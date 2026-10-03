[CmdletBinding()]
param(
    [string]$Executable = '',
    [string]$HistoryDirectory = "$PSScriptRoot/../build/benchmark-results",
    [string]$Label = 'local',
    [string]$Cpu = '',
    [string[]]$Workload = @(),
    [ValidateRange(0, 100000)][int]$WarmupSteps = 30,
    [ValidateRange(1, 100000)][int]$SampleSteps = 120,
    [ValidateRange(3, 100)][int]$Repetitions = 3,
    [ValidateRange(0, 32)][int]$Workers = 1,
    [switch]$DisableProfiling,
    [ValidateRange(0, 100)][double]$RelativeThreshold = 0.20,
    [ValidateRange(0, 10000)][double]$AbsoluteThresholdMs = 0.05,
    [ValidateRange(0, 100)][double]$NoiseMultiplier = 3.0,
    [ValidateRange(0, 100)][double]$MaximumRelativeSpread = 0.25
)

$ErrorActionPreference = 'Stop'
if (!$Executable) {
    $benchmarkPackagedExecutable = Join-Path $PSScriptRoot '../rigid_bodies_benchmark.exe'
    $Executable = if (Test-Path -LiteralPath $benchmarkPackagedExecutable -PathType Leaf) { $benchmarkPackagedExecutable } else { "$PSScriptRoot/../build/tools/rigid_bodies_benchmark.exe" }
}
$benchmarkExecutable = (Resolve-Path -LiteralPath $Executable).Path
$benchmarkHistory = [System.IO.Path]::GetFullPath($HistoryDirectory)
New-Item -ItemType Directory -Path $benchmarkHistory -Force | Out-Null
$benchmarkStamp = [DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$benchmarkRecord = Join-Path $benchmarkHistory "$benchmarkStamp-$([Guid]::NewGuid().ToString('N').Substring(0, 8)).json"
$benchmarkArguments = @('--warmup', "$WarmupSteps", '--samples', "$SampleSteps", '--repetitions', "$Repetitions", '--workers', "$Workers", '--label', $Label, '--output', $benchmarkRecord)
if ($DisableProfiling) { $benchmarkArguments += '--no-profiling' }
if ($Cpu) { $benchmarkArguments += @('--cpu', $Cpu) }
foreach ($benchmarkWorkload in $Workload) { $benchmarkArguments += @('--workload', $benchmarkWorkload) }
& $benchmarkExecutable @benchmarkArguments
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
$benchmarkCurrent = Get-Content -LiteralPath $benchmarkRecord -Raw | ConvertFrom-Json
Write-Host "Recorded $benchmarkRecord"

# Select the newest completely matching report. Different hardware, compiler/configuration,
# sampling, worker counts, or scene content establishes a separate result series.
$benchmarkKeys = @('machine', 'platform', 'architecture', 'cpu', 'hardware_threads', 'compiler', 'build', 'project_version')
$benchmarkSettingKeys = @('warmup_steps', 'sample_steps', 'repetitions', 'time_step_s', 'seed', 'worker_count', 'profiling')
$benchmarkBaseline = $null
foreach ($benchmarkFile in (Get-ChildItem -LiteralPath $benchmarkHistory -Filter '*.json' -File | Sort-Object Name -Descending)) {
    if ($benchmarkFile.FullName -eq $benchmarkRecord) { continue }
    $benchmarkStatusPath = "$($benchmarkFile.FullName).comparison-status"
    if ((Test-Path -LiteralPath $benchmarkStatusPath) -and ((Get-Content -LiteralPath $benchmarkStatusPath -Raw).Trim() -ne '0')) { continue }
    try { $benchmarkPrevious = Get-Content -LiteralPath $benchmarkFile.FullName -Raw | ConvertFrom-Json }
    catch { Write-Warning "Skipping unreadable result $($benchmarkFile.Name)"; continue }
    if ($benchmarkPrevious.format -ne 'rigid-bodies-benchmark' -or $benchmarkPrevious.version.major -ne 1 -or $benchmarkPrevious.suite_revision -ne 1) { continue }
    $benchmarkMatches = $true
    foreach ($benchmarkKey in $benchmarkKeys) {
        if (!$benchmarkPrevious.metadata.$benchmarkKey -or $benchmarkPrevious.metadata.$benchmarkKey -ne $benchmarkCurrent.metadata.$benchmarkKey) { $benchmarkMatches = $false }
    }
    foreach ($benchmarkKey in $benchmarkSettingKeys) {
        if ($null -eq $benchmarkPrevious.settings.$benchmarkKey -or $benchmarkPrevious.settings.$benchmarkKey -ne $benchmarkCurrent.settings.$benchmarkKey) { $benchmarkMatches = $false }
    }
    foreach ($benchmarkResult in $benchmarkCurrent.results) {
        $benchmarkOldResult = @($benchmarkPrevious.results | Where-Object { $_.workload -eq $benchmarkResult.workload })
        if ($benchmarkOldResult.Count -ne 1 -or $benchmarkOldResult[0].content_fingerprint -ne $benchmarkResult.content_fingerprint) { $benchmarkMatches = $false }
    }
    if ($benchmarkMatches) { $benchmarkBaseline = $benchmarkFile.FullName; break }
}
if ($null -eq $benchmarkBaseline) {
    Write-Host 'No compatible previous recording; this run establishes a baseline.'
    Set-Content -LiteralPath "$benchmarkRecord.comparison-status" -Value '0' -Encoding ascii
    exit 0
}
Write-Host "Comparing with $benchmarkBaseline"
$benchmarkCulture = [System.Globalization.CultureInfo]::InvariantCulture
& $benchmarkExecutable --compare $benchmarkRecord --baseline $benchmarkBaseline `
    --relative-threshold $RelativeThreshold.ToString($benchmarkCulture) `
    --absolute-threshold-ms $AbsoluteThresholdMs.ToString($benchmarkCulture) `
    --noise-multiplier $NoiseMultiplier.ToString($benchmarkCulture) `
    --maximum-relative-spread $MaximumRelativeSpread.ToString($benchmarkCulture)
$benchmarkExitCode = $LASTEXITCODE
# Keep failed candidates as evidence, but never silently promote a confirmed regression into the
# next automatic baseline. Otherwise an unchanged regression would pass on its second invocation.
Set-Content -LiteralPath "$benchmarkRecord.comparison-status" -Value "$benchmarkExitCode" -Encoding ascii
exit $benchmarkExitCode
