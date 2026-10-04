#requires -Version 7.0
[CmdletBinding()]
param(
    [ValidateRange(1, 128)]
    [int]$Jobs = [Environment]::ProcessorCount,
    [string]$Python = 'python',
    [string]$Tag,
    [string]$MakeNsis,
    [switch]$NoDownload
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
$buildDirectory = Join-Path $projectRoot 'build'
$releaseDirectory = [IO.Path]::GetFullPath((Join-Path $buildDirectory 'release\current'))
$allowedRoot = [IO.Path]::GetFullPath($buildDirectory).TrimEnd('\') + '\'
if (-not $releaseDirectory.StartsWith($allowedRoot, [StringComparison]::OrdinalIgnoreCase))
{
    throw 'The release directory is outside the build tree.'
}
New-Item -ItemType Directory -Path $buildDirectory -Force | Out-Null
if (Test-Path -LiteralPath $releaseDirectory) { Remove-Item -LiteralPath $releaseDirectory -Recurse -Force }

function Assert-ExitCode([string]$Operation)
{
    if ($LASTEXITCODE -ne 0) { throw "$Operation failed (exit $LASTEXITCODE)." }
}

Push-Location $projectRoot
Start-Transcript -LiteralPath (Join-Path $buildDirectory 'release.log') -Force | Out-Null
try
{
    $version = & (Join-Path $PSScriptRoot 'version.ps1') -Tag $Tag
    & (Join-Path $PSScriptRoot 'test-version.ps1')
    & $Python scripts/test_compare_determinism.py
    Assert-ExitCode 'Determinism comparator tests'
    & $Python scripts/test_static_analysis.py
    Assert-ExitCode 'Static-analysis helper tests'

    # Both build systems copy changed resources but do not remove deleted ones. Start their
    # staging trees empty so a local release has the same files as a fresh CI checkout.
    foreach ($directory in @('assets', 'config'))
    {
        $staged = [IO.Path]::GetFullPath((Join-Path $buildDirectory $directory))
        if (-not $staged.StartsWith($allowedRoot, [StringComparison]::OrdinalIgnoreCase))
        {
            throw 'The runtime staging directory is outside the build tree.'
        }
        if (Test-Path -LiteralPath $staged) { Remove-Item -LiteralPath $staged -Recurse -Force }
    }
    & (Join-Path $projectRoot 'build.ps1') -Tests -Jobs $Jobs -NonInteractive
    & $Python scripts/generate_visual_studio.py --check
    Assert-ExitCode 'Visual Studio project check'

    $application = Join-Path $buildDirectory 'rigid_bodies.exe'
    $cmakeHash = (Get-FileHash -LiteralPath $application -Algorithm SHA256).Hash
    $msbuild = Join-Path $env:VSINSTALLDIR 'MSBuild\Current\Bin\amd64\MSBuild.exe'
    & $msbuild RigidBodies.sln /p:Configuration=Release /p:Platform=x64 "/m:$Jobs" /nologo /v:minimal
    Assert-ExitCode 'Native Visual Studio build'
    $nativeHash = (Get-FileHash -LiteralPath $application -Algorithm SHA256).Hash
    if ($nativeHash -cne $cmakeHash) { throw 'Visual Studio and CMake produced different application executables.' }
    Write-Host "CMake and Visual Studio executable SHA-256: $($cmakeHash.ToLowerInvariant())"

    & (Join-Path $buildDirectory 'tools\rigid_bodies_determinism_probe.exe') --verify --output build/trace.json
    Assert-ExitCode 'Determinism replay'
    & $Python scripts/compare_determinism.py tests/fixtures/determinism/reference-v1.json build/trace.json
    Assert-ExitCode 'Determinism reference comparison'

    $installerOptions = @{ BuildDirectory = $buildDirectory; NoDownload = $NoDownload }
    if ($MakeNsis) { $installerOptions.MakeNsis = $MakeNsis }
    & (Join-Path $PSScriptRoot 'build-installer.ps1') @installerOptions
    $baseName = "Rigid-Bodies-$version-windows-x64"
    $installer = Join-Path $buildDirectory "installer\$baseName-setup.exe"
    $testOptions = @{ Installer = $installer; BuildDirectory = $buildDirectory }
    if ($MakeNsis) { $testOptions.MakeNsis = $MakeNsis }
    if ($NoDownload) { $testOptions.NoDownload = $true }
    & (Join-Path $PSScriptRoot 'test-installer.ps1') @testOptions

    New-Item -ItemType Directory -Path $releaseDirectory -Force | Out-Null
    Copy-Item -LiteralPath $installer -Destination $releaseDirectory
    # The installer builder creates both formats from the same staged payload.
    Copy-Item -LiteralPath (Join-Path $buildDirectory "installer\$baseName.zip") -Destination $releaseDirectory

    $commit = git rev-parse HEAD
    Assert-ExitCode 'Reading the source commit'
    $workingChanges = @(git status --porcelain --untracked-files=normal)
    Assert-ExitCode 'Reading the working tree status'
    if ($Tag -and $workingChanges.Count -ne 0) { throw 'A tagged release must be built from a clean working tree.' }
    $provenance = [ordered]@{
        version = $version
        commit = $commit
        working_tree_modified = ($workingChanges.Count -ne 0)
        platform = 'windows-x64'
        visual_studio = $env:VSCMD_VER
        msvc_toolset = $env:VCToolsVersion
        windows_sdk = $env:WindowsSDKVersion
        application_sha256 = $cmakeHash.ToLowerInvariant()
        cmake_visual_studio_match = $true
        created_utc = [DateTime]::UtcNow.ToString('o')
    }
    $provenance | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $releaseDirectory "$baseName-build.json") -Encoding utf8
    $releaseNotes = Join-Path $projectRoot "docs/releases/$version.md"
    if (Test-Path -LiteralPath $releaseNotes -PathType Leaf)
    {
        Copy-Item -LiteralPath $releaseNotes -Destination (Join-Path $releaseDirectory 'RELEASE-NOTES.md')
    }
    $checksums = Get-ChildItem -LiteralPath $releaseDirectory -File | Sort-Object Name | ForEach-Object {
        "$((Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash.ToLowerInvariant())  $($_.Name)"
    }
    Set-Content -LiteralPath (Join-Path $releaseDirectory 'SHA256SUMS') -Value $checksums -Encoding ascii
    Write-Host "Release $version passed validation. Packages: $releaseDirectory" -ForegroundColor Green
}
finally
{
    Stop-Transcript | Out-Null
    Pop-Location
}
