#requires -Version 7.0
[CmdletBinding()]
param(
    [Parameter(Mandatory)][string]$BuildDirectory,
    [string[]]$RuntimeFiles = @('rigid_bodies.exe', 'SDL3.dll'),
    [string]$Dumpbin
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

if (-not $Dumpbin)
{
    $command = Get-Command 'dumpbin.exe' -ErrorAction SilentlyContinue
    if ($command) { $Dumpbin = $command.Source }
    else
    {
        $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
        if (Test-Path -LiteralPath $vswhere)
        {
            $vs = (& $vswhere -latest -prerelease -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath | Select-Object -First 1)
            if ($vs)
            {
                $versionFile = Join-Path $vs 'VC\Auxiliary\Build\Microsoft.VCToolsVersion.default.txt'
                if (Test-Path -LiteralPath $versionFile)
                {
                    $toolset = (Get-Content -LiteralPath $versionFile -Raw).Trim()
                    $Dumpbin = Join-Path $vs "VC\Tools\MSVC\$toolset\bin\Hostx64\x64\dumpbin.exe"
                }
            }
        }
    }
}
if (-not $Dumpbin -or -not (Test-Path -LiteralPath $Dumpbin -PathType Leaf))
{
    throw 'Runtime verification needs dumpbin.exe from the Visual Studio C++ workload. Open Developer PowerShell for Visual Studio or supply -Dumpbin.'
}

$root = (Resolve-Path -LiteralPath $BuildDirectory).Path
$packaged = [Collections.Generic.HashSet[string]]::new([StringComparer]::OrdinalIgnoreCase)
foreach ($file in $RuntimeFiles)
{
    if ([IO.Path]::GetFileName($file) -ne $file) { throw "Runtime filenames must be relative leaf names: $file" }
    if (-not (Test-Path -LiteralPath (Join-Path $root $file) -PathType Leaf)) { throw "Missing packaged runtime file: $file" }
    [void]$packaged.Add($file)
}

# Explicit OS components, not whatever happens to exist on the build machine.
# Do not add vcruntime/msvcp DLLs here: those require a redistribution decision.
$system = [Collections.Generic.HashSet[string]]::new([StringComparer]::OrdinalIgnoreCase)
foreach ($file in @('KERNEL32.dll', 'USER32.dll', 'SHELL32.dll', 'GDI32.dll', 'WINMM.dll', 'IMM32.dll', 'ole32.dll', 'OLEAUT32.dll', 'VERSION.dll', 'ADVAPI32.dll', 'SETUPAPI.dll'))
{
    [void]$system.Add($file)
}

$imports = [ordered]@{}
foreach ($file in $RuntimeFiles)
{
    $report = @(& $Dumpbin '/NOLOGO' '/DEPENDENTS' (Join-Path $root $file) 2>&1)
    if ($LASTEXITCODE -ne 0) { throw "dumpbin could not inspect $file (exit $LASTEXITCODE): $($report -join ' ')" }
    $dependencies = @($report | ForEach-Object { if ($_ -match '^\s+([^\r\n]+\.dll)\s*$') { $Matches[1].Trim() } } | Sort-Object -Unique)
    if ($dependencies.Count -eq 0) { throw "No DLL imports were found in $file. Check the input binary and dumpbin output." }
    foreach ($dependency in $dependencies)
    {
        if ($packaged.Contains($dependency) -or $system.Contains($dependency) -or $dependency -match '^(api-ms-win-|ext-ms-win-)[A-Za-z0-9_.-]+\.dll$') { continue }
        throw "$file imports $dependency, which is neither bundled nor an approved Windows component. Add the required runtime to the package or restore static linkage; do not rely on a DLL installed only on the build machine."
    }
    $imports[$file] = $dependencies
}
Write-Host "Runtime dependency check passed for $($RuntimeFiles -join ', ')."
# Every bundled binary was inspected, so DLL-to-DLL imports are checked as well.
return $imports
