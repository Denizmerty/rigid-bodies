[CmdletBinding()]
param(
    [ValidateRange(1, 128)]
    [int]$Jobs = [Environment]::ProcessorCount,
    [switch]$Clean,
    [switch]$CleanOnly,
    [switch]$Installer,
    [switch]$Tests,
    [switch]$Run,
    [switch]$NonInteractive
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$projectRoot = $PSScriptRoot
$buildDirectory = Join-Path $projectRoot 'build'
$application = Join-Path $buildDirectory 'rigid_bodies.exe'

function Assert-ChildPath([string]$Path)
{
    $root = [IO.Path]::GetFullPath($projectRoot).TrimEnd('\') + '\'
    $candidate = [IO.Path]::GetFullPath($Path)
    if (-not $candidate.StartsWith($root, [StringComparison]::OrdinalIgnoreCase))
    {
        throw "Path is outside the project: $candidate"
    }
    return $candidate
}

function Get-VisualStudio
{
    $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
    if (-not (Test-Path -LiteralPath $vswhere))
    {
        throw 'Visual Studio Installer is unavailable.'
    }
    $installation = (& $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath).Trim()
    if (-not $installation)
    {
        throw 'A Visual Studio installation with the C++ toolchain is required.'
    }
    return $installation
}

function Get-BuildTool([string]$Name, [string]$VisualStudioRelativePath, [string]$VisualStudio)
{
    $command = Get-Command $Name -ErrorAction SilentlyContinue
    if ($command)
    {
        return $command.Source
    }

    $tool = Join-Path $VisualStudio $VisualStudioRelativePath
    if (-not (Test-Path -LiteralPath $tool))
    {
        throw "$Name was not found at $tool."
    }
    return $tool
}

$buildDirectory = Assert-ChildPath $buildDirectory
# CMake keeps its own files in build\cmake. A CMake cache at the top of build\ is the previous
# layout, whose test executables would sit beside the application, so that directory is recreated.
if (-not $CleanOnly -and (Test-Path -LiteralPath (Join-Path $buildDirectory 'CMakeCache.txt')))
{
    Write-Host 'Recreating build\ from its previous layout.'
    $Clean = $true
}
if (($Clean -or $CleanOnly) -and (Test-Path -LiteralPath $buildDirectory))
{
    Remove-Item -LiteralPath $buildDirectory -Recurse -Force
}
if ($CleanOnly)
{
    Write-Host "Removed $buildDirectory"
    exit 0
}

$visualStudio = Get-VisualStudio
$developerShell = Join-Path $visualStudio 'Common7\Tools\Microsoft.VisualStudio.DevShell.dll'
Import-Module $developerShell
Enter-VsDevShell -VsInstallPath $visualStudio -SkipAutomaticLocation -DevCmdArguments '-arch=x64 -host_arch=x64' | Out-Null
$cmake = Get-BuildTool 'cmake.exe' 'Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe' $visualStudio
$ninja = Get-BuildTool 'ninja.exe' 'Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe' $visualStudio
$ctest = Join-Path (Split-Path -Parent $cmake) 'ctest.exe'
# The development preset builds the same application plus the tests and tools.
$preset = if ($Tests) { 'development' } else { 'default' }

Push-Location $projectRoot
try
{
    & $cmake --preset $preset "-DCMAKE_MAKE_PROGRAM=$ninja"
    if ($LASTEXITCODE -ne 0) { throw "CMake configuration failed (exit $LASTEXITCODE)." }

    & $cmake --build --preset $preset --parallel $Jobs
    if ($LASTEXITCODE -ne 0) { throw "CMake build failed (exit $LASTEXITCODE)." }

    if ($Tests)
    {
        & $ctest --preset development
        if ($LASTEXITCODE -ne 0) { throw "Tests failed (exit $LASTEXITCODE)." }
    }
}
finally
{
    Pop-Location
}

if (-not (Test-Path -LiteralPath $application))
{
    throw "The build did not produce $application."
}
$topLevelExecutables = @(Get-ChildItem -LiteralPath $buildDirectory -File -Filter '*.exe')
if ($topLevelExecutables.Count -ne 1 -or $topLevelExecutables[0].Name -ne 'rigid_bodies.exe')
{
    throw "The build directory must contain only rigid_bodies.exe at its top level; tests and tools belong in build\tests and build\tools."
}

if ($Installer)
{
    & (Join-Path $projectRoot 'scripts\build-installer.ps1') -BuildDirectory $buildDirectory
    if ($LASTEXITCODE -ne 0) { throw "Installer build failed (exit $LASTEXITCODE)." }
}

Write-Host ''
Write-Host 'Build complete.' -ForegroundColor Green
Write-Host "  Executable: $application"
if ($Run)
{
    Start-Process -FilePath $application -WorkingDirectory $buildDirectory | Out-Null
}
