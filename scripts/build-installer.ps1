#requires -Version 7.0
[CmdletBinding()]
param(
    [string]$BuildDirectory = (Join-Path (Split-Path $PSScriptRoot -Parent) 'build'),
    [string]$OutputDirectory = (Join-Path (Split-Path $PSScriptRoot -Parent) 'build\installer'),
    [string]$MakeNsis,
    [switch]$NoDownload,
    [ValidatePattern('^[a-zA-Z0-9-]+$')][string]$TestInstanceId,
    [ValidatePattern('^\d+\.\d+\.\d+$')][string]$TestVersion
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$ProgressPreference = 'SilentlyContinue'
$projectRoot = Split-Path $PSScriptRoot -Parent
$nsisVersion = '3.12'
$nsisArchiveHash = '56581f90db321581c5381193d796fffcf2d24b2f8fed2160a6c6a3baa67f2c4f'
$nsisUrl = 'https://sourceforge.net/projects/nsis/files/NSIS%203/3.12/nsis-3.12.zip/download'

function Get-Identity([string]$Name)
{
    $identity = Get-Content -LiteralPath (Join-Path $projectRoot 'src\RigidBodies.Core\include\rigidbodies\project_identity.hpp') -Raw -Encoding UTF8
    $match = [regex]::Match($identity, ('(?m)^#define RIGIDBODIES_' + [regex]::Escape($Name) + ' "([^"]+)"\r?$'))
    if (-not $match.Success) { throw "Could not read project identity field $Name." }
    return $match.Groups[1].Value
}

function Remove-PackageWork([string]$Path)
{
    $resolved = [IO.Path]::GetFullPath($Path)
    $allowed = [IO.Path]::GetFullPath($outputPath).TrimEnd('\') + '\'
    if (-not $resolved.StartsWith($allowed, [StringComparison]::OrdinalIgnoreCase) -or (Split-Path $resolved -Leaf) -notmatch '^\.package-[a-f0-9]{32}$')
    {
        throw "Refusing to remove an unexpected packaging directory: $resolved"
    }
    if (Test-Path -LiteralPath $resolved) { Remove-Item -LiteralPath $resolved -Recurse -Force }
}

function Get-MakeNsis
{
    if ($MakeNsis)
    {
        if (-not (Test-Path -LiteralPath $MakeNsis -PathType Leaf)) { throw "makensis was not found at $MakeNsis." }
        return (Resolve-Path -LiteralPath $MakeNsis).Path
    }
    $installed = Get-Command 'makensis.exe' -ErrorAction SilentlyContinue
    if ($installed -and ((& $installed.Source '/VERSION' | Out-String).Trim() -eq "v$nsisVersion")) { return $installed.Source }
    $cache = Join-Path $projectRoot 'build\tool-cache'
    $archive = Join-Path $cache "nsis-$nsisVersion.zip"
    if (-not (Test-Path -LiteralPath $archive))
    {
        if ($NoDownload) { throw 'NSIS is not installed or cached and -NoDownload was specified.' }
        New-Item -ItemType Directory -Force -Path $cache | Out-Null
        Invoke-WebRequest -Uri $nsisUrl -OutFile $archive
        $signature = [IO.File]::ReadAllBytes($archive)
        if ($signature.Length -lt 2 -or $signature[0] -ne 0x50 -or $signature[1] -ne 0x4b)
        {
            $refresh = [regex]::Match((Get-Content -LiteralPath $archive -Raw), '<meta http-equiv="refresh" content="[0-9]+; url=([^"]+)"')
            if (-not $refresh.Success) { throw 'The NSIS download was not a ZIP archive.' }
            Invoke-WebRequest -Uri ([Net.WebUtility]::HtmlDecode($refresh.Groups[1].Value)) -OutFile $archive
        }
    }
    $actual = (Get-FileHash -LiteralPath $archive -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($actual -ne $nsisArchiveHash) { throw "NSIS checksum mismatch: $actual. Remove $archive and retry." }
    $toolRoot = Join-Path $work 'nsis'
    Expand-Archive -LiteralPath $archive -DestinationPath $toolRoot
    return Join-Path $toolRoot "nsis-$nsisVersion\makensis.exe"
}

if ($TestVersion -and -not $TestInstanceId) { throw '-TestVersion is only allowed with an isolated -TestInstanceId.' }
$version = Get-Identity 'VERSION_STRING'
$buildPath = (Resolve-Path -LiteralPath $BuildDirectory).Path
foreach ($item in @('rigid_bodies.exe', 'SDL3.dll', 'assets', 'config'))
{
    if (-not (Test-Path -LiteralPath (Join-Path $buildPath $item))) { throw "Build output is missing $item." }
}
$fileVersion = [Diagnostics.FileVersionInfo]::GetVersionInfo((Join-Path $buildPath 'rigid_bodies.exe'))
$exeVersion = $fileVersion.ProductVersion
if ($exeVersion -ne $version -or $fileVersion.FileVersion -ne $version) { throw "Application product/file version differs from the project version $version. Rebuild the application before packaging." }
$runtimeImports = & (Join-Path $PSScriptRoot 'verify-runtime-dependencies.ps1') -BuildDirectory $buildPath
if ($TestVersion) { $version = $TestVersion }
if ($version -notmatch '^\d+\.\d+\.\d+$' -or @($version.Split('.') | Where-Object { [long]$_ -gt 65535 }).Count) { throw "Invalid Windows product version: $version" }
New-Item -ItemType Directory -Force -Path $OutputDirectory | Out-Null
$outputPath = (Resolve-Path -LiteralPath $OutputDirectory).Path
$work = Join-Path $outputPath ('.package-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $work | Out-Null
$payload = Join-Path $work 'payload'
try
{
    New-Item -ItemType Directory -Path $payload | Out-Null
    foreach ($item in @('rigid_bodies.exe', 'SDL3.dll', 'assets', 'config'))
    {
        Copy-Item -LiteralPath (Join-Path $buildPath $item) -Destination $payload -Recurse
    }
    foreach ($item in @('README.md', 'LICENSE')) { Copy-Item -LiteralPath (Join-Path $projectRoot $item) -Destination $payload }
    New-Item -ItemType Directory -Path (Join-Path $payload 'docs') | Out-Null
    Copy-Item -LiteralPath (Join-Path $projectRoot 'docs\DISTRIBUTION.md') -Destination (Join-Path $payload 'docs')
    $files = @(Get-ChildItem -LiteralPath $payload -File -Recurse | Sort-Object FullName)
    $manifest = foreach ($file in $files)
    {
        $relative = [IO.Path]::GetRelativePath($payload, $file.FullName).Replace('\', '/')
        if ($relative -match '[\r\n"$]' -or $relative -match '[^\x20-\x7E]') { throw "Unsupported package filename: $relative" }
        $hash = (Get-FileHash -LiteralPath $file.FullName -Algorithm SHA256).Hash.ToLowerInvariant()
        "$hash  $relative"
    }
    Set-Content -LiteralPath (Join-Path $payload 'PACKAGE-MANIFEST.sha256') -Value $manifest -Encoding ascii
    $packageFiles = @(Get-ChildItem -LiteralPath $payload -File -Recurse | ForEach-Object { [IO.Path]::GetRelativePath($payload, $_.FullName) })
    $packageFiles += @('PACKAGE-FILES.txt', 'Uninstall.exe')
    Set-Content -LiteralPath (Join-Path $payload 'PACKAGE-FILES.txt') -Value ($packageFiles | Sort-Object) -Encoding ascii
    $compiler = Get-MakeNsis
    $compilerVersion = (& $compiler '/VERSION' | Out-String).Trim()
    if ($compilerVersion -ne "v$nsisVersion") { throw "NSIS $nsisVersion is required; $compiler reports $compilerVersion." }
    $defines = @('/WX', '/V3', "/DVERSION=$version", "/DAUTHOR=$(Get-Identity 'AUTHOR')", "/DCONTACT=$(Get-Identity 'CONTACT')", "/DCOPYRIGHT=$(Get-Identity 'COPYRIGHT')", "/DPAYLOAD=$payload", "/DOUTPUT_DIRECTORY=$outputPath", "/DBRANDING_DIRECTORY=$(Join-Path $projectRoot 'packaging\installer')", "/DICON_FILE=$(Join-Path $projectRoot 'assets\branding\rigid-bodies.ico')")
    if ($TestInstanceId) { $defines += "/DTEST_INSTANCE=$TestInstanceId" }
    & $compiler @defines (Join-Path $projectRoot 'packaging\rigid-bodies.nsi')
    if ($LASTEXITCODE -ne 0) { throw "makensis failed (exit $LASTEXITCODE)." }
    $suffix = if ($TestInstanceId) { "-test-$TestInstanceId" } else { '' }
    $installer = Join-Path $outputPath "Rigid-Bodies-$version-windows-x64$suffix-setup.exe"
    if (-not (Test-Path -LiteralPath $installer)) { throw "Installer was not created: $installer" }
    $hash = (Get-FileHash -LiteralPath $installer -Algorithm SHA256).Hash.ToLowerInvariant()
    Set-Content -LiteralPath "$installer.sha256" -Value "$hash  $(Split-Path $installer -Leaf)" -Encoding ascii
    $metadata = [ordered]@{ version = $version; applicationVersion = $exeVersion; architecture = 'x64'; installer = (Split-Path $installer -Leaf); sha256 = $hash; nsisVersion = $nsisVersion; runtime = @('rigid_bodies.exe', 'SDL3.dll'); runtimeImports = $runtimeImports; testInstance = $TestInstanceId }
    if (-not $TestInstanceId)
    {
        $portable = Join-Path $outputPath "Rigid-Bodies-$version-windows-x64.zip"
        # The portable archive has no installer bookkeeping or uninstaller.
        Remove-Item -LiteralPath (Join-Path $payload 'PACKAGE-FILES.txt')
        Compress-Archive -Path (Join-Path $payload '*') -DestinationPath $portable -Force
        $portableHash = (Get-FileHash -LiteralPath $portable -Algorithm SHA256).Hash.ToLowerInvariant()
        Set-Content -LiteralPath "$portable.sha256" -Value "$portableHash  $(Split-Path $portable -Leaf)" -Encoding ascii
        $metadata.portable = Split-Path $portable -Leaf
        $metadata.portableSha256 = $portableHash
    }
    $metadata | ConvertTo-Json | Set-Content -LiteralPath "$installer.json" -Encoding utf8
    Write-Host "Installer: $installer" -ForegroundColor Green
    Write-Host "SHA-256 : $hash"
}
finally { Remove-PackageWork $work }
