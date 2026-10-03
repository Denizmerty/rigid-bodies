#requires -Version 7.0
[CmdletBinding()]
param(
    [Alias('InstallerPath')][string]$Installer,
    [string]$BuildDirectory = (Join-Path (Split-Path $PSScriptRoot -Parent) 'build'),
    [string]$MakeNsis,
    [switch]$NoDownload,
    [switch]$SkipRenderSmoke
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
$testId = [guid]::NewGuid().ToString('N')
$testRoot = [IO.Path]::GetFullPath((Join-Path $projectRoot "build\installer-tests\$testId"))
$allowedRoot = [IO.Path]::GetFullPath((Join-Path $projectRoot 'build\installer-tests')).TrimEnd('\') + '\'
if (-not $testRoot.StartsWith($allowedRoot, [StringComparison]::OrdinalIgnoreCase)) { throw 'Installer test root is outside the build tree.' }
$installRoot = Join-Path $testRoot 'installed'
$fixtureBuild = Join-Path $testRoot 'build'
$output = Join-Path $testRoot 'packages'
$appKey = "HKCU:\Software\RigidBodies-InstallerTest-$testId"
$uninstallKey = "HKCU:\Software\Microsoft\Windows\CurrentVersion\Uninstall\RigidBodies-InstallerTest-$testId"
$shortcutRoot = Join-Path ([Environment]::GetFolderPath('Programs')) "Rigid Bodies Installer Test $testId"
$originalAppData = $env:APPDATA
$passed = $false

function Assert-True([bool]$Condition, [string]$Message)
{
    if (-not $Condition) { throw $Message }
}

function Invoke-Setup([string]$Path, [int]$ExpectedExit = 0, [string]$Destination = $installRoot)
{
    $process = Start-Process -FilePath $Path -ArgumentList @('/S', "/D=$Destination") -Wait -PassThru -WindowStyle Hidden
    Assert-True ($process.ExitCode -eq $ExpectedExit) "Installer exited $($process.ExitCode); expected $ExpectedExit."
}

function Assert-Payload
{
    foreach ($line in Get-Content -LiteralPath (Join-Path $installRoot 'PACKAGE-MANIFEST.sha256'))
    {
        Assert-True ($line -match '^([0-9a-f]{64})  (.+)$') "Malformed checksum line: $line"
        $expected = $Matches[1]
        $file = [IO.Path]::GetFullPath((Join-Path $installRoot $Matches[2]))
        Assert-True ($file.StartsWith($installRoot + '\', [StringComparison]::OrdinalIgnoreCase)) 'Package checksum path escaped the install directory.'
        Assert-True (Test-Path -LiteralPath $file -PathType Leaf) "Missing installed file: $file"
        Assert-True ((Get-FileHash -LiteralPath $file -Algorithm SHA256).Hash -eq $expected) "Checksum mismatch: $file"
    }
    Assert-True (-not (Test-Path -LiteralPath (Join-Path $installRoot '.rigid-bodies-update'))) 'Completed setup left its transaction directory behind.'
}

function Invoke-Uninstall
{
    # NSIS normally launches a temporary child and its bootstrapper returns first.
    # Run our own temporary copy with _?= so tests observe the actual removal exit code.
    $runner = Join-Path $testRoot 'uninstall-runner.exe'
    Copy-Item -LiteralPath (Join-Path $installRoot 'Uninstall.exe') -Destination $runner -Force
    return Start-Process -FilePath $runner -ArgumentList @('/S', "_?=$installRoot") -Wait -PassThru -WindowStyle Hidden
}

function Assert-Registration([string]$ExpectedVersion)
{
    $record = Get-ItemProperty -LiteralPath $uninstallKey
    Assert-True ($record.DisplayVersion -eq $ExpectedVersion) 'Installed Apps has the wrong version.'
    Assert-True ($record.InstallLocation -eq $installRoot) 'Installed Apps has the wrong installation path.'
    Assert-True ((Get-ItemProperty -LiteralPath $appKey).InstallDirectory -eq $installRoot) 'The installation moved during an update.'
    $records = @(Get-ChildItem 'HKCU:\Software\Microsoft\Windows\CurrentVersion\Uninstall' | Where-Object PSChildName -eq "RigidBodies-InstallerTest-$testId")
    Assert-True ($records.Count -eq 1) 'Setup created duplicate Installed Apps records.'
    $shell = New-Object -ComObject WScript.Shell
    $shortcut = $shell.CreateShortcut((Join-Path $shortcutRoot 'Rigid Bodies.lnk'))
    Assert-True ($shortcut.TargetPath -eq (Join-Path $installRoot 'rigid_bodies.exe')) 'The Start-menu shortcut target changed.'
    [Runtime.InteropServices.Marshal]::ReleaseComObject($shell) | Out-Null
}

try
{
    if ($Installer)
    {
        $Installer = (Resolve-Path -LiteralPath $Installer).Path
        $metadata = Get-Content -LiteralPath "$Installer.json" -Raw | ConvertFrom-Json
        Assert-True (-not $metadata.testInstance) 'The release artifact must not use a test namespace.'
        Assert-True ((Get-FileHash -LiteralPath $Installer -Algorithm SHA256).Hash -eq $metadata.sha256) 'Release installer checksum mismatch.'
    }
    New-Item -ItemType Directory -Path $fixtureBuild, $output | Out-Null
    foreach ($item in @('rigid_bodies.exe', 'SDL3.dll', 'assets', 'config'))
    {
        Copy-Item -LiteralPath (Join-Path $BuildDirectory $item) -Destination $fixtureBuild -Recurse
    }
    $version = [Diagnostics.FileVersionInfo]::GetVersionInfo((Join-Path $fixtureBuild 'rigid_bodies.exe')).ProductVersion
    $parts = $version.Split('.')
    $nextVersion = '{0}.{1}.{2}' -f $parts[0], $parts[1], ([int]$parts[2] + 1)
    $obsolete = Join-Path $fixtureBuild 'assets\installer-test-obsolete.txt'
    $introduced = Join-Path $fixtureBuild 'assets\installer-test-introduced.txt'
    Set-Content -LiteralPath $obsolete -Value 'Old package fixture.'
    $packageArgs = @{ BuildDirectory = $fixtureBuild; OutputDirectory = $output; TestInstanceId = $testId; NoDownload = $NoDownload }
    if ($MakeNsis) { $packageArgs.MakeNsis = $MakeNsis }
    & (Join-Path $PSScriptRoot 'build-installer.ps1') @packageArgs
    $baseInstaller = Join-Path $output "Rigid-Bodies-$version-windows-x64-test-$testId-setup.exe"
    Remove-Item -LiteralPath $obsolete
    Set-Content -LiteralPath $introduced -Value 'New package fixture.'
    & (Join-Path $PSScriptRoot 'build-installer.ps1') @packageArgs -TestVersion $nextVersion
    $nextInstaller = Join-Path $output "Rigid-Bodies-$nextVersion-windows-x64-test-$testId-setup.exe"

    $env:APPDATA = Join-Path $testRoot 'user-data'
    $preferences = Join-Path $env:APPDATA 'RigidBodies\personal-test.txt'
    New-Item -ItemType Directory -Path (Split-Path $preferences -Parent) | Out-Null
    Set-Content -LiteralPath $preferences -Value 'Keep my preferences.'
    Invoke-Setup $baseInstaller
    Assert-Payload
    Assert-Registration $version
    Set-Content -LiteralPath (Join-Path $installRoot 'personal-test.txt') -Value 'Keep my files.'
    Set-Content -LiteralPath (Join-Path $installRoot 'assets\personal-test.json') -Value '{}'
    $ignoredDestination = Join-Path $testRoot 'must-not-be-created'
    Invoke-Setup $baseInstaller -Destination $ignoredDestination
    Assert-Payload
    Assert-Registration $version
    Assert-True (-not (Test-Path -LiteralPath $ignoredDestination)) 'Reinstall created a second installation.'

    # Simulate a rollback interrupted after one old file was already restored.
    $transaction = Join-Path $installRoot '.rigid-bodies-update'
    New-Item -ItemType Directory -Path (Join-Path $transaction 'stage'), (Join-Path $transaction 'backup') | Out-Null
    Set-Content -LiteralPath (Join-Path $transaction 'state.ini') -Encoding ascii -Value @('[Transaction]', "Owner=RigidBodies-InstallerTest-$testId")
    Copy-Item -LiteralPath (Join-Path $installRoot 'PACKAGE-FILES.txt') -Destination (Join-Path $transaction 'old-files.txt')
    Copy-Item -LiteralPath (Join-Path $installRoot 'PACKAGE-FILES.txt') -Destination (Join-Path $transaction 'stage-files.txt')
    Set-Content -LiteralPath (Join-Path $transaction 'old-journal.txt') -Encoding ascii -Value @('README.md', 'LICENSE')
    $backupLicense = Join-Path $transaction 'backup\LICENSE'
    # Both resolved paths are children of this run's validated install root.
    Move-Item -LiteralPath (Join-Path $installRoot 'LICENSE') -Destination $backupLicense
    $readmeHash = (Get-FileHash -LiteralPath (Join-Path $installRoot 'README.md')).Hash
    $locked = [IO.File]::Open($backupLicense, 'Open', 'Read', 'None')
    try { Invoke-Setup $baseInstaller -ExpectedExit 14 }
    finally { $locked.Dispose() }
    Assert-True ((Get-FileHash -LiteralPath (Join-Path $installRoot 'README.md')).Hash -eq $readmeHash) 'Recovery deleted an already restored file.'
    Assert-True (Test-Path -LiteralPath $backupLicense) 'Failed recovery deleted its backup.'
    Invoke-Setup $baseInstaller
    Assert-Payload
    Assert-Registration $version

    # A linked package directory must not redirect an upgrade outside its install root.
    $outsideAssets = Join-Path $testRoot 'outside-assets'
    $installedAssets = Join-Path $installRoot 'assets'
    if (-not ([IO.Path]::GetFullPath($installedAssets).StartsWith($testRoot + '\', [StringComparison]::OrdinalIgnoreCase)) -or
        -not ([IO.Path]::GetFullPath($outsideAssets).StartsWith($testRoot + '\', [StringComparison]::OrdinalIgnoreCase))) { throw 'Unexpected junction test path.' }
    Move-Item -LiteralPath $installedAssets -Destination $outsideAssets
    try
    {
        New-Item -ItemType Junction -Path $installedAssets -Target $outsideAssets | Out-Null
        Invoke-Setup $nextInstaller -ExpectedExit 13
        Assert-True (Test-Path -LiteralPath (Join-Path $outsideAssets 'branding\rigid-bodies.ico')) 'Setup followed a junction and changed its target.'
    }
    finally
    {
        # Remove the junction itself, without recursing into its target.
        if (Test-Path -LiteralPath $installedAssets) { [IO.Directory]::Delete($installedAssets) }
        Move-Item -LiteralPath $outsideAssets -Destination $installedAssets
    }
    Assert-Payload

    $locked = [IO.File]::Open((Join-Path $installRoot 'SDL3.dll'), 'Open', 'Read', 'None')
    try { Invoke-Setup $nextInstaller -ExpectedExit 13 }
    finally { $locked.Dispose() }
    Assert-Payload
    Assert-Registration $version

    # A new package file colliding with a personal file forces rollback after backup.
    $conflict = Join-Path $installRoot 'assets\installer-test-introduced.txt'
    Set-Content -LiteralPath $conflict -Value 'Keep this personal file.'
    Invoke-Setup $nextInstaller -ExpectedExit 14
    Assert-Payload
    Assert-Registration $version
    Assert-True ((Get-Content -LiteralPath $conflict -Raw).Trim() -eq 'Keep this personal file.') 'Rollback changed a personal file.'
    Remove-Item -LiteralPath $conflict

    Invoke-Setup $nextInstaller -Destination $ignoredDestination
    Assert-Payload
    Assert-Registration $nextVersion
    Assert-True (-not (Test-Path -LiteralPath (Join-Path $installRoot 'assets\installer-test-obsolete.txt'))) 'Upgrade left an obsolete package file.'
    Assert-True (Test-Path -LiteralPath (Join-Path $installRoot 'assets\installer-test-introduced.txt')) 'Upgrade did not install a new package file.'
    Invoke-Setup $baseInstaller -ExpectedExit 12
    Assert-Payload
    Assert-Registration $nextVersion
    if (-not $SkipRenderSmoke)
    {
        $smoke = Start-Process -FilePath (Join-Path $installRoot 'rigid_bodies.exe') -ArgumentList '--render-smoke' -Wait -PassThru -WindowStyle Hidden -RedirectStandardError (Join-Path $testRoot 'render-smoke.log')
        Assert-True ($smoke.ExitCode -eq 0) 'The installed application failed its rendering smoke test.'
    }
    $locked = [IO.File]::Open((Join-Path $installRoot 'SDL3.dll'), 'Open', 'Read', 'None')
    try
    {
        $blocked = Invoke-Uninstall
        Assert-True ($blocked.ExitCode -eq 13) 'Uninstall did not refuse a locked file.'
    }
    finally { $locked.Dispose() }
    Assert-Payload
    # Packaged files may acquire read-only attributes; that must not strand the uninstaller.
    (Get-Item -LiteralPath (Join-Path $installRoot 'SDL3.dll')).IsReadOnly = $true
    $uninstall = Invoke-Uninstall
    Assert-True ($uninstall.ExitCode -eq 0) "Uninstaller exited $($uninstall.ExitCode)."
    Assert-True (-not (Test-Path -LiteralPath (Join-Path $installRoot 'rigid_bodies.exe'))) 'Uninstall left the application executable.'
    Assert-True (-not (Test-Path -LiteralPath $uninstallKey)) 'Uninstall left its Installed Apps entry.'
    Assert-True (-not (Test-Path -LiteralPath $appKey)) 'Uninstall left its installation registry key.'
    Assert-True (-not (Test-Path -LiteralPath $shortcutRoot)) 'Uninstall left its Start-menu directory.'
    foreach ($file in @($preferences, (Join-Path $installRoot 'personal-test.txt'), (Join-Path $installRoot 'assets\personal-test.json')))
    {
        Assert-True (Test-Path -LiteralPath $file) "Uninstall removed a personal file: $file"
    }
    $passed = $true
    Write-Host 'Installer checks passed: clean install, repair, locked files, rollback, interrupted recovery, junction safety, upgrade, downgrade protection, stable shortcuts, checksums, user files and uninstall.' -ForegroundColor Green
}
finally
{
    $env:APPDATA = $originalAppData
    # These names are generated for this test run. The normal app keys and shortcuts are never touched.
    foreach ($key in @($appKey, $uninstallKey))
    {
        if (Test-Path -LiteralPath $key) { Remove-Item -LiteralPath $key -Recurse -Force }
    }
    $expectedShortcut = Join-Path ([Environment]::GetFolderPath('Programs')) "Rigid Bodies Installer Test $testId"
    if ($shortcutRoot -eq $expectedShortcut -and (Test-Path -LiteralPath $shortcutRoot)) { Remove-Item -LiteralPath $shortcutRoot -Recurse -Force }
    if ($passed)
    {
        $resolved = [IO.Path]::GetFullPath($testRoot)
        if (-not $resolved.StartsWith($allowedRoot, [StringComparison]::OrdinalIgnoreCase) -or (Split-Path $resolved -Leaf) -ne $testId) { throw 'Unexpected test cleanup path.' }
        Remove-Item -LiteralPath $resolved -Recurse -Force
    }
    else { Write-Warning "Installer test files kept for diagnosis: $testRoot" }
}
