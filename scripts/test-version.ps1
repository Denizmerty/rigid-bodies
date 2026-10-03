#requires -Version 7.0
[CmdletBinding()]
param()

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
$buildRoot = [IO.Path]::GetFullPath((Join-Path $projectRoot 'build')).TrimEnd('\') + '\'
$fixture = [IO.Path]::GetFullPath((Join-Path $buildRoot ('version-test-' + [Guid]::NewGuid().ToString('N'))))
if (-not $fixture.StartsWith($buildRoot, [StringComparison]::OrdinalIgnoreCase)) { throw 'Invalid version-test directory.' }
$versionScript = Join-Path $PSScriptRoot 'version.ps1'
$utf8 = [Text.UTF8Encoding]::new($false)
$identity = Join-Path $fixture 'src\RigidBodies.Core\include\rigidbodies\project_identity.hpp'
$snapshot = Join-Path $fixture 'tests\fixtures\ui\test.rows.txt'
$readme = Join-Path $fixture 'README.md'
$assertions = 0

function Assert-Equal($Actual, $Expected, [string]$Message)
{
    if ($Actual -cne $Expected) { throw $Message }
    $script:assertions++
}

function Assert-Rejected([scriptblock]$Action, [string]$Message)
{
    $rejected = $false
    try { & $Action | Out-Null } catch { $rejected = $true }
    if (-not $rejected) { throw $Message }
    $script:assertions++
}

try
{
    New-Item -ItemType Directory -Path (Split-Path $identity -Parent), (Split-Path $snapshot -Parent) -Force | Out-Null
    [IO.File]::WriteAllText($identity, "#define RIGIDBODIES_VERSION_MAJOR 1`r`n#define RIGIDBODIES_VERSION_MINOR 0`r`n#define RIGIDBODIES_VERSION_PATCH 0`r`n#define RIGIDBODIES_VERSION_STRING `"1.0.0`"`r`n#define RIGIDBODIES_COPYRIGHT `"Copyright © 2026 Deniz Mert Yayla`"`r`n", $utf8)
    [IO.File]::WriteAllText($readme, "The current version is ``1.0.0``.`r`nDocument version: 1.0`r`n", $utf8)
    [IO.File]::WriteAllText($snapshot, "kind=19`tlabel=Version`tvalue=1.0.0`tunit=`r`n", $utf8)
    Assert-Equal (& $versionScript -ProjectRoot $fixture -Tag v1.0.0) '1.0.0' 'Reading a valid version failed.'
    Assert-Rejected { & $versionScript -ProjectRoot $fixture -Tag v1.0.1 } 'Mismatched tag was accepted.'
    foreach ($invalid in @('01.0.0', '1.2', '1.2.3-beta', '65536.0.0', '1.99999999999999999.0', '../1.0.0'))
    {
        Assert-Rejected { & $versionScript -ProjectRoot $fixture -Version $invalid } "Invalid version was accepted: $invalid"
    }
    Assert-Equal (& $versionScript -ProjectRoot $fixture -Version 1.0.1) '1.0.1' 'Version update failed.'
    Assert-Equal (& $versionScript -ProjectRoot $fixture -Tag v1.0.1) '1.0.1' 'Updated version metadata is inconsistent.'
    $updated = [IO.File]::ReadAllText($identity, $utf8)
    Assert-Equal ($updated.Contains('Copyright © 2026 Deniz Mert Yayla')) $true 'The copyright was changed.'
    Assert-Equal ([IO.File]::ReadAllText($readme, $utf8).Contains('Document version: 1.0')) $true 'The document version was changed.'
    [IO.File]::WriteAllText($snapshot, "kind=19`tlabel=Version`tvalue=0.9.0`tunit=`r`n", $utf8)
    Assert-Rejected { & $versionScript -ProjectRoot $fixture -Version 1.0.2 } 'A stale snapshot was accepted.'
    Assert-Equal ([IO.File]::ReadAllText($identity, $utf8)) $updated 'A rejected update partially changed files.'
    [IO.File]::WriteAllText($identity, $updated.Replace('VERSION_PATCH 1', 'VERSION_PATCH 3'), $utf8)
    Assert-Rejected { & $versionScript -ProjectRoot $fixture } 'Mismatched numeric version was accepted.'
    Write-Host "Version helper: $assertions checks passed." -ForegroundColor Green
}
finally
{
    if (Test-Path -LiteralPath $fixture) { Remove-Item -LiteralPath $fixture -Recurse -Force }
}
