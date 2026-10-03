#requires -Version 7.0
[CmdletBinding(SupportsShouldProcess)]
param(
    [string]$Version,
    [string]$Tag,
    [string]$ProjectRoot = (Split-Path $PSScriptRoot -Parent)
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$utf8 = [Text.UTF8Encoding]::new($false)
$identityPath = Join-Path $ProjectRoot 'src\RigidBodies.Core\include\rigidbodies\project_identity.hpp'
$identity = [IO.File]::ReadAllText($identityPath, $utf8)
$versionPattern = '(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)'

function Read-IdentityField([string]$Name, [string]$Pattern)
{
    $matches = [regex]::Matches($identity, ('(?m)^#define RIGIDBODIES_' + $Name + ' ' + $Pattern + '\r?$'))
    if ($matches.Count -ne 1) { throw "Expected one project identity field: $Name." }
    return $matches[0].Groups[1].Value
}

function Assert-Version([string]$Value)
{
    if ($Value -cnotmatch "^$versionPattern`$") { throw "Invalid version: $Value. Use MAJOR.MINOR.PATCH without leading zeroes." }
    foreach ($part in $Value.Split('.'))
    {
        $number = 0
        if (-not [int]::TryParse($part, [ref]$number) -or $number -gt 65535)
        {
            throw 'Each version component must be between 0 and 65535 for Windows executable metadata.'
        }
    }
}

$current = Read-IdentityField 'VERSION_STRING' '"([^"\r\n]+)"'
Assert-Version $current
$components = @('MAJOR', 'MINOR', 'PATCH') | ForEach-Object { Read-IdentityField "VERSION_$_" '(0|[1-9][0-9]*)' }
if (($components -join '.') -cne $current) { throw 'Numeric and string versions disagree in project_identity.hpp.' }
$target = if ($Version) { $Version } else { $current }
Assert-Version $target
if ($Tag -and $Tag -cne "v$target") { throw "Tag $Tag does not match version $target; expected v$target." }

# Check every file before writing any of them. Only product-version fields are changed;
# document schemas, dependency versions and physics reference values have their own versions.
$changes = [ordered]@{}
$readmePath = Join-Path $ProjectRoot 'README.md'
$readme = [IO.File]::ReadAllText($readmePath, $utf8)
$readmePattern = '(?m)^The current version is `([^`]+)`\.\r?$'
$readmeMatch = [regex]::Matches($readme, $readmePattern)
if ($readmeMatch.Count -ne 1 -or $readmeMatch[0].Groups[1].Value -cne $current)
{
    throw 'The README current-version line does not match project_identity.hpp.'
}
$changes[$readmePath] = $readme.Replace("The current version is ``$current``.", "The current version is ``$target``.")

$snapshots = @(Get-ChildItem -LiteralPath (Join-Path $ProjectRoot 'tests\fixtures\ui') -File -Filter '*.rows.txt')
if ($snapshots.Count -eq 0) { throw 'No UI text snapshots were found.' }
foreach ($snapshot in $snapshots)
{
    $text = [IO.File]::ReadAllText($snapshot.FullName, $utf8)
    $versionRows = [regex]::Matches($text, '\tlabel=Version\tvalue=([^\t\r\n]+)\t')
    if ($versionRows.Count -ne 1 -or $versionRows[0].Groups[1].Value -cne $current)
    {
        throw "The About version in $($snapshot.Name) does not match project_identity.hpp."
    }
    $changes[$snapshot.FullName] = $text.Replace("`tlabel=Version`tvalue=$current`t", "`tlabel=Version`tvalue=$target`t")
}

$parts = $target.Split('.')
for ($index = 0; $index -lt 3; ++$index)
{
    $field = @('MAJOR', 'MINOR', 'PATCH')[$index]
    $identity = $identity.Replace("#define RIGIDBODIES_VERSION_$field $($components[$index])", "#define RIGIDBODIES_VERSION_$field $($parts[$index])")
}
$identity = $identity.Replace("#define RIGIDBODIES_VERSION_STRING `"$current`"", "#define RIGIDBODIES_VERSION_STRING `"$target`"")
$changes[$identityPath] = $identity
if ($Version -and $target -cne $current -and $PSCmdlet.ShouldProcess($ProjectRoot, "Set product version to $target"))
{
    foreach ($entry in $changes.GetEnumerator()) { [IO.File]::WriteAllText($entry.Key, $entry.Value, $utf8) }
    Write-Host "Updated version $current to $target. Rebuild before packaging."
}
Write-Output $target
