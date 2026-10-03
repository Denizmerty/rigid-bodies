[CmdletBinding(SupportsShouldProcess, ConfirmImpact = 'Medium')]
param()

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$projectRoot = [System.IO.Path]::GetFullPath($PSScriptRoot)
$comparison = [System.StringComparison]::OrdinalIgnoreCase

$protectedRoots = @(
    [System.IO.Path]::GetFullPath((Join-Path $projectRoot '.git')),
    [System.IO.Path]::GetFullPath((Join-Path $projectRoot '_development_archive')),
    [System.IO.Path]::GetFullPath((Join-Path $projectRoot 'vendor'))
)

function Test-IsWithinProject {
    param([Parameter(Mandatory)][string]$Path)

    $fullPath = [System.IO.Path]::GetFullPath($Path)
    $rootPrefix = $projectRoot.TrimEnd('\', '/') + [System.IO.Path]::DirectorySeparatorChar
    return $fullPath.StartsWith($rootPrefix, $comparison)
}

function Test-IsProtected {
    param([Parameter(Mandatory)][string]$Path)

    $fullPath = [System.IO.Path]::GetFullPath($Path)
    foreach ($protectedRoot in $protectedRoots) {
        if ($fullPath.Equals($protectedRoot, $comparison) -or
            $fullPath.StartsWith($protectedRoot.TrimEnd('\', '/') + [System.IO.Path]::DirectorySeparatorChar, $comparison)) {
            return $true
        }
    }

    return $false
}

$removedTargets = 0
$scheduledTreeRoots = [System.Collections.Generic.List[string]]::new()

function Test-IsWithinScheduledTree {
    param([Parameter(Mandatory)][string]$Path)

    $fullPath = [System.IO.Path]::GetFullPath($Path)
    foreach ($treeRoot in $scheduledTreeRoots) {
        if ($fullPath.Equals($treeRoot, $comparison) -or
            $fullPath.StartsWith($treeRoot.TrimEnd('\', '/') + [System.IO.Path]::DirectorySeparatorChar, $comparison)) {
            return $true
        }
    }

    return $false
}

function Remove-BuildTarget {
    param([Parameter(Mandatory)][System.IO.FileSystemInfo]$Item)

    if (-not (Test-IsWithinProject -Path $Item.FullName)) {
        throw "Refusing to remove a path outside the project: $($Item.FullName)"
    }
    if (Test-IsProtected -Path $Item.FullName) {
        return
    }

    if ($PSCmdlet.ShouldProcess($Item.FullName, 'Remove generated build artifact')) {
        Remove-Item -LiteralPath $Item.FullName -Recurse -Force
    }
    $script:removedTargets++
}

# Build systems and IDEs place their complete output trees under these root names.
$rootOutputNames = @('.vs', '_deps', 'Debug', 'Release', 'dist', 'out', 'x64')
Get-ChildItem -LiteralPath $projectRoot -Force -Directory | Where-Object {
    $_.Name -eq 'build' -or
    $_.Name -like 'build-*' -or
    $rootOutputNames -contains $_.Name
} | ForEach-Object {
    $scheduledTreeRoots.Add([System.IO.Path]::GetFullPath($_.FullName))
    Remove-BuildTarget -Item $_
}

# Remove CMake and Visual Studio intermediate directories left by an in-source or custom build.
$generatedDirectoryNames = @('.vs', '_deps', 'CMakeFiles', 'Debug', 'Release', 'Testing', 'ipch', 'x64')
Get-ChildItem -LiteralPath $projectRoot -Recurse -Force -Directory -ErrorAction SilentlyContinue |
    Where-Object {
        $generatedDirectoryNames -contains $_.Name -and
        -not (Test-IsProtected -Path $_.FullName) -and
        -not (Test-IsWithinScheduledTree -Path $_.FullName)
    } |
    Sort-Object { $_.FullName.Length } -Descending |
    ForEach-Object {
        if (Test-Path -LiteralPath $_.FullName) {
            $scheduledTreeRoots.Add([System.IO.Path]::GetFullPath($_.FullName))
            Remove-BuildTarget -Item $_
        }
    }

$generatedFileNames = @(
    '.ninja_deps',
    '.ninja_log',
    'build.ninja',
    'CMakeCache.txt',
    'cmake_install.cmake',
    'compile_commands.json',
    'CTestTestfile.cmake',
    'install_manifest.txt',
    'rules.ninja'
)

$artifactExtensions = @(
    '.a', '.coverage', '.dll', '.dylib', '.exe', '.exp', '.gcda', '.gcno',
    '.idb', '.ilk', '.lastbuildstate', '.lib', '.lo', '.obj', '.o', '.pch',
    '.pdb', '.profdata', '.profraw', '.res', '.so', '.suo', '.tlog', '.user',
    '.vc.db', '.vc.opendb'
)

Get-ChildItem -LiteralPath $projectRoot -Recurse -Force -File -ErrorAction SilentlyContinue |
    Where-Object {
        if (Test-IsProtected -Path $_.FullName) {
            return $false
        }
        if (Test-IsWithinScheduledTree -Path $_.FullName) {
            return $false
        }

        $lowerName = $_.Name.ToLowerInvariant()
        $lowerExtension = $_.Extension.ToLowerInvariant()
        return $generatedFileNames -contains $lowerName -or
            $artifactExtensions -contains $lowerExtension -or
            $lowerName.EndsWith('.vc.db') -or
            $lowerName.EndsWith('.vc.opendb')
    } |
    ForEach-Object {
        Remove-BuildTarget -Item $_
    }

if ($WhatIfPreference) {
    Write-Host "Clean inspection complete: $removedTargets generated target(s) would be removed."
} else {
    Write-Host "Project clean complete: removed $removedTargets generated target(s)."
}
