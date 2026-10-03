# Windows distribution

Rigid Bodies is distributed under the [MIT License](../LICENSE).
Copyright © 2026 Deniz Mert Yayla <denizmerty@gmail.com>.

## Build and package

Build the application and its installer with:

```powershell
pwsh .\build.ps1 -Clean -Installer -NonInteractive
```

The executable is written to `build\rigid_bodies.exe`. Packaging produces the following files in
`build\installer`, with the version read from `project_identity.hpp`:

- `Rigid-Bodies-<version>-windows-x64-setup.exe`: per-user NSIS installer.
- `Rigid-Bodies-<version>-windows-x64.zip`: portable application with the same runtime files.
- A `.sha256` file for each download, plus installer `.json` build metadata.

Packaging rejects an executable whose product version differs from the source header. It requires
NSIS 3.12, either supplied through `-MakeNsis`, found on `PATH`, or downloaded from the pinned URL.
The download is checked against its SHA-256 checksum and cached under `build\tool-cache`.
`-NoDownload` uses an installed or cached compiler without network access. Temporary package and
compiler directories are removed after the build.

Before creating either package, `verify-runtime-dependencies.ps1` checks the executable and bundled
DLL imports with Visual Studio's `dumpbin`. Every dependency must be in the payload or on the explicit
Windows-component list. Missing libraries and accidental dependencies on a machine-installed MSVC
runtime fail the package build.

The same packaging script is used locally and by the Windows release workflow. See
[Releasing](RELEASING.md) for version changes, validation, tags and GitHub releases.

## Runtime dependencies

The application, RmlUi, FreeType and the MSVC runtime are statically linked. SDL is supplied as
`SDL3.dll` beside `rigid_bodies.exe`. No separate Visual C++ Redistributable installation is needed.

This arrangement keeps deployment small and predictable. The libraries built into the application
have no independent plugin or update mechanism, so rebuilding them with the application avoids
extra DLL compatibility and search-path problems. SDL already has a supported shared-library
boundary and is packaged with its matching build. Dependency updates are shipped and tested as a
complete release; replacing individual files in an installed copy is not a supported update method.

The payload contains runtime assets, default configuration, documentation, the project license and
third-party license notices in `assets\licenses`. This software is based in part on the work of the
[FreeType Team](https://freetype.org).

## Installation and updates

Setup installs for the current user under `%LOCALAPPDATA%\Programs\Rigid Bodies` by default and
requires no administrator access. It creates one entry in Windows Installed Apps and one Start-menu
folder. The executable remains at the same path across releases, so existing shortcuts keep working.

Run the newer installer to update. Setup finds the existing installation even if a different `/D=`
path is supplied, replaces its files and updates the existing Installed Apps entry. Installing the
same version repairs its packaged files. Older installers refuse to replace a newer version; to
downgrade, uninstall first and then install the older release.

Setup extracts the complete new payload before moving any installed files. It checks for files in
use and asks you to close the application before continuing. Updates keep a backup and recovery
journals until replacement finishes. A failed replacement restores the previous files; after an
interrupted operation, running setup again resumes recovery. Setup refuses linked package paths
and conflicts with personal files introduced under new package filenames.

Uninstall removes only files recorded as part of the package and directories left empty afterward.
Saved scenes, preferences and other files under `%APPDATA%\RigidBodies` remain. Unrelated files
placed in the installation directory also remain, so that directory may still exist after uninstall.
Keep custom scenes and edited configuration in the application's user-data directory; packaged
defaults are replaced when repairing or updating.

The old development installer predating this release did not record the package file list required
for safe updates. If you installed that development build, uninstall it once before installing this
release. Back up any custom files you placed inside its installation directory first. Releases from
this version onward support direct upgrades.

Silent installation and removal are supported:

```powershell
& '.\Rigid-Bodies-<version>-windows-x64-setup.exe' /S
& "$env:LOCALAPPDATA\Programs\Rigid Bodies\Uninstall.exe" /S
```

For a custom first-install location, append `/D=C:\path\without quotes` as the final installer
argument. Silent setup returns `0` on success, `11` if another setup is running, `12` for a blocked
downgrade, `13` when files are in use, and `14` when installation or recovery cannot finish.
Uninstall additionally uses `15` for a mismatched installation and `16` for removal errors.

## Verification

```powershell
pwsh .\scripts\test-installer.ps1
```

The test builds packages with a unique test registry and Start-menu namespace. It checks clean
installation, same-version repair, a subsequent package version, downgrade rejection, locked files,
rollback, recovery, checksums, stable shortcut paths and uninstall. Its next-version fixture changes
packaged assets while reusing the current application binary; it tests installer upgrade behavior
without changing the project's version. Normal installed copies are never used by these tests.

The installer is currently unsigned. Windows may show an unknown-publisher or reputation warning.

## Installer artwork

The installer shares the application's tumbling orange block, blue motion trail and slate palette.
Its standard Windows controls remain familiar and follow display scaling. Source artwork is in
`packaging\installer`; the rendered bitmaps are committed so ordinary builds need no graphics tools.
To regenerate them after an artwork change:

```powershell
python packaging\make_installer_art.py
```

This command requires ImageMagick (`magick`). The application icon is maintained separately in
`packaging\icon`; `packaging\make_app_icon.py` regenerates `assets\branding\rigid-bodies.ico` using
Python with Pillow and ImageMagick. Its small-size SVG variants are adjusted for pixel legibility.
