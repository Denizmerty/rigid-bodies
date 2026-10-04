# Windows releases

The local release command and GitHub Actions use the same build and packaging scripts. Windows
x64 is the supported release target. Linux and macOS have no CI or release jobs at present.

## Build and test a release locally

Install Visual Studio 2022 or later with **Desktop development with C++**, PowerShell 7 or newer,
Python 3.10 or newer, and Git. Run from the project directory:

```powershell
pwsh ./scripts/release.ps1 -Jobs 4
```

The script validates the product version, runs the PowerShell and Python tooling tests, builds the
CMake development preset, and runs all application tests. It checks the generated Visual Studio
projects, builds the native solution, and requires the two application executables to match byte
for byte. It also checks deterministic replay against the committed reference trace.

Then it builds the NSIS installer and portable ZIP from the same staged files, checks the runtime
dependencies, and runs isolated installation tests. Those tests cover installation, reinstall,
upgrade, downgrade rejection and uninstall. They use separate registry keys, shortcuts and files
under `build`; they do not replace an installed copy of Rigid Bodies.

Successful output is written to `build/release/current`:

| File | Contents |
| --- | --- |
| `Rigid-Bodies-<version>-windows-x64-setup.exe` | Per-user NSIS installer |
| `Rigid-Bodies-<version>-windows-x64.zip` | The same runtime files for manual extraction |
| `Rigid-Bodies-<version>-windows-x64-build.json` | Source commit, compiler versions and executable hash |
| `RELEASE-NOTES.md` | Version-specific notes, when `docs/releases/<version>.md` exists |
| `SHA256SUMS` | SHA-256 hashes of the packages, build record and any release notes |

The installer is the recommended download for normal use and upgrades. The ZIP does not register
an installation or manage upgrades; extract each version into its own directory.

The release command accepts `-Python <executable>` when Python is not on `PATH`, and
`-MakeNsis <makensis.exe> -NoDownload` for an existing NSIS installation. Otherwise the installer
builder downloads the pinned NSIS version and checks its SHA-256 before using it. Application
dependencies come from `vendor`, so their source and runtime versions travel with each commit.

`./build.ps1` remains the quick application build. `pwsh ./build.ps1 -Installer` builds and packages
the application without running the complete release validation. Neither command publishes files.

## Prepare a new version

`src/RigidBodies.Core/include/rigidbodies/project_identity.hpp` is the source of the product version.
The GUI, Windows executable properties, both build systems and the NSIS installer read it. Keep
the version as three numbers, with no leading zeroes or prerelease suffix; each component must fit
in Windows' 16-bit version fields.

For example, to prepare the next patch release:

```powershell
pwsh ./scripts/version.ps1 -Version 1.0.1
git diff
git add src/RigidBodies.Core/include/rigidbodies/project_identity.hpp README.md tests/fixtures/ui
git commit -m "Prepare version 1.0.1"
git push origin master
```

The helper checks the old values before updating the header, the README current-version line and
the UI snapshots. It preserves copyright text, dependency versions, saved-document formats and
physics reference data. Run it without `-Version` to check consistency and print the current version.
`-WhatIf` previews an update without writing files.

Write user-facing release notes in `docs/releases/<version>.md` and include them in the version
commit. The release build copies them into its tested artifacts and checksum manifest, and the
publication job uses that exact file as the GitHub release description. Versions without this
file fall back to GitHub's generated release notes.

Wait for **Windows CI / Build, test and package** to pass on that commit. Then publish an annotated
tag for that exact commit:

```powershell
git tag -a v1.0.1 -m "Rigid Bodies 1.0.1"
git push origin v1.0.1
```

Pushing the tag starts **Windows release**. It repeats the complete build and validation on a
fresh Windows runner, uploads the tested files to a draft GitHub release, and publishes it only
after all files are present and their checksums, version and source commit have been verified.
There is no separate manual installer rebuild or manual release-publication step.

## CI and repository access

**Windows CI** runs on pushes to `master`, pull requests targeting `master`, and manual dispatches.
Each successful run retains its tested packages for 14 days, so a build can be tried without
creating a release. Failed runs retain build and test logs for the same period.

**Windows release** accepts only the `Denizmerty` account in the `Denizmerty/rigid-bodies`
repository. A release tag must match the product version and point to a commit on `master`. The
source checks run before any script from that tag is executed. A tagged build must have a clean
working tree. Build jobs have read-only repository access; a separate publication job has
`contents: write`, downloads the tested artifacts and never executes project code. Pull requests
cannot publish releases and receive no release credentials. Action dependencies are pinned to
reviewed commit hashes.

The repository rules in `.github/rulesets` allow only the owner's account to update `master` or
create release tags. Separate rules forbid deletion and force-pushing of `master` and release
tags. The checked-in JSON records the intended settings; GitHub's repository settings enforce
them. `.github/CODEOWNERS` identifies the reviewer for future contributions.

To retry a failed release, rerun its failed jobs or manually run **Windows release** from `master`
with the existing tag. A partially uploaded draft can be completed. Published releases are never
overwritten by the workflow; corrections require a new version and tag. Publishing an older
maintenance release does not replace a newer version as GitHub's latest release. Do not move or
reuse tags.

The Windows runner image is pinned to `windows-2025`; its servicing updates can still change MSVC
or the Windows SDK. The build record captures those versions, and the byte comparison always
uses the same installed toolchain for CMake and Visual Studio. When updating a dependency, change
its vendored files and license notices together, then let the full release checks validate it.

Installers are currently unsigned. The checksum detects a damaged or changed download but does
not replace a publisher signature. Code signing can be added to the package stage when a signing
certificate or service is available, with checksums calculated after signing.
