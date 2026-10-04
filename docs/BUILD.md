# Building

## Requirements

- Windows: Visual Studio 2022 or later with **Desktop development with C++**. The native solution
  uses the installation's default MSVC toolset and Windows SDK; `build.ps1` uses the same toolset
  through the CMake and Ninja that the installation includes.
- Installer and release scripts: PowerShell 7 or newer (`pwsh`). The full release checks also
  need Python 3.10 or newer and Git. GitHub Actions supplies these tools.
- Other platforms: a C++17 compiler, CMake 3.24 or newer, and Ninja.

## Building on Windows

| | Visual Studio | Script |
| --- | --- | --- |
| Start | Open `RigidBodies.sln`, press **Build** or **Local Windows Debugger** | `.\build.ps1` |
| Build system | Native MSBuild projects in `visualstudio\` | CMake + Ninja (`CMakePresets.json`) |
| Application | `build\rigid_bodies.exe` | `build\rigid_bodies.exe` |
| Tests and tools | **Test** configuration | `.\build.ps1 -Tests` |

Both builds compile the same files with the same MSVC toolset and code-generation options, link the
same libraries in the same order, and stage the same `SDL3.dll`, `LICENSE`, `assets` and `config`. The
executable is identical byte for byte whichever way it was built, and a build of either kind
updates `build\rigid_bodies.exe`. The builds share these settings and file lists:

- The Visual Studio projects list the product's files. `cmake\visual_studio_projects.cmake` reads
  those lists, so a file added to a project in Visual Studio is part of the CMake build too.
- `visualstudio\RigidBodies.props` and `RigidBodies.Code.props` set the same options as the
  `RigidBodies.BuildOptions` target and `cmake\dependencies.cmake`; change them together.
- The test and tool projects are generated from CMake's own model of those targets by
  `scripts\generate_visual_studio.py`, and continuous integration checks that they are current.

### Output layout

| Path | Contents | Written by |
| --- | --- | --- |
| `build\` | `rigid_bodies.exe`, `SDL3.dll`, `LICENSE`, `assets\`, `config\` | both builds |
| `build\tests\` | One executable per test suite, plus `SDL3.dll` | both builds |
| `build\tools\` | Benchmark, determinism probe, energy drift, restitution comparison | both builds |
| `build\cmake\` | CMake's cache, Ninja files, objects and libraries | CMake |
| `build\msbuild\` | MSBuild's objects, static libraries and logs | Visual Studio |

`rigid_bodies.exe` is the only executable at the top of `build\`; `build.ps1` and the installer
script check that.

Product version, author, contact, copyright and license metadata are shared in
`src/RigidBodies.Core/include/rigidbodies/project_identity.hpp`. Both builds use that header for
the GUI and executable properties; CMake reads its numeric version for `PROJECT_VERSION`, and
the installer reads its string metadata. CMake also installs `LICENSE` beside the executable.
Use `pwsh ./scripts/version.ps1 -Version 1.0.1` to prepare a version change. It also updates
the README and the About panel's test snapshots. See [Releasing](RELEASING.md) for the complete
build, test and publication process.

### The Visual Studio solution

The solution has two configurations. **Release** (the one Visual Studio opens in) builds the
application and the libraries it links. **Test** builds the same, plus every test and tool. Visual
Studio opens a solution in its alphabetically first configuration, which is why there is no
configuration named Debug in this solution. Adding one would change the default configuration and
output layout. You can still debug Release with the green button.

Solution Explorer groups the projects by role:

- **Application**: `RigidBodies.App` (the startup project), `RigidBodies.AppCore`, `RigidBodies.Ui`,
  `RigidBodies.Render` and `RigidBodies.Core`.
- **Simulation**: `RigidBodies.Physics` and `RigidBodies.Math`.
- **Third party**: FreeType and RmlUi, built from `vendor\`.
- **Tests** (by module) and **Tools**.
- **Build**, **Documentation**, **Scripts** and **Test support**: the files that are not compiled.

Inside each project, folders mirror the source tree: `include` and `src` for every module, and the
application's `resources`, `assets` and `config`.

To add or remove a test or tool, edit `CMakeLists.txt`, then run:

```powershell
.\build.ps1 -Tests
python scripts\generate_visual_studio.py
```

### Script options

| Option | Effect |
| --- | --- |
| `-Clean` | Remove the build directory before compiling |
| `-CleanOnly` | Remove the build directory and stop |
| `-Jobs N` | Set the parallel compilation limit |
| `-Tests` | Also build the tests and tools, and run the tests |
| `-Installer` | Build the NSIS installer after the application |
| `-Run` | Launch the application after a successful build |

## CMake

The direct CMake workflow uses the same presets and directories:

```bash
cmake --preset default
cmake --build --preset default

# With tests and tools:
cmake --preset development
cmake --build --preset development
ctest --preset development
```

The presets put CMake's own files in `build/cmake` and the outputs in `build/`. A configuration
without a preset writes the outputs to its binary directory instead, still with tests in `tests/`
and tools in `tools/`.

## Dependencies

FreeType and RmlUi are compiled from `vendor/` by both builds, with their file lists read from
`visualstudio/thirdparty`. On Windows x64 SDL3 is the prebuilt runtime and import library in
`vendor/SDL3`. Other platforms find an installed SDL3 3.4 or fetch the same release; set
`RIGIDBODIES_FETCH_DEPENDENCIES=OFF` to require an installed package. The developer overlay uses
`vendor/DearImGui`.

The application can be checked without showing a window:

```powershell
.\build\rigid_bodies.exe --render-smoke
```

`--render-benchmark` renders hidden frames for about thirty seconds with vertical sync off and
prints the mean and 95th-percentile frame time, followed by per-phase medians for simulation,
scene recording, interface, device submission and presentation.

For visual review, `--screenshot <file.png>` renders the interface offscreen and writes one frame
without opening a visible window. Optional arguments fix the conditions so captures are
reproducible across machines: `--size WxH` (window size in physical pixels), `--display-scale S`
(simulated monitor density, e.g. 1.25 or 2), `--ui-scale S` (the text-size preference, 0.75 to 2),
`--theme workbench_dark|workbench_light|workbench_projector`, `--scenario <id>`,
`--run-frames N` (advance the simulation N sixtieths of a second first) and `--state a,b,c`, which
applies interface states in order: `guide`, `noguide`, `inspector`, `noinspector`, `measure`,
`graph`, `collisions`, `runs`, `world`, `library`, `preferences`, `shortcuts`, `about`, `search`,
`menu`, `add`, `show`, `playback_speed`, `present`, `spotlight`, `unlock`, `performance`, `nohints`, `select`,
`selectall`, `context`, `hover`, `draw`, `draw-options`, `reduce-motion` and `run`.
