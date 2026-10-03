# Rigid Bodies

A 2D playground for exploring classical Newtonian mechanics.

Build a setup, watch it move, change a value, and compare the result. Objects have shape and mass
distribution, with sizes and masses suited to familiar classroom experiments. On-screen arrows,
measurements and guides help explain what is happening.

The simulation includes deterministic fixed steps, continuous collision detection, joints,
springs, air forces and editable compound shapes. You can browse experiments, move and edit
objects, undo changes, save setups, import shapes and capture PNG images. Saved scenario and shape
documents retain their editable geometry and physical settings.

The renderer distinguishes wood, steel, foam and glass, and shows a hatched ground, scale bar,
centre-of-mass markers and labelled vectors. Arrows use colour and head shape to distinguish
quantities. Labels avoid overlap and follow the text-size preference. Contact shadows, motion
trails and impact effects help show motion. Reduce motion turns off blur, impact flashes, sparks,
dust and smooth transitions.

The current version is `1.0.0`.

## License and author

Copyright © 2026 Deniz Mert Yayla <denizmerty@gmail.com>.

Rigid Bodies is distributed under the [MIT License](LICENSE). Bundled dependencies retain their
own licenses; their notices are in `assets/licenses`, `assets/fonts`, and `vendor`.

## Building

On Windows there are two equivalent ways to build, and they produce the same program: both compile
the same files with the same MSVC toolset and options, link them in the same order, and write the
same `build\rigid_bodies.exe`, byte for byte, beside the same `SDL3.dll`, `LICENSE`, `assets` and `config`.

- **Visual Studio.** Open `RigidBodies.sln` and press **Build** or the green **Local Windows
  Debugger** button. The solution builds directly with MSBuild and MSVC. It
  opens in its **Release | x64** configuration with the application as the startup project and
  needs no setup step. The **Test** configuration also builds the tests into `build\tests` and the
  command-line tools into `build\tools`; the application never depends on them.
- **Script.** `.\build.ps1` builds the same application with CMake and Ninja. `.\build.ps1 -Tests`
  also builds the tests and tools and runs the tests.

Requirements: Visual Studio 2022 or later with **Desktop development with C++** (the script uses the
CMake and Ninja that installation includes). Other platforms need a C++17 compiler, CMake 3.24 or
newer, and Ninja. The direct CMake commands are:

```bash
cmake --preset default
cmake --build --preset default
```

SDL3, FreeType and RmlUi come from `vendor/`, so a Windows x64 build downloads nothing. See
[docs/BUILD.md](docs/BUILD.md) for the output layout, the solution's structure and the developer
options.

## Windows installer

Run `pwsh ./build.ps1 -Installer` with PowerShell 7 or newer to create
`build/installer/Rigid-Bodies-<version>-windows-x64-setup.exe`. It installs for the current user without
administrator access and can be removed through Windows Installed Apps or its Start menu shortcut.
The installer is unsigned.

See [docs/DISTRIBUTION.md](docs/DISTRIBUTION.md) for checksums, silent installation, and
building the installer from source.

## Releases and CI

[Windows CI](https://github.com/Denizmerty/rigid-bodies/actions/workflows/quality.yml) builds, tests
and packages changes to `master` and incoming pull requests. The same checks run locally with
`pwsh ./scripts/release.ps1`. They cover both build systems, the installed application, and updates
between versions. Successful runs provide an installer, a portable ZIP and checksums.

An owner-created version tag starts the release workflow, which builds and verifies the packages
before publishing them on [GitHub Releases](https://github.com/Denizmerty/rigid-bodies/releases).
[docs/RELEASING.md](docs/RELEASING.md) explains version updates and the tag-based release process.
Build, packaging and maintenance scripts are tracked with the source so releases can be reproduced
from a checkout.

## Running

The current Release executable is:

```text
build/rigid_bodies.exe
```

A build-tree run uses the assets and configuration staged beside the executable. Preferences and
interface state live under the platform's RigidBodies user-data directory; saved setups, exported shapes and
captures use that directory by default but can be placed elsewhere through the native dialogs.

Experiments open **Ready** at t = 0 and are framed inside the visible stage. The command bar groups
transport (back to start, Play/Pause, step, speed), tools (Select & move, Throw, Pull), history
(Undo/Redo) and the workspace (Add, Library, Guide, Inspector, Show, Measure, menu). As the window
narrows, it first hides workspace labels, then tool labels, then the Add and Play labels. Less-used
controls move into the top of the main menu with their icons and shortcuts. Play/Pause, the tools
and the menu stay on the bar. Icon-only controls keep their names as tooltips and accessible labels.
The status line reports time,
selection, the lab integrator and view height. The Guide explains the experiment; the contextual
Inspector edits World, objects, joints and springs; Measure contains Energy, Graph, Collisions,
Theory checks and Runs. The Library searches bundled experiments and saved setups. **Ctrl+K**
searches every command and control, then runs the action or opens and focuses the matching setting.

On the stage:

- Click selects the smallest eligible object within a scale-aware tolerance; Shift-click extends
  the selection. Shift-drag on empty space selects free and driven objects inside a rectangle.
- Drag selected objects in Select mode, throw them in Throw mode, or attach a temporary force in
  Pull mode. A gesture begins only after four logical pixels.
- When Ready or Paused, use the selected object's velocity and rotation handles or the World's
  gravity compass. Joint and spring glyphs are selectable.
- Right-click opens the target's context menu, selecting an object first unless it is already part
  of the selection, so the menu's commands act on what was clicked; Shift+F10 or the Menu key opens
  it for the current selection. Shift+A adds a snapped Ball, Box or Plank at the focus-rectangle centre.
- Draw mode supports snapping, rubber-band previews, smooth click-drags, node/tangent editing and
  double-click insertion. Double-clicking an authored shape edits the exact part under the pointer.
  The draw bar keeps Discard and Apply in view at every width; material, snapping, precision and
  the selected point's edge and join settings are also in its **Options** popover.
- Hovering first shows a silhouette, then a read-only card. Impact markers can be inspected from
  the stage. Cursor glyphs distinguish open-hand pan, grab and pen interaction.

**F5** enters Present mode. A slim strip shows the experiment title, large playback controls,
a clock, and Spotlight, Lock and Exit controls. The current Guide step appears in large text on a
card at the bottom of the stage, with Previous, Next and Show me. Camera framing keeps the subject
above the card. The optional spotlight draws a soft circle around the pointer, scaled for the
display. Demonstration lock starts on and disables file actions and destructive edits with an
explanation. Playback, Throw, Pull and Guide variables remain available. The first time you enter
Present mode, the caption card offers the high-contrast Projector theme.
Performance statistics are available through **View → Performance overlay**; the old Diagnostics
panel is retired.

First launch shows three short hints you can dismiss. In Preferences, you can show them again,
choose Dark, Light or Projector, set the text size from 75–200%, enable Reduce motion, and save
your choice of whether to show the Performance overlay. All learner commands are available from
the keyboard except Throw/Pull gestures and placing shape nodes, which require a pointer.
The Motion fields let you set velocity from the keyboard for the
Throw learning task.

| Input | Effect |
| --- | --- |
| Left click / Shift-click | Select an object / extend or reduce the selection |
| Shift-drag empty stage | Add free and driven objects inside a marquee |
| Right drag or middle drag | Pan after the four-pixel gesture threshold |
| Right click / Shift+F10 / Menu | Open the context menu under the pointer / for the selection |
| Scroll wheel | Zoom about the pointer |
| Space / Shift+Space | Play or pause / play until the next impact |
| . / Shift+. | Single step / step ten times |
| R / Shift+R | Back to start / replay from the starting setup |
| F / Shift+F | Frame the moving subject / selected group |
| V / T / P | Select & move / Throw / Pull |
| D | Draw a shape |
| Shift+A | Open Add Ball, Box, Plank |
| Ctrl+A / Delete | Select all free objects / delete the eligible selection |
| Ctrl+Z / Ctrl+Shift+Z / Ctrl+Y | Undo / redo / redo |
| Ctrl+G / Ctrl+Shift+G | Combine selected authored shapes / separate parts |
| [ / ] | Select the previous / next object |
| Alt+Arrow / Shift+Alt+Arrow | Nudge the selection 1 cm / 10 cm |
| L / S / M | Open Library / toggle Show / toggle Measure |
| W / G / I | Open World / toggle Guide / pin Inspector |
| F10 | Open the main menu |
| Ctrl+K | Open Command search |
| Ctrl+, | Open Preferences |
| F1 / ? | Open the keyboard reference / Shortcuts |
| F5 | Enter or leave Present mode |
| Left / Right in Present | Previous / next Guide step |
| Ctrl+S / Ctrl+Shift+S | Save setup / Save setup as |
| Ctrl+O / Ctrl+I | Open setup / import shape |
| Tab / Shift+Tab | Move forward / backward between controls |
| F6 / Shift+F6 | Move between Guide, Inspector, Measure and the scene |
| Enter | Activate the focused control; commit typed number fields |
| Esc | Cancel the current edit or popup, then leave the active tool, selection and Present mode in order |
| F12 | Developer inspection in Debug and RelWithDebInfo builds |
| Ctrl+Q | Quit |

Number fields accept exact values with optional units, such as `2 kg`, `200 g`, `30°` or
`0.5 m/s`; a value typed without a unit is read in the unit shown beside the field, so `2.0`
beside `mm` means two millimetres. Each adjustment creates one undo entry when you release the
control or press Enter. Esc restores the previous value. Dots and revert controls show setup changes. **Back to start** uses
the current starting setup; **Restore original** returns to the authored experiment. Lab settings
that remain after a restore are listed separately.

Measure records a current and previous trace continuously. Runs are sampled at 40 Hz for up to
60 seconds, kept only for the current session, and bounded to ten per experiment and 48 MiB total.
Starred runs are protected. Up to twelve pinned values can report At end, Maximum, Minimum, At first
impact or At t = …; Compare shows A, B, Δ and Δ %, with a warning when starting setups differ by
more than one change. Graph can overlay the previous run and one kept comparison run.

PNG capture has two areas, chosen in Preferences: **Stage only** crops to the simulation stage
and contains no interface, while **Whole window** composites the learner interface over the scene
exactly as shown.

Settings are documented in `config/application.cfg`. Every key is optional; malformed or unknown
values are reported and skipped so configuration cannot prevent startup. See
[docs/CONTENT_FORMAT.md](docs/CONTENT_FORMAT.md) for content persistence and compatibility.

## Layout

```
src/RigidBodies.Math/        Vectors, rotations, transforms, bounds, polygon geometry
src/RigidBodies.Physics/     Bodies, shapes, materials, forces, integration, collision, worlds
src/RigidBodies.Core/        Logging, configuration, resource location, project identity
src/RigidBodies.Render/      Camera, draw lists, render device, scene renderer, layers
src/RigidBodies.Ui/          Panels, commands, interface backends
src/RigidBodies.App/         Window, input, session, run loop, Windows resources
tests/                       Simulation, rendering, interface, and session test suites
tools/                       Command-line diagnostics: benchmark, determinism probe, energy drift
fuzz/                        Document fuzzing harness and corpus
assets/                      Run-time assets
config/                      Default configuration
vendor/                      SDL3, FreeType, RmlUi and Dear ImGui, used by both builds
visualstudio/                Native Visual Studio projects and property sheets
cmake/                       CMake modules
scripts/, packaging/         Maintenance, benchmark and installer scripts
docs/                        Architecture and build documentation
```

The dependency between modules points one way: `Math` ← `Physics` ← `Render` ← `Ui` ← `App`, with
`Core` available to everything above `Physics`. `Math` and `Physics` have no windowing, rendering or interface dependencies, so
you can create, step and measure a world without a display. [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) describes the module
boundaries.

## Technology

| Concern | Choice |
| --- | --- |
| Language | C++17 |
| Build | CMake, with dependencies found or fetched |
| Platform, window, input, timing | SDL3 |
| Rendering | OpenGL 3.3 shaders with MSAA and instancing; shared antialiased geometry and SDL fallback |
| Interface | RmlUi documents, FreeType, bundled Inter and Phosphor icons; the built-in overlay is a fallback |
| Developer tooling | Dear ImGui inspection in development configurations, separate from the viewer interface |

Releases and CI currently target Windows x64. The source retains its Linux and macOS support for
future builds. Platform-specific code stays in the module that handles the operating system.

## Conventions

Four-space indentation, Allman braces, `snake_case` for functions and variables, `PascalCase` for
types, and a trailing underscore on private members. Include units in quantity names, such as
`mass_kg`, `position_m` and `torque_n_m`, so readers can tell what a value represents. The `.clang-format` and `.clang-tidy` configurations are authoritative.
