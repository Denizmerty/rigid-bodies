# Architecture

The application separates mathematics, physics, rendering, interface and application code into
modules with one-way dependencies.

## The one-way dependency

```
Math  <-  Physics  <-  Render  <-  Ui  <-  App
                          ^        ^       ^
                          +--------+-------+---- Core
```

`RigidBodies.Math` and `RigidBodies.Physics` depend only on the standard library. They can run
without a window, renderer or interface, which supports headless tests and batch experiments.
Configure with `RIGIDBODIES_BUILD_APP=OFF` to build and test them without third-party packages.
A dependency-related failure in that configuration indicates that a higher-level dependency has
entered the simulation modules.

The renderer reads the world and maintains its own trajectory history. Physics objects have no
rendering callbacks or display history. This keeps the simulation independent of how it is shown.

`RigidBodies.Core` provides logging, configuration and resource paths to the modules above Physics.
The simulation does not use Core, so it has no dependency on filesystem access or logging.

## Replaceable components

The following interfaces support multiple implementations:

| Interface | Purpose |
| --- | --- |
| `Integrator` | Compare integration methods in the same experiment |
| `BroadPhase` | Compare a spatial tree against exhaustive pair testing |
| `NarrowPhase` | Use suitable algorithms for different shape pairs |
| `ContactSolver` | Compare approaches to solving contact impulses |
| `ForceGenerator` | Enable or disable individual physical effects |
| `Constraint` | Solve joint rows together while supporting custom sequential constraints |
| `Shape` | Add shape types without changing the solver |
| `RenderDevice` | Share meshes, font atlases and export targets between OpenGL and SDL |
| `UiBackend` | Present the same controls through RmlUi or the overlay fallback |

The world, camera, session and application each have one implementation and use concrete types.

## Collision implementations

`NullNarrowPhase` reports no contact for any pair, and `NullContactSolver` leaves every velocity
untouched. They implement the interfaces with no collision response.

They remain useful for isolated dynamics experiments. Production worlds use
`DynamicTreeBroadPhase`, `CollisionNarrowPhase`, and
`SequentialImpulseContactSolver`. The brute-force broad phase remains an exact reference. The
world owns manifold persistence and event lifetimes; the solver owns transient constraints, using
body IDs and local anchors rather than pointers into a world. Snapshots clone both independently.

## The simulation step

`World::step` calls each phase in this order:

```
apply forces
    -> integrate velocity
        -> detect collisions
            -> solve velocity        (several passes)
                -> integrate position
                    -> constrain swept motion
                        -> solve position (several passes)
```

The velocity solver runs after force integration and before positions advance. This lets it
correct the updated velocities before bodies move into one another. Applying that correction a
step later would cause visible penetration and popping.

The sequence includes sleeping, prescribed paths, numerical bounds, and staged
integrators. Verlet and RK4 predict free motion using isolated force probes, then commit their
predicted displacement plus the contact solver's velocity correction. The correction is measured
against the bounded velocity passed to the solver, keeping numerical caps separate from physical
impulses.

World-owned spring pairs run after the one-body force generators. Connected bodies use
synchronized trial states in Verlet/RK4, so both ends of a spring use the same integration stage.
Worlds without springs can still use the single-body/custom integrator interface. Trial force registrations retain shared identity and canonical
traversal while staying detached from live state.

Built-in joints prepare and warm-start Jacobian rows through the constraint interface. Connected
dynamic islands solve these rows as a bounded impulse system between contact passes. Custom constraints without rows retain their sequential callbacks. Position correction
remains separate from velocity impulses. Breaking is decided from the completed substep's load
and releases subsequent solves; joint reports retain the failure load. Stable registration keys,
definition revisions, collision filtering, wake propagation, and snapshot cloning belong to the
world.

Shape conversion stays in the dependency-free physics module. `Outline` retains authored
nodes and tangent handles; `AuthoredShape` freezes separate render and collision polygons, render
triangles, and convex collision cells. All cells from one logical part share an immutable
`AuthoredPartDefinition`, including material and local placement. Rendering follows the logical
outline while collision uses the cells. The App's `ShapeEditor` owns a draft and a bounded geometry
cache; `SimulationSession` commits validated edits through transactional authored-body operations.
Snapshots retain immutable authored geometry and copy mutable body state. Successful authoring
transactions invalidate live body pointers; handles remain valid for a part edit, while assembly
and separation replace their source handles.

Contact impulse applications report signed energy changes and external drive work. Responding
contacts are then grouped into bounded impact reports for each body pair. Snapshots preserve
this physics evidence. Read-only energy and reference-experiment APIs remain dependency-free.
The session retains display histories, handles pause-on-impact between substeps, and routes
validated property edits without replacing body handles. Rendering converts SI values only for
display and computes vector scales without modifying the world.

## Force attachment and measurement

`World::add_force_generator(generator)` registers a field for all bodies;
`World::add_force_generator(body_id, generator)` attaches it only to a particular live body.
Global generators run first in registration order, each visiting canonical body order. Body
attachments then run in canonical body order and their own registration order. Duplicate
registrations are additive, and removing a pointer removes all its registrations within the chosen
scope. Destroying or clearing a body removes its attachments. Generation checks prevent an old
attachment from following a reused slot.

The world supplies each generator's name as its default force channel. Direct body calls can pass
an explicit channel name; unnamed direct calls use `external`. Contributions with equal names
combine, and channel reports sort by name. Forces applied away from the centre of mass record both
the force and its torque in the same channel.

Pending accumulators feed the integrator. Just before velocity integration, the world copies their
totals and channels into retained applied loads. Clearing the pending accumulators at the end of
the step leaves this report intact; the next valid step replaces it, including when all loads have
become zero. Drawing and inspection can then read the report between steps and while paused without applying
forces again. Substepping reports the last substep, not a frame average. Direct applied forces last
for one physics substep; sustained effects belong in a force generator.

These reports cover external loads. They exclude contact impulses, damping and measured velocity
changes.
Rendering uses the retained net force at the interpolated centre of mass and derives the external
force acceleration from mass. The inspector shows individual named forces and torques, so a zero
net force does not hide opposing contributions.

## Fixed simulation steps

`TimeStepper` belongs to the Physics module so headless and interactive runs use the same pacing
rules. A fixed step makes simulation results independent of display frame rate. The renderer uses
the stepper's interpolation fraction to draw between simulation states.

Each returned fixed step runs `substep_count()` calls to `World::step(substep_s(), ...)`.
The first substep captures the previous body pose; later substeps preserve that same endpoint.
Rendering interpolates the centre of mass and unwrapped angle across the whole fixed step, and
selection tests the same displayed pose. Explicit placement setters synchronize both endpoints,
while integrators and position solvers use `set_simulated_pose` to preserve the previous state.

The per-frame budget counts physics substeps, so increasing substeps cannot bypass it. Whole steps
that exceed the budget are discarded, with the remaining fraction retained for rendering. Both
stall clamping and budget loss are reported in simulated seconds. This slows the simulation under
load without building an unbounded backlog; it does not impose a wall-clock deadline on one step.

## Bodies, frames, and the centre of mass

A body stores the placement of its own frame and, separately, where its centre of mass sits within
that frame. Velocity always refers to the centre of mass, because that is where the equations of
motion separate into a translation and a rotation.

For a symmetric shape, the frame origin and centre of mass coincide. For compound objects such
as a hammer or chair, they can differ, especially when the parts have different densities.
Rotation must use the centre of mass to produce the correct motion.
`SemiImplicitEulerIntegrator::integrate_position` advances the centre of mass and then places the
frame so that it still holds that centre.

## Body identifiers

`BodyId` is a slot index paired with a generation counter. The interface holds a selection across a
scenario change; a raw pointer would dangle and an index alone would silently address whichever body
next occupied the slot. The generation makes a stale identifier detectable, and the physics test
suite covers exactly that case.

Iteration uses a separate ordering from slot allocation. `create_body(definition, stable_key)`
accepts an optional unique key; keyed bodies sort lexicographically before unkeyed bodies, which
retain creation order even when a freed slot is reused. Built-in scenarios provide keys. The
world also normalizes and sorts candidate pairs in that order, so a broad-phase implementation's
enumeration order does not alter the solve order. Reproducing construction in a different order
requires the same stable keys.

## World snapshots

`World::snapshot` captures bodies, allocator state, ordering keys, environment, time, statistics,
contacts, forces, springs and their identifier allocator, constraints, and the selected integration
and collision implementations.
Force registrations retain their shared identity across global and body scopes within a snapshot,
while their mutable state is detached from the live world. Pending and applied load reports are
copied with each body.
`World::restore` clones that captured state before replacing the live world, so snapshots can be
reused and failed restores leave the live world unchanged. Scenario reset uses its starting
snapshot and reacquires force references after restoring.

Component `clone()` implementations must copy their mutable state independently. Built-in
components support this; a custom component without snapshot support fails explicitly instead of
sharing live mutable state. Shapes are cloned too, preserving sharing within the snapshot. Custom
components must not keep pointers to live bodies; body IDs are restored, while pointers and
references to the old world contents must be reacquired. Snapshot restoration also restores
the allocator, so handles from a discarded future timeline must also be discarded by the caller.

Snapshots are in-memory replay state, separate from the on-disk scenario format.

## Shapes are shared and immutable

A `Shape` is immutable and held by `shared_ptr`, so several bodies can safely share its geometry.
Each collider stores its own placement. Compound bodies can therefore reuse a shape at several
positions and orientations.

Every polygon shape is convex and wound counter-clockwise, enforced at construction. The narrow
phase, the mass-property computation, and the triangle fan in the render device all rely on it. A
concave outline from the authoring tools is decomposed before it can reach a collider.

## Two dimensions, one out-of-plane thickness

The simulation is planar. Materials use bulk density in kilograms per cubic metre, and each
collider has a `depth_m` for its out-of-plane thickness. Mass is area times depth times density.
This lets scenarios use familiar material densities, such as those of steel and oak, while still
modelling motion in two dimensions.

## Drawing and render devices

Scene and interface rendering record a `DrawList`, a flat command buffer in device pixels. Scene
code projects world coordinates through the camera; interface code already works in pixels.
`RenderDevice` can replay both lists through the same graphics backend.

Recorded primitives compile into feathered indexed meshes. Layer sorting is stable and
batching joins only adjacent compatible texture/clip state, preserving translucent composition.
The compiler reuses per-thread scratch buffers, so a frame of thousands of strokes does not
allocate per primitive. The OpenGL 3.3 core device packs a whole compiled list into one vertex,
index and instance upload per submit and draws runs of compatible meshes with one call each:
untextured vertices carry a negative texture coordinate that the shader treats as white, so
geometry interleaved with text keeps a single draw call. It renders through multisampled targets
and hardware instances; the SDL device replays the same geometry and expands instances.
`--render-benchmark` reports mean and p95 frame time with per-phase medians (simulation, scene
recording, interface, device submission, presentation). Immutable shared texture pixels
outlive recorded commands and native uploads are reclaimed when their source ownership ends.
The shared FreeType atlas supplies scene and fallback text with proportional letters and tabular
digits. It hints only vertically, so glyph advances stay the rounded design widths that the scene
renderer mirrors when it reserves label space without a device round trip.

Feathering is the compiler's antialiasing: exposed boundary edges gain a band that fades to zero
alpha over one device pixel per unit of display scale. Interface meshes are marked
`pixel_aligned`, because laid-out boxes sit on whole pixels; their axis-aligned grid edges get no
band, keeping rules and dividers sharp and one pixel wide. On a
multisampled target the device sets `DrawCompileOptions::multisampled` and interface meshes skip
feathering altogether, leaving rounded corners to the coverage samples so they carry the same
weight as the straight edges beside them. Scene meshes keep their band on every target.

`SceneRenderer` handles material shading and impact particles. Each physical material maps to a
`MaterialAppearance` with surface, shade, highlight, rim, grain, gloss and edge-light properties.
A fixed light from the top left gives all bodies consistent lighting.

Shading is tessellated on the CPU into vertex-coloured meshes. This lets the SDL fallback produce
the same image without programmable shaders, and lets PNG capture and render tests compare the
compiled geometry. The shading budget keeps scene recording to a small part of frame time;
`--render-benchmark` reports that cost.

Stage labels use a shared placement pass. Bodies, vectors, springs and joints submit requests
with a priority, anchor and preferred position. The pass tries positions around each anchor and
avoids existing labels, arrow shafts and reserved areas. It remembers the previous frame's choice
to reduce movement, removes secondary text such as mass or "not to scale" when space is tight,
and draws a thin leader line when a label moves away from its subject.

Labels scale with display density and the text-size preference. Body names come from
`SceneRenderer::set_body_names`, using the same names as the Inspector, context menu and hover
card. App overlays reserve their areas through `set_overlay_areas` before placement, including the
gravity dial, selection grips and the tether of a velocity handle beyond a capped arrow.
`set_overlay_rings` helps labels avoid the selection's rotation ring.

When a hover card is visible, `SceneRenderSettings::label_replaced` suppresses the matching body
label. Identical measurements shared by several bodies are shown once with "each" appended.
Body and connection labels avoid covering their subjects. Present mode enlarges stage text,
chips and handles together.

Force arrows form a free-body diagram for each dynamic body:

- External loads are read from force channels and drawn at the centre of mass. Labels identify
  Weight, Air drag, Magnus lift, Attraction, Pull and Spring. Weight has a filled arrowhead; the
  other loads have hollow heads, so they can be distinguished without colour.
- Contact force uses the contact colour and an open head. It is calculated from solved normal
  and friction impulses, averaged over at least a frame of substeps to smooth alternation between
  solver steps. Sleeping bodies show the equilibrium value, so their force arrows balance.
- A fixed support's push, or a selected body's contact with one neighbour, approaches from outside
  with its head on the contact face. If it would overlap the body's own arrows, it starts at the
  centre of mass instead. Several contacts combine under the label Net contact. Selecting the
  body also shows each neighbour's contribution as "Push from ...", placed within the body
  exerting it, beside that body's arrows.
- A load that would end inside its own body can be drawn beyond the outline, connected by a thin
  leader. This is allowed only where there is free space; arrows never enter neighbouring bodies.
  Joint reactions use the force colour at the focused body's anchor, beside the link and short
  of its far end.

Arrows are limited to the visible stage, with only a small margin outside the framing area, so
they stay clear of panels. A shortened arrow has a slanted break. A minimum-length arrow for a
very small value has a dotted shaft. Its label says "not to scale", or shows a miniature mark
when there is too little room for text.

In manual mode, force scale is based on scene weights so both a 0.1 kg ball and a 30 kg crate have
readable arrows without crowding stacks or shelf lanes. With gravity off, the scale follows the
applied loads and decreases when they remain well below it. The key announces scale changes.
It sits beside the scale bar in a corner clear of arrows, moving bodies and overlays, and shows
the drawn length for a round-number force.

Arrow casings use the label-background colour for contrast and share one mesh per frame. Scales
are logical pixels per unit, multiplied by display density once per frame. This keeps the layout
consistent on 200% laptop displays and 100% projectors. Velocity handles use the displayed factor
and hold it constant during a drag. Selecting bodies keeps their arrows prominent and dims the
others. Trails keep at most one point per device pixel before tessellation, which halves scene
vertices in the benchmark.

The physics step records energy changes without affecting the simulation. Measure reads work
from contacts, air, dampers, joints, motors, applied loads and edits between steps. Potential
energy is evaluated at the middle of the last step, using the quantity semi-implicit Euler
conserves during free flight. A resting body reports `m g h` and `k x^2 / 2` exactly.

Motor work is measured on its own joint row. If a travel stop later absorbs that energy, it is
recorded as a joint loss without reducing the recorded motor input. Stored spring energy belongs
to the moving bodies it connects; a mass attached to a fixed anchor owns all of that energy.

The Energy tab compares starting energy with current mechanical energy plus losses minus added
energy. The shares chart groups kinetic energy, potential energy and each kind of loss into a
budget totalling 100%. Height-based potential uses the first solid static surface below an object
along its gravity direction as zero height. If there is no surface below, it uses the lowest
position recorded across that object's runs.

A completed physics step supplies immutable contact evidence, and the renderer owns bounded visual
histories and seeds. Repeated rendering does not resample an impact. Theme easing consumes
presentation time once; PNG capture replays the already recorded scene and cannot change simulation
time or body state.

## Camera coordinates

World coordinates use metres with positive y pointing up. Device coordinates use pixels with
positive y pointing down. `Camera2D` performs the conversion, so drawing code uses one coordinate
system at a time.

The view is specified by the visible world height. The viewport height sets the pixel scale,
and its aspect ratio determines how much of the world is visible horizontally.

**Frame subject** fits the experiment into the focus rectangle, the part of the stage no panel
covers, and the status line reports that rectangle's height. A bundled experiment may author the
region; otherwise the session frames its free objects, their joints' permitted travel, the points
their forces pull towards and a 1.5 s look-ahead run on a restored copy of the world, which adds
small fixtures the motion strikes and stops following anything that leaves. The live world is
never stepped for framing.

## Interface commands

Panels read a `UiModel` assembled each frame and emit `UiCommand` values to request changes.
`SimulationSession` builds that view from the simulation and applies validated commands. Panels
have no direct access to modify the world or camera.

The command enumeration lists the actions available to the interface. Adding an action requires
an explicit handler in the session, keeping simulation changes and validation in one place. The
relativity experiment adds one, `set_relativity_speed`: the Probe speed field, its slider, the
preset chips and the speed keys all send it, with `nudge` or `preset` in its detail for the keys.

Tabs, disclosures, checklist choices, search text, selected cards and open surface stacks are
view-only state owned by `ViewState`. They never enter `UiModel`, physics snapshots or undo. The
document event boundary mutates that state; panel construction receives it read-only.

Continuous controls use the same command seam with an explicit `UiEditPhase`. A `preview` starts
one pending transaction from the pre-drag state, updates the visible value, and holds simulation
time without restarting histories or posting completion feedback. `commit` applies those deferred
side effects once and records one undo entry; `cancel` restores the transaction's original state
exactly. Commits for the same control can coalesce only when their wall-clock timestamps are no
more than one second apart. Wall time is supplied by the application and never enters the physics
world, so coalescing cannot affect deterministic stepping. The probe speed is the one exception to
holding time: its preview leaves the clocks running so their rates visibly change, and cancel
restores the speed without rewinding them.

## Two backends behind the interface

The overlay backend draws panels through the project renderer. It needs no package beyond the
renderer, cannot fail to start, and is therefore the fallback when a requested backend is
unavailable.

RmlUi is the default presentation. `PanelBuilder` also records semantic rows, so the document
backend can use proportional font metrics, wrap text, manage keyboard focus, and scroll or reflow
the panels without changing their commands. The document and stylesheet in `assets/ui` control
surfaces and typography. Indexed textured meshes retain font atlas ownership and clipping in the
draw list. Accelerated SDL devices retain RmlUi's premultiplied alpha; the software device converts
to straight alpha for its surface-blit path and preserves rounded fans by submitting their
untextured triangles individually. The shared compiler feathers exposed off-grid document boundaries
on the fallback, while the OpenGL device relies on multisample coverage for them. The scene and
overlay use their own Inter atlas through the same device seam.

Panels describe rows and commands. Each backend handles their layout and presentation.

Tests require every active, non-developer command to be reachable through both a document row
and an overlay hotspot across the standard fixture states. The overlay is one keyed scrolling list, including typed number entry, while the document
reconciles elements by semantic `(host, key, instance)` identity. The window commands (minimize,
maximize or restore) are the one exemption: only the document backend draws the window's title bar,
and under the overlay the platform's own title bar does their work.

## The window's own title bar

On Windows, `window.custom_title_bar` is on by default. The interface draws the app icon on the
left of the top strip and window controls on the right. The borderless window retains Windows
caption, system-menu and resizing styles so snapping, maximizing, minimizing and animations work.
`WindowChrome` subclasses the window procedure to handle this behaviour.

Windows keeps invisible borders outside three edges; the top resize edge is inside the window.
Because SDL sizes borderless windows without those borders, `WindowChrome` adjusts the outer and
minimum sizes. Other hit tests call `UiContext::window_part`. The document backend checks the
topmost element and the toolbar column at that point. Unoccupied areas act as the draggable caption.

The three window controls are reported as native caption buttons. They work on the first click
when the window is inactive, and maximize supports Windows 11 snap layouts. Windows reports hover
and press states for the interface to draw. The close button, Alt+F4 and taskbar close all use the
Quit command, including its unsaved-change confirmation.

During native move, resize and menu loops, an event watch draws frames on live exposure events so
the interface updates throughout the operation. Other platforms, high-contrast mode and the
overlay backend use the platform title bar.

`SimulationSession` owns edit history. Frozen world snapshots include the reset
baseline, stepping state, selection, authoring draft, and explicitly chosen physics preferences.
Commands and captured pointer gestures form bounded transactions; invalid batch property edits
cannot partially update a selection. Undo/redo restore a checkpoint and pause, while appearance
preferences remain independent. A temporary body-attached spring samples the actual integration
stage for pointer pulling and is removed before a committed history record. Dear ImGui is an App
development adapter that emits the same validated commands and records six timing phases. It is
excluded from viewer builds.

## Configuration defaults and errors

`ApplicationConfig` starts with working defaults. Missing files are allowed. Unknown keys and
malformed values are reported and skipped, leaving the defaults in place so configuration errors
do not prevent startup.

## Resources located from the executable

`ResourcePaths` searches an ordered list of locations relative to the executable, covering both
the build tree and installed layouts. This avoids depending on the directory from which the app
was launched. Both builds copy `assets` and `config` beside the executable, so development runs
exercise the same resource loading as installed runs.

Assets are read-only during normal use. Captures and default save locations use a per-user
directory resolved per platform; explicit content file dialogs may select another location.

## Versioned content

The Physics module owns a dependency-free JSON codec and versioned scenario/shape documents.
The catalogue validates its complete directory and prerequisite graph before publishing new
descriptions. Population builds a separate world and commits only a valid arrangement. Persistent
document body IDs preserve references and extension fields independently of runtime handles.
Authored parts retain source outlines and optional standalone shape provenance through edits,
assembly, snapshots, and persistence; tessellation is regenerated from those sources.

The App adapter adds saved pacing and camera settings and native file dialogs. SDL callbacks
publish paths through a shared mailbox without touching the session. The main thread performs
validated, undoable scene changes or stages a complete file before replacing the destination.
Edit history shares immutable document sources, so each gesture does not duplicate the JSON tree.
[CONTENT_FORMAT.md](CONTENT_FORMAT.md) defines compatibility and the boundary between persistent
arrangements and transient solver evidence.

## Interface mechanisms

The interface is coordinated by six mechanisms:

- **Layout manager.** `compute_layout` is the sole authority for command bar, draw bar, stage,
  focus rectangle, docks, rails, sheets, Measure, Present strip, Present caption, and title bar,
  app mark and window control geometry. It
  arbitrates side surfaces (one docked side in Medium, one sheet at a time in Narrow), computes
  occlusion-aware camera framing, and publishes the same rectangles to input, scene rendering and
  both interface backends. Where only one side can dock, `UiContext` gives it to the most recent
  intent: opening an experiment with a guide gives it to the Guide, selecting an object (when the
  Inspector can dock) or asking for the Inspector or World gives it to the Inspector, and asking
  for the Guide gives it back. Toolbar contents are fitted by the document backend rather than the
  layout: rows carry a `RowPresentation` (icon, shortcut, badge, emphasis, the step at which the
  label hides, the step from which a title may shrink behind an ellipsis, and the step at which
  the row folds), and the backend applies steps in order until the strip fits, folding
  command-bar rows into the main menu.
- **View state.** `ViewState` owns tabs, disclosures, surface stacks, search history, first-run
  hints and Present state. This state can change presentation but never enters `UiModel`, physics,
  setup snapshots or undo. Only the documented preferences and dismissed hints are persisted.
- **Input router.** One ordered router resolves text editing, captured drags, transients, sheets,
  tools, selection, keyboard mode and Present. The nine-step Escape ladder therefore cancels one
  innermost state at a time. The four-logical-pixel gesture threshold is shared by every tool,
  handles and secondary-button panning. A pointer leaving for the title bar or out of the window
  reaches both the interface and the stage, so neither keeps a stale hover, and `frame_hit` turns a
  window part into the platform's resize, caption or caption-button area.
- **Notification service.** `Notifier` stores typed, source-keyed messages; `ToastPresenter`
  handles active wall time, replacement, limits and Present filtering. Inline notices, banners
  and toasts remain semantically distinct, and reveal actions open and focus the owning control.
- **Run recorder.** `RunRecorder` samples deterministic float series at 40 Hz, shares immutable
  buffers with history snapshots and enforces per-experiment retention, the global star cap and
  48 MiB live budget. Runs are session-only; comparison and graph selection remain view state.
- **Content reader.** Versioned JSON is decoded in Physics without UI dependencies, then the App
  layer resolves guide variables through the control registry and stages complete documents before
  committing them. Unknown extension data is retained, while runtime evidence such as selection,
  impacts, recorded runs, open surfaces and Present mode is intentionally excluded.

Direct manipulation publishes hover, handle, connection, marker and context-menu hit areas as
screen-space UI model data. The scene owns geometric picking and transactional drags; panels own
only the commands they emit. Command search consumes the same key and control registries, so an
action uses its existing handler and a setting result opens the control that already edits it. Present is a non-persistent chrome state layered on those same
mechanisms; its demonstration lock is enforced both while controls are built and while keys are
dispatched.

## Special relativity beside the world

The Chasing light experiment models a point probe at a constant, learner-set speed close to c. It
runs beside the World rather than inside it: rigid-body dynamics stay Newtonian, and the World
holds only the probe's static track.

`physics/special_relativity.hpp` is dependency-free like the rest of Physics. `lorentz_factors`
derives γ, γ − 1, βγ, 1/γ and the rapidity from β without cancellation at either end of [0, 1). It
carries 1 − β, which is exact for β ≥ ½, and forms γ − 1 as (βγ)²/(γ + 1), so both 0.9999999 and
10⁻⁹ keep their digits. `maximum_speed_fraction` caps the speed at 0.9999999, where c − v is still
about 30 m/s, and the session, the probe and the document codec each enforce it. Below it, a moving
speed is never slower than `minimum_moving_speed_fraction` (10⁻¹², about 0.3 mm/s): the speed field,
the session and the document reader refuse anything between rest and that floor.
`speed_fraction_on_ladder` treats a speed a rounding error from a rung of the speed ladder as that
rung, so a typed 99.999 % is 0.99999 c, with its preset chip, its limit notice and its next step.
`RelativisticProbe` holds the probe, the clock it carries and the light pulse that leaves the start
line with it on every lap of a track ten light-nanoseconds long. Each constant-speed segment is
exact, so step size and step count change a result only by rounding, and a large step runs whole
laps in constant time. The probe is copyable, so undo snapshots hold it by value.
`physics/relativity_document.hpp` reads and writes the document's `relativity` object, and
`validate_document_header` accepts the `special_relativity` feature only for scenarios
([CONTENT_FORMAT.md](CONTENT_FORMAT.md#special-relativity)).

`SimulationSession` keeps the probe in an optional member that opening an experiment, undo and
redo, and Back to start replace. The probe advances in the World's own fixed steps, by each
substep times `relativity_lab_seconds_per_world_second` (10⁻⁹): a world second, which is a screen
second at 1×, is a lab nanosecond. It interpolates between the same two steps as the bodies. Its
speed belongs to the starting setup and the live probe at once: a speed edit is one "Change probe
speed" parameter entry that never rewinds or pauses the clocks, and undo restores the speed alone.
Because the clocks keep running, the run's graph is marked "Speed 0.5 c → 0.9 c" wherever their rate
changed: where a drag first moved the speed, at both ends of a cancelled drag, at an undo or redo
and at a per-item revert.
Commands that act on Newtonian objects, such as adding, drawing, importing, deleting, throwing,
pulling and playing until the next impact, are refused at one gate with one notice per opened
experiment. Saving writes the speed into the document and the setup fingerprint includes it, so a
speed edit marks a saved setup as changed while running never does.

Each frame the session hands `SceneRenderer::set_relativity_stage` a `RelativityStage` POD built
from the interpolated sample; Newtonian experiments pass `std::nullopt`. The track itself is the
World's static marker body, drawn by the body pass on the static-body layer (−8).
`relativity_stage.cpp` draws the start and finish lines, the light-nanosecond marks and their
numbers, and the light lane on layer −6, above the track body; the pulse and the probe on layer 0;
the clock faces and fixed plates on layer 20; and the instrument band across the top of the
framing area on the instrument layer. It then records the drawn layout for the probe click and the
tests. The scale key keeps clear of the band, faces, track and plates, and is left out of a stage
too short for the apparatus, where every corner holds one of them; the
shared label pass places the probe's clock plate clear of the pulse and the mark numbers.
`scene_style.hpp` holds the typography, stroke and mesh helpers it shares with the scene
renderer, so stage text is measured the same way wherever it is drawn. Three tiers (full, compact
and minimal) follow the framing size, and every tier draws both clock faces: smaller faces and
fewer plates as the stage shrinks, down to the minimal tier, where the probe's face is its marker
and the lab face stands beside its plate under the track. While the framing size keeps changing,
hysteresis keeps the tier near a threshold; once it holds still for a frame, the session settles
on the largest tier that fits, so a passing size, such as Present's caption before it is
measured, never leaves the stage in a smaller tier. The band's height depends only on the
framing width, the text scale and the units, never on the values shown, so the session's framing
reserves exactly that height above the track instead of shrinking the camera's focus rectangle.

The interface reads the experiment from `UiModel::relativity`, which the session fills from the
same physics helpers and the same interpolated sample the stage draws. `measure_tabs.hpp` gives
the Measure tab set for a model: Relativity, Graph and Runs here, and Energy, Graph, Collisions,
Runs and Theory checks otherwise. `RunRecorder` adds a relativity block of proper time τ, t − τ
and γ − 1 to each sample of a relativity run, so Graph and Runs plot the clocks over lab time.
Core's `speed_fraction` format truncates towards rest so a speed below c never reads as c, and
Core groups long numbers with a thin space, which the bundled Inter faces draw as a blank glyph.
