# Content document format 1.0

Rigid Bodies stores arrangements and independent authored shapes as UTF-8 JSON. The physics
module reads both formats using only the C++17 standard library. The application version
and document version `1.0` are independent: a program release does not automatically change the
file format.

## Envelope and compatibility

Every document is an object with `format` and `version`:

```json
{
  "format": "rigid-bodies.scenario",
  "version": { "major": 1, "minor": 0 },
  "metadata": { "id": "example", "title": "Example" },
  "world": { "bodies": [] }
}
```

`rigid-bodies.scenario` identifies arrangements; `rigid-bodies.shape` identifies authored shapes.
The major number must be `1`, and the minor number must be a nonnegative integer. Later minor
revisions may add optional information without changing the meaning, units, or defaults of
existing fields. An older reader accepts those revisions and retains their original version.
A different major number is rejected before the live world changes. Version 1 is the first public
file format.

The optional `required_features` array declares features that a reader must understand. It must
be an array of distinct strings, each naming a feature the reader supports. Scenario documents
support one feature, `special_relativity` (see [Special relativity](#special-relativity)); shape
documents support none. Any other requirement is rejected, even when the version number would
otherwise be compatible. Unknown physics component kinds also fail explicitly; they are never
silently dropped or replaced by a default component.

Unknown object fields are retained in the source document. Scenario capture with that source
updates known fields while retaining extensions on matching objects. Body extensions follow the
persistent document `id`, even when simulation slot allocation changes. For other object arrays,
matching uses the collection position when its length and component type still agree. Removed
objects remain removed; replacing a component type discards fields that belonged to the previous
type. This preserves optional extensions for readers that understand them.

New required behavior, changed units, or changed interpretation of an existing field requires a
new major format, or an explicit required feature understood by the reader. Future major-format
support must include a conversion that updates the document contents. Editing the version number
alone leaves the old contents in place.

JSON is strict: duplicate object keys, comments, trailing commas, invalid UTF-8, unpaired Unicode
surrogates, nonfinite numbers, and extra content after the root are rejected. Object key ordering
and whitespace have no meaning. Numeric output retains enough precision to round-trip a physics
`double`; all coordinates and physical quantities use SI units. A two-dimensional vector is
`[x, y]`, angles are radians, and nullable optional values use JSON `null`.

## Scenario metadata and catalogue rules

An individual scenario requires nonempty string `metadata.id` and `metadata.title`. Optional
`summary` is explanatory text; `concepts` and `prerequisites` are arrays of nonempty strings;
`suggested_order` is a nonnegative integer. Omitted arrays are empty, and omitted order is zero.
Standalone arrangements can carry prerequisite information without shipping the referenced
lessons, and can be opened without registration in a catalogue.

The bundled catalogue has these additional requirements:

| Field | Catalogue requirement |
| --- | --- |
| `id` | Unique across the directory; 1–128 lowercase letters, digits, underscores, or hyphens |
| `title`, `summary` | Both nonempty |
| `concepts` | Nonempty list, with no empty or duplicate entries |
| `prerequisites` | Unique scenario IDs that exist in the same catalogue; the graph must be acyclic |
| `suggested_order` | Nonnegative integer; sorting uses this first, then ID |

The first suggested scenario is the default. `assets/scenarios` contains 25 complete arrangements,
with explicit ordering and concepts. The application reads its delivered asset directory before
configuring the session. Headless tests and tools have a build-configured source asset directory.
`available_scenarios()`, `find_scenario()`, and `load_scenario()` still expose descriptions and
populate worlds through the same runtime path. `initialize_scenario_catalogue()` validates the entire directory
before replacing it; failure preserves all previous descriptions, source documents, and loads.
A successful replacement invalidates existing catalogue views and pointers.

Only regular files with a `.json` extension are read, case-insensitively. Filename order has no
meaning. Adding a document changes the catalogue on the next application start; there is no live
filesystem watcher. For the complete teaching metadata and example physics, use the documents in
[assets/scenarios](../assets/scenarios).

## Scenario world

`world.bodies` is required and may be empty. Other sections use version-1 defaults. Files
written by the application include their current values explicitly. `world.forces` is the complete global
force list: absent or empty means there are no global force generators, including gravity.
Setting `settings.gravity_m_s2` alone does not install a gravity generator.

| Section | Contents |
| --- | --- |
| `settings` | Gravity, air density/velocity/viscosity, broad-phase margin, constraint-graph switch, and nested solver/sleep/limits/collision settings |
| `algorithms` | Integrator, broad phase, narrow phase, and contact solver selections |
| `potential_energy_reference_height_m` | Height used as zero gravitational potential energy |
| `bodies` | Bodies, their primitive or authored parts, and per-body effects |
| `forces` | Global force generators |
| `springs` | Pairwise linear and angular springs |
| `joints` | Distance, revolute, prismatic, and weld constraints |

The `settings` fields are `gravity_m_s2`, `air_density_kg_m3`, `air_velocity_m_s`,
`air_dynamic_viscosity_pa_s`, `broad_phase_margin_m`, and `constraint_graph_enabled`. Their nested
objects contain:

- `solver`: `velocity_iterations`, `position_iterations`, `linear_slop_m`,
  `position_correction_fraction`, `restitution_threshold_m_s`, `warm_starting`, and
  `maximum_position_correction_m`.
- `sleep`: `enabled`, `linear_speed_m_s`, `angular_speed_rad_s`, `linear_acceleration_m_s2`,
  `angular_acceleration_rad_s2`, and `quiet_duration_s`.
- `limits`: `maximum_position_m`, `maximum_linear_speed_m_s`, `maximum_angular_speed_rad_s`, and
  `maximum_orientation_rad`.
- `collision`: `continuous`, `contact_margin_m`, `matching_tolerance_m`, `sweep_tolerance_m`,
  `maximum_sweep_iterations`, `friction_mixing`, and `restitution_mixing`. Mixing choices are
  `geometric_mean`, `arithmetic_mean`, `minimum`, and `maximum`.

If `algorithms` is present, all four selections are required:

| Selection | Supported values |
| --- | --- |
| `integrator` | `semi_implicit_euler`, `velocity_verlet`, `runge_kutta_4` |
| `broad_phase.type` | `brute_force`, or `dynamic_tree` with optional `base_margin_m` and `displacement_multiplier` |
| `narrow_phase` | `collision`, `none` |
| `contact_solver` | `sequential_impulse`, `none` |

### Bodies, materials, and parts

Each body requires a unique nonempty string `id`, a `type` (`static`, `kinematic`, or `dynamic`),
and a `parts` array. Body references use this document ID, never an in-memory slot or pointer.
`stable_key` separately retains canonical simulation ordering; nonempty live keys must be unique.

The remaining body fields are `name`, `position_m`, `orientation_rad`, `linear_velocity_m_s`,
`angular_velocity_rad_s`, `linear_damping`, `angular_damping`, `gravity_scale`, `fixed_rotation`,
`sleep_enabled`, `awake`, `mass_override_kg`, `pending_force_n`, `pending_torque_n_m`, `forces`,
and `motion`. A `null` mass override uses geometric mass. A non-null override must be positive.
Fixed rotation requires zero angular velocity. Pending force and torque preserve loads already
applied for the next step.

Every part has a `type` and `shape`. Both part types support `depth_m`, nullable
`density_override_kg_m3` and `drag_coefficient_override`, `is_sensor`, `local_transform`, `filter`,
and `material`. A transform contains `translation_m: [x, y]` and `rotation_rad`; a filter contains
integer `category`, `mask`, and `group` values.

A material stores `name`, `density_kg_m3`, `restitution`, `static_friction`, `kinetic_friction`,
`drag_coefficient`, `rolling_friction_m`, `spinning_friction_m`, `friction_axis_local`, and
`friction_anisotropy_ratio`. Restitution is between zero and one. Physical magnitudes such as
density, damping, depth, friction, and drag must satisfy the same validity checks as the physics
objects they create; malformed numeric types are errors rather than coercions.

A `primitive` part additionally supports nullable `shell_thickness_m`. Its shape is one of:

| `shape.type` | Required geometry |
| --- | --- |
| `circle` | Positive `radius_m` and `center_m: [x, y]` |
| `segment` | `start_m` and `end_m` |
| `convex_polygon` | `vertices_m`, an array of 3–4096 coordinate pairs |

An `authored` part supports `name` and stores the `outline` and `options` described below inside
its `shape`. It is written once per logical part, even when collision uses several convex cells.
Render outlines, triangles and convex decomposition are rebuilt from the source when loading,
so the file contains only one copy of the geometry.
An optional `source_document` on an authored part contains its original standalone shape
envelope, or is null. It preserves imported metadata and extensions while the outer part shape
remains the authoritative geometry. Saving synchronizes this envelope with the edited outline.

### Forces and prescribed motion

Both world and body force lists accept the following types, with an optional `enabled` flag:

| `type` | Fields |
| --- | --- |
| `uniform_gravity` | Uses the world's gravity vector and each body's gravity scale |
| `point_attractor` | `position_m`, `stiffness_n_m` |
| `aerodynamic_drag` | `estimate_outline_coefficient`, `reynolds_correction`, `angular_drag`, `magnus_lift`, `angular_drag_coefficient`, `magnus_efficiency`, `maximum_lift_coefficient` |

A body `motion` is `null` or one of the following objects. Only kinematic bodies accept it.
Every motion supports nonnegative `elapsed_s`, preserving the current path phase on reopen.

| `type` | Parameters |
| --- | --- |
| `linear` | `origin_m`, `initial_orientation_rad`, `velocity_m_s`, `angular_velocity_rad_s` |
| `harmonic` | `origin_m`, `initial_orientation_rad`, `translation_amplitude_m`, `rotation_amplitude_rad`, `frequency_hz`, `phase_rad` |
| `circular` | `center_m`, `radius_m`, `angular_speed_rad_s`, `phase_rad`, `orientation_offset_rad`, `orient_to_path` |

### Springs and joints

Springs refer to `first` and `second` body IDs, and support `stable_key` and `enabled`.
A `linear` spring adds `local_anchor_first_m`, `local_anchor_second_m`, `rest_length_m`,
`stiffness_n_m`, and `damping_n_s_m`. An `angular` spring adds `rest_angle_rad`,
`stiffness_n_m_rad`, and `damping_n_m_s_rad`.

Joints also refer to `first` and `second` body IDs. Common fields are `stable_key`, `enabled`,
`broken`, `local_anchor_first_m`, `local_anchor_second_m`, `collide_connected`, `break_force_n`,
and `break_torque_n_m`. A null break threshold means no finite breaking limit.

| `type` | Additional fields |
| --- | --- |
| `distance` | `length_m` |
| `revolute` | `reference_angle_rad`, `limits_enabled`, `lower_angle_rad`, `upper_angle_rad`, `motor_enabled`, `motor_speed_rad_s`, `maximum_motor_torque_n_m` |
| `prismatic` | `reference_angle_rad`, `local_axis_first`, `limits_enabled`, `lower_translation_m`, `upper_translation_m`, `motor_enabled`, `motor_speed_m_s`, `maximum_motor_force_n` |
| `weld` | `reference_angle_rad` |

References to missing bodies, duplicate stable keys, invalid joint definitions, and unsupported
component types reject the whole arrangement. User-defined C++ integrators, collision algorithms,
force generators, shapes, or constraints without a format-1 codec cause save to fail explicitly.

## Special relativity

A scenario that lists `special_relativity` in `required_features` is a relativity experiment. It
must also carry a top-level `relativity` object with both of these fields:

```json
{
  "required_features": ["special_relativity"],
  "relativity": { "rest_mass_kg": 1, "speed_fraction_c": 0 }
}
```

| Field | Meaning | Accepted values |
| --- | --- | --- |
| `rest_mass_kg` | The probe's rest mass | Finite, from 0.000001 to 1,000,000 kilograms |
| `speed_fraction_c` | The probe's speed as a fraction of the speed of light | 0 (rest), or finite from 0.000000000001 to 0.9999999 |

A missing object, a missing field, a value of another JSON type or a value out of range rejects
the document with a message that names the field, before the session changes. Parsing,
populating a world, capture from a source document and the catalogue all apply this check. A
`relativity` object in a document that does not declare the feature is an ordinary extension: it
is retained and ignored, and the document opens as a Newtonian experiment.

The probe is not a body. It moves along the x axis at the set speed on a repeating track of ten
light-nanoseconds (about 3 m) that starts at x = 0, and carries its own clock. Its speed is a
setting, so the starting and current speeds are always the same. The `world` section is still
required: the bundled `chasing_light` document uses it for the probe's track, one static body with
`role: "marker"`, and sets gravity to zero. Saving writes the current speed and rest mass into
`relativity` and keeps any unknown members of that object.

The application frames a relativity experiment from the stage size, leaving room for the band of
readings above the track. A saved `presentation.view` is kept until the stage, the text size or
the units change; a top-level `view` is used only when no stage size is known, as in headless
sessions.

## Standalone authored shapes

A shape document has `format: "rigid-bodies.shape"`, `version`, optional `metadata` with `title`
and `summary`, and a required `shape` object:

```json
{
  "format": "rigid-bodies.shape",
  "version": { "major": 1, "minor": 0 },
  "metadata": { "title": "Triangle", "summary": "A reusable outline." },
  "shape": {
    "outline": {
      "closed": true,
      "nodes": [
        { "position_m": [0, 0] },
        { "position_m": [1, 0] },
        { "position_m": [0, 1] }
      ]
    }
  }
}
```

Every node requires `position_m`. Optional `incoming_handle_m` and `outgoing_handle_m` are offsets
relative to that position, not absolute control points. `outgoing_edge` is `line` (the default)
or `cubic`. `continuity` is `corner` (the default), `aligned`, or `mirrored`. The outline must be
closed, nondegenerate, and valid for authoring; self-intersections and exhausted geometry budgets
produce diagnostics before anything is imported.

`shape.options` supports `render_tolerance_m`, `collision_tolerance_m`,
`simplification_tolerance_m`, `concavity_tolerance_m`, `max_render_vertices`,
`max_collision_vertices`, `max_subdivision_depth`, `minimum_area_m2`, and
`duplicate_tolerance_m`. Omitted fields use `ShapeAuthoringOptions` defaults. Shape capture with
its original `ShapeDocument` preserves unknown additive fields, including extensions on source
nodes matched by index. Deleted nodes remain deleted.

A standalone shape contains geometry and compilation settings. It has no body pose, material,
force, or mechanism dependency. The application imports it as a selected dynamic body at the
camera centre with the default oak-wood part properties. Export uses the active valid draft, or
the currently selected authored part when no editor is active.

An imported logical part retains its standalone source through editing, assembly, separation,
undo/redo, Reset, and arrangement save/reopen. Exporting that part or its active draft preserves
compatible extra fields, minor version, and summary. Scenario files carry this provenance in the
part's `source_document`. Exporting a new draft creates a new shape document.

## Application state and file operations

The desktop application adds optional top-level `playback` and `presentation` objects to scenarios.
The former top-level `view` remains accepted for compatibility, but new saves write
`presentation.view`:

| Object | Fields |
| --- | --- |
| `playback` | `fixed_step_s`, `substeps`, `time_scale`, `maximum_substeps_per_frame`, `maximum_frame_time_s` |
| `presentation` | `layers`; `view` with `center_x_m`, `center_y_m`, `height_m` |

A bundled experiment may also carry a top-level `view` with `center_x_m`, `center_y_m`, `height_m`
and an optional `width_m`: the region **Frame subject** fits inside the visible stage.
When a width is provided, it fits both dimensions. Without it the application frames the free objects, their joints' travel,
the points their forces pull towards and where a short look-ahead of the motion takes them. Saves
leave an authored `view` unchanged.

The physics module retains these fields but does not interpret them. The application validates
them before changing the session. Fixed step is between 0.0001 and 0.02 seconds; the per-frame
substep budget is 1–4096, and the substep count must fit within it. Time scale is nonnegative;
maximum frame time is positive. View height must be within the camera's supported range, and
each centre coordinate must be within plus or minus one billion metres so the grid remains
numerically representable.

**Save setup** writes the current starting setup; **Save as…** can instead write the current
moment. The Save details sheet supplies a title and an Include the guide choice. A titled setup
gets a stable slug-derived `metadata.id`, records the originating experiment in
`metadata.based_on`, belongs to the `make_your_own` collection, and is listed under Library › My
setups for the session. A save always writes the current presentation layers and camera view.
Run records, stars, pinned values, comparison choices and other `ViewState` are not written.

Opening a valid arrangement loads its world, saved playback speed and presentation view; it creates
a new starting/original baseline, pauses at t = 0, clears transient measurements and records an
undoable structural edit. The keep-between-experiments preference determines whether the application retains the current
lab settings or uses the saved solver settings. Undo restores the previous experiment,
baseline and in-memory runs; Back to start returns to the opened setup. Importing a shape is also
undoable. An invalid document leaves the previous session intact. Native file dialogs pause the
simulation, including on cancellation, so it resumes only through Play.

Save and export dialogs default to the user's RigidBodies data folder (`%APPDATA%/RigidBodies` on
Windows), with suggested names `arrangement.rbscenario.json` and `shape.rbshape.json`; the viewer
can choose another destination. Writes first complete a temporary file in the destination
folder and replace the requested file only after writing succeeds. A failed write or replacement
keeps the previous destination. Normal startup does not save user documents.

Documents store the physical arrangement: body poses and velocities,
materials and source geometry, world settings and algorithms, force registrations, springs,
joints and their broken states, prescribed-motion phase, and pending loads. Simulation elapsed
time, accumulated step count, contact/warm-start caches, quiet-time counters, collision reports,
render trails, impact particles, and education graph history are rebuilt. Reopening an in-flight
scene restores that arrangement, but its subsequent motion may differ because the solver history
has been rebuilt. In-memory undo snapshots also keep the solver state needed for exact replay.
A relativity experiment's clock readings, lap count and light race are never stored, so a saved
current moment reopens at t = 0 with both clocks at zero and the saved speed.

## Resource limits

| Item | Limit |
| --- | ---: |
| JSON document / application content file | 16 MiB |
| JSON nesting | 64 levels |
| JSON values | 1,000,000 |
| Individual JSON string | 1 MiB |
| Scenario strings consumed by the physics schema | 65,536 bytes |
| Concepts / prerequisites | 256 entries each |
| Bodies | 4,096 |
| Logical body parts across a scenario | 16,384 |
| Springs / joints | 16,384 each |
| Global forces / forces per body | 1,024 each |
| Authored source nodes / maximum render vertices | 4,096 each |
| Maximum authored collision vertices | 512 |
| Maximum curve subdivision depth | 24 |
| Authored coordinate magnitude | 1,000,000,000 metres |
| Shape title / summary | 256 / 4,096 bytes |
| Catalogue | 256 files, 8 MiB per file, 64 MiB total |

Physical constructors and the authoring compiler impose their own validity checks in addition to
these storage bounds. Readers validate a complete result before replacing the current document or world. Limits and
validation errors are reported to the caller and to the content status in the desktop interface.
