# Scenarios

Each `*.scenario.json` file is a complete starting arrangement using the versioned
`rigid-bodies.scenario` format. The catalogue reads these files at application startup; adding or
editing a document changes the available arrangements without recompiling the application.
Headless tools and tests use the same description-and-populate interface.

The 24 supplied documents include authored outlines, materials, gravity and air settings, body-specific forces,
prescribed motion, springs, and joints. Simulation quantities use SI units. Each document describes a
starting state. Contact caches and rendering effects are rebuilt when the experiment runs.

The `metadata` object contains:

- `id`: a unique identifier of 1–128 lowercase letters, digits, underscores, or hyphens.
- `title` and `summary`: the name and description shown in the Library.
- `concepts`: the nonempty list of concepts this arrangement demonstrates.
- `prerequisites`: scenario identifiers to explore first; an empty list means none.
- `suggested_order`: a nonnegative integer used to order the catalogue. Equal values sort by ID.

All prerequisites must refer to documents in this directory and form an acyclic graph. The
catalogue validates every document before replacing its previous contents, so a malformed file,
duplicate ID, or broken reference cannot leave a partially loaded catalogue. The directory is
limited to 256 documents, 8 MiB per file, and 64 MiB in total; unrelated non-JSON files are ignored.

The envelope declares `format`, `version.major`, and `version.minor`. Compatible minor versions
may add fields, and the immutable source is retained so those fields can be preserved on save.
Unsupported major versions and unknown physics component kinds are rejected with an explanation
before changing the world. See [the document format](../../docs/CONTENT_FORMAT.md) for the complete
schema and compatibility rules.

Setups you create and shapes you export default to the per-user data folder. You can choose a
different location in the file dialog.
