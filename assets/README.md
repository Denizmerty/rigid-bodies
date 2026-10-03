# Assets

This directory contains the files the application loads at runtime. Both builds and the installer
copy it beside the executable, using the same relative paths.

| Directory | Holds |
| --- | --- |
| `ui/` | Documents and stylesheets for the document interface backend. |
| `fonts/` | Typefaces for the document interface backend. |
| `scenarios/` | Experiment definitions loaded from JSON files. |
| `licenses/` | Notices for the platform, document, font-engine, and development dependencies. |
| `branding/` | The application icon, built from `packaging/icon` (see docs/DISTRIBUTION.md). |

The default RmlUi interface loads its document, stylesheets, Inter text faces and Phosphor icon
subset from here.
The optional overlay fallback draws with the built-in font. Dependency notices are staged with
both build-tree and packaged runs; the Inter font license is in `fonts/`.

The application reads these assets without modifying them. Settings and saved scenarios default
to the per-user data directory, which `ResourcePaths` locates on each platform.
