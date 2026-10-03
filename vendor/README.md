# Third-party dependencies

Both builds use the dependencies in this directory, so the Visual Studio solution and
`build.ps1` link the same libraries:

- **SDL 3.4.16**: public headers, the x64 import library and the runtime DLL. SDL stays a shared
  runtime component and is copied beside the application, the tests and the installer payload.
  The binaries were built locally from the upstream `release-3.4.16` sources.
- **FreeType 2.13.3**: headers and source, compiled as a static library with its stock
  configuration. The native project is `visualstudio/thirdparty/FreeType.vcxproj`; the CMake build
  reads its file list.
- **RmlUi 6.1 Core**: headers and source, compiled as a static library with its precompiled header.
  The native project is `visualstudio/thirdparty/RmlUi.vcxproj`; the CMake build reads its file
  list.
- **Dear ImGui 1.91.9b**: source for the developer overlay of CMake Debug and RelWithDebInfo builds.

Their upstream license files are kept in their respective directories. Files the builds do not
compile (other FreeType modules, RmlUi's optional plugins) are left as upstream ships them.
