# Document parser fuzzing

The scenario, shape, and JSON parsers share one fuzz harness in
`fuzz/document_fuzz.cpp`. Ordinary CTest runs exercise the tracked corpus and a
fixed sequence of byte mutations. An optional Clang libFuzzer executable explores
coverage-guided mutations with AddressSanitizer and UndefinedBehaviorSanitizer.
Neither path needs the desktop application or downloaded dependencies.

The harness calls the production parsers. Accepted JSON must serialize and parse
without changing its canonical representation. Accepted shape documents must
produce finite tessellation, collision cells, and approximation bounds. Accepted
scenarios must populate a world with finite poses, velocities, mass properties,
and collider bounds, then serialize, reparse, and recreate the same arrangement.
Rejected documents must preserve their output arguments. A rejected scenario
load must preserve an existing world's handles and complete serialized arrangement.
Exceptions, allocation failures and sanitizer errors propagate out of the harness so failures
remain visible.

The corpus includes primitive, mechanism, authored, prescribed-motion, and
special-relativity scenarios; convex, concave, and curved outlines; malformed
Unicode, duplicate keys, numeric overflow, excessive nesting, unsupported
versions, missing body references, and self intersections. The deterministic smoke test also mutates numeric
extremes and feeds raw invalid UTF-8 and embedded NUL bytes. Mutation ordering uses
a fixed integer generator rather than implementation-dependent random distributions.

## Routine smoke tests

Build and run the smoke tests with the development preset:

```sh
cmake --preset development
cmake --build --preset development --target RigidBodies.Physics.document_fuzz.Tests
ctest --preset development -R document_fuzz
```

Each tracked seed runs unchanged and receives 96 repeatable byte mutations.
Additional tests check semantic boundaries and the harness resource limits.
Keep every discovered regression as a minimal corpus input or focused parser
test. The deterministic smoke tests cover a fixed set of cases; use libFuzzer for broader exploration.

## Coverage-guided runs

Use a Clang toolchain with its libFuzzer and sanitizer runtimes installed. For
example, on Linux:

```sh
cmake -S . -B build -G Ninja \
  -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DRIGIDBODIES_BUILD_APP=OFF -DRIGIDBODIES_BUILD_TESTS=OFF \
  -DRIGIDBODIES_WARNINGS_AS_ERRORS=ON \
  -DRIGIDBODIES_ENABLE_SANITIZERS=ON -DRIGIDBODIES_BUILD_FUZZERS=ON
cmake --build build --target RigidBodies.DocumentFuzzer
mkdir -p build/corpus
cp fuzz/corpus/* build/corpus/
build/tools/rigidbodies_document_fuzzer build/corpus \
  -dict=fuzz/documents.dict -max_len=16384 -runs=10000 \
  -timeout=10 -rss_limit_mb=2048 -artifact_prefix=build/
```

Compile both the Math and Physics libraries and the entry-point executable with coverage
instrumentation. The CMake fuzz option applies
`-fsanitize=fuzzer-no-link` to those libraries and links the executable with
`-fsanitize=fuzzer`. The sanitizer option instruments the production library code
as well. Coverage-guided document fuzzing is supported on Linux; the portable CTest mutations
remain available with ordinary MSVC, GCC, and Clang.

The coverage campaign is configured in the Linux quality workflow. The portable CTest mutation
suite remains available on every supported toolchain.

Use a writable corpus under `build`, as libFuzzer adds coverage-increasing inputs
to its first corpus directory. The tracked `fuzz/corpus` remains a small reviewed
set of regression seeds. For a longer local campaign, replace `-runs=10000` with
`-max_total_time=3600`. Replay a failure by passing its artifact path instead of
the corpus directory, with the same executable and sanitizer configuration.
Record the compiler, sanitizer options, invocation, and minimized input alongside
any bug fix. Report a platform as passing only after running and checking its results.

## Resource limits and scope

Each raw input is limited to 16 KiB. All inputs within that limit reach the JSON
parser. Valid JSON still receives canonical roundtrip checks before semantic
budget filtering. Semantic decoding is limited to 1,024 JSON values, 24 entries
per array, four authored outlines, and no tessellation budget above the production
defaults of 512 render vertices, 128 collision vertices, and subdivision depth
16. Inputs beyond those semantic budgets still exercise JSON parsing and writing.
The filter leaves malformed types, negative counts, numeric extremes, and schema
errors for the real document parsers to reject.

These limits keep the fuzz harness fast. The document format has separate production limits. Existing focused parser tests cover production size/depth
limits and rejection behavior. The harness parses documents without stepping the simulation. Physics validation and property
tests cover dynamics, conservation and convergence, keeping solver workloads out of parser fuzzing.
