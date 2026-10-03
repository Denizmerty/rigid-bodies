# Performance measurements

`rigid_bodies_benchmark` runs the simulation without a window. It uses the same scenario catalogue
and physics library as the application. Each repetition constructs the same initial world, advances
a fixed number of untimed warmup steps, and records a fixed number of simulation steps. World
construction, document parsing, console output, and report writing are outside the timed interval.
The generator uses specified 32-bit integer arithmetic and a recorded seed. Identical repetitions
must finish with identical body-state digests; a mismatch fails the measurement.

Build Release before comparing performance with the shipped application. Stop other builds,
benchmarks and heavy applications; use the same power mode and allow the machine to cool between
long runs. Record the machine and workload with each timing result so comparisons have a clear basis.

```powershell
cmake --preset development
cmake --build --preset development --target RigidBodies.Benchmark
.\scripts\benchmark.ps1 -Label 'local-change' -Workers 1 -DisableProfiling
.\scripts\benchmark.ps1 -Label 'local-change-parallel' -Workers 0
```

The benchmark script writes a new timestamped, unique JSON file in `build/benchmark-results`. It then finds
the newest compatible previous recording and compares it. Nothing is overwritten or removed.
A comparison-status sidecar prevents a confirmed regression from becoming the next automatic
baseline; a repeat run continues to compare with the earlier accepted recording.
The script works from both the source checkout and the portable distribution: it first looks for
`rigid_bodies_benchmark.exe` beside the package's `scripts` directory, then falls back to the source
tree's `build/tools` executable, which the development preset and the Visual Studio solution's
**Test** configuration both build. `-Executable PATH` overrides that choice, and `-HistoryDirectory
PATH` chooses a writable recording directory when the installation folder is read-only.
The executable can also run and compare directly, including on systems without PowerShell:

```text
rigid_bodies_benchmark --list
rigid_bodies_benchmark --workers 1 --no-profiling --output serial.json
rigid_bodies_benchmark --workers 0 --output parallel.json
rigid_bodies_benchmark --workload dense_256 --samples 240 --repetitions 5 --output dense.json
rigid_bodies_benchmark --compare current.json --baseline previous.json
```

Use `--scenarios PATH` if the assets are neither staged beside the executable nor available at the
configured source path. The packaged executable automatically uses its neighboring `assets/scenarios`
directory. `--cpu NAME` supplies the CPU identity on platforms that do not expose the Windows
`PROCESSOR_IDENTIFIER` environment variable; give each physical machine a distinct `--machine NAME`
if its hostname is absent or shared. `--label TEXT` identifies a build or experiment.

## Workloads

| ID | Initial arrangement | Purpose |
| --- | --- | --- |
| `scene_stack` | `stable_stack` catalogue document | Small representative resting contact scene |
| `scene_aerodynamics` | `asymmetric_aerodynamics` catalogue document | Distributed drag and torque evaluation |
| `scene_authored` | `shape_workshop` catalogue document | Decomposed authored collision shapes |
| `sparse_128` | 128 circles and boxes, spaced 1.2 m apart | Small sparse broad phase and traversal |
| `sparse_512` | 512 circles and boxes, spaced 1.2 m apart | Larger sparse broad phase and traversal |
| `dense_256` | 256 touching circles and boxes | Pair generation, narrow phase, and contact solving |
| `stacks_128` | Sixteen stacks of eight boxes on one floor | Independent contact groups and shared support |
| `mechanisms_96` | Twenty-four chains, each four dynamic links and one anchor | Joint graphs and independent mechanisms |
| `sleeping_512` | 128 stationary groups of four boxes on one floor | Sleeping groups; zero gravity, seeded contacts |

Generated scale scenes turn off continuous collision detection to measure ordinary broad/narrow
phase work without unrelated swept-motion costs. Their motion is slow and their timestep is small.
Catalogue scenes retain their authored settings. The sleeping fixture performs one untimed cache
initialization step before sleeping its bodies, so measurements exclude initial contact wake-up.
Scene documents are fingerprinted from their complete normalized source. Generated arrangements
have a suite workload revision that must change if their construction changes.

Defaults are 30 warmup steps, 120 measured steps, three fresh repetitions, timestep 1/120 s,
seed 17329, and one worker. A worker setting of zero selects bounded automatic concurrency;
one is the serial reference. The maximum explicit count is 32. Keep separate serial and parallel
histories so regression comparisons use the same worker settings.
Profiling is on by default and records phase median/p95/min/max alongside externally timed whole
steps. `--no-profiling` measures throughput without phase timers and is a separate comparison key.
Reference captures use serial execution without profiling.

## Recorded format and comparison

Reports use the `rigid-bodies-benchmark` version 1 JSON format and suite revision 1. They record:

- Machine, CPU, architecture, hardware thread count, platform, compiler, build, project version,
  recording time, and a free label.
- Warmup/sample/repetition counts, timestep, generator seed, worker setting, and profiling setting.
- Workload/source fingerprint; per-repetition step and optional phase timings in milliseconds;
  body, pair, contact, constraint, island, and sleeping counts; actual broad/narrow worker counts
  when profiling is enabled; final body-state digest. Zero runtime evidence means unavailable in
  the reference; its whole-step timing remains comparable with unprofiled serial runs.

Timing percentiles use the nearest-rank definition. Repetitions remain separate in the report;
combining them into one large pool would hide run-to-run variation. The comparison checks both
median step time and p95 step time. A metric is a confirmed regression only when **every candidate
repetition** exceeds the **slowest baseline repetition** by more than the largest of:

- 20% of the baseline median across repetitions;
- an absolute floor of 0.05 ms;
- three times the baseline median absolute deviation across repetitions.

Both baseline and candidate must have at least three repetitions. A metric whose repetition range
exceeds the larger of the absolute floor and 25% of its median is reported as noisy and skipped.
A single slow run is inconclusive under these rules. When a comparison is skipped, increase the
repetition count and rerun with other activity stopped before drawing a conclusion. `--relative-threshold`, `--absolute-threshold-ms`,
`--noise-multiplier`, and `--maximum-relative-spread` adjust the policy explicitly.

Machine/build identity, sampling settings, and workload fingerprints must match. Dates and labels
are descriptive and need not match. Missing optional phase timings do not prevent whole-step
comparison. An incompatible baseline is skipped, and a separate history starts for the new configuration. Exit code 0 means the measurement completed with no
confirmed regression (possibly with explicit skips), 1 means invalid input or measurement failure,
and 2 means at least one confirmed regression. Review the reported compared-workload count along
with the exit code. Timing thresholds are never part of CTest: functional tests use synthetic timing
reports to check comparison, compatibility, invalid input, and noise handling.

## Profiling and interpretation

The developer overlay reports simulation phase timings and collision worker utilization from the
most recent substep. It is a diagnostic view, while benchmark files provide repeatable recordings.
Phase clocks are opt-in at the world level. They measure elapsed wall time, including synchronization,
so concurrent tasks are not summed as if they ran serially. Whole-step timing includes phase timer
and bookkeeping costs. The phase totals can differ from the outer measurement.

Compare serial before/after results to assess storage, traversal, and island costs. Then compare
separate parallel measurements to assess scheduling on a particular workload. Small scenes can be
faster serially; worker startup is warmed up, and the execution policy retains serial work below its
batch threshold. Sleeping results should be read together with their sleeping-body counts, and
contact results with their pair/manifold work: different physical evolution can change the amount
of work even when source arrangement and timestep are identical.

## Run-recording memory budget

The session run recorder samples at 40 Hz and keeps a rolling 60-second window. Every stored value
is a four-byte float. At the maximum recorded scene size, one sample contains 457 floats: nine
scene values and fourteen quantities for each of 32 free objects. A full run therefore uses
`457 × 4 × 2,400 = 4,387,200` bytes.

At most ten runs are retained per experiment. All experiments and the current recording share a
48 MiB (50,331,648-byte) live budget. When a run is kept, retention first enforces the per-experiment
limit, then removes the globally oldest unstarred records until the kept series leave one full-run
buffer free. No more than eight records may be starred, and starred records are never removed
automatically. Buffers reserve their maximum sample capacity when recording begins, so sampling
does not allocate per frame. Run data remains session-only and is never persisted.

`RigidBodies.RunRecording.Tests` asserts the exact full-run byte count, records ten attempted full
runs in each of two experiments, checks every intermediate budget, and verifies the expected
cross-experiment eviction order. It also exercises the ten-run and eight-star limits, the rolling
window, deterministic series, and that recording writes no files.
