# Determinism reference v1

`reference-v1.json` contains all 220 physical step records from the revision-1 workload in
`tests/support/determinism_workload.hpp`, generated on Windows x64 with MSVC 1951 after the probe's
`--verify` repetition, one/two/four-worker comparisons, and five checkpoint replay checks passed.
Machine metadata is excluded from physical comparisons; the trace itself is reviewed and versioned.

Compare another build without regenerating the reference:

```text
rigid_bodies_determinism_probe --output candidate.json --workers 4 --verify
python scripts/compare_determinism.py tests/fixtures/determinism/reference-v1.json candidate.json
```

The first two workloads require exact portable numeric identity. The remaining three allow only
the fixed SI-value tolerances documented in `docs/DETERMINISM.md`; identities, ordering, topology,
features and events always remain exact. Storing every step lets the comparison catch differences that disappear before the next checkpoint. See that document for reference-update requirements.
