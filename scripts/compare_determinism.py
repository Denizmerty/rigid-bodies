#!/usr/bin/env python3
"""Compare complete versioned physics traces, with exact and bounded numeric contracts.

The fixed tolerance applies only beneath a record's `values` field in a `bounded`
scenario. Keys, array order, identities, integer counters and events must match
exactly. Tolerances cannot be overridden on the command line.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

ABSOLUTE_TOLERANCE = 2.0e-9
RELATIVE_TOLERANCE = 2.0e-8
SCENARIOS = (
    ("dyadic_elastic_impacts", "exact", 1.0 / 128.0, 72),
    ("dyadic_storage_replay", "exact", 1.0 / 128.0, 32),
    ("contacts_and_sensors", "bounded", 1.0 / 120.0, 56),
    ("joints_and_forces", "bounded", 1.0 / 120.0, 48),
    ("continuous_collision", "bounded", 1.0 / 60.0, 12),
)
TOPOLOGY_KEYS = {
    "bodies", "pairs", "contacts", "pair_events", "contact_events", "joints",
    "islands", "ccd_clamped", "sweep_limits", "limit_events",
}
VALUE_KEYS = {"bodies", "contacts", "joints", "elapsed_s", "energy_j", "momentum_kg_m_s"}


class TraceError(ValueError):
    """Invalid or nonconforming determinism trace."""


def _reject_constant(value: str) -> None:
    raise TraceError(f"nonfinite JSON number {value}")


def _object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise TraceError(f"duplicate JSON object key {key!r}")
        result[key] = value
    return result


def _finite(value: Any, path: str) -> None:
    if isinstance(value, float) and not math.isfinite(value):
        raise TraceError(f"{path}: nonfinite number")
    if isinstance(value, dict):
        for key, item in value.items():
            _finite(item, f"{path}.{key}")
    elif isinstance(value, list):
        for index, item in enumerate(value):
            _finite(item, f"{path}[{index}]")


def _keys(value: Any, expected: set[str], path: str) -> None:
    if not isinstance(value, dict) or set(value) != expected:
        raise TraceError(f"{path}: object fields must be {sorted(expected)}")


def _numeric_tree(value: Any, path: str) -> None:
    if isinstance(value, dict):
        for key, item in value.items():
            _numeric_tree(item, f"{path}.{key}")
    elif isinstance(value, list):
        for index, item in enumerate(value):
            _numeric_tree(item, f"{path}[{index}]")
    elif type(value) not in (int, float):
        raise TraceError(f"{path}: values must contain only finite numbers")


def validate(trace: Any) -> None:
    _finite(trace, "trace")
    if not isinstance(trace, dict):
        raise TraceError("trace must be an object")
    expected = {"format", "schema_version", "workload_revision", "seed", "scenarios"}
    _keys(trace, expected | ({"metadata"} if "metadata" in trace else set()), "trace")
    if trace["format"] != "rigid-bodies-determinism" or type(trace["schema_version"]) is not int or trace["schema_version"] != 1:
        raise TraceError("unsupported trace format or schema version")
    if type(trace["workload_revision"]) is not int or trace["workload_revision"] != 1 or trace["seed"] != "fixed-dyadic-and-mechanics-v1":
        raise TraceError("unsupported workload revision or seed")
    if "metadata" in trace and not isinstance(trace["metadata"], dict):
        raise TraceError("metadata must be an object")
    scenarios = trace["scenarios"]
    if not isinstance(scenarios, list) or len(scenarios) != len(SCENARIOS):
        raise TraceError("all five scenarios must be present in canonical order")
    for index, (name, contract, dt, steps) in enumerate(SCENARIOS):
        scenario = scenarios[index]
        path = f"scenarios[{index}]"
        _keys(scenario, {"id", "numeric_contract", "dt_s", "steps", "records"}, path)
        if scenario["id"] != name or scenario["numeric_contract"] != contract:
            raise TraceError(f"{path}: expected workload {name!r} with {contract!r} numeric contract")
        if type(scenario["dt_s"]) not in (int, float) or scenario["dt_s"] != dt or type(scenario["steps"]) is not int or scenario["steps"] != steps:
            raise TraceError(f"{path}: workload time step or length changed")
        records = scenario["records"]
        if not isinstance(records, list) or len(records) != steps:
            raise TraceError(f"{path}: every one of {steps} records is required")
        for frame, record in enumerate(records):
            frame_path = f"{path}.records[{frame}]"
            _keys(record, {"step", "topology", "values"}, frame_path)
            if type(record["step"]) is not int or record["step"] != frame + 1:
                raise TraceError(f"{frame_path}: noncanonical step number")
            _keys(record["topology"], TOPOLOGY_KEYS, f"{frame_path}.topology")
            _keys(record["values"], VALUE_KEYS, f"{frame_path}.values")
            for field in ("bodies", "contacts", "joints"):
                identities, numbers = record["topology"][field], record["values"][field]
                if not isinstance(identities, list) or not isinstance(numbers, list) or len(identities) != len(numbers):
                    raise TraceError(f"{frame_path}: {field} identity/value count mismatch")
            if not record["topology"]["bodies"]:
                raise TraceError(f"{frame_path}: an empty workload cannot certify determinism")
            _numeric_tree(record["values"], f"{frame_path}.values")


def load(path: Path) -> dict[str, Any]:
    try:
        result = json.loads(path.read_text(encoding="utf-8"), parse_constant=_reject_constant, object_pairs_hook=_object)
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise TraceError(f"{path}: {error}") from error
    validate(result)
    return result


def _compare(expected: Any, actual: Any, path: str, *, numeric: bool = False, bounded: bool = False) -> None:
    if isinstance(expected, dict):
        if not isinstance(actual, dict) or set(expected) != set(actual):
            raise TraceError(f"{path}: missing or added fields")
        for key in expected:
            _compare(expected[key], actual[key], f"{path}.{key}", numeric=numeric, bounded=bounded)
    elif isinstance(expected, list):
        if not isinstance(actual, list) or len(expected) != len(actual):
            raise TraceError(f"{path}: missing or added array records")
        for index, (left, right) in enumerate(zip(expected, actual)):
            _compare(left, right, f"{path}[{index}]", numeric=numeric, bounded=bounded)
    elif numeric and type(expected) in (int, float) and type(actual) in (int, float):
        difference = abs(expected - actual)
        tolerance = ABSOLUTE_TOLERANCE + RELATIVE_TOLERANCE * max(abs(expected), abs(actual)) if bounded else 0.0
        if difference > tolerance:
            raise TraceError(f"{path}: {actual!r} differs from {expected!r} by {difference:.9g}, allowed {tolerance:.9g}")
    elif type(expected) is not type(actual) or expected != actual:
        raise TraceError(f"{path}: exact field changed: {expected!r} -> {actual!r}")


def compare(reference: dict[str, Any], candidate: dict[str, Any]) -> None:
    validate(reference)
    validate(candidate)
    for index, (expected, actual) in enumerate(zip(reference["scenarios"], candidate["scenarios"])):
        for frame, (left, right) in enumerate(zip(expected["records"], actual["records"])):
            path = f"{expected['id']}.records[{frame}]"
            _compare(left["topology"], right["topology"], f"{path}.topology")
            _compare(left["values"], right["values"], f"{path}.values", numeric=True, bounded=SCENARIOS[index][1] == "bounded")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference", type=Path)
    parser.add_argument("candidates", type=Path, nargs="+")
    arguments = parser.parse_args(argv)
    try:
        reference = load(arguments.reference)
        for path in arguments.candidates:
            compare(reference, load(path))
            print(f"PASS {path}: exact portable values; exact topology/events; general abs={ABSOLUTE_TOLERANCE:g}, rel={RELATIVE_TOLERANCE:g}")
    except TraceError as error:
        print(f"FAIL {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
