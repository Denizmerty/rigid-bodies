#!/usr/bin/env python3
"""Adversarial tests for the trace comparison gate (no external packages)."""

import copy
import json
import math
from pathlib import Path
import tempfile
import unittest

import compare_determinism as gate


def sample():
    scenarios = []
    for name, contract, dt, steps in gate.SCENARIOS:
        records = []
        for step in range(1, steps + 1):
            topology = {key: [] for key in gate.TOPOLOGY_KEYS}
            topology.update(bodies=[{"id": "body:0:1", "awake": True, "type": 2, "colliders": 1}], ccd_clamped=0, sweep_limits=0, limit_events=0)
            values = dict(bodies=[{"position_m": [0.25, 0.0]}], contacts=[], joints=[], elapsed_s=step * dt, energy_j=1.0, momentum_kg_m_s=[1.0, 0.0])
            records.append(dict(step=step, topology=topology, values=values))
        scenarios.append(dict(id=name, numeric_contract=contract, dt_s=dt, steps=steps, records=records))
    return dict(format="rigid-bodies-determinism", schema_version=1, workload_revision=1, seed="fixed-dyadic-and-mechanics-v1", scenarios=scenarios)


class ComparisonTests(unittest.TestCase):
    def setUp(self):
        self.reference = sample()
        self.candidate = copy.deepcopy(self.reference)

    def frame(self, index=2):
        return self.candidate["scenarios"][index]["records"][0]

    def rejected(self):
        with self.assertRaises(gate.TraceError):
            gate.compare(self.reference, self.candidate)

    def test_identical_complete_trace(self):
        gate.compare(self.reference, self.candidate)

    def test_machine_metadata_is_not_simulation_state(self):
        self.candidate["metadata"] = {"platform": "another OS", "requested_workers": 4}
        gate.compare(self.reference, self.candidate)

    def test_small_general_rounding_is_accepted(self):
        self.frame()["values"]["energy_j"] += 1.0e-9
        gate.compare(self.reference, self.candidate)

    def test_general_change_outside_tolerance_is_rejected(self):
        self.frame()["values"]["energy_j"] += 1.0e-5
        self.rejected()

    def test_absolute_tolerance_near_zero_is_bounded(self):
        self.frame()["values"]["bodies"][0]["position_m"][1] = 3.0e-9
        self.rejected()

    def test_relative_tolerance_scales_large_values(self):
        self.reference["scenarios"][2]["records"][0]["values"]["energy_j"] = 1.0e6
        self.frame()["values"]["energy_j"] = 1.0e6 + 0.01
        gate.compare(self.reference, self.candidate)
        self.frame()["values"]["energy_j"] = 1.0e6 + 0.03
        self.rejected()

    def test_exact_subset_rejects_even_one_ulp(self):
        self.frame(0)["values"]["energy_j"] = math.nextafter(1.0, 2.0)
        self.rejected()

    def test_contract_cannot_be_downgraded(self):
        self.candidate["scenarios"][0]["numeric_contract"] = "bounded"
        self.rejected()

    def test_body_generation_is_exact(self):
        self.frame()["topology"]["bodies"][0]["id"] = "body:0:2"
        self.rejected()

    def test_numeric_topology_counters_are_exact(self):
        self.frame()["topology"]["ccd_clamped"] = 1.0e-12
        self.rejected()

    def test_boolean_is_not_integer_state(self):
        self.frame()["topology"]["bodies"][0]["awake"] = 1
        self.rejected()

    def test_event_cannot_be_added(self):
        self.frame()["topology"]["contact_events"].append({"kind": "begin", "pair": ["body:0:1", 0, "body:1:1", 0], "sensor": False})
        self.rejected()

    def test_missing_or_reordered_record_is_rejected(self):
        records = self.candidate["scenarios"][0]["records"]
        records[0], records[1] = records[1], records[0]
        self.rejected()
        del records[0]
        self.rejected()

    def test_missing_scenario_is_rejected(self):
        self.candidate["scenarios"].pop()
        self.rejected()

    def test_added_or_missing_numeric_field_is_rejected(self):
        self.frame()["values"]["bodies"][0]["hidden"] = 0.0
        self.rejected()

    def test_missing_body_values_are_rejected(self):
        self.frame()["values"]["bodies"].clear()
        self.rejected()

    def test_nonfinite_values_and_metadata_are_rejected(self):
        for value in (math.nan, math.inf, -math.inf):
            self.candidate = copy.deepcopy(self.reference)
            self.frame()["values"]["energy_j"] = value
            self.rejected()
        self.candidate = copy.deepcopy(self.reference)
        self.candidate["metadata"] = {"bad": math.inf}
        self.rejected()

    def test_numeric_string_is_rejected(self):
        self.frame()["values"]["energy_j"] = "1.0"
        self.rejected()

    def test_empty_matching_documents_do_not_pass(self):
        with self.assertRaises(gate.TraceError):
            gate.compare({}, {})

    def test_loader_rejects_duplicate_keys_constants_and_overflow(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.json"
            for text in ('{"schema_version":1,"schema_version":1}', '{"value":NaN}', '{"value":Infinity}', '{"value":1e999}'):
                path.write_text(text, encoding="utf-8")
                with self.assertRaises(gate.TraceError):
                    gate.load(path)

    def test_loader_roundtrip(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trace.json"
            path.write_text(json.dumps(self.reference), encoding="utf-8")
            gate.compare(self.reference, gate.load(path))


if __name__ == "__main__":
    unittest.main()
