#!/usr/bin/env python3
"""Check that the static-analysis gate fails closed and excludes dependency sources."""

import contextlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import run_static_analysis as gate


class StaticAnalysisTests(unittest.TestCase):
    def test_database_deduplicates_and_excludes_dependencies(self):
        root = Path(__file__).resolve().parent.parent
        entries = [{"file": name, "directory": str(root)} for name in
                   ("src/example.cpp", "src/example.cpp", "tests/example.cpp",
                    "fuzz/example.cpp", "tools/example.cpp", "build/_deps/library.cpp")]
        self.assertEqual(len(gate.project_sources(entries, root)), 4)

    def test_empty_project_database_is_an_error(self):
        with self.assertRaises(ValueError):
            gate.project_sources([], Path.cwd())

    def run_gate(self, result=None, error=None):
        root = Path(__file__).resolve().parent.parent
        with tempfile.TemporaryDirectory() as directory:
            build = Path(directory)
            output = build / "report.json"
            (build / "compile_commands.json").write_text(json.dumps([
                {"file": str(root / "src" / "example.cpp"), "directory": str(root)}]), encoding="utf-8")
            arguments = ["run_static_analysis.py", "--build", str(build),
                         "--output", str(output), "--jobs", "1"]
            with patch.object(sys, "argv", arguments), patch.object(
                    gate.subprocess, "run", return_value=result, side_effect=error) as launch:
                with contextlib.redirect_stdout(io.StringIO()):
                    status = gate.main()
            command = launch.call_args.args[0]
            self.assertIn("--warnings-as-errors=*", command)
            self.assertEqual(launch.call_args.kwargs["timeout"], 300)
            return status, json.loads(output.read_text(encoding="utf-8"))

    def test_success_report(self):
        status, report = self.run_gate(subprocess.CompletedProcess([], 0, ""))
        self.assertEqual((status, report["translation_units"], report["failed"]), (0, 1, 0))

    def test_diagnostic_failure_is_preserved(self):
        status, report = self.run_gate(subprocess.CompletedProcess([], 1, "unsafe access"))
        self.assertEqual((status, report["failed"]), (1, 1))
        self.assertEqual(report["results"][0]["diagnostics"], "unsafe access")

    def test_missing_tool_is_failure(self):
        status, report = self.run_gate(error=FileNotFoundError("clang-tidy missing"))
        self.assertEqual((status, report["failed"]), (1, 1))

    def test_zero_exit_configuration_error_is_failure(self):
        status, report = self.run_gate(subprocess.CompletedProcess(
            [], 0, ".clang-tidy:18:1: error: unknown key 'SystemHeaders'\nError parsing .clang-tidy: Invalid argument"))
        self.assertEqual((status, report["failed"]), (1, 1))

    def test_zero_exit_processing_error_is_failure(self):
        status, report = self.run_gate(subprocess.CompletedProcess([], 0, "Error while processing example.cpp."))
        self.assertEqual((status, report["failed"]), (1, 1))

    def test_timeout_is_failure(self):
        status, report = self.run_gate(error=subprocess.TimeoutExpired("clang-tidy", 300))
        self.assertEqual((status, report["failed"]), (1, 1))


if __name__ == "__main__":
    unittest.main()
