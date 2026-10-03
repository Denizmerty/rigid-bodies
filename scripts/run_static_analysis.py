#!/usr/bin/env python3
"""Run the repository's clang-tidy policy on every compiled project source, without Git."""

import argparse
import concurrent.futures
import json
from pathlib import Path
import re
import subprocess
import sys


def diagnostic_failure(output):
    # Some clang-tidy versions exit zero after rejecting YAML and silently use default checks.
    # Treat these diagnostics as failures even when the process exits successfully.
    return re.search(r"(?im)^.*\berror:\s|^error parsing\b|^error while processing\b", output) is not None


def project_sources(database, root):
    sources = set()
    for entry in database:
        source = Path(entry["file"])
        if not source.is_absolute():
            source = Path(entry["directory"]) / source
        source = source.resolve()
        try:
            relative = source.relative_to(root)
        except ValueError:
            continue
        if relative.parts[0] in {"src", "tests", "tools", "fuzz"}:
            sources.add(source)
    if not sources:
        raise ValueError("Compilation database contains no project translation units")
    return sorted(sources)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", type=Path, required=True)
    parser.add_argument("--clang-tidy", default="clang-tidy")
    parser.add_argument("--jobs", type=int, default=2)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=int, default=300)
    args = parser.parse_args()
    if not 1 <= args.jobs <= 32 or args.timeout <= 0:
        parser.error("jobs must be 1..32 and timeout must be positive")
    root = Path(__file__).resolve().parent.parent
    build = args.build.resolve()
    sources = project_sources(json.loads((build / "compile_commands.json").read_text(encoding="utf-8")), root)

    def inspect(source):
        command = [args.clang_tidy, "-p", str(build), "--warnings-as-errors=*", str(source)]
        if sys.platform == "win32":
            # Keep the standalone gate aligned with CMake's MSVC policy. Clang-tidy does not
            # infer exception support from /EHsc, and its opt-in enum checker diagnoses the
            # standard library's internal filesystem bitmask rather than project code.
            command.extend([
                "--extra-arg=/EHsc",
                "--checks=-clang-analyzer-optin.core.EnumCastOutOfRange",
            ])
        try:
            result = subprocess.run(command, cwd=root, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, text=True, encoding="utf-8",
                                    errors="replace", timeout=args.timeout, check=False)
            status = result.returncode or (1 if diagnostic_failure(result.stdout) else 0)
            return {"file": source.relative_to(root).as_posix(), "exit_code": status,
                    "diagnostics": result.stdout}
        except (OSError, subprocess.TimeoutExpired) as error:
            return {"file": source.relative_to(root).as_posix(), "exit_code": 1,
                    "diagnostics": str(error)}

    reports = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
        futures = [pool.submit(inspect, source) for source in sources]
        for future in concurrent.futures.as_completed(futures):
            report = future.result()
            reports.append(report)
            status = "PASS" if report["exit_code"] == 0 else "FAIL"
            print(f"[{len(reports)}/{len(sources)}] {status} {report['file']}", flush=True)
            if report["exit_code"] != 0:
                print(report["diagnostics"], flush=True)
    reports.sort(key=lambda report: report["file"])
    failed = sum(report["exit_code"] != 0 for report in reports)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({"translation_units": len(reports), "failed": failed,
                                      "results": reports}, indent=2) + "\n", encoding="utf-8")
    print(f"clang-tidy: {len(reports) - failed}/{len(reports)} project translation units passed")
    return 1 if failed else 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, TypeError) as error:
        print(f"Static analysis configuration error: {error}", file=sys.stderr)
        sys.exit(1)
