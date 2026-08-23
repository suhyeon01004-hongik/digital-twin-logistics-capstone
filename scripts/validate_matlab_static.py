#!/usr/bin/env python3
"""Perform static integrity checks for MATLAB/Simulink assets without MATLAB."""

from __future__ import annotations

import csv
import re
import sys
from pathlib import Path
from zipfile import BadZipFile, ZipFile


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
TWIN_ROOT = REPOSITORY_ROOT / "matlab" / "digital_twin"
MODEL_NAMES = ("milemate_digital_twin.slx", "baseline_auto_parcel_loading_process.slx")
MODEL_UUID_PATTERN = re.compile(r"<P Name=\"ModelUUID\">([^<]+)</P>")
BASELINE_ROOT = REPOSITORY_ROOT / "matlab" / "tests" / "baseline"
BASELINE_STEM = "parcel_regression_results_20260624_200629"


def inspect_historical_baseline(failures: list[str]) -> None:
    """Check archived result consistency without treating it as a current run."""
    summary_path = BASELINE_ROOT / "parcel_regression_summary_20260624_200629.txt"
    csv_path = BASELINE_ROOT / f"{BASELINE_STEM}.csv"
    harness_path = TWIN_ROOT / "run_parcel_manual_regression_suite.m"
    config_path = TWIN_ROOT / "parcel_manual_config.m"

    missing = [path for path in (summary_path, csv_path, harness_path, config_path) if not path.is_file()]
    if missing:
        failures.extend(f"missing MATLAB baseline input: {path}" for path in missing)
        return

    summary = summary_path.read_text(encoding="utf-8-sig", errors="ignore")
    summary_match = re.search(r"^failures\s+(\d+)\s*$", summary, re.MULTILINE)
    if not summary_match:
        failures.append(f"failure count missing from historical summary: {summary_path}")
        return
    summary_failures = int(summary_match.group(1))

    with csv_path.open(encoding="utf-8-sig", newline="") as handle:
        rows = list(csv.DictReader(handle))
    required = {
        "phase", "targetId", "steps", "success", "collision", "rotation",
        "refuge", "reinsert", "waitTotal", "message",
    }
    if not rows or not required.issubset(rows[0]):
        failures.append(f"historical baseline CSV has an incomplete schema: {csv_path}")
        return

    failed_rows = [row for row in rows if int(row["success"]) == 0]
    if len(failed_rows) != summary_failures:
        failures.append(
            "historical baseline mismatch: "
            f"summary failures={summary_failures}, CSV failures={len(failed_rows)}"
        )
        return

    harness = harness_path.read_text(encoding="utf-8-sig", errors="ignore")
    limit_match = re.search(r"'maxCirculationSteps',\s*(\d+)", harness)
    step_limit = int(limit_match.group(1)) if limit_match else None
    config = config_path.read_text(encoding="utf-8-sig", errors="ignore")
    sample_match = re.search(r"cfg\.sampleTimeSec\s*=\s*([0-9.]+)", config)
    sample_time = float(sample_match.group(1)) if sample_match else None

    print(
        "[INFO] historical MATLAB result is internally consistent: "
        f"rows={len(rows)}, recorded failures={len(failed_rows)}"
    )
    for row in failed_rows:
        steps = int(row["steps"])
        is_timeout = step_limit is not None and steps >= step_limit
        simulated_seconds = steps * sample_time if sample_time is not None else None
        classification = "step-limit timeout/stall" if is_timeout else "unspecified failure"
        duration = f", simulated_time={simulated_seconds:.1f}s" if simulated_seconds is not None else ""
        print(
            "[INFO] historical failed case (not a current run): "
            f"phase={row['phase']}, target=P{row['targetId']}, steps={steps}{duration}, "
            f"collision={row['collision']}, rotation={row['rotation']}, "
            f"refuge={row['refuge']}, reinsert={row['reinsert']}, wait={row['waitTotal']}, "
            f"classification={classification}, message={row['message']}"
        )


def main() -> int:
    failures: list[str] = []
    uuids: dict[str, str] = {}

    for name in MODEL_NAMES:
        path = TWIN_ROOT / name
        if not path.is_file():
            failures.append(f"missing Simulink model: {path}")
            continue
        try:
            with ZipFile(path) as archive:
                bad_member = archive.testzip()
                if bad_member:
                    failures.append(f"corrupt Simulink member: {name}:{bad_member}")
                    continue
                blockdiagram = archive.read("simulink/blockdiagram.xml").decode("utf-8", errors="replace")
        except (BadZipFile, KeyError) as exc:
            failures.append(f"invalid Simulink package {name}: {exc}")
            continue
        match = MODEL_UUID_PATTERN.search(blockdiagram)
        if not match:
            failures.append(f"ModelUUID missing: {name}")
            continue
        uuids[name] = match.group(1)
        print(f"[PASS] Simulink package integrity: {name} ({path.stat().st_size} bytes)")

    matlab_files = sorted(TWIN_ROOT.rglob("*.m"))
    empty = [str(path.relative_to(REPOSITORY_ROOT)) for path in matlab_files if not path.read_text(encoding="utf-8-sig", errors="ignore").strip()]
    if empty:
        failures.append(f"empty MATLAB files: {empty}")
    else:
        print(f"[PASS] MATLAB source files are non-empty: {len(matlab_files)} files")

    if len(set(uuids.values())) == 1 and len(uuids) == len(MODEL_NAMES):
        print(f"[INFO] Simulink models share one lineage UUID: {next(iter(uuids.values()))}")

    inspect_historical_baseline(failures)

    for failure in failures:
        print(f"[FAIL] {failure}", file=sys.stderr)
    print(f"\nSummary: {len(failures)} failed")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
