#!/usr/bin/env python3
"""Run MileMate MATLAB regression suites in isolated MATLAB processes."""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from datetime import datetime
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
TWIN_ROOT = REPOSITORY_ROOT / "matlab" / "digital_twin"
SUITE_FUNCTIONS = {
    "manual": "run_parcel_manual_regression_suite",
    "e2e": "run_parcel_final_e2e_suite",
}


def matlab_string(value: str | Path) -> str:
    """Return a MATLAB single-quoted string literal."""
    return "'" + str(value).replace("'", "''") + "'"


def build_batch_expression(twin_root: Path, function_name: str, profile: str) -> str:
    return (
        f"cd({matlab_string(twin_root.as_posix())}); "
        "fprintf('MILEMATE MATLAB %s\\n', version); "
        f"{function_name}({matlab_string(profile)});"
    )


def selected_suites(selection: str) -> list[str]:
    return ["manual", "e2e"] if selection == "all" else [selection]


def executable_exists(command: str) -> bool:
    return bool(shutil.which(command) or Path(command).is_file())


def run_and_tee(command: list[str], env: dict[str, str], log_path: Path) -> int:
    with log_path.open("w", encoding="utf-8", newline="") as log:
        process = subprocess.Popen(
            command,
            cwd=REPOSITORY_ROOT,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
        )
        assert process.stdout is not None
        for line in process.stdout:
            print(line, end="")
            log.write(line)
        return process.wait()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=("quick", "standard"), default="quick")
    parser.add_argument("--suite", choices=("manual", "e2e", "all"), default="all")
    parser.add_argument("--matlab", default=os.environ.get("MATLAB_BIN", "matlab"))
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = (args.output_dir or REPOSITORY_ROOT / "matlab" / "tests" / "runs" / stamp).resolve()
    suites = selected_suites(args.suite)

    commands: list[tuple[str, list[str]]] = []
    for suite in suites:
        expression = build_batch_expression(TWIN_ROOT.resolve(), SUITE_FUNCTIONS[suite], args.profile)
        commands.append((suite, [args.matlab, "-batch", expression]))

    print(f"profile={args.profile} output={output_dir}")
    for suite, command in commands:
        print(f"[{suite}] {subprocess.list2cmdline(command)}")
    if args.dry_run:
        return 0

    if not executable_exists(args.matlab):
        print(
            f"[FAIL] MATLAB executable not found: {args.matlab}. "
            "Install MATLAB or pass --matlab /absolute/path/to/matlab.",
            file=sys.stderr,
        )
        return 2

    output_dir.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["REFUGE_TEST_OUTPUT_DIR"] = str(output_dir)
    failed: list[str] = []
    for suite, command in commands:
        print(f"\n=== MATLAB {suite} regression ({args.profile}) ===")
        return_code = run_and_tee(command, env, output_dir / f"matlab-{suite}.log")
        if return_code == 0:
            print(f"[PASS] MATLAB {suite} regression")
        else:
            failed.append(suite)
            print(f"[FAIL] MATLAB {suite} regression exit={return_code}", file=sys.stderr)

    if failed:
        print(f"\nSummary: failed suites={','.join(failed)}; outputs={output_dir}", file=sys.stderr)
        return 1
    print(f"\nSummary: all MATLAB suites passed; outputs={output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
