#!/usr/bin/env python3
"""Validate the committed portfolio-evidence bundle and its manifest."""

from __future__ import annotations

import csv
import hashlib
import json
import sys
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
EVIDENCE_ROOT = REPOSITORY_ROOT / "docs" / "evidence"
EXPECTED_ROWS = {
    "perception/obb-training-results.csv": 107,
    "perception/segmentation-training-results.csv": 30,
    "perception/registration-results.csv": 3,
    "control/parcel-regression-20260624-194647.csv": 9,
    "control/final-e2e-20260624-195504.csv": 2,
}
REQUIRED_FILES = {
    "README.md",
    "manifest.json",
    *EXPECTED_ROWS,
    "perception/obb-training-metrics.png",
    "perception/segmentation-training-metrics.png",
    "perception/segmentation-confusion-matrix.png",
    "perception/registration-montage.jpg",
    "perception/registration-measurement-summary.png",
    "control/parcel-regression-target-steps.png",
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    failures: list[str] = []
    for relative in sorted(REQUIRED_FILES):
        if not (EVIDENCE_ROOT / relative).is_file():
            failures.append(f"missing evidence file: {relative}")

    manifest_path = EVIDENCE_ROOT / "manifest.json"
    if manifest_path.is_file():
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        entries = {entry["path"]: entry for entry in manifest.get("files", [])}
        for relative, entry in entries.items():
            path = EVIDENCE_ROOT / relative
            if not path.is_file():
                failures.append(f"manifest target missing: {relative}")
                continue
            if path.stat().st_size != entry["bytes"]:
                failures.append(f"manifest size mismatch: {relative}")
            if sha256(path) != entry["sha256"]:
                failures.append(f"manifest SHA-256 mismatch: {relative}")
        print(f"[PASS] evidence manifest verified: {len(entries)} files")

    for relative, expected in EXPECTED_ROWS.items():
        path = EVIDENCE_ROOT / relative
        if not path.is_file():
            continue
        with path.open(encoding="utf-8-sig", newline="") as handle:
            actual = sum(1 for _ in csv.DictReader(handle))
        if actual != expected:
            failures.append(f"unexpected CSV rows: {relative}: expected={expected}, actual={actual}")
        else:
            print(f"[PASS] evidence CSV rows: {relative} ({actual})")

    text_files = list(EVIDENCE_ROOT.rglob("*.csv")) + list(EVIDENCE_ROOT.rglob("*.yaml"))
    forbidden = ("C:" + "\\Users\\", "/home/" + "suhyeon/", "/home/" + "minho/")
    for path in text_files:
        text = path.read_text(encoding="utf-8-sig", errors="ignore")
        for marker in forbidden:
            if marker in text:
                failures.append(f"machine-specific path leaked in {path.relative_to(EVIDENCE_ROOT)}")

    for failure in failures:
        print(f"[FAIL] {failure}", file=sys.stderr)
    print(f"\nSummary: {len(failures)} failed")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
