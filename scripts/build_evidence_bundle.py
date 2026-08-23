#!/usr/bin/env python3
"""Build a compact, reproducible portfolio-evidence bundle from archived runs."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import shutil
from pathlib import Path

import matplotlib.pyplot as plt
import yaml


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = REPOSITORY_ROOT / "docs" / "evidence"


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def copy_file(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)


def write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def save_figure(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    plt.tight_layout()
    plt.savefig(path, dpi=180, bbox_inches="tight")
    plt.close()


def plot_training(rows: list[dict[str, str]], output: Path, task: str) -> None:
    epochs = [int(float(row["epoch"])) for row in rows]
    if task == "obb":
        metric_series = [
            ("Precision", "metrics/precision(B)"),
            ("Recall", "metrics/recall(B)"),
            ("mAP50", "metrics/mAP50(B)"),
            ("mAP50-95", "metrics/mAP50-95(B)"),
        ]
        loss_series = [
            ("Box", "val/box_loss"),
            ("Class", "val/cls_loss"),
            ("DFL", "val/dfl_loss"),
            ("Angle", "val/angle_loss"),
        ]
        title = "YOLO11s-OBB validation history"
    else:
        metric_series = [
            ("Box mAP50", "metrics/mAP50(B)"),
            ("Box mAP50-95", "metrics/mAP50-95(B)"),
            ("Mask mAP50", "metrics/mAP50(M)"),
            ("Mask mAP50-95", "metrics/mAP50-95(M)"),
        ]
        loss_series = [
            ("Box", "val/box_loss"),
            ("Mask", "val/seg_loss"),
            ("Class", "val/cls_loss"),
            ("DFL", "val/dfl_loss"),
        ]
        title = "YOLO11 segmentation validation history"

    figure, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    for label, key in metric_series:
        axes[0].plot(epochs, [float(row[key]) for row in rows], label=label, linewidth=2)
    axes[0].set_ylim(0, 1)
    axes[0].set_xlabel("Epoch")
    axes[0].set_ylabel("Metric")
    axes[0].grid(alpha=0.25)
    axes[0].legend(fontsize=8)

    for label, key in loss_series:
        axes[1].plot(epochs, [float(row[key]) for row in rows], label=label, linewidth=2)
    axes[1].set_xlabel("Epoch")
    axes[1].set_ylabel("Validation loss")
    axes[1].grid(alpha=0.25)
    axes[1].legend(fontsize=8)
    figure.suptitle(title, fontsize=14, fontweight="bold")
    save_figure(output)


def sanitize_training_config(source: Path, destination: Path) -> None:
    config = yaml.safe_load(source.read_text(encoding="utf-8-sig"))
    keys = (
        "task", "model", "epochs", "patience", "batch", "imgsz", "optimizer",
        "seed", "deterministic", "pretrained", "amp", "lr0", "weight_decay",
    )
    clean = {key: config.get(key) for key in keys if key in config}
    model = clean.get("model")
    if isinstance(model, str) and ("/" in model or "\\" in model):
        clean["model"] = Path(model).name
    clean["source_note"] = "Machine-specific paths removed from the archived Ultralytics args file."
    destination.write_text(yaml.safe_dump(clean, sort_keys=False, allow_unicode=True), encoding="utf-8")


def process_registration(source_dir: Path, output_dir: Path) -> None:
    source = json.loads((source_dir / "results.json").read_text(encoding="utf-8"))
    rows: list[dict[str, object]] = []
    for index, item in enumerate(source, start=1):
        measured_long, measured_short = item["box"]["size_mm"]
        reference_long, reference_short = item["box"]["reference_size_mm"]
        rows.append(
            {
                "sample": index,
                "box_id": item["box_id"],
                "destination": item["destination"],
                "qr_decode_method": item["qr_decode_method"],
                "valid_parcel": int(bool(item["valid_parcel"])),
                "detection_count": item["detection_count"],
                "best_confidence": max(item["confidences"]),
                "yaw_deg": item["box"]["yaw_deg"],
                "measured_long_mm": measured_long,
                "measured_short_mm": measured_short,
                "reference_long_mm": reference_long,
                "reference_short_mm": reference_short,
                "abs_error_long_mm": abs(measured_long - reference_long),
                "abs_error_short_mm": abs(measured_short - reference_short),
            }
        )
    fieldnames = list(rows[0])
    write_csv(output_dir / "registration-results.csv", fieldnames, rows)
    copy_file(source_dir / "montage.jpg", output_dir / "registration-montage.jpg")

    labels = [f"Sample {row['sample']}" for row in rows]
    x_values = list(range(len(rows)))
    width = 0.36
    figure, axes = plt.subplots(1, 2, figsize=(11, 4.2))
    axes[0].bar(
        [value - width / 2 for value in x_values],
        [float(row["abs_error_long_mm"]) for row in rows],
        width,
        label="Long side",
    )
    axes[0].bar(
        [value + width / 2 for value in x_values],
        [float(row["abs_error_short_mm"]) for row in rows],
        width,
        label="Short side",
    )
    axes[0].set_xticks(x_values, labels)
    axes[0].set_ylabel("Absolute error (mm)")
    axes[0].set_title("Reference-size measurement error")
    axes[0].grid(axis="y", alpha=0.25)
    axes[0].legend()

    axes[1].bar(labels, [float(row["best_confidence"]) for row in rows], color="#4c78a8")
    axes[1].set_ylim(0.8, 1.0)
    axes[1].set_ylabel("Best confidence")
    axes[1].set_title("Detection confidence")
    axes[1].grid(axis="y", alpha=0.25)
    figure.suptitle("Three archived platform-registration samples", fontsize=14, fontweight="bold")
    save_figure(output_dir / "registration-measurement-summary.png")


def process_matlab(regression_csv: Path, e2e_csv: Path, output_dir: Path) -> None:
    regression_destination = output_dir / "parcel-regression-20260624-194647.csv"
    e2e_destination = output_dir / "final-e2e-20260624-195504.csv"
    copy_file(regression_csv, regression_destination)
    copy_file(e2e_csv, e2e_destination)

    rows = [row for row in read_csv(regression_csv) if row["phase"] == "scan"]
    labels = [f"P{row['targetId']}" for row in rows]
    steps = [int(row["steps"]) for row in rows]
    colors = ["#59a14f" if int(row["success"]) else "#e15759" for row in rows]
    plt.figure(figsize=(10, 4.8))
    bars = plt.bar(labels, steps, color=colors)
    plt.axhline(36000, color="#444444", linestyle="--", linewidth=1.5, label="36,000-step limit")
    for bar, value in zip(bars, steps, strict=True):
        plt.text(bar.get_x() + bar.get_width() / 2, value + 600, f"{value:,}", ha="center", fontsize=8)
    plt.ylim(0, 40500)
    plt.ylabel("Simulation steps")
    plt.xlabel("Target parcel")
    plt.title("Historical target-retrieval regression (2026-06-24)", fontweight="bold")
    plt.grid(axis="y", alpha=0.25)
    plt.legend()
    save_figure(output_dir / "parcel-regression-target-steps.png")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_manifest(output_root: Path) -> None:
    files = []
    for path in sorted(output_root.rglob("*")):
        if path.is_file() and path.name != "manifest.json":
            files.append(
                {
                    "path": path.relative_to(output_root).as_posix(),
                    "bytes": path.stat().st_size,
                    "sha256": sha256(path),
                }
            )
    (output_root / "manifest.json").write_text(
        json.dumps({"files": files}, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--obb-dir", type=Path, required=True)
    parser.add_argument("--seg-dir", type=Path, required=True)
    parser.add_argument("--registration-dir", type=Path, required=True)
    parser.add_argument("--matlab-regression-csv", type=Path, required=True)
    parser.add_argument("--matlab-e2e-csv", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    output_root = args.output.resolve()
    perception_dir = output_root / "perception"
    control_dir = output_root / "control"
    perception_dir.mkdir(parents=True, exist_ok=True)
    control_dir.mkdir(parents=True, exist_ok=True)

    obb_rows = read_csv(args.obb_dir / "results.csv")
    copy_file(args.obb_dir / "results.csv", perception_dir / "obb-training-results.csv")
    sanitize_training_config(args.obb_dir / "args.yaml", perception_dir / "obb-training-config.yaml")
    plot_training(obb_rows, perception_dir / "obb-training-metrics.png", "obb")

    seg_rows = read_csv(args.seg_dir / "results.csv")
    copy_file(args.seg_dir / "results.csv", perception_dir / "segmentation-training-results.csv")
    sanitize_training_config(args.seg_dir / "args.yaml", perception_dir / "segmentation-training-config.yaml")
    copy_file(args.seg_dir / "confusion_matrix_normalized.png", perception_dir / "segmentation-confusion-matrix.png")
    plot_training(seg_rows, perception_dir / "segmentation-training-metrics.png", "segmentation")

    process_registration(args.registration_dir, perception_dir)
    process_matlab(args.matlab_regression_csv, args.matlab_e2e_csv, control_dir)
    write_manifest(output_root)
    print(f"evidence bundle written: {output_root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
