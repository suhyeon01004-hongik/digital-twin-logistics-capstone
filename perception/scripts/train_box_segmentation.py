#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

from ultralytics import YOLO

from path_config import DATA_ROOT, RUNS_ROOT

def parse_args():
    parser = argparse.ArgumentParser(
        description="Fine-tune a smaller YOLO segmentation model for platform box detection."
    )
    parser.add_argument(
        "--data",
        default=str(DATA_ROOT / "processed" / "combined_box_segmentation" / "data.yaml"),
    )
    parser.add_argument("--model", default="yolo11s-seg.pt")
    parser.add_argument("--imgsz", type=int, default=512)
    parser.add_argument("--epochs", type=int, default=80)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--device", default="0")
    parser.add_argument(
        "--project",
        default=str(RUNS_ROOT / "box_segmentation"),
    )
    parser.add_argument("--name", default="box_yolo11s_seg_platform")
    parser.add_argument("--export-engine", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    model = YOLO(args.model)
    results = model.train(
        data=args.data,
        imgsz=args.imgsz,
        epochs=args.epochs,
        batch=args.batch,
        device=args.device,
        project=args.project,
        name=args.name,
        task="segment",
    )

    if args.export_engine:
        run_dir = Path(results.save_dir)
        best = run_dir / "weights" / "best.pt"
        export_model = YOLO(str(best))
        export_model.export(format="engine", imgsz=args.imgsz, half=True, device=args.device)


if __name__ == "__main__":
    main()
