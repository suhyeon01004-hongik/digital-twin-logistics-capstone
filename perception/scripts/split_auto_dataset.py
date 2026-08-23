#!/usr/bin/env python3
from __future__ import annotations

import argparse
import random
import shutil
from pathlib import Path

from path_config import DATA_ROOT

IMAGE_EXTS = (".jpg", ".jpeg", ".png", ".bmp")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Move a balanced subset of auto-collected YOLO samples from train to val."
    )
    parser.add_argument(
        "--dataset",
        default=str(DATA_ROOT / "interim" / "auto_qr_box_seg"),
    )
    parser.add_argument("--val-ratio", type=float, default=0.15)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument(
        "--copy",
        action="store_true",
        help="Copy samples to val instead of moving them out of train.",
    )
    return parser.parse_args()


def sample_kind(label_path: Path) -> str:
    if not label_path.exists():
        return "negative"
    return "positive" if label_path.read_text(encoding="utf-8").strip() else "negative"


def collect_samples(dataset: Path):
    train_images = dataset / "train" / "images"
    train_labels = dataset / "train" / "labels"
    samples = {"positive": [], "negative": []}

    for image_path in sorted(train_images.iterdir()):
        if image_path.suffix.lower() not in IMAGE_EXTS:
            continue
        label_path = train_labels / f"{image_path.stem}.txt"
        samples[sample_kind(label_path)].append((image_path, label_path))

    return samples


def transfer_sample(image_path: Path, label_path: Path, dataset: Path, copy: bool) -> None:
    val_images = dataset / "val" / "images"
    val_labels = dataset / "val" / "labels"
    val_images.mkdir(parents=True, exist_ok=True)
    val_labels.mkdir(parents=True, exist_ok=True)

    dst_image = val_images / image_path.name
    dst_label = val_labels / f"{image_path.stem}.txt"
    if dst_image.exists() and dst_label.exists():
        return

    action = shutil.copy2 if copy else shutil.move
    action(str(image_path), str(dst_image))
    if label_path.exists():
        action(str(label_path), str(dst_label))
    else:
        dst_label.write_text("", encoding="utf-8")


def main():
    args = parse_args()
    dataset = Path(args.dataset).expanduser()
    samples = collect_samples(dataset)
    rng = random.Random(args.seed)

    moved = {"positive": 0, "negative": 0}
    for kind, items in samples.items():
        rng.shuffle(items)
        count = int(round(len(items) * args.val_ratio))
        if len(items) > 0:
            count = max(1, count)
        for image_path, label_path in items[:count]:
            transfer_sample(image_path, label_path, dataset, args.copy)
            moved[kind] += 1

    mode = "copied" if args.copy else "moved"
    print(
        f"Validation split {mode}: positive={moved['positive']}, "
        f"negative={moved['negative']}, dataset={dataset}"
    )


if __name__ == "__main__":
    main()
