#!/usr/bin/env python3
from __future__ import annotations

import argparse
import shutil
from pathlib import Path

import cv2
import numpy as np


IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert YOLO segmentation/polygon labels to YOLO OBB labels."
    )
    parser.add_argument(
        "--source",
        action="append",
        required=True,
        help="Source YOLO dataset directory. Can be passed multiple times.",
    )
    parser.add_argument(
        "--output",
        required=True,
        help="Output YOLO OBB dataset directory.",
    )
    parser.add_argument(
        "--class-name",
        default="box",
        help="Class name for data.yaml.",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Remove output directory before conversion.",
    )
    return parser.parse_args()


def split_name(split: str) -> str:
    if split in ("valid", "validation"):
        return "val"
    if split in ("train", "val", "test"):
        return split
    return "train"


def find_split_dirs(dataset_dir: Path) -> list[tuple[str, Path, Path]]:
    found = []
    for split in ("train", "val", "valid", "test"):
        images_dir = dataset_dir / split / "images"
        labels_dir = dataset_dir / split / "labels"
        if images_dir.exists():
            found.append((split_name(split), images_dir, labels_dir))
    return found


def image_size(path: Path) -> tuple[int, int] | None:
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        return None
    height, width = image.shape[:2]
    return width, height


def yolo_bbox_to_points(values: list[float], width: int, height: int) -> np.ndarray:
    cx, cy, bw, bh = values
    x1 = (cx - bw / 2.0) * width
    y1 = (cy - bh / 2.0) * height
    x2 = (cx + bw / 2.0) * width
    y2 = (cy + bh / 2.0) * height
    return np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]], dtype=np.float32)


def yolo_polygon_to_points(values: list[float], width: int, height: int) -> np.ndarray:
    coords = np.array(values, dtype=np.float32).reshape(-1, 2)
    coords[:, 0] *= width
    coords[:, 1] *= height
    return coords


def order_points_clockwise(points: np.ndarray) -> np.ndarray:
    center = points.mean(axis=0)
    angles = np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0])
    ordered = points[np.argsort(angles)]
    start = int(np.argmin(ordered[:, 0] + ordered[:, 1]))
    return np.roll(ordered, -start, axis=0)


def points_to_obb_line(cls: str, points: np.ndarray, width: int, height: int) -> str | None:
    if len(points) < 3:
        return None
    rect = cv2.minAreaRect(points.astype(np.float32))
    box = cv2.boxPoints(rect)
    box = order_points_clockwise(box)
    box[:, 0] = np.clip(box[:, 0] / width, 0.0, 1.0)
    box[:, 1] = np.clip(box[:, 1] / height, 0.0, 1.0)
    values = " ".join(f"{v:.6f}" for v in box.reshape(-1))
    return f"{cls} {values}"


def convert_label_line(line: str, width: int, height: int) -> str | None:
    parts = line.strip().split()
    if not parts:
        return None
    cls = parts[0]
    try:
        values = [float(value) for value in parts[1:]]
    except ValueError:
        return None

    if len(values) == 4:
        points = yolo_bbox_to_points(values, width, height)
    elif len(values) >= 6 and len(values) % 2 == 0:
        points = yolo_polygon_to_points(values, width, height)
    else:
        return None

    return points_to_obb_line(cls, points, width, height)


def convert_label_file(label_path: Path, width: int, height: int) -> str:
    if not label_path.exists():
        return ""
    lines = []
    for raw_line in label_path.read_text(encoding="utf-8", errors="ignore").splitlines():
        converted = convert_label_line(raw_line, width, height)
        if converted is not None:
            lines.append(converted)
    return "\n".join(lines) + ("\n" if lines else "")


def copy_dataset(source_dir: Path, output_dir: Path) -> tuple[int, int, int]:
    dataset_prefix = source_dir.name
    image_count = 0
    label_count = 0
    skipped_count = 0

    for split, images_dir, labels_dir in find_split_dirs(source_dir):
        out_images_dir = output_dir / split / "images"
        out_labels_dir = output_dir / split / "labels"
        out_images_dir.mkdir(parents=True, exist_ok=True)
        out_labels_dir.mkdir(parents=True, exist_ok=True)

        for image_path in sorted(images_dir.iterdir()):
            if image_path.suffix.lower() not in IMAGE_EXTS:
                continue
            size = image_size(image_path)
            if size is None:
                skipped_count += 1
                continue
            width, height = size

            stem = f"{dataset_prefix}__{image_path.stem}"
            out_image_path = out_images_dir / f"{stem}{image_path.suffix.lower()}"
            out_label_path = out_labels_dir / f"{stem}.txt"
            label_path = labels_dir / f"{image_path.stem}.txt"

            shutil.copy2(image_path, out_image_path)
            label_text = convert_label_file(label_path, width, height)
            out_label_path.write_text(label_text, encoding="utf-8")

            image_count += 1
            if label_text.strip():
                label_count += 1

    return image_count, label_count, skipped_count


def write_data_yaml(output_dir: Path, class_name: str) -> None:
    (output_dir / "data.yaml").write_text(
        "\n".join(
            [
                f"path: {output_dir}",
                "train: train/images",
                "val: val/images",
                "test: test/images",
                "names:",
                f"  0: {class_name}",
                "",
            ]
        ),
        encoding="utf-8",
    )


def main() -> None:
    args = parse_args()
    output_dir = Path(args.output).expanduser().resolve()
    source_dirs = [Path(source).expanduser().resolve() for source in args.source]

    if args.overwrite and output_dir.exists():
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    total_images = 0
    total_labeled = 0
    total_skipped = 0
    for source_dir in source_dirs:
        images, labeled, skipped = copy_dataset(source_dir, output_dir)
        print(
            f"{source_dir}: images={images}, labeled={labeled}, "
            f"empty_or_negative={images - labeled}, skipped={skipped}"
        )
        total_images += images
        total_labeled += labeled
        total_skipped += skipped

    write_data_yaml(output_dir, args.class_name)
    print(
        f"Done: output={output_dir}, images={total_images}, "
        f"labeled={total_labeled}, empty_or_negative={total_images - total_labeled}, "
        f"skipped={total_skipped}"
    )


if __name__ == "__main__":
    main()
