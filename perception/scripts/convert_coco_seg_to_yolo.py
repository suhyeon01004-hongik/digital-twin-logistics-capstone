#!/usr/bin/env python3
import argparse
import json
import os
from pathlib import Path

from path_config import DATA_ROOT



def normalize_polygon(poly, width, height):
    values = []
    for i in range(0, len(poly), 2):
        x = min(max(float(poly[i]) / width, 0.0), 1.0)
        y = min(max(float(poly[i + 1]) / height, 0.0), 1.0)
        values.extend((x, y))
    return values


def write_split(raw_root, out_root, split, category_map):
    annotation_path = raw_root / split / "_annotations.coco.json"
    with annotation_path.open("r", encoding="utf-8") as f:
        coco = json.load(f)

    image_by_id = {image["id"]: image for image in coco["images"]}
    labels_by_image = {image_id: [] for image_id in image_by_id}

    for ann in coco["annotations"]:
        image = image_by_id.get(ann["image_id"])
        class_id = category_map.get(ann["category_id"])
        if image is None or class_id is None:
            continue

        width = float(image["width"])
        height = float(image["height"])
        for poly in ann.get("segmentation", []):
            if not isinstance(poly, list) or len(poly) < 6 or len(poly) % 2:
                continue
            normalized = normalize_polygon(poly, width, height)
            labels_by_image[ann["image_id"]].append((class_id, normalized))

    labels_dir = out_root / "labels" / split
    labels_dir.mkdir(parents=True, exist_ok=True)

    for image_id, image in image_by_id.items():
        label_path = labels_dir / (Path(image["file_name"]).stem + ".txt")
        with label_path.open("w", encoding="utf-8") as f:
            for class_id, normalized in labels_by_image[image_id]:
                coords = " ".join(f"{v:.6f}" for v in normalized)
                f.write(f"{class_id} {coords}\n")

    images_dir = out_root / "images" / split
    images_dir.mkdir(parents=True, exist_ok=True)
    for image in image_by_id.values():
        source = raw_root / split / image["file_name"]
        target = images_dir / image["file_name"]
        relative_source = os.path.relpath(source, images_dir)
        if target.is_symlink():
            if os.readlink(target) == relative_source:
                continue
            target.unlink()
        elif target.exists():
            raise FileExistsError(f"{target} already exists and is not a symlink")
        target.symlink_to(relative_source)

    return len(image_by_id), sum(len(v) for v in labels_by_image.values())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--raw-root", default=str(DATA_ROOT / "interim" / "box_detection_coco"))
    parser.add_argument("--out-root", default=str(DATA_ROOT / "processed" / "box_segmentation_yolo"))
    args = parser.parse_args()

    raw_root = Path(args.raw_root).resolve()
    out_root = Path(args.out_root).resolve()
    out_root.mkdir(parents=True, exist_ok=True)

    with (raw_root / "train" / "_annotations.coco.json").open("r", encoding="utf-8") as f:
        coco = json.load(f)

    categories = [cat for cat in coco["categories"] if cat["name"] == "Box"]
    if not categories:
        raise RuntimeError("Could not find the Box category in the COCO annotations")

    category_map = {categories[0]["id"]: 0}

    totals = {}
    for split in ("train", "valid", "test"):
        totals[split] = write_split(raw_root, out_root, split, category_map)

    data_yaml = out_root / "data.yaml"
    data_yaml.write_text(
        "\n".join(
            [
                f"path: {out_root}",
                "train: images/train",
                "val: images/valid",
                "test: images/test",
                "names:",
                "  0: Box",
                "",
            ]
        ),
        encoding="utf-8",
    )

    for split, (image_count, label_count) in totals.items():
        print(f"{split}: {image_count} images, {label_count} polygons")
    print(f"Wrote {data_yaml}")


if __name__ == "__main__":
    main()
