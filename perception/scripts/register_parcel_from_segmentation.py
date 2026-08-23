#!/usr/bin/env python3
"""Register parcel boxes from YOLO segmentation and QR codes.

The model may output several Box instances for one physical parcel. This script
merges the masks, accepts the parcel only when a decoded QR lies inside the
merged box mask, then stores ID/destination/size/yaw in SQLite.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import sqlite3

import cv2
import numpy as np
from pyzbar import pyzbar
from ultralytics import YOLO

from path_config import DATA_ROOT, MODEL_ROOT, RUNS_ROOT, model_from_env

IMAGE_SUFFIXES = {".bmp", ".jpg", ".jpeg", ".png", ".tif", ".tiff"}


def default_model_path() -> Path:
    return model_from_env(
        "MILEMATE_SEG_MODEL_PATH",
        MODEL_ROOT / "box_segmentation" / "best.pt",
    )


def iter_images(source: Path) -> list[Path]:
    if source.is_file():
        return [source]
    if not source.is_dir():
        raise FileNotFoundError(f"source does not exist: {source}")
    return sorted(p for p in source.iterdir() if p.suffix.lower() in IMAGE_SUFFIXES)


def normalize_yaw(yaw: float) -> float:
    """Normalize an undirected line angle to [-90, 90) degrees."""
    while yaw >= 90.0:
        yaw -= 180.0
    while yaw < -90.0:
        yaw += 180.0
    return yaw


def long_edge_yaw_deg(box_points: np.ndarray) -> float:
    """Return the angle from image +x axis to the rectangle long edge.

    The image coordinate system is used: +x points right and +y points down.
    Therefore positive yaw means clockwise rotation on the displayed image.
    Because a box edge has no arrow direction, the result is normalized to
    [-90, 90) degrees.
    """
    longest = None
    longest_len = -1.0
    for idx in range(4):
        p0 = box_points[idx].astype(np.float32)
        p1 = box_points[(idx + 1) % 4].astype(np.float32)
        vec = p1 - p0
        length = float(np.linalg.norm(vec))
        if length > longest_len:
            longest = vec
            longest_len = length

    if longest is None or longest_len <= 0.0:
        return 0.0
    yaw = float(np.degrees(np.arctan2(longest[1], longest[0])))
    return normalize_yaw(yaw)


def resize_mask(mask: np.ndarray, width: int, height: int) -> np.ndarray:
    mask_u8 = ((mask > 0.5).astype(np.uint8)) * 255
    if mask_u8.shape != (height, width):
        mask_u8 = cv2.resize(mask_u8, (width, height), interpolation=cv2.INTER_NEAREST)
    return mask_u8


def merged_mask_from_result(result, width: int, height: int) -> np.ndarray:
    union = np.zeros((height, width), dtype=np.uint8)
    if result.masks is None:
        return union

    masks = result.masks.data.cpu().numpy()
    for mask in masks:
        union = cv2.bitwise_or(union, resize_mask(mask, width, height))
    return union


def remove_small_components(mask: np.ndarray, min_area_ratio: float) -> np.ndarray:
    height, width = mask.shape[:2]
    min_area = int(height * width * min_area_ratio)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    if count <= 1:
        return mask

    kept = np.zeros_like(mask)
    component_areas = stats[1:, cv2.CC_STAT_AREA]
    largest_label = int(np.argmax(component_areas)) + 1

    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area >= min_area:
            kept[labels == label] = 255

    if not kept.any():
        kept[labels == largest_label] = 255
    return kept


def estimate_rotated_box(mask: np.ndarray):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [c for c in contours if cv2.contourArea(c) > 0]
    if not contours:
        return None

    points = np.vstack(contours)
    rect = cv2.minAreaRect(points)
    (cx, cy), (rw, rh), _ = rect
    long_px = max(float(rw), float(rh))
    short_px = min(float(rw), float(rh))
    box_points = cv2.boxPoints(rect).astype(np.int32)
    yaw = long_edge_yaw_deg(box_points)
    return {
        "center_px": [float(cx), float(cy)],
        "size_px": [long_px, short_px],
        "yaw_deg": float(yaw),
        "box_points_px": box_points.tolist(),
    }


def add_mm_measurements(box_info: dict | None, long_mm: float, short_mm: float) -> None:
    if box_info is None:
        return

    long_px, short_px = box_info["size_px"]
    long_scale = long_mm / long_px
    short_scale = short_mm / short_px
    mm_per_px = (long_scale + short_scale) / 2.0

    box_info["reference_size_mm"] = [float(long_mm), float(short_mm)]
    box_info["mm_per_px"] = float(mm_per_px)
    box_info["size_mm"] = [float(long_px * mm_per_px), float(short_px * mm_per_px)]


def decode_qr(image: np.ndarray) -> tuple[str | None, list[list[int]] | None, str | None]:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    adaptive = cv2.adaptiveThreshold(
        cv2.GaussianBlur(gray, (5, 5), 0),
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY,
        11,
        2,
    )
    _, otsu = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    variants = (
        ("raw", image),
        ("gray", gray),
        ("adaptive_threshold", adaptive),
        ("otsu_threshold", otsu),
    )

    for method, variant in variants:
        decoded = pyzbar.decode(variant)
        if not decoded:
            continue
        obj = decoded[0]
        qr_data = obj.data.decode("utf-8", errors="replace").strip()
        polygon = [[int(p.x), int(p.y)] for p in obj.polygon]
        return qr_data, polygon, method
    return None, None, None


def qr_center(qr_polygon: list[list[int]] | None) -> tuple[int, int] | None:
    if not qr_polygon:
        return None
    points = np.array(qr_polygon, dtype=np.float32)
    center = points.mean(axis=0)
    return int(round(float(center[0]))), int(round(float(center[1])))


def qr_is_inside_box(mask: np.ndarray, qr_polygon: list[list[int]] | None) -> bool:
    center = qr_center(qr_polygon)
    if center is None:
        return False
    x, y = center
    height, width = mask.shape[:2]
    if x < 0 or y < 0 or x >= width or y >= height:
        return False
    return bool(mask[y, x] > 0)


def destination_from_qr(qr_data: str | None) -> str | None:
    if not qr_data:
        return None
    return qr_data[0]


def init_database(db_path: Path) -> sqlite3.Connection:
    db_path.parent.mkdir(parents=True, exist_ok=True)
    conn = sqlite3.connect(db_path)
    conn.execute("PRAGMA foreign_keys = ON")
    conn.execute(
        """
        CREATE TABLE IF NOT EXISTS boxes (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            box_uid TEXT UNIQUE,
            qr_data TEXT UNIQUE,
            destination TEXT,
            latest_yaw_deg REAL,
            latest_long_mm REAL,
            latest_short_mm REAL,
            created_at TEXT NOT NULL,
            updated_at TEXT NOT NULL
        )
        """
    )
    conn.execute(
        """
        CREATE TABLE IF NOT EXISTS detections (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            box_id INTEGER NOT NULL,
            image_path TEXT NOT NULL,
            output_image_path TEXT NOT NULL,
            qr_data TEXT,
            destination TEXT,
            yaw_deg REAL,
            long_mm REAL,
            short_mm REAL,
            long_px REAL,
            short_px REAL,
            mm_per_px REAL,
            yolo_detection_count INTEGER NOT NULL,
            confidences_json TEXT NOT NULL,
            qr_decode_method TEXT,
            created_at TEXT NOT NULL,
            FOREIGN KEY(box_id) REFERENCES boxes(id)
        )
        """
    )
    return conn


def box_uid(box_id: int) -> str:
    return f"BOX-{box_id:02d}"


def get_or_create_box(
    conn: sqlite3.Connection,
    qr_data: str | None,
    destination: str | None,
    box_info: dict | None,
) -> tuple[int, str]:
    now = datetime.now().isoformat(timespec="seconds")
    latest_yaw = None if box_info is None else box_info["yaw_deg"]
    latest_long = None if box_info is None else box_info["size_mm"][0]
    latest_short = None if box_info is None else box_info["size_mm"][1]

    row = None
    if qr_data:
        row = conn.execute("SELECT id, box_uid FROM boxes WHERE qr_data = ?", (qr_data,)).fetchone()

    if row is None:
        cur = conn.execute(
            """
            INSERT INTO boxes (
                qr_data, destination, latest_yaw_deg, latest_long_mm,
                latest_short_mm, created_at, updated_at
            )
            VALUES (?, ?, ?, ?, ?, ?, ?)
            """,
            (qr_data, destination, latest_yaw, latest_long, latest_short, now, now),
        )
        box_id = int(cur.lastrowid)
        uid = box_uid(box_id)
        conn.execute("UPDATE boxes SET box_uid = ? WHERE id = ?", (uid, box_id))
        return box_id, uid

    box_id, uid = int(row[0]), str(row[1])
    conn.execute(
        """
        UPDATE boxes
        SET destination = COALESCE(?, destination),
            latest_yaw_deg = ?,
            latest_long_mm = ?,
            latest_short_mm = ?,
            updated_at = ?
        WHERE id = ?
        """,
        (destination, latest_yaw, latest_long, latest_short, now, box_id),
    )
    return box_id, uid


def save_detection(
    conn: sqlite3.Connection,
    box_id: int,
    image_path: Path,
    output_image: Path,
    qr_data: str | None,
    destination: str | None,
    box_info: dict | None,
    detection_count: int,
    confidences: list[float],
    qr_decode_method: str | None,
) -> None:
    now = datetime.now().isoformat(timespec="seconds")
    long_mm = short_mm = long_px = short_px = mm_per_px = yaw = None
    if box_info is not None:
        long_mm, short_mm = box_info["size_mm"]
        long_px, short_px = box_info["size_px"]
        mm_per_px = box_info["mm_per_px"]
        yaw = box_info["yaw_deg"]

    conn.execute(
        """
        INSERT INTO detections (
            box_id, image_path, output_image_path, qr_data, destination, yaw_deg,
            long_mm, short_mm, long_px, short_px, mm_per_px,
            yolo_detection_count, confidences_json, qr_decode_method, created_at
        )
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """,
        (
            box_id,
            str(image_path),
            str(output_image),
            qr_data,
            destination,
            yaw,
            long_mm,
            short_mm,
            long_px,
            short_px,
            mm_per_px,
            detection_count,
            json.dumps(confidences),
            qr_decode_method,
            now,
        ),
    )


def annotate_image(
    image: np.ndarray,
    mask: np.ndarray,
    box_info: dict | None,
    box_uid_text: str | None,
    destination: str | None,
    qr_polygon: list[list[int]] | None,
    valid_parcel: bool,
    invalid_reason: str | None,
) -> np.ndarray:
    vis = image.copy()
    if mask.any():
        overlay = vis.copy()
        overlay[mask > 0] = (255, 80, 0)
        vis = cv2.addWeighted(overlay, 0.45, vis, 0.55, 0)

    if box_info is not None:
        points = np.array(box_info["box_points_px"], dtype=np.int32)
        cv2.drawContours(vis, [points], 0, (0, 255, 255), 2)
        long_px, short_px = box_info["size_px"]
        if "size_mm" in box_info:
            long_mm, short_mm = box_info["size_mm"]
            size_text = f"{long_mm:.1f}x{short_mm:.1f}mm"
        else:
            size_text = f"{long_px:.0f}x{short_px:.0f}px"
        cv2.putText(
            vis,
            f"yaw={box_info['yaw_deg']:.1f} size={size_text}",
            (8, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        if box_uid_text:
            dst_text = "" if destination is None else f" dest={destination}"
            cv2.putText(
                vis,
                f"id={box_uid_text}{dst_text}",
                (8, 56),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
        elif not valid_parcel:
            cv2.putText(
                vis,
                f"invalid: {invalid_reason}",
                (8, 56),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
    else:
        cv2.putText(
            vis,
            "no box",
            (8, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )

    if qr_polygon and len(qr_polygon) >= 4:
        qr_points = np.array(qr_polygon, dtype=np.int32).reshape((-1, 1, 2))
        cv2.polylines(vis, [qr_points], True, (0, 255, 0), 2)
    return vis


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="YOLO-seg parcel box size/yaw inference with merged masks."
    )
    parser.add_argument("--source", type=Path, default=DATA_ROOT / "raw" / "registration_input")
    parser.add_argument("--model", type=Path, default=default_model_path())
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=RUNS_ROOT / "parcel_registration",
    )
    parser.add_argument("--imgsz", type=int, default=960)
    parser.add_argument("--conf", type=float, default=0.50)
    parser.add_argument("--iou", type=float, default=0.50)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--min-component-area-ratio", type=float, default=0.02)
    parser.add_argument("--box-long-mm", type=float, default=200.0)
    parser.add_argument("--box-short-mm", type=float, default=145.0)
    parser.add_argument("--db-path", type=Path)
    parser.add_argument("--save-mask", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    image_paths = iter_images(args.source)
    if not image_paths:
        raise RuntimeError(f"no images found: {args.source}")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    db_path = args.db_path or (args.output_dir / "box_perception.sqlite3")
    conn = init_database(db_path)
    model = YOLO(str(args.model))
    results = model.predict(
        source=[str(p) for p in image_paths],
        imgsz=args.imgsz,
        conf=args.conf,
        iou=args.iou,
        device=args.device,
        retina_masks=True,
        save=False,
        verbose=False,
    )

    records = []
    for result in results:
        image_path = Path(result.path)
        image = cv2.imread(str(image_path))
        if image is None:
            raise RuntimeError(f"failed to read image: {image_path}")
        height, width = image.shape[:2]

        detection_count = 0 if result.boxes is None else len(result.boxes)
        confidences = (
            []
            if result.boxes is None or result.boxes.conf is None
            else [float(c) for c in result.boxes.conf.cpu().numpy()]
        )

        mask = merged_mask_from_result(result, width, height)
        mask = remove_small_components(mask, args.min_component_area_ratio)
        box_info = estimate_rotated_box(mask)
        add_mm_measurements(box_info, args.box_long_mm, args.box_short_mm)

        qr_data, qr_polygon, qr_decode_method = decode_qr(image)
        destination = destination_from_qr(qr_data)
        qr_inside_box = qr_is_inside_box(mask, qr_polygon)
        valid_parcel = box_info is not None and qr_data is not None and qr_inside_box
        invalid_reason = None
        if box_info is None:
            invalid_reason = "no box"
        elif qr_data is None:
            invalid_reason = "no QR"
        elif not qr_inside_box:
            invalid_reason = "QR outside box"

        output_image = args.output_dir / f"{image_path.stem}_box_yaw.jpg"
        uid = None
        if valid_parcel:
            box_id, uid = get_or_create_box(conn, qr_data, destination, box_info)
        annotated = annotate_image(
            image,
            mask,
            box_info,
            uid,
            destination,
            qr_polygon,
            valid_parcel,
            invalid_reason,
        )
        cv2.imwrite(str(output_image), annotated)
        if args.save_mask:
            cv2.imwrite(str(args.output_dir / f"{image_path.stem}_mask.png"), mask)
        if valid_parcel:
            save_detection(
                conn,
                box_id,
                image_path,
                output_image,
                qr_data,
                destination,
                box_info,
                detection_count,
                confidences,
                qr_decode_method,
            )
            conn.commit()

        record = {
            "box_id": uid,
            "qr_data": qr_data,
            "qr_decode_method": qr_decode_method,
            "qr_inside_box": qr_inside_box,
            "valid_parcel": valid_parcel,
            "destination": destination,
            "image": str(image_path),
            "output_image": str(output_image),
            "detection_count": detection_count,
            "confidences": confidences,
            "merged_mask_area_px": int((mask > 0).sum()),
            "box": box_info,
        }
        records.append(record)

        if not valid_parcel:
            print(f"{image_path.name}: rejected ({invalid_reason}) qr={qr_data}")
        else:
            long_px, short_px = box_info["size_px"]
            long_mm, short_mm = box_info["size_mm"]
            print(
                f"{image_path.name}: id={uid} dest={destination} qr={qr_data} "
                f"yaw={box_info['yaw_deg']:.2f} "
                f"size={long_mm:.1f}x{short_mm:.1f}mm "
                f"({long_px:.1f}x{short_px:.1f}px, "
                f"{box_info['mm_per_px']:.4f}mm/px)"
            )

    output_json = args.output_dir / "results.json"
    output_json.write_text(json.dumps(records, indent=2), encoding="utf-8")
    print(f"saved: {args.output_dir}")
    print(f"db: {db_path}")


if __name__ == "__main__":
    main()
