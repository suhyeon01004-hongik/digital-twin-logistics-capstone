#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import time
from pathlib import Path
from typing import Any, Optional

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from .perception_geometry import (
    clamp,
    classify_parcel_size,
    edge_axis_yaws_deg,
    yaw_error_to_camera_axis,
)
from .runtime_paths import repository_root

try:
    from ultralytics import YOLO
except ImportError as exc:  # pragma: no cover - runtime dependency
    raise SystemExit("ultralytics is required for parcel perception") from exc

try:
    import torch
except ImportError:  # pragma: no cover - runtime dependency
    torch = None

try:
    import tensorrt as trt  # noqa: F401
    TENSORRT_AVAILABLE = True
except Exception:
    TENSORRT_AVAILABLE = False

try:
    from pyzbar import pyzbar
    PYZBAR_IMPORT_ERROR = ""
except Exception as exc:  # QR is optional for loading.
    pyzbar = None
    PYZBAR_IMPORT_ERROR = str(exc)


REPOSITORY_ROOT = repository_root(__file__)
REPOSITORY_MODEL_PATH = REPOSITORY_ROOT / "artifacts" / "models" / "box_obb_s_512" / "best.pt"


def configured_model_path() -> Path:
    """Return the portable model path used by source and installed runs."""

    configured = os.environ.get("MILEMATE_MODEL_PATH", "").strip()
    return Path(configured).expanduser() if configured else REPOSITORY_MODEL_PATH


DEFAULT_MODEL_PATH = configured_model_path()
FALLBACK_MODEL_PATH = REPOSITORY_MODEL_PATH

def as_bool(value: Any) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


def draw_axis_line(
    image: np.ndarray,
    center: list[float],
    yaw_deg: float,
    half_len_px: float,
    color: tuple[int, int, int],
    thickness: int = 2,
) -> None:
    rad = np.radians(float(yaw_deg))
    dx = float(np.cos(rad)) * float(half_len_px)
    dy = float(np.sin(rad)) * float(half_len_px)
    p0 = (int(round(float(center[0]) - dx)), int(round(float(center[1]) - dy)))
    p1 = (int(round(float(center[0]) + dx)), int(round(float(center[1]) + dy)))
    cv2.line(image, p0, p1, color, thickness)


def add_unique_qr(
    decoded: list[dict[str, Any]],
    text: str,
    center_px: list[float],
    source: str,
    polygon: Optional[list[list[float]]] = None,
):
    text = str(text or "").strip()
    if not text:
        return
    for item in decoded:
        if item.get("data") == text:
            return
    decoded.append({
        "data": text,
        "center_px": center_px,
        "source": source,
        "polygon": polygon or [],
    })


def decode_qr_with_pyzbar(image: np.ndarray) -> list[dict[str, Any]]:
    if pyzbar is None:
        return []
    try:
        detections = pyzbar.decode(image)
    except Exception:
        return []
    decoded: list[dict[str, Any]] = []
    for det in detections:
        try:
            text = det.data.decode("utf-8", errors="ignore").strip()
        except Exception:
            text = ""
        pts = getattr(det, "polygon", None) or []
        if pts:
            polygon = [[float(p.x), float(p.y)] for p in pts]
            center_x = float(sum(p.x for p in pts) / len(pts))
            center_y = float(sum(p.y for p in pts) / len(pts))
        else:
            rect = det.rect
            polygon = [
                [float(rect.left), float(rect.top)],
                [float(rect.left + rect.width), float(rect.top)],
                [float(rect.left + rect.width), float(rect.top + rect.height)],
                [float(rect.left), float(rect.top + rect.height)],
            ]
            center_x = float(rect.left + rect.width / 2.0)
            center_y = float(rect.top + rect.height / 2.0)
        add_unique_qr(decoded, text, [center_x, center_y], "pyzbar", polygon)
    return decoded


def decode_qr_with_opencv(image: np.ndarray) -> list[dict[str, Any]]:
    decoded: list[dict[str, Any]] = []
    detector = cv2.QRCodeDetector()
    variants = [("raw", image)]
    try:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        variants.append(("gray", gray))
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        variants.append((
            "adaptive",
            cv2.adaptiveThreshold(
                blurred,
                255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY,
                11,
                2,
            ),
        ))
    except Exception:
        pass

    for name, variant in variants:
        try:
            ok, decoded_info, points, _ = detector.detectAndDecodeMulti(variant)
            if ok and points is not None:
                for text, pts in zip(decoded_info, points, strict=False):
                    pts = np.asarray(pts, dtype=np.float32).reshape(-1, 2)
                    center = [float(np.mean(pts[:, 0])), float(np.mean(pts[:, 1]))]
                    polygon = [[float(x), float(y)] for x, y in pts]
                    add_unique_qr(decoded, text, center, "opencv_multi_" + name, polygon)
        except Exception:
            pass
        if decoded:
            return decoded
        try:
            text, points, _ = detector.detectAndDecode(variant)
            if points is not None:
                pts = np.asarray(points, dtype=np.float32).reshape(-1, 2)
                center = [float(np.mean(pts[:, 0])), float(np.mean(pts[:, 1]))]
                polygon = [[float(x), float(y)] for x, y in pts]
            else:
                center = [0.0, 0.0]
                polygon = []
            add_unique_qr(decoded, text, center, "opencv_" + name, polygon)
        except Exception:
            pass
        if decoded:
            return decoded
    return decoded


def decode_qr(image: np.ndarray) -> list[dict[str, Any]]:
    decoded: list[dict[str, Any]] = []
    for item in decode_qr_with_pyzbar(image) + decode_qr_with_opencv(image):
        add_unique_qr(
            decoded,
            item.get("data", ""),
            item.get("center_px", [0.0, 0.0]),
            item.get("source", "qr"),
            item.get("polygon", []),
        )
    return decoded


def offset_qrs(qrs: list[dict[str, Any]], offset_x: int, offset_y: int) -> list[dict[str, Any]]:
    shifted = []
    for qr in qrs:
        item = dict(qr)
        center = item.get("center_px") or [0.0, 0.0]
        item["center_px"] = [float(center[0]) + offset_x, float(center[1]) + offset_y]
        polygon = item.get("polygon") or []
        item["polygon"] = [
            [float(point[0]) + offset_x, float(point[1]) + offset_y]
            for point in polygon
            if len(point) >= 2
        ]
        shifted.append(item)
    return shifted


def decode_qr_in_candidate_rois(
    image: np.ndarray,
    candidates: list[dict[str, Any]],
    max_candidates: int,
    margin_px: int,
) -> list[dict[str, Any]]:
    height, width = image.shape[:2]
    decoded: list[dict[str, Any]] = []
    for cand in sorted(candidates, key=lambda item: item["confidence"], reverse=True)[:max_candidates]:
        points = cand["points"].astype(np.float32)
        x, y, w, h = cv2.boundingRect(points.astype(np.int32))
        x0 = max(0, x - margin_px)
        y0 = max(0, y - margin_px)
        x1 = min(width, x + w + margin_px)
        y1 = min(height, y + h + margin_px)
        if x1 <= x0 or y1 <= y0:
            continue
        for qr in offset_qrs(decode_qr(image[y0:y1, x0:x1]), x0, y0):
            add_unique_qr(
                decoded,
                qr.get("data", ""),
                qr.get("center_px", [0.0, 0.0]),
                qr.get("source", "qr_roi"),
                qr.get("polygon", []),
            )
    return decoded


def draw_qr_debug(debug: np.ndarray, qrs: list[dict[str, Any]]):
    for qr in qrs:
        polygon = qr.get("polygon") or []
        pts = np.asarray(polygon, dtype=np.float32).reshape(-1, 2) if polygon else np.empty((0, 2), dtype=np.float32)
        if len(pts) >= 3:
            cv2.polylines(debug, [pts.astype(np.int32)], True, (255, 0, 255), 3)
        center = qr.get("center_px") or []
        if len(center) >= 2:
            cx = int(float(center[0]))
            cy = int(float(center[1]))
            cv2.circle(debug, (cx, cy), 5, (255, 0, 255), -1)
            cv2.putText(debug, "QR", (cx + 8, cy - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)


def point_in_poly(point: tuple[float, float], poly: np.ndarray) -> bool:
    return cv2.pointPolygonTest(poly.astype(np.float32), point, False) >= 0


def destination_from_qr(text: str) -> str:
    text = str(text or "").strip()
    if not text:
        return ""
    try:
        payload = json.loads(text)
        for key in ("destination", "dest", "region", "area"):
            if key in payload:
                return str(payload[key])
    except Exception:
        pass
    if "," in text:
        parts = [part.strip() for part in text.split(",")]
        if len(parts) >= 2:
            return parts[-1]
    return ""


class ParcelPerceptionNode(Node):
    def __init__(self):
        super().__init__("parcel_perception_node")
        self.declare_parameter("image_topic", "/hk_camera/rgb/compressed")
        self.declare_parameter("detection_topic", "/platform/parcel_detection")
        self.declare_parameter("debug_topic", "/platform/parcel_detection/debug_image/compressed")
        self.declare_parameter("model_path", str(configured_model_path()))
        self.declare_parameter("model_task", "obb")
        self.declare_parameter("confidence", 0.35)
        self.declare_parameter("imgsz", 512)
        self.declare_parameter("device", "0")
        self.declare_parameter("half", True)
        self.declare_parameter("max_det", 5)
        self.declare_parameter("inference_rate_hz", 30.0)
        self.declare_parameter("process_every_n_frames", 1)
        self.declare_parameter("alignment_edge", "short")
        self.declare_parameter("alignment_axis", "image_y")
        self.declare_parameter("publish_debug", True)
        self.declare_parameter("debug_jpeg_quality", 60)
        self.declare_parameter("debug_max_width", 960)
        self.declare_parameter("qr_roi_max_candidates", 2)
        self.declare_parameter("qr_roi_margin_px", 32)
        self.declare_parameter("qr_required", False)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.detection_topic = str(self.get_parameter("detection_topic").value)
        self.debug_topic = str(self.get_parameter("debug_topic").value)
        self.model_task = str(self.get_parameter("model_task").value)
        self.confidence = float(self.get_parameter("confidence").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.device = str(self.get_parameter("device").value)
        self.half = as_bool(self.get_parameter("half").value)
        self.max_det = int(self.get_parameter("max_det").value)
        self.inference_rate_hz = max(0.1, float(self.get_parameter("inference_rate_hz").value))
        self.process_every_n_frames = max(1, int(self.get_parameter("process_every_n_frames").value))
        self.alignment_edge = str(self.get_parameter("alignment_edge").value).strip().lower()
        if self.alignment_edge not in {"short", "long"}:
            self.alignment_edge = "short"
        self.alignment_axis = str(self.get_parameter("alignment_axis").value).strip().lower()
        if self.alignment_axis not in {"image_x", "image_y", "x", "y", "horizontal", "vertical"}:
            self.alignment_axis = "image_y"
        self.publish_debug = as_bool(self.get_parameter("publish_debug").value)
        self.debug_jpeg_quality = int(clamp(float(self.get_parameter("debug_jpeg_quality").value), 20.0, 95.0))
        self.debug_max_width = int(max(0, float(self.get_parameter("debug_max_width").value)))
        self.qr_roi_max_candidates = max(1, int(self.get_parameter("qr_roi_max_candidates").value))
        self.qr_roi_margin_px = max(0, int(self.get_parameter("qr_roi_margin_px").value))
        self.qr_required = as_bool(self.get_parameter("qr_required").value)

        model_path = Path(str(self.get_parameter("model_path").value)).expanduser()
        if not model_path.exists() and FALLBACK_MODEL_PATH.exists():
            model_path = FALLBACK_MODEL_PATH
        if not model_path.exists():
            raise RuntimeError(f"parcel model does not exist: {model_path}")
        cuda_available = bool(torch is not None and torch.cuda.is_available())
        if model_path.suffix == ".engine" and not TENSORRT_AVAILABLE:
            if FALLBACK_MODEL_PATH.exists():
                self.get_logger().error(
                    "TensorRT engine requested, but Python TensorRT is not installed; "
                    f"falling back to CPU PyTorch model: {FALLBACK_MODEL_PATH}"
                )
                model_path = FALLBACK_MODEL_PATH
            else:
                raise RuntimeError("TensorRT engine requested, but Python TensorRT is not installed")
        if not cuda_available and model_path.suffix != ".engine":
            if self.device.lower() != "cpu":
                self.get_logger().error(
                    f"CUDA is not available in Python torch ({getattr(torch, '__version__', 'no torch')}); "
                    f"using CPU inference instead of device '{self.device}'"
                )
                self.device = "cpu"
            self.half = False
        self.model = YOLO(str(model_path), task=self.model_task)
        self.model_path = model_path

        if pyzbar is None:
            self.get_logger().warn(
                "pyzbar QR decoder is disabled"
                + (f": {PYZBAR_IMPORT_ERROR}" if PYZBAR_IMPORT_ERROR else "")
                + "; falling back to OpenCV QRCodeDetector"
            )

        self.detection_pub = self.create_publisher(String, self.detection_topic, 10)
        self.debug_pub = self.create_publisher(CompressedImage, self.debug_topic, 2)
        self.image_topics = self.camera_topic_candidates(self.image_topic)
        self.image_subs = [
            self.create_subscription(
                CompressedImage,
                topic,
                lambda msg, topic=topic: self.image_callback(msg, topic),
                5,
            )
            for topic in self.image_topics
        ]
        self.frame_count = 0
        self.latest_msg: Optional[CompressedImage] = None
        self.latest_msg_topic = ""
        self.active_image_topic = ""
        self.processing = False
        self.last_log_time = 0.0
        self.last_frame_time = 0.0
        self.last_frame_wait_log_time = 0.0
        self.last_detection_seen = False
        self.timer = self.create_timer(1.0 / self.inference_rate_hz, self.timer_callback)
        self.get_logger().info(
            f"parcel perception ready image_topics={self.image_topics} model={model_path} "
            f"task={self.model_task} align={self.alignment_edge}->{self.alignment_axis} conf={self.confidence} "
            f"device={self.device} half={self.half} rate={self.inference_rate_hz:.1f}Hz "
            f"debug={self.publish_debug} q={self.debug_jpeg_quality} maxw={self.debug_max_width} "
            f"qr_roi={self.qr_roi_max_candidates}/{self.qr_roi_margin_px}px "
            f"qr_required={self.qr_required}"
        )

    def camera_topic_candidates(self, configured_topic: str) -> list[str]:
        candidates = []
        for topic in (configured_topic, "/hk_camera/rgb/compressed", "/hik_camera/rgb/compressed"):
            topic = str(topic or "").strip()
            if topic and topic not in candidates:
                candidates.append(topic)
        return candidates

    def image_callback(self, msg: CompressedImage, topic: str):
        self.frame_count += 1
        if self.frame_count % self.process_every_n_frames != 0:
            return
        self.latest_msg = msg
        self.latest_msg_topic = topic
        self.active_image_topic = topic
        self.last_frame_time = time.monotonic()

    def timer_callback(self):
        if self.processing:
            return
        if self.latest_msg is None:
            now = time.monotonic()
            if now - self.last_frame_time > 2.0 and now - self.last_frame_wait_log_time > 2.0:
                self.last_frame_wait_log_time = now
                self.get_logger().warn(f"waiting for camera frame topics={self.image_topics}")
            return
        self.processing = True
        msg = self.latest_msg
        topic = self.latest_msg_topic
        self.latest_msg = None
        self.latest_msg_topic = ""
        try:
            self.process_image_msg(msg, topic)
        except Exception as exc:
            self.get_logger().error(f"parcel perception failed: {exc}", throttle_duration_sec=1.0)
        finally:
            self.processing = False

    def process_image_msg(self, msg: CompressedImage, topic: str):
        image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            return
        payload, debug = self.detect(image)
        payload["stamp_sec"] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1.0e-9
        payload["image_topic"] = topic
        if self.publish_debug and debug is not None:
            encode_start = time.perf_counter()
            if self.debug_max_width > 0 and debug.shape[1] > self.debug_max_width:
                scale = self.debug_max_width / float(debug.shape[1])
                debug = cv2.resize(
                    debug,
                    (self.debug_max_width, max(1, int(round(debug.shape[0] * scale)))),
                    interpolation=cv2.INTER_AREA,
                )
            ok, encoded = cv2.imencode(".jpg", debug, [int(cv2.IMWRITE_JPEG_QUALITY), self.debug_jpeg_quality])
            payload["debug_encode_ms"] = round((time.perf_counter() - encode_start) * 1000.0, 2)
            if ok:
                payload["debug_bytes"] = int(len(encoded))
                out = CompressedImage()
                out.header = msg.header
                out.format = "jpeg"
                out.data = encoded.tobytes()
                self.debug_pub.publish(out)
        self.detection_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))
        self.log_payload(payload)

    def log_payload(self, payload: dict[str, Any]):
        now = time.monotonic()
        present = bool(payload.get("present"))
        if now - self.last_log_time < 1.0 and present == self.last_detection_seen:
            return
        self.last_log_time = now
        self.last_detection_seen = present
        if present:
            self.get_logger().info(
                "parcel detected "
                f"type={payload.get('parcel_type')} conf={float(payload.get('confidence') or 0.0):.2f} "
                f"yaw={float(payload.get('yaw_error_deg') or 0.0):.1f} "
                f"size={float(payload.get('long_mm') or 0.0):.0f}x{float(payload.get('short_mm') or 0.0):.0f}mm "
                f"qr={'Y' if payload.get('qr_data') else 'N'} "
                f"qr_seen={int(payload.get('qr_seen') or 0)} source={payload.get('qr_source') or '-'} "
                f"qr_ms={float(payload.get('qr_ms') or 0.0):.1f} "
                f"infer_ms={float(payload.get('infer_ms') or 0.0):.1f} "
                f"debug_ms={float(payload.get('debug_encode_ms') or 0.0):.1f}"
            )
        else:
            self.get_logger().info(
                f"parcel not detected qr_seen={payload.get('qr_seen', 0)} "
                f"qr_ms={float(payload.get('qr_ms') or 0.0):.1f} "
                f"infer_ms={float(payload.get('infer_ms') or 0.0):.1f} "
                f"debug_ms={float(payload.get('debug_encode_ms') or 0.0):.1f}",
                throttle_duration_sec=1.0,
            )

    def detect(self, image: np.ndarray) -> tuple[dict[str, Any], np.ndarray]:
        debug = image.copy()
        predict_kwargs = {
            "source": image,
            "imgsz": self.imgsz,
            "conf": self.confidence,
            "verbose": False,
            "device": self.device,
            "max_det": self.max_det,
            "retina_masks": False,
        }
        if self.model_path.suffix != ".engine":
            predict_kwargs["half"] = bool(self.half and self.device.lower() != "cpu")
        infer_start = time.perf_counter()
        result = self.model.predict(**predict_kwargs)[0]
        infer_ms = (time.perf_counter() - infer_start) * 1000.0
        candidates = self.obb_candidates(result)
        if not candidates:
            qrs = []
            qr_ms = 0.0
            payload = self.no_detection_payload(qrs)
            payload["qr_ms"] = round(qr_ms, 2)
            payload["infer_ms"] = round(infer_ms, 2)
            return payload, debug

        qr_start = time.perf_counter()
        qrs = decode_qr_in_candidate_rois(
            image,
            candidates,
            self.qr_roi_max_candidates,
            self.qr_roi_margin_px,
        )
        qr_ms = (time.perf_counter() - qr_start) * 1000.0
        draw_qr_debug(debug, qrs)

        selected = self.select_candidate(candidates, qrs)
        if selected is None or (self.qr_required and not selected.get("qr_data")):
            payload = self.no_detection_payload(qrs)
            payload["qr_ms"] = round(qr_ms, 2)
            payload["infer_ms"] = round(infer_ms, 2)
            return payload, debug

        points = selected["points"]
        size = classify_parcel_size(selected["long_px"], selected["short_px"])
        long_yaw = float(selected["long_axis_yaw_deg"])
        short_yaw = float(selected["short_axis_yaw_deg"])
        target_edge_yaw = short_yaw if self.alignment_edge == "short" else long_yaw
        yaw_error = yaw_error_to_camera_axis(target_edge_yaw, self.alignment_axis)

        qr_data = str(selected.get("qr_data") or "")
        qr_source = str(selected.get("qr_source") or "")
        qr_polygon = selected.get("qr_polygon") or []
        destination = destination_from_qr(qr_data)
        cv2.polylines(debug, [points.astype(np.int32)], True, (0, 255, 0), 2)
        center = selected["center_px"]
        cv2.circle(debug, (int(center[0]), int(center[1])), 4, (0, 255, 255), -1)
        draw_axis_line(debug, center, long_yaw, selected["long_px"] * 0.45, (255, 0, 255), 2)
        draw_axis_line(debug, center, short_yaw, selected["short_px"] * 0.45, (255, 180, 0), 2)
        label = f"{size['label']} {self.alignment_edge}->{self.alignment_axis}={yaw_error:.1f} qr={'Y' if qr_data else 'N'}"
        cv2.putText(debug, label, (int(center[0]) - 80, int(center[1]) - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        return {
            "present": True,
            "parcel_type": int(size["type"]),
            "parcel_label": str(size["label"]),
            "long_mm": float(size["long_mm"]),
            "short_mm": float(size["short_mm"]),
            "height_mm": 75.0,
            "min_mm": float(min(size["long_mm"], size["short_mm"])),
            "confidence": float(selected["confidence"]),
            "size_score_px": float(size["score_px"]),
            "long_px": float(selected["long_px"]),
            "short_px": float(selected["short_px"]),
            "center_px": [float(center[0]), float(center[1])],
            "yaw_deg": long_yaw,
            "long_axis_yaw_deg": long_yaw,
            "short_axis_yaw_deg": short_yaw,
            "target_edge_yaw_deg": target_edge_yaw,
            "yaw_error_deg": float(yaw_error),
            "alignment_edge": self.alignment_edge,
            "alignment_axis": self.alignment_axis,
            "qr_data": qr_data,
            "qr_seen": len(qrs),
            "qr_source": qr_source,
            "qr_polygon": qr_polygon,
            "destination": destination,
            "qr_required": bool(self.qr_required),
            "source": "camera_qr_optional",
            "qr_ms": round(qr_ms, 2),
            "infer_ms": round(infer_ms, 2),
        }, debug

    def obb_candidates(self, result) -> list[dict[str, Any]]:
        candidates = []
        obb = getattr(result, "obb", None)
        if obb is None or getattr(obb, "xyxyxyxy", None) is None:
            return candidates
        points_arr = obb.xyxyxyxy.cpu().numpy()
        confs = obb.conf.cpu().numpy() if getattr(obb, "conf", None) is not None else np.ones((len(points_arr),))
        for points, conf in zip(points_arr, confs, strict=False):
            pts = points.reshape(4, 2).astype(np.float32)
            edge_lengths = [
                float(np.linalg.norm(pts[(idx + 1) % 4] - pts[idx]))
                for idx in range(4)
            ]
            long_px = max(edge_lengths)
            short_px = min(edge_lengths)
            if long_px < 10.0 or short_px < 10.0:
                continue
            long_yaw, short_yaw = edge_axis_yaws_deg(pts)
            candidates.append(
                {
                    "points": pts,
                    "confidence": float(conf),
                    "long_px": long_px,
                    "short_px": short_px,
                    "yaw_deg": long_yaw,
                    "long_axis_yaw_deg": long_yaw,
                    "short_axis_yaw_deg": short_yaw,
                    "center_px": [float(np.mean(pts[:, 0])), float(np.mean(pts[:, 1]))],
                }
            )
        return candidates

    def select_candidate(self, candidates: list[dict[str, Any]], qrs: list[dict[str, Any]]) -> Optional[dict[str, Any]]:
        qr_selected = None
        for cand in sorted(candidates, key=lambda item: item["confidence"], reverse=True):
            for qr in qrs:
                center = qr.get("center_px") or [0.0, 0.0]
                if point_in_poly((float(center[0]), float(center[1])), cand["points"]):
                    qr_selected = dict(cand)
                    qr_selected["qr_data"] = qr.get("data", "")
                    qr_selected["qr_source"] = qr.get("source", "")
                    qr_selected["qr_polygon"] = qr.get("polygon", [])
                    return qr_selected
        selected = dict(max(candidates, key=lambda item: item["confidence"]))
        selected["qr_data"] = ""
        selected["qr_source"] = ""
        selected["qr_polygon"] = []
        return selected

    def no_detection_payload(self, qrs: list[dict[str, Any]]) -> dict[str, Any]:
        return {
            "present": False,
            "qr_seen": len(qrs),
            "qr_required": bool(self.qr_required),
            "source": "camera_qr_optional",
        }


def main():
    rclpy.init()
    node = ParcelPerceptionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
