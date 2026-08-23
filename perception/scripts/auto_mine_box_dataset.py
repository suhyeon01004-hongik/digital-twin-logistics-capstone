#!/usr/bin/env python3
import argparse
from datetime import datetime
from pathlib import Path
import random
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from ultralytics import YOLO

from path_config import DATA_ROOT, MODEL_ROOT, model_from_env

DEFAULT_MODEL = model_from_env(
    "MILEMATE_SEG_MODEL_PATH",
    MODEL_ROOT / "box_segmentation" / "best.pt",
)
DEFAULT_OUTPUT_ROOT = DATA_ROOT / "raw" / "auto_mined"


class AutoMineBoxDataset(Node):
    def __init__(self, args):
        super().__init__("auto_mine_box_dataset")
        self.args = args
        self.bridge = CvBridge()
        self.model = YOLO(str(Path(args.model).expanduser().resolve()), task="segment")
        self.session_dir = self.make_session_dir(args)
        self.last_save_time = 0.0
        self.frame_count = 0
        self.saved_positive = 0
        self.saved_negative = 0
        self.saved_review = 0
        self.random = random.Random(args.seed)

        self.write_data_yaml()
        self.subscription = self.create_subscription(
            Image,
            args.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"자동 dataset 수집 시작: topic={args.image_topic}, model={args.model}, "
            f"output={self.session_dir}, high_conf={args.high_conf}, "
            f"negative_conf={args.negative_conf}"
        )

    @staticmethod
    def make_session_dir(args):
        session = args.session or datetime.now().strftime("%Y%m%d_%H%M%S")
        session_dir = Path(args.output_root).expanduser().resolve() / session
        for split in ("train", "valid"):
            (session_dir / "images" / split).mkdir(parents=True, exist_ok=True)
            (session_dir / "labels" / split).mkdir(parents=True, exist_ok=True)
        (session_dir / "review" / "images").mkdir(parents=True, exist_ok=True)
        return session_dir

    def write_data_yaml(self):
        data_yaml = self.session_dir / "data.yaml"
        data_yaml.write_text(
            "\n".join(
                [
                    f"path: {self.session_dir}",
                    "train: images/train",
                    "val: images/valid",
                    "test: images/valid",
                    "",
                    "nc: 1",
                    "names: ['Box']",
                    "",
                ]
            ),
            encoding="utf-8",
        )

    def image_callback(self, msg):
        self.frame_count += 1
        if self.args.every_n_frames > 1 and self.frame_count % self.args.every_n_frames != 0:
            return
        if self.args.max_images > 0 and self.total_saved >= self.args.max_images:
            self.shutdown_when_done()
            return

        now = time.monotonic()
        if now - self.last_save_time < self.args.interval_sec:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            result = self.model.predict(
                source=frame,
                conf=self.args.negative_conf,
                max_det=self.args.max_det,
                verbose=False,
                retina_masks=True,
            )[0]
            labels, max_conf = self.make_segmentation_labels(result)

            if labels:
                self.save_training_sample(frame, labels, "positive")
                self.saved_positive += 1
                self.last_save_time = now
            elif max_conf < self.args.negative_conf and self.args.save_negatives:
                self.save_training_sample(frame, [], "negative")
                self.saved_negative += 1
                self.last_save_time = now
            elif self.args.save_review:
                self.save_review_image(frame)
                self.saved_review += 1
                self.last_save_time = now

            if self.args.max_images > 0 and self.total_saved >= self.args.max_images:
                self.shutdown_when_done()
        except Exception as exc:
            self.get_logger().error(f"자동 dataset 수집 실패: {exc}")

    @property
    def total_saved(self):
        return self.saved_positive + self.saved_negative

    def make_segmentation_labels(self, result):
        labels = []
        max_conf = 0.0
        if result.boxes is None or result.masks is None:
            return labels, max_conf

        confs = result.boxes.conf.detach().cpu().tolist()
        classes = result.boxes.cls.detach().cpu().tolist()
        polygons = result.masks.xyn
        for conf, cls, polygon in zip(confs, classes, polygons, strict=False):
            conf = float(conf)
            max_conf = max(max_conf, conf)
            if conf < self.args.high_conf:
                continue
            points = []
            for x, y in polygon:
                x = min(max(float(x), 0.0), 1.0)
                y = min(max(float(y), 0.0), 1.0)
                points.extend((x, y))
            if len(points) < 6:
                continue
            coords = " ".join(f"{value:.6f}" for value in points)
            labels.append(f"{int(cls)} {coords}")
        return labels, max_conf

    def choose_split(self):
        return "valid" if self.random.random() < self.args.valid_ratio else "train"

    def save_training_sample(self, frame, labels, kind):
        split = self.choose_split()
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        index = self.total_saved + 1
        stem = f"{kind}_{timestamp}_{index:06d}"
        image_path = self.session_dir / "images" / split / f"{stem}.jpg"
        label_path = self.session_dir / "labels" / split / f"{stem}.txt"
        cv2.imwrite(str(image_path), frame, [cv2.IMWRITE_JPEG_QUALITY, self.args.jpeg_quality])
        label_path.write_text("\n".join(labels) + ("\n" if labels else ""), encoding="utf-8")
        self.get_logger().info(
            f"저장: {kind} split={split} total={self.total_saved + 1} "
            f"pos={self.saved_positive} neg={self.saved_negative} file={image_path.name}"
        )

    def save_review_image(self, frame):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        path = self.session_dir / "review" / "images" / f"review_{timestamp}_{self.saved_review + 1:06d}.jpg"
        cv2.imwrite(str(path), frame, [cv2.IMWRITE_JPEG_QUALITY, self.args.jpeg_quality])
        self.get_logger().info(f"검수 후보 저장: {path.name}")

    def shutdown_when_done(self):
        self.get_logger().info(
            f"수집 완료: positive={self.saved_positive}, negative={self.saved_negative}, "
            f"review={self.saved_review}, dataset={self.session_dir}"
        )
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description="YOLO 결과 기반 자동 dataset 수집기")
    parser.add_argument("--image-topic", default="/hk_camera/rgb")
    parser.add_argument("--model", default=str(DEFAULT_MODEL))
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT_ROOT))
    parser.add_argument("--session", default="")
    parser.add_argument("--high-conf", type=float, default=0.85)
    parser.add_argument("--negative-conf", type=float, default=0.25)
    parser.add_argument("--interval-sec", type=float, default=0.5)
    parser.add_argument("--every-n-frames", type=int, default=1)
    parser.add_argument("--max-images", type=int, default=0)
    parser.add_argument("--max-det", type=int, default=5)
    parser.add_argument("--valid-ratio", type=float, default=0.1)
    parser.add_argument("--jpeg-quality", type=int, default=95)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--no-negatives", dest="save_negatives", action="store_false")
    parser.add_argument("--no-review", dest="save_review", action="store_false")
    parser.set_defaults(save_negatives=True, save_review=True)
    args, ros_args = parser.parse_known_args()

    args.high_conf = min(max(args.high_conf, 0.0), 1.0)
    args.negative_conf = min(max(args.negative_conf, 0.0), args.high_conf)
    args.interval_sec = max(0.0, args.interval_sec)
    args.every_n_frames = max(1, args.every_n_frames)
    args.valid_ratio = min(max(args.valid_ratio, 0.0), 0.5)
    args.jpeg_quality = min(max(args.jpeg_quality, 1), 100)

    rclpy.init(args=ros_args)
    node = AutoMineBoxDataset(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.get_logger().info(
                f"수집 종료: positive={node.saved_positive}, negative={node.saved_negative}, "
                f"review={node.saved_review}"
            )
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
