#!/usr/bin/env python3
import argparse
from datetime import datetime
from pathlib import Path
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from path_config import DATA_ROOT

DEFAULT_OUTPUT_ROOT = DATA_ROOT / "raw" / "captures"


class BoxImageCapture(Node):
    def __init__(self, args):
        super().__init__("box_image_capture")
        self.args = args
        self.bridge = CvBridge()
        self.saved_count = 0
        self.frame_count = 0
        self.last_save_time = 0.0
        self.output_dir = self.make_output_dir(args)

        self.subscription = self.create_subscription(
            Image,
            args.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"이미지 수집 시작: topic={args.image_topic}, output={self.output_dir}, "
            f"interval={args.interval_sec}s, max_images={args.max_images}"
        )

    @staticmethod
    def make_output_dir(args):
        output_root = Path(args.output_root).expanduser().resolve()
        session = args.session or datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = output_root / session / "images"
        output_dir.mkdir(parents=True, exist_ok=True)
        return output_dir

    def should_save(self):
        if self.args.max_images > 0 and self.saved_count >= self.args.max_images:
            return False
        if self.args.every_n_frames > 1 and self.frame_count % self.args.every_n_frames != 0:
            return False
        now = time.monotonic()
        if now - self.last_save_time < self.args.interval_sec:
            return False
        return True

    def image_callback(self, msg):
        self.frame_count += 1
        if not self.should_save():
            if self.args.max_images > 0 and self.saved_count >= self.args.max_images:
                self.shutdown_when_done()
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if self.args.resize_width > 0:
                frame = self.resize_by_width(frame, self.args.resize_width)

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            filename = self.output_dir / f"box_{timestamp}_{self.saved_count + 1:06d}.jpg"
            cv2.imwrite(str(filename), frame, [cv2.IMWRITE_JPEG_QUALITY, self.args.jpeg_quality])

            self.saved_count += 1
            self.last_save_time = time.monotonic()
            self.get_logger().info(f"저장됨 [{self.saved_count}]: {filename}")

            if self.args.max_images > 0 and self.saved_count >= self.args.max_images:
                self.shutdown_when_done()
        except Exception as exc:
            self.get_logger().error(f"이미지 저장 실패: {exc}")

    @staticmethod
    def resize_by_width(frame, width):
        h, w = frame.shape[:2]
        if w <= 0 or width == w:
            return frame
        scale = width / float(w)
        height = max(1, int(round(h * scale)))
        return cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)

    def shutdown_when_done(self):
        self.get_logger().info(f"목표 이미지 수집 완료: {self.saved_count}장")
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description="박스 인식 데이터셋용 이미지 수집기")
    parser.add_argument("--image-topic", default="/hk_camera/rgb")
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT_ROOT))
    parser.add_argument("--session", default="", help="저장 세션 이름. 비우면 현재 시간이 사용된다.")
    parser.add_argument("--interval-sec", type=float, default=0.5, help="이미지 저장 최소 시간 간격.")
    parser.add_argument("--every-n-frames", type=int, default=1, help="N 프레임마다 저장 후보로 사용.")
    parser.add_argument("--max-images", type=int, default=0, help="0이면 Ctrl+C 전까지 계속 저장.")
    parser.add_argument("--resize-width", type=int, default=0, help="0이면 원본 크기 저장.")
    parser.add_argument("--jpeg-quality", type=int, default=95)
    args, ros_args = parser.parse_known_args()

    args.every_n_frames = max(1, args.every_n_frames)
    args.interval_sec = max(0.0, args.interval_sec)
    args.jpeg_quality = max(1, min(100, args.jpeg_quality))

    rclpy.init(args=ros_args)
    node = BoxImageCapture(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.get_logger().info(f"수집 종료: 총 {node.saved_count}장 저장")
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
