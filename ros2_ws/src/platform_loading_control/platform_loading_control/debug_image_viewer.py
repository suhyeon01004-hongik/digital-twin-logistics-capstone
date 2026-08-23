#!/usr/bin/env python3
from __future__ import annotations

import os

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class DebugImageViewer(Node):
    def __init__(self):
        super().__init__("platform_debug_image_viewer")
        self.declare_parameter("image_topic", "/platform/parcel_detection/debug_image/compressed")
        self.declare_parameter("window_name", "YOLO Parcel Detection")
        self.declare_parameter("resize_width", 960)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.window_name = str(self.get_parameter("window_name").value)
        self.resize_width = int(self.get_parameter("resize_width").value)
        self.gui_available = bool(os.environ.get("DISPLAY"))

        if self.gui_available:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        else:
            self.get_logger().warn("DISPLAY is not set; YOLO debug image window will not open")

        self.create_subscription(CompressedImage, self.image_topic, self.image_callback, 5)
        self.get_logger().info(f"YOLO debug viewer listening: {self.image_topic}")

    def image_callback(self, msg: CompressedImage):
        if not self.gui_available:
            return
        image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            return
        if self.resize_width > 0 and image.shape[1] > self.resize_width:
            scale = self.resize_width / float(image.shape[1])
            image = cv2.resize(image, (self.resize_width, max(1, int(image.shape[0] * scale))))
        cv2.imshow(self.window_name, image)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            self.get_logger().info("YOLO debug viewer close requested")
            rclpy.shutdown()

    def destroy_node(self):
        if self.gui_available:
            try:
                cv2.destroyWindow(self.window_name)
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = DebugImageViewer()
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
