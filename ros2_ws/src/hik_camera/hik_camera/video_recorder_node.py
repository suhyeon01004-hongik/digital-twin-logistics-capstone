#!/usr/bin/env python3
from datetime import datetime
from pathlib import Path
from queue import Full, Queue
import threading

import cv2
import numpy as np
import rclpy
from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool


def float_descriptor(description, minimum, maximum, step):
    return ParameterDescriptor(
        description=description,
        floating_point_range=[
            FloatingPointRange(from_value=minimum, to_value=maximum, step=step)
        ],
    )


def int_descriptor(description, minimum, maximum, step):
    return ParameterDescriptor(
        description=description,
        integer_range=[IntegerRange(from_value=minimum, to_value=maximum, step=step)],
    )


class VideoRecorderNode(Node):
    def __init__(self):
        super().__init__("hik_video_recorder")

        self.declare_parameter("image_topic", "/hik_camera/rgb")
        self.declare_parameter(
            "record_video",
            False,
            ParameterDescriptor(description="Write subscribed image frames to a video file."),
        )
        self.declare_parameter(
            "video_path",
            "",
            ParameterDescriptor(
                description="Output video path. Empty creates ~/Videos/hik_camera_<timestamp>.mp4."
            ),
        )
        self.declare_parameter(
            "video_codec",
            "mp4v",
            ParameterDescriptor(description="FourCC codec used by OpenCV VideoWriter."),
        )
        self.declare_parameter(
            "video_fps",
            30.0,
            float_descriptor("Recorded video FPS.", 1.0, 240.0, 1.0),
        )
        self.declare_parameter(
            "queue_size",
            120,
            int_descriptor("Maximum frames buffered for the writer thread.", 1, 1000, 1),
        )

        image_topic = self.get_parameter("image_topic").value
        queue_size = int(self.get_parameter("queue_size").value)

        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.recording_publisher = self.create_publisher(Bool, "~/recording", status_qos)
        self.recording_path_publisher = self.create_publisher(String, "~/recording_path", status_qos)
        self.recording_service = self.create_service(SetBool, "~/set_recording", self.on_set_recording)
        self.subscription = self.create_subscription(Image, image_topic, self.on_image, 10)
        self.add_on_set_parameters_callback(self.on_parameters)

        self.lock = threading.Lock()
        self.frame_queue = Queue(maxsize=queue_size)
        self.writer = None
        self.writer_size = None
        self.video_output_path = None
        self.record_video = bool(self.get_parameter("record_video").value)
        self.running = True
        self.writer_thread = threading.Thread(target=self.writer_loop, daemon=True)
        self.writer_thread.start()

        self.publish_recording_status()
        self.get_logger().info(f"Recording subscriber listening on {image_topic}")

    def on_parameters(self, params):
        updates = {p.name: p.value for p in params}
        try:
            if any(name in updates for name in ("record_video", "video_path", "video_codec", "video_fps")):
                self.reconfigure_writer(bool(updates.get("record_video", self.record_video)))
            return SetParametersResult(successful=True)
        except Exception as exc:
            return SetParametersResult(successful=False, reason=str(exc))

    def on_set_recording(self, request, response):
        result = self.set_parameters(
            [Parameter("record_video", Parameter.Type.BOOL, bool(request.data))]
        )[0]
        response.success = result.successful
        response.message = "Recording started" if request.data else "Recording stopped"
        if not result.successful:
            response.message = result.reason
        return response

    def resolve_video_path(self):
        configured_path = str(self.get_parameter("video_path").value).strip()
        if configured_path:
            return Path(configured_path).expanduser()
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return Path.home() / "Videos" / f"hik_camera_{timestamp}.mp4"

    def publish_recording_status(self):
        recording_msg = Bool()
        recording_msg.data = bool(self.record_video)
        self.recording_publisher.publish(recording_msg)

        path_msg = String()
        path_msg.data = str(self.video_output_path) if self.video_output_path else ""
        self.recording_path_publisher.publish(path_msg)

    def release_writer_locked(self):
        if self.writer is not None:
            self.writer.release()
            self.writer = None
        self.writer_size = None

    def reconfigure_writer(self, enabled):
        with self.lock:
            self.release_writer_locked()
            self.record_video = enabled
            self.video_output_path = None
            while not self.frame_queue.empty():
                self.frame_queue.get_nowait()
            self.publish_recording_status()

    def ensure_writer_locked(self, frame):
        if self.writer is not None:
            return

        height, width = frame.shape[:2]
        path = self.resolve_video_path()
        path.parent.mkdir(parents=True, exist_ok=True)

        codec = str(self.get_parameter("video_codec").value)
        if len(codec) != 4:
            raise RuntimeError(f"video_codec must be a 4-character FourCC code, got '{codec}'")

        fps = float(self.get_parameter("video_fps").value)
        fourcc = cv2.VideoWriter_fourcc(*codec)
        writer = cv2.VideoWriter(str(path), fourcc, fps, (width, height), True)
        if not writer.isOpened():
            raise RuntimeError(f"Failed to open video writer: {path}")

        self.writer = writer
        self.writer_size = (width, height)
        self.video_output_path = path
        self.publish_recording_status()
        self.get_logger().info(f"Recording video to {path}")

    def image_to_bgr(self, msg):
        if msg.encoding == "rgb8":
            rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if msg.encoding == "bgr8":
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3).copy()
        if msg.encoding == "mono8":
            mono = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)
        if msg.encoding == "bayer_bggr8":
            raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            return cv2.cvtColor(raw, cv2.COLOR_BayerBG2BGR)
        if msg.encoding == "bayer_gbrg8":
            raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            return cv2.cvtColor(raw, cv2.COLOR_BayerGB2BGR)
        if msg.encoding == "bayer_grbg8":
            raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            return cv2.cvtColor(raw, cv2.COLOR_BayerGR2BGR)
        if msg.encoding == "bayer_rggb8":
            raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            return cv2.cvtColor(raw, cv2.COLOR_BayerRG2BGR)
        raise RuntimeError(f"Unsupported image encoding for recording: {msg.encoding}")

    def on_image(self, msg):
        with self.lock:
            recording = self.record_video
        if not recording:
            return

        try:
            frame = self.image_to_bgr(msg)
            self.frame_queue.put_nowait(frame.copy())
        except Full:
            self.get_logger().warn("Recorder queue is full; dropping frame", throttle_duration_sec=2.0)
        except Exception as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=2.0)

    def writer_loop(self):
        while self.running:
            frame = self.frame_queue.get()
            if frame is None:
                break
            try:
                with self.lock:
                    if not self.record_video:
                        continue
                    self.ensure_writer_locked(frame)
                    height, width = frame.shape[:2]
                    if self.writer_size != (width, height):
                        raise RuntimeError("Image size changed during recording; restart recording.")
                    self.writer.write(frame)
            except Exception as exc:
                self.get_logger().error(str(exc), throttle_duration_sec=2.0)

    def destroy_node(self):
        self.running = False
        try:
            self.frame_queue.put_nowait(None)
        except Full:
            self.frame_queue.get_nowait()
            self.frame_queue.put_nowait(None)
        if self.writer_thread.is_alive():
            self.writer_thread.join(timeout=2.0)
        with self.lock:
            self.release_writer_locked()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VideoRecorderNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
