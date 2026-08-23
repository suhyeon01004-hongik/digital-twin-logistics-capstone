#!/usr/bin/env python3
# ruff: noqa: F403,F405

import os
import sys
import threading
from ctypes import byref, c_ubyte, cast, memset, sizeof, string_at, POINTER
from pathlib import Path

import numpy as np
import cv2
import rclpy
from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image


MVS_IMPORT = Path(
    os.environ.get("MILEMATE_MVS_IMPORT", "/opt/MVS/Samples/64/Python/MvImport")
).expanduser()
if str(MVS_IMPORT) not in sys.path:
    sys.path.append(str(MVS_IMPORT))

try:
    from MvCameraControl_class import *  # noqa: F401,F403
except Exception as exc:  # pragma: no cover - depends on host MVS install
    raise RuntimeError(
        "Hikrobot MVS Python bindings were not found. Install MVS SDK first."
    ) from exc


MONO_PIXEL_FORMATS = {
    PixelType_Gvsp_Mono8,
    PixelType_Gvsp_Mono10,
    PixelType_Gvsp_Mono10_Packed,
    PixelType_Gvsp_Mono12,
    PixelType_Gvsp_Mono12_Packed,
    PixelType_Gvsp_Mono14,
    PixelType_Gvsp_Mono16,
}

HB_PIXEL_FORMATS = {
    PixelType_Gvsp_HB_Mono8,
    PixelType_Gvsp_HB_Mono10,
    PixelType_Gvsp_HB_Mono10_Packed,
    PixelType_Gvsp_HB_Mono12,
    PixelType_Gvsp_HB_Mono12_Packed,
    PixelType_Gvsp_HB_Mono16,
    PixelType_Gvsp_HB_RGB8_Packed,
    PixelType_Gvsp_HB_BGR8_Packed,
    PixelType_Gvsp_HB_RGBA8_Packed,
    PixelType_Gvsp_HB_BGRA8_Packed,
    PixelType_Gvsp_HB_RGB16_Packed,
    PixelType_Gvsp_HB_BGR16_Packed,
    PixelType_Gvsp_HB_RGBA16_Packed,
    PixelType_Gvsp_HB_BGRA16_Packed,
    PixelType_Gvsp_HB_YUV422_Packed,
    PixelType_Gvsp_HB_YUV422_YUYV_Packed,
    PixelType_Gvsp_HB_BayerGR8,
    PixelType_Gvsp_HB_BayerRG8,
    PixelType_Gvsp_HB_BayerGB8,
    PixelType_Gvsp_HB_BayerBG8,
    PixelType_Gvsp_HB_BayerRBGG8,
    PixelType_Gvsp_HB_BayerGB10,
    PixelType_Gvsp_HB_BayerGB10_Packed,
    PixelType_Gvsp_HB_BayerBG10,
    PixelType_Gvsp_HB_BayerBG10_Packed,
    PixelType_Gvsp_HB_BayerRG10,
    PixelType_Gvsp_HB_BayerRG10_Packed,
    PixelType_Gvsp_HB_BayerGR10,
    PixelType_Gvsp_HB_BayerGR10_Packed,
    PixelType_Gvsp_HB_BayerGB12,
    PixelType_Gvsp_HB_BayerGB12_Packed,
    PixelType_Gvsp_HB_BayerBG12,
    PixelType_Gvsp_HB_BayerBG12_Packed,
    PixelType_Gvsp_HB_BayerRG12,
    PixelType_Gvsp_HB_BayerRG12_Packed,
    PixelType_Gvsp_HB_BayerGR12,
    PixelType_Gvsp_HB_BayerGR12_Packed,
}

BAYER8_ENCODINGS = {
    PixelType_Gvsp_BayerGR8: "bayer_grbg8",
    PixelType_Gvsp_BayerRG8: "bayer_rggb8",
    PixelType_Gvsp_BayerGB8: "bayer_gbrg8",
    PixelType_Gvsp_BayerBG8: "bayer_bggr8",
}

CV_BAYER_TO_BGR = {
    "bayer_grbg8": cv2.COLOR_BAYER_GR2BGR,
    "bayer_rggb8": cv2.COLOR_BAYER_RG2BGR,
    "bayer_gbrg8": cv2.COLOR_BAYER_GB2BGR,
    "bayer_bggr8": cv2.COLOR_BAYER_BG2BGR,
}


def ok(ret):
    return ret == 0


def hexret(ret):
    return f"0x{ret:08x}"


def float_descriptor(description, minimum, maximum, step):
    range_kwargs = {"from_value": minimum, "to_value": maximum}
    if step is not None:
        range_kwargs["step"] = step
    return ParameterDescriptor(
        description=description,
        floating_point_range=[FloatingPointRange(**range_kwargs)],
    )


def int_descriptor(description, minimum, maximum, step):
    return ParameterDescriptor(
        description=description,
        integer_range=[IntegerRange(from_value=minimum, to_value=maximum, step=step)],
    )


class HikrobotCameraNode(Node):
    def __init__(self):
        super().__init__("hik_camera")

        self.declare_parameter("frame_id", "hik_camera")
        self.declare_parameter("image_topic", "/hik_camera/rgb")
        self.declare_parameter("compressed_image_topic", "/hik_camera/rgb/compressed")
        self.declare_parameter(
            "publish_raw",
            False,
            ParameterDescriptor(description="Publish sensor_msgs/Image frames."),
        )
        self.declare_parameter(
            "publish_compressed",
            True,
            ParameterDescriptor(description="Publish sensor_msgs/CompressedImage JPEG frames."),
        )
        self.declare_parameter(
            "jpeg_quality",
            80,
            int_descriptor("JPEG compression quality for compressed images.", 1, 100, 1),
        )
        self.declare_parameter(
            "device_index",
            0,
            int_descriptor("MVS device index from enumeration order.", 0, 32, 1),
        )
        self.declare_parameter(
            "exposure_auto",
            True,
            ParameterDescriptor(description="Enable camera ExposureAuto=Continuous."),
        )
        self.declare_parameter(
            "exposure_time",
            4333.0,
            float_descriptor("Manual exposure time in microseconds.", 100.0, 50000.0, None),
        )
        self.declare_parameter(
            "gain_auto",
            False,
            ParameterDescriptor(description="Enable camera GainAuto=Continuous."),
        )
        self.declare_parameter(
            "gain",
            0.0,
            float_descriptor("Manual analog gain.", 0.0, 30.0, 0.1),
        )
        self.declare_parameter(
            "frame_rate_enable",
            True,
            ParameterDescriptor(description="Enable AcquisitionFrameRate control."),
        )
        self.declare_parameter(
            "frame_rate",
            30.0,
            float_descriptor("Acquisition frame rate in Hz.", 1.0, 240.0, 1.0),
        )
        self.declare_parameter(
            "timeout_ms",
            1000,
            int_descriptor("Frame grab timeout in milliseconds.", 10, 5000, 10),
        )
        self.declare_parameter(
            "publish_color",
            True,
            ParameterDescriptor(
                description="Convert Bayer/mono frames to rgb8 before publishing. False publishes native fast Bayer/mono frames."
            ),
        )

        self.frame_id = self.get_parameter("frame_id").value
        image_topic = self.get_parameter("image_topic").value
        compressed_image_topic = self.get_parameter("compressed_image_topic").value
        self.timeout_ms = int(self.get_parameter("timeout_ms").value)
        self.publish_color = bool(self.get_parameter("publish_color").value)
        self.publish_raw = bool(self.get_parameter("publish_raw").value)
        self.publish_compressed = bool(self.get_parameter("publish_compressed").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.publisher = self.create_publisher(Image, image_topic, 10)
        self.compressed_publisher = self.create_publisher(
            CompressedImage, compressed_image_topic, 10
        )

        self.cam = None
        self.cam_lock = threading.RLock()
        self.grabbing = False
        self.running = False
        self.grab_thread = None
        self.param_lock = threading.Lock()

        MvCamera.MV_CC_Initialize()
        self.open_camera()
        self.apply_all_camera_parameters()
        self.start_grabbing()
        self.add_on_set_parameters_callback(self.on_parameters)
        self.hide_internal_parameters()

        self.running = True
        self.grab_thread = threading.Thread(target=self.grab_loop, daemon=True)
        self.grab_thread.start()
        topics = []
        if self.publish_raw:
            topics.append(image_topic)
        if self.publish_compressed:
            topics.append(compressed_image_topic)
        self.get_logger().info(
            f"Publishing Hikrobot camera images on {', '.join(topics) if topics else 'no topics'}"
        )

    def hide_internal_parameters(self):
        visible = {
            "exposure_auto",
            "exposure_time",
            "frame_rate",
            "frame_rate_enable",
            "gain",
            "gain_auto",
            "jpeg_quality",
        }
        for name in list(self._parameters.keys()):
            if name in visible or name == "use_sim_time":
                continue
            try:
                self.undeclare_parameter(name)
            except Exception:
                pass

    def open_camera(self):
        device_list = MV_CC_DEVICE_INFO_LIST()
        tlayer_type = (
            MV_GIGE_DEVICE
            | MV_USB_DEVICE
            | MV_GENTL_GIGE_DEVICE
            | MV_GENTL_CAMERALINK_DEVICE
            | MV_GENTL_CXP_DEVICE
            | MV_GENTL_XOF_DEVICE
        )
        ret = MvCamera.MV_CC_EnumDevices(tlayer_type, device_list)
        if not ok(ret):
            raise RuntimeError(f"Enum devices failed: {hexret(ret)}")
        if device_list.nDeviceNum == 0:
            raise RuntimeError("No Hikrobot/MVS camera was found.")

        device_index = int(self.get_parameter("device_index").value)
        if device_index >= device_list.nDeviceNum:
            raise RuntimeError(
                f"device_index={device_index} out of range; found {device_list.nDeviceNum} devices"
            )

        dev_info = cast(device_list.pDeviceInfo[device_index], POINTER(MV_CC_DEVICE_INFO)).contents
        self.log_device_info(dev_info, device_index)

        self.cam = MvCamera()
        ret = self.cam.MV_CC_CreateHandle(dev_info)
        if not ok(ret):
            raise RuntimeError(f"Create handle failed: {hexret(ret)}")

        ret = self.cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
        if not ok(ret):
            self.cam.MV_CC_DestroyHandle()
            raise RuntimeError(f"Open device failed: {hexret(ret)}")

        self.set_enum_string("TriggerMode", "Off", warn_only=True)

    def start_grabbing(self):
        ret = self.cam.MV_CC_StartGrabbing()
        if not ok(ret):
            self.cam.MV_CC_CloseDevice()
            self.cam.MV_CC_DestroyHandle()
            raise RuntimeError(f"Start grabbing failed: {hexret(ret)}")
        self.grabbing = True

    def stop_grabbing(self):
        if self.grabbing:
            self.cam.MV_CC_StopGrabbing()
            self.grabbing = False

    def restart_grabbing_around(self, action):
        with self.cam_lock:
            was_grabbing = self.grabbing
            if was_grabbing:
                self.stop_grabbing()
            try:
                action()
            finally:
                if was_grabbing:
                    self.start_grabbing()

    def log_device_info(self, dev_info, index):
        if dev_info.nTLayerType == MV_USB_DEVICE:
            usb = dev_info.SpecialInfo.stUsb3VInfo
            model = bytes(usb.chModelName).split(b"\0", 1)[0].decode(errors="ignore")
            serial = bytes(usb.chSerialNumber).split(b"\0", 1)[0].decode(errors="ignore")
            self.get_logger().info(f"Using USB camera [{index}]: model={model}, serial={serial}")
        elif dev_info.nTLayerType in (MV_GIGE_DEVICE, MV_GENTL_GIGE_DEVICE):
            gige = dev_info.SpecialInfo.stGigEInfo
            model = bytes(gige.chModelName).split(b"\0", 1)[0].decode(errors="ignore")
            self.get_logger().info(f"Using GigE camera [{index}]: model={model}")
        else:
            self.get_logger().info(f"Using MVS camera [{index}], transport={dev_info.nTLayerType}")

    def set_enum_string(self, name, value, warn_only=False):
        ret = self.cam.MV_CC_SetEnumValueByString(name, value)
        if not ok(ret):
            message = f"Set {name}={value} failed: {hexret(ret)}"
            if warn_only:
                self.get_logger().warn(message)
            else:
                raise RuntimeError(message)

    def set_enum(self, name, value, warn_only=False):
        ret = self.cam.MV_CC_SetEnumValue(name, int(value))
        if not ok(ret):
            message = f"Set {name}={value} failed: {hexret(ret)}"
            if warn_only:
                self.get_logger().warn(message)
            else:
                raise RuntimeError(message)

    def set_float(self, name, value, warn_only=False):
        ret = self.cam.MV_CC_SetFloatValue(name, float(value))
        if not ok(ret):
            message = f"Set {name}={value} failed: {hexret(ret)}"
            if warn_only:
                self.get_logger().warn(message)
            else:
                raise RuntimeError(message)

    def get_float(self, name, warn_only=False):
        value = MVCC_FLOATVALUE()
        memset(byref(value), 0, sizeof(MVCC_FLOATVALUE))
        ret = self.cam.MV_CC_GetFloatValue(name, value)
        if ok(ret):
            return value.fCurValue

        message = f"Get {name} failed: {hexret(ret)}"
        if warn_only:
            self.get_logger().warn(message)
            return None
        raise RuntimeError(message)

    def set_bool(self, name, value, warn_only=False):
        ret = self.cam.MV_CC_SetBoolValue(name, bool(value))
        if not ok(ret):
            message = f"Set {name}={value} failed: {hexret(ret)}"
            if warn_only:
                self.get_logger().warn(message)
            else:
                raise RuntimeError(message)

    def apply_all_camera_parameters(self):
        with self.param_lock:
            self.restart_grabbing_around(
                lambda: (
                    self.apply_exposure_unlocked(
                        bool(self.get_parameter("exposure_auto").value),
                        float(self.get_parameter("exposure_time").value),
                    ),
                    self.apply_gain_unlocked(
                        bool(self.get_parameter("gain_auto").value),
                        float(self.get_parameter("gain").value),
                    ),
                    self.apply_frame_rate_unlocked(
                        bool(self.get_parameter("frame_rate_enable").value),
                        float(self.get_parameter("frame_rate").value),
                    ),
                )
            )

    def apply_exposure_unlocked(self, auto, exposure_time):
        self.set_enum("ExposureAuto", 2 if auto else 0)
        if not auto:
            self.set_float("ExposureTime", exposure_time)
            actual = self.get_float("ExposureTime", warn_only=True)
            if actual is not None:
                self.get_logger().info(
                    f"ExposureTime requested={exposure_time:.2f}us actual={actual:.2f}us"
                )

    def apply_gain_unlocked(self, auto, gain):
        self.set_enum("GainAuto", 2 if auto else 0)
        if not auto:
            self.set_float("Gain", gain)
            actual = self.get_float("Gain", warn_only=True)
            if actual is not None:
                self.get_logger().info(f"Gain requested={gain:.2f} actual={actual:.2f}")

    def apply_frame_rate_unlocked(self, enabled, frame_rate):
        self.set_bool("AcquisitionFrameRateEnable", enabled)
        if enabled:
            self.set_float("AcquisitionFrameRate", frame_rate)
            actual = self.get_float("AcquisitionFrameRate", warn_only=True)
            if actual is not None:
                self.get_logger().info(
                    f"AcquisitionFrameRate requested={frame_rate:.2f} actual={actual:.2f}"
                )

    def on_parameters(self, params):
        updates = {p.name: p.value for p in params}
        try:
            with self.param_lock:
                if "timeout_ms" in updates:
                    self.timeout_ms = int(updates["timeout_ms"])
                if "frame_id" in updates:
                    self.frame_id = str(updates["frame_id"])
                if "publish_color" in updates:
                    self.publish_color = bool(updates["publish_color"])
                if "publish_raw" in updates:
                    self.publish_raw = bool(updates["publish_raw"])
                if "publish_compressed" in updates:
                    self.publish_compressed = bool(updates["publish_compressed"])
                if "jpeg_quality" in updates:
                    self.jpeg_quality = int(updates["jpeg_quality"])
                exposure_auto = bool(updates.get("exposure_auto", self.get_parameter("exposure_auto").value))
                exposure_time = float(updates.get("exposure_time", self.get_parameter("exposure_time").value))
                gain_auto = bool(updates.get("gain_auto", self.get_parameter("gain_auto").value))
                gain = float(updates.get("gain", self.get_parameter("gain").value))
                frame_rate_enable = bool(
                    updates.get("frame_rate_enable", self.get_parameter("frame_rate_enable").value)
                )
                frame_rate = float(updates.get("frame_rate", self.get_parameter("frame_rate").value))

                def apply_updates():
                    if "exposure_auto" in updates or "exposure_time" in updates:
                        self.apply_exposure_unlocked(exposure_auto, exposure_time)
                    if "gain_auto" in updates or "gain" in updates:
                        self.apply_gain_unlocked(gain_auto, gain)
                    if "frame_rate_enable" in updates or "frame_rate" in updates:
                        self.apply_frame_rate_unlocked(frame_rate_enable, frame_rate)

                if any(
                    name in updates
                    for name in (
                        "exposure_auto",
                        "exposure_time",
                        "gain_auto",
                        "gain",
                        "frame_rate_enable",
                        "frame_rate",
                    )
                ):
                    self.restart_grabbing_around(apply_updates)

            return SetParametersResult(successful=True)
        except Exception as exc:
            return SetParametersResult(successful=False, reason=str(exc))

    def convert_frame(self, frame):
        width = frame.stFrameInfo.nWidth
        height = frame.stFrameInfo.nHeight
        src_pixel = frame.stFrameInfo.enPixelType

        if not self.publish_color:
            if src_pixel in BAYER8_ENCODINGS:
                data_len = width * height
                image_data = string_at(frame.pBufAddr, data_len)
                return image_data, BAYER8_ENCODINGS[src_pixel], width
            if src_pixel == PixelType_Gvsp_Mono8:
                data_len = width * height
                image_data = string_at(frame.pBufAddr, data_len)
                return image_data, "mono8", width
            if src_pixel == PixelType_Gvsp_RGB8_Packed:
                data_len = width * height * 3
                image_data = string_at(frame.pBufAddr, data_len)
                return image_data, "rgb8", width * 3
            if src_pixel == PixelType_Gvsp_BGR8_Packed:
                data_len = width * height * 3
                image_data = string_at(frame.pBufAddr, data_len)
                return image_data, "bgr8", width * 3

        decode_buffer = None
        src_data = frame.pBufAddr
        src_len = frame.stFrameInfo.nFrameLen

        if src_pixel in HB_PIXEL_FORMATS:
            decode_param = MV_CC_HB_DECODE_PARAM()
            decode_len = width * height * 3
            decode_buffer = (c_ubyte * decode_len)()
            decode_param.pSrcBuf = frame.pBufAddr
            decode_param.nSrcLen = frame.stFrameInfo.nFrameLen
            decode_param.pDstBuf = decode_buffer
            decode_param.nDstBufSize = decode_len
            ret = self.cam.MV_CC_HBDecode(decode_param)
            if not ok(ret):
                raise RuntimeError(f"HB decode failed: {hexret(ret)}")
            src_data = decode_param.pDstBuf
            src_len = decode_param.nDstBufLen
            src_pixel = decode_param.enDstPixelType

        is_mono = src_pixel in MONO_PIXEL_FORMATS
        dst_pixel = PixelType_Gvsp_Mono8 if is_mono else PixelType_Gvsp_RGB8_Packed
        channels = 1 if is_mono else 3
        dst_len = width * height * channels
        dst_buffer = (c_ubyte * dst_len)()

        convert_param = MV_CC_PIXEL_CONVERT_PARAM_EX()
        memset(byref(convert_param), 0, sizeof(convert_param))
        convert_param.nWidth = width
        convert_param.nHeight = height
        convert_param.pSrcData = src_data
        convert_param.nSrcDataLen = src_len
        convert_param.enSrcPixelType = src_pixel
        convert_param.enDstPixelType = dst_pixel
        convert_param.pDstBuffer = dst_buffer
        convert_param.nDstBufferSize = dst_len

        ret = self.cam.MV_CC_ConvertPixelTypeEx(convert_param)
        if not ok(ret):
            raise RuntimeError(f"Pixel conversion failed: {hexret(ret)}")

        if is_mono:
            mono = np.frombuffer(dst_buffer, dtype=np.uint8, count=dst_len).reshape(height, width)
            image = np.repeat(mono[:, :, np.newaxis], 3, axis=2)
            encoding = "rgb8"
            step = width * 3
        else:
            image = np.frombuffer(dst_buffer, dtype=np.uint8, count=dst_len).reshape(height, width, 3)
            encoding = "rgb8"
            step = width * 3

        # Copy out of the ctypes buffer before the next SDK call reuses memory.
        return image.tobytes(), encoding, step

    def encode_compressed_image(self, image_data, encoding, width, height):
        if encoding == "rgb8":
            rgb = np.frombuffer(image_data, dtype=np.uint8).reshape(height, width, 3)
            image = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        elif encoding == "bgr8":
            image = np.frombuffer(image_data, dtype=np.uint8).reshape(height, width, 3)
        elif encoding == "mono8":
            image = np.frombuffer(image_data, dtype=np.uint8).reshape(height, width)
        elif encoding in CV_BAYER_TO_BGR:
            bayer = np.frombuffer(image_data, dtype=np.uint8).reshape(height, width)
            image = cv2.cvtColor(bayer, CV_BAYER_TO_BGR[encoding])
        else:
            raise RuntimeError(f"Cannot JPEG-compress unsupported encoding: {encoding}")

        quality = max(1, min(100, int(self.jpeg_quality)))
        ok_encode, encoded = cv2.imencode(
            ".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        )
        if not ok_encode:
            raise RuntimeError("JPEG compression failed")
        return encoded.tobytes()

    def grab_loop(self):
        while rclpy.ok() and self.running:
            frame = MV_FRAME_OUT()
            memset(byref(frame), 0, sizeof(frame))
            with self.cam_lock:
                ret = self.cam.MV_CC_GetImageBuffer(frame, self.timeout_ms)
                if not ok(ret):
                    self.get_logger().warn(f"No camera frame: {hexret(ret)}", throttle_duration_sec=2.0)
                    continue

                try:
                    image_data, encoding, step = self.convert_frame(frame)
                finally:
                    self.cam.MV_CC_FreeImageBuffer(frame)

            if not ok(ret):
                self.get_logger().warn(f"No camera frame: {hexret(ret)}", throttle_duration_sec=2.0)
                continue

            try:
                if not rclpy.ok() or not self.running:
                    break
                stamp = self.get_clock().now().to_msg()
                height = frame.stFrameInfo.nHeight
                width = frame.stFrameInfo.nWidth

                if self.publish_raw:
                    msg = Image()
                    msg.header.stamp = stamp
                    msg.header.frame_id = self.frame_id
                    msg.height = height
                    msg.width = width
                    msg.encoding = encoding
                    msg.is_bigendian = False
                    msg.step = step
                    msg.data = image_data
                    self.publisher.publish(msg)

                if self.publish_compressed:
                    compressed_msg = CompressedImage()
                    compressed_msg.header.stamp = stamp
                    compressed_msg.header.frame_id = self.frame_id
                    compressed_msg.format = "jpeg"
                    compressed_msg.data = self.encode_compressed_image(
                        image_data, encoding, width, height
                    )
                    self.compressed_publisher.publish(compressed_msg)
            except Exception as exc:
                if not rclpy.ok() or not self.running:
                    break
                self.get_logger().error(str(exc), throttle_duration_sec=2.0)

    def destroy_node(self):
        self.running = False
        if self.grab_thread and self.grab_thread.is_alive():
            self.grab_thread.join(timeout=2.0)
        if self.cam is not None:
            with self.cam_lock:
                self.stop_grabbing()
                self.cam.MV_CC_CloseDevice()
                self.cam.MV_CC_DestroyHandle()
                self.cam = None
        MvCamera.MV_CC_Finalize()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HikrobotCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
