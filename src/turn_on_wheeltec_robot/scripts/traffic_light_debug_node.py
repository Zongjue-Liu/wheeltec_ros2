#!/usr/bin/env python3
import argparse
import ctypes
import os
from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


@dataclass
class GreenDecision:
    raw_state: str
    stable_state: str
    green_pixels: int
    bright_green_pixels: int
    green_ratio: float
    bright_green_ratio: float
    mean_green_v: float


@dataclass
class Detection:
    xyxy: tuple[float, float, float, float]
    conf: float
    cls_id: int


CLASS_NAMES = {
    0: "traffic_light",
    1: "stop_sign",
    2: "slow_sign",
    3: "pedestrian",
}

CLASS_BOX_COLORS = {
    0: (0, 255, 255),
    1: (0, 0, 255),
    2: (255, 0, 255),
    3: (255, 180, 0),
}


def str_to_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    value = str(value).strip().lower()
    return value in ("1", "true", "yes", "on")


class UltralyticsDetector:
    def __init__(self, model_path: str, conf: float):
        from ultralytics import YOLO

        self.model = YOLO(model_path)
        self.conf = conf

    def predict(self, frame: np.ndarray) -> list[Detection]:
        detections = []
        results = self.model.predict(frame, imgsz=640, conf=self.conf, verbose=False)
        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                cls_id = int(box.cls[0]) if box.cls is not None else 0
                detections.append(
                    Detection(
                        xyxy=tuple(float(v) for v in box.xyxy[0].tolist()),
                        conf=float(box.conf[0]),
                        cls_id=cls_id,
                    )
                )
        return detections


class CudaRuntime:
    HOST_TO_DEVICE = 1
    DEVICE_TO_HOST = 2

    def __init__(self):
        lib = None
        for name in ("libcudart.so", "libcudart.so.12", "libcudart.so.12.0"):
            try:
                lib = ctypes.CDLL(name)
                break
            except OSError:
                continue
        if lib is None:
            raise RuntimeError("could not load libcudart.so")

        self.lib = lib
        self.lib.cudaMalloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]
        self.lib.cudaMalloc.restype = ctypes.c_int
        self.lib.cudaFree.argtypes = [ctypes.c_void_p]
        self.lib.cudaFree.restype = ctypes.c_int
        self.lib.cudaMemcpyAsync.argtypes = [
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_size_t,
            ctypes.c_int,
            ctypes.c_void_p,
        ]
        self.lib.cudaMemcpyAsync.restype = ctypes.c_int
        self.lib.cudaStreamCreate.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
        self.lib.cudaStreamCreate.restype = ctypes.c_int
        self.lib.cudaStreamSynchronize.argtypes = [ctypes.c_void_p]
        self.lib.cudaStreamSynchronize.restype = ctypes.c_int
        self.lib.cudaStreamDestroy.argtypes = [ctypes.c_void_p]
        self.lib.cudaStreamDestroy.restype = ctypes.c_int

    def _check(self, code: int, action: str):
        if code != 0:
            raise RuntimeError(f"{action} failed with CUDA error code {code}")

    def malloc(self, nbytes: int) -> ctypes.c_void_p:
        ptr = ctypes.c_void_p()
        self._check(self.lib.cudaMalloc(ctypes.byref(ptr), nbytes), "cudaMalloc")
        return ptr

    def free(self, ptr: ctypes.c_void_p):
        if ptr and ptr.value:
            self._check(self.lib.cudaFree(ptr), "cudaFree")

    def stream_create(self) -> ctypes.c_void_p:
        stream = ctypes.c_void_p()
        self._check(self.lib.cudaStreamCreate(ctypes.byref(stream)), "cudaStreamCreate")
        return stream

    def stream_destroy(self, stream: ctypes.c_void_p):
        if stream and stream.value:
            self._check(self.lib.cudaStreamDestroy(stream), "cudaStreamDestroy")

    def memcpy_async(
        self,
        dst: ctypes.c_void_p,
        src: ctypes.c_void_p,
        nbytes: int,
        kind: int,
        stream: ctypes.c_void_p,
    ):
        self._check(
            self.lib.cudaMemcpyAsync(dst, src, nbytes, kind, stream), "cudaMemcpyAsync"
        )

    def stream_sync(self, stream: ctypes.c_void_p):
        self._check(self.lib.cudaStreamSynchronize(stream), "cudaStreamSynchronize")


class TensorRTDetector:
    def __init__(self, engine_path: str, conf: float, nms_iou: float = 0.45):
        import tensorrt as trt

        self.trt = trt
        self.conf = conf
        self.nms_iou = nms_iou
        self.cuda = CudaRuntime()
        self.logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f, trt.Runtime(self.logger) as runtime:
            self.engine = runtime.deserialize_cuda_engine(f.read())
        if self.engine is None:
            raise RuntimeError(f"failed to load TensorRT engine: {engine_path}")

        self.context = self.engine.create_execution_context()
        self.input_name = None
        self.output_names = []
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            mode = self.engine.get_tensor_mode(name)
            if mode == trt.TensorIOMode.INPUT:
                self.input_name = name
            else:
                self.output_names.append(name)
        if self.input_name is None or not self.output_names:
            raise RuntimeError("TensorRT engine must have one input and at least one output")

        input_shape = tuple(self.engine.get_tensor_shape(self.input_name))
        if any(dim < 0 for dim in input_shape):
            input_shape = (1, 3, 640, 640)
            self.context.set_input_shape(self.input_name, input_shape)
        self.input_shape = tuple(input_shape)
        self.input_dtype = trt.nptype(self.engine.get_tensor_dtype(self.input_name))
        if len(self.input_shape) != 4:
            raise RuntimeError(f"unsupported TensorRT input shape: {self.input_shape}")
        self.input_h = int(self.input_shape[2])
        self.input_w = int(self.input_shape[3])

        self.stream = self.cuda.stream_create()
        self.device_buffers = {}
        self.host_outputs = {}

        self.host_input = np.empty(self.input_shape, dtype=self.input_dtype)
        self.device_buffers[self.input_name] = self.cuda.malloc(self.host_input.nbytes)
        self.context.set_tensor_address(
            self.input_name, int(self.device_buffers[self.input_name].value)
        )

        for name in self.output_names:
            shape = tuple(self.context.get_tensor_shape(name))
            if any(dim < 0 for dim in shape):
                shape = tuple(self.engine.get_tensor_shape(name))
            dtype = trt.nptype(self.engine.get_tensor_dtype(name))
            host = np.empty(shape, dtype=dtype)
            self.host_outputs[name] = host
            self.device_buffers[name] = self.cuda.malloc(host.nbytes)
            self.context.set_tensor_address(name, int(self.device_buffers[name].value))

    def close(self):
        for ptr in getattr(self, "device_buffers", {}).values():
            self.cuda.free(ptr)
        if hasattr(self, "stream"):
            self.cuda.stream_destroy(self.stream)

    def _letterbox(self, frame: np.ndarray) -> tuple[np.ndarray, float, tuple[float, float]]:
        h, w = frame.shape[:2]
        gain = min(self.input_w / w, self.input_h / h)
        new_w = int(round(w * gain))
        new_h = int(round(h * gain))
        resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        canvas = np.full((self.input_h, self.input_w, 3), 114, dtype=np.uint8)
        pad_x = (self.input_w - new_w) / 2
        pad_y = (self.input_h - new_h) / 2
        left = int(round(pad_x - 0.1))
        top = int(round(pad_y - 0.1))
        canvas[top : top + new_h, left : left + new_w] = resized
        return canvas, gain, (pad_x, pad_y)

    def _preprocess(self, frame: np.ndarray) -> tuple[np.ndarray, float, tuple[float, float]]:
        letterboxed, gain, pad = self._letterbox(frame)
        rgb = cv2.cvtColor(letterboxed, cv2.COLOR_BGR2RGB)
        nchw = np.transpose(rgb, (2, 0, 1))[None]
        arr = np.ascontiguousarray(nchw, dtype=np.float32) / 255.0
        if self.input_dtype == np.float16:
            arr = arr.astype(np.float16)
        return arr, gain, pad

    def _infer_raw(self, inp: np.ndarray) -> list[np.ndarray]:
        np.copyto(self.host_input, inp)
        self.cuda.memcpy_async(
            self.device_buffers[self.input_name],
            self.host_input.ctypes.data_as(ctypes.c_void_p),
            self.host_input.nbytes,
            CudaRuntime.HOST_TO_DEVICE,
            self.stream,
        )
        ok = self.context.execute_async_v3(stream_handle=int(self.stream.value))
        if not ok:
            raise RuntimeError("TensorRT execute_async_v3 failed")
        for name, host in self.host_outputs.items():
            self.cuda.memcpy_async(
                host.ctypes.data_as(ctypes.c_void_p),
                self.device_buffers[name],
                host.nbytes,
                CudaRuntime.DEVICE_TO_HOST,
                self.stream,
            )
        self.cuda.stream_sync(self.stream)
        return [host.copy() for host in self.host_outputs.values()]

    def _postprocess(
        self,
        outputs: list[np.ndarray],
        original_shape: tuple[int, int],
        gain: float,
        pad: tuple[float, float],
    ) -> list[Detection]:
        out = outputs[0]
        if out.ndim == 3:
            out = out[0]
        # YOLOv8 detect export is normally (4 + classes, anchors).
        if out.ndim == 2 and out.shape[0] < out.shape[1]:
            out = out.T
        if out.ndim != 2 or out.shape[1] < 5:
            raise RuntimeError(f"unsupported TensorRT output shape: {outputs[0].shape}")

        boxes_xywh = out[:, :4].astype(np.float32)
        class_scores = out[:, 4:].astype(np.float32)
        cls_ids = np.argmax(class_scores, axis=1)
        scores = class_scores[np.arange(class_scores.shape[0]), cls_ids]
        keep = scores >= self.conf
        if not np.any(keep):
            return []

        boxes_xywh = boxes_xywh[keep]
        scores = scores[keep]
        cls_ids = cls_ids[keep]

        cx, cy, bw, bh = boxes_xywh.T
        x1 = cx - bw / 2
        y1 = cy - bh / 2
        x2 = cx + bw / 2
        y2 = cy + bh / 2
        pad_x, pad_y = pad
        h0, w0 = original_shape
        x1 = (x1 - pad_x) / gain
        x2 = (x2 - pad_x) / gain
        y1 = (y1 - pad_y) / gain
        y2 = (y2 - pad_y) / gain
        x1 = np.clip(x1, 0, w0 - 1)
        x2 = np.clip(x2, 0, w0 - 1)
        y1 = np.clip(y1, 0, h0 - 1)
        y2 = np.clip(y2, 0, h0 - 1)

        nms_boxes = [
            [float(a), float(b), float(c - a), float(d - b)]
            for a, b, c, d in zip(x1, y1, x2, y2)
        ]
        indices = cv2.dnn.NMSBoxes(
            nms_boxes, scores.tolist(), self.conf, self.nms_iou
        )
        if len(indices) == 0:
            return []

        detections = []
        for idx in np.array(indices).reshape(-1):
            detections.append(
                Detection(
                    xyxy=(
                        float(x1[idx]),
                        float(y1[idx]),
                        float(x2[idx]),
                        float(y2[idx]),
                    ),
                    conf=float(scores[idx]),
                    cls_id=int(cls_ids[idx]),
                )
            )
        return detections

    def predict(self, frame: np.ndarray) -> list[Detection]:
        inp, gain, pad = self._preprocess(frame)
        outputs = self._infer_raw(inp)
        return self._postprocess(outputs, frame.shape[:2], gain, pad)


def create_detector(model_path: str, conf: float):
    if os.path.splitext(model_path)[1].lower() == ".engine":
        return TensorRTDetector(model_path, conf)
    return UltralyticsDetector(model_path, conf)


def ros_image_to_bgr(msg: Image) -> np.ndarray:
    data = np.frombuffer(msg.data, dtype=np.uint8)
    enc = msg.encoding.lower()

    if enc in ("bgr8",):
        return data.reshape(msg.height, msg.step)[:, : msg.width * 3].reshape(
            msg.height, msg.width, 3
        )

    if enc in ("rgb8",):
        rgb = data.reshape(msg.height, msg.step)[:, : msg.width * 3].reshape(
            msg.height, msg.width, 3
        )
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    if enc in ("yuv422_yuy2", "yuyv", "yuv422"):
        yuyv = data.reshape(msg.height, msg.step)[:, : msg.width * 2].reshape(
            msg.height, msg.width, 2
        )
        return cv2.cvtColor(yuyv, cv2.COLOR_YUV2BGR_YUY2)

    raise ValueError(f"unsupported image encoding: {msg.encoding}")


def bgr_to_ros_image(frame: np.ndarray, header) -> Image:
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    msg = Image()
    msg.header = header
    msg.height, msg.width = frame_rgb.shape[:2]
    msg.encoding = "rgb8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = frame_rgb.tobytes()
    return msg


def ros_depth_to_meters(msg: Image) -> np.ndarray:
    data = np.frombuffer(msg.data, dtype=np.uint8)
    enc = msg.encoding.lower()

    if enc in ("16uc1", "mono16"):
        rows = data.reshape(msg.height, msg.step)[:, : msg.width * 2].copy()
        depth_mm = rows.view(np.uint16).reshape(msg.height, msg.width)
        return depth_mm.astype(np.float32) * 0.001

    if enc == "32fc1":
        rows = data.reshape(msg.height, msg.step)[:, : msg.width * 4].copy()
        return rows.view(np.float32).reshape(msg.height, msg.width)

    raise ValueError(f"unsupported depth encoding: {msg.encoding}")


def estimate_box_distance_m(
    depth_m: np.ndarray,
    rgb_shape: tuple[int, int],
    xyxy: tuple[float, float, float, float],
    min_depth_m: float,
    max_depth_m: float,
) -> float | None:
    if depth_m is None or depth_m.size == 0:
        return None

    rgb_h, rgb_w = rgb_shape
    depth_h, depth_w = depth_m.shape[:2]
    x1, y1, x2, y2 = xyxy

    # Use the center part of the box. It avoids poles, borders, and background.
    box_w = max(1.0, x2 - x1)
    box_h = max(1.0, y2 - y1)
    cx1 = x1 + box_w * 0.35
    cx2 = x2 - box_w * 0.35
    cy1 = y1 + box_h * 0.35
    cy2 = y2 - box_h * 0.35
    if cx2 <= cx1 or cy2 <= cy1:
        cx = (x1 + x2) * 0.5
        cy = (y1 + y2) * 0.5
        cx1, cx2 = cx - 3, cx + 3
        cy1, cy2 = cy - 3, cy + 3

    scale_x = depth_w / max(1, rgb_w)
    scale_y = depth_h / max(1, rgb_h)
    dx1 = int(np.clip(round(cx1 * scale_x), 0, depth_w - 1))
    dx2 = int(np.clip(round(cx2 * scale_x), 0, depth_w - 1))
    dy1 = int(np.clip(round(cy1 * scale_y), 0, depth_h - 1))
    dy2 = int(np.clip(round(cy2 * scale_y), 0, depth_h - 1))

    if dx2 <= dx1:
        dx2 = min(depth_w, dx1 + 1)
    if dy2 <= dy1:
        dy2 = min(depth_h, dy1 + 1)

    roi = depth_m[dy1:dy2, dx1:dx2]
    valid = roi[np.isfinite(roi) & (roi >= min_depth_m) & (roi <= max_depth_m)]
    if valid.size < 4:
        return None
    return float(np.median(valid))


def classify_light_hsv(roi_bgr: np.ndarray) -> tuple[str, dict[str, int]]:
    if roi_bgr.size == 0:
        return "UNKNOWN", {"red": 0, "yellow": 0, "green": 0}

    h, w = roi_bgr.shape[:2]
    # Avoid frame edges/background inside a loose YOLO box.
    x1 = int(w * 0.15)
    x2 = int(w * 0.85)
    y1 = int(h * 0.05)
    y2 = int(h * 0.95)
    crop = roi_bgr[y1:y2, x1:x2]
    if crop.size == 0:
        crop = roi_bgr

    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    hue = hsv[:, :, 0]
    sat = hsv[:, :, 1]
    val = hsv[:, :, 2]

    bgr_i = crop.astype(np.int16)
    blue = bgr_i[:, :, 0]
    green_ch = bgr_i[:, :, 1]
    red_ch = bgr_i[:, :, 2]

    # Yellow lamps often appear orange or nearly white in camera frames. Use
    # HSV plus BGR channel dominance so the short yellow phase is not lost.
    red_mask = (
        ((((hue <= 10) | (hue >= 168)) & (sat >= 55) & (val >= 95)))
        | ((red_ch >= 130) & ((red_ch - green_ch) >= 35) & ((red_ch - blue) >= 45))
    )
    yellow_mask = (
        (((hue >= 10) & (hue <= 45) & (sat >= 35) & (val >= 115)))
        | (
            (red_ch >= 115)
            & (green_ch >= 95)
            & (val >= 130)
            & ((red_ch - blue) >= 35)
            & ((green_ch - blue) >= 25)
            & (np.abs(red_ch - green_ch) <= 95)
        )
    )
    green_mask = (
        (((hue >= 38) & (hue <= 98) & (sat >= 45) & (val >= 85)))
        | ((green_ch >= 105) & ((green_ch - red_ch) >= 20) & ((green_ch - blue) >= 35))
    )

    red = red_mask.astype(np.uint8) * 255
    yellow = yellow_mask.astype(np.uint8) * 255
    green = green_mask.astype(np.uint8) * 255

    kernel = np.ones((3, 3), np.uint8)
    red = cv2.morphologyEx(red, cv2.MORPH_OPEN, kernel)
    yellow = cv2.morphologyEx(yellow, cv2.MORPH_OPEN, kernel)
    green = cv2.morphologyEx(green, cv2.MORPH_OPEN, kernel)

    counts = {
        "red": int(cv2.countNonZero(red)),
        "yellow": int(cv2.countNonZero(yellow)),
        "green": int(cv2.countNonZero(green)),
    }
    color, count = max(counts.items(), key=lambda kv: kv[1])

    # Small lamps are tiny, and yellow is short-lived, so keep this threshold
    # low in the color-debug path while still filtering single-pixel noise.
    min_pixels = max(4, int(crop.shape[0] * crop.shape[1] * 0.001))
    if count < min_pixels:
        return "UNKNOWN", counts
    return color.upper(), counts


def detect_lit_green(roi_bgr: np.ndarray) -> tuple[bool, dict[str, float]]:
    """Return True only for a bright lit green lamp, not a dim green lens."""
    if roi_bgr.size == 0:
        return False, {
            "green_pixels": 0,
            "bright_green_pixels": 0,
            "bright_red_pixels": 0,
            "bright_yellow_pixels": 0,
            "green_ratio": 0.0,
            "bright_green_ratio": 0.0,
            "mean_green_v": 0.0,
        }

    h, w = roi_bgr.shape[:2]
    # The lamp itself is normally near the center of the YOLO box. Cropping
    # removes black pole/background and makes HSV thresholds less fragile.
    x1 = int(w * 0.18)
    x2 = int(w * 0.82)
    y1 = int(h * 0.05)
    y2 = int(h * 0.90)
    crop = roi_bgr[y1:y2, x1:x2]
    if crop.size == 0:
        crop = roi_bgr

    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    hue = hsv[:, :, 0]
    sat = hsv[:, :, 1]
    val = hsv[:, :, 2]

    # Loose green finds green material/lens. Bright masks require actual LED
    # emission, and are used to reject an unlit green lens beside a lit red lamp.
    green = (hue >= 42) & (hue <= 92) & (sat >= 55) & (val >= 70)
    bright_green = (hue >= 45) & (hue <= 88) & (sat >= 90) & (val >= 185)
    bright_red = (((hue <= 12) | (hue >= 168)) & (sat >= 90) & (val >= 165))
    bright_yellow = (hue >= 15) & (hue <= 38) & (sat >= 80) & (val >= 165)

    green_u8 = green.astype(np.uint8) * 255
    bright_u8 = bright_green.astype(np.uint8) * 255
    red_u8 = bright_red.astype(np.uint8) * 255
    yellow_u8 = bright_yellow.astype(np.uint8) * 255
    kernel = np.ones((3, 3), np.uint8)
    green_u8 = cv2.morphologyEx(green_u8, cv2.MORPH_OPEN, kernel)
    bright_u8 = cv2.morphologyEx(bright_u8, cv2.MORPH_OPEN, kernel)
    red_u8 = cv2.morphologyEx(red_u8, cv2.MORPH_OPEN, kernel)
    yellow_u8 = cv2.morphologyEx(yellow_u8, cv2.MORPH_OPEN, kernel)

    green_pixels = int(cv2.countNonZero(green_u8))
    bright_green_pixels = int(cv2.countNonZero(bright_u8))
    bright_red_pixels = int(cv2.countNonZero(red_u8))
    bright_yellow_pixels = int(cv2.countNonZero(yellow_u8))
    area = max(1, crop.shape[0] * crop.shape[1])
    green_ratio = green_pixels / area
    bright_green_ratio = bright_green_pixels / area
    mean_green_v = float(np.mean(val[bright_u8 > 0])) if bright_green_pixels else 0.0

    # Require a bright green core and make it dominate other lit colors.
    # A non-lit green lens may have many green pixels, but lacks a strong
    # high-value core; when red is lit, bright_red_pixels should dominate.
    min_bright_pixels = max(10, int(area * 0.003))
    competing_lit_pixels = max(bright_red_pixels, bright_yellow_pixels)
    lit = (
        bright_green_pixels >= min_bright_pixels
        and bright_green_ratio >= 0.003
        and mean_green_v >= 200.0
        and bright_green_pixels >= max(12, int(competing_lit_pixels * 1.4))
    )
    return lit, {
        "green_pixels": green_pixels,
        "bright_green_pixels": bright_green_pixels,
        "bright_red_pixels": bright_red_pixels,
        "bright_yellow_pixels": bright_yellow_pixels,
        "green_ratio": green_ratio,
        "bright_green_ratio": bright_green_ratio,
        "mean_green_v": mean_green_v,
    }


class TrafficLightDebugNode(Node):
    def __init__(
        self,
        model_path: str,
        conf: float,
        green_only: bool,
        green_hold_sec: float,
        image_topic: str,
        depth_topic: str,
        use_depth: bool,
        stop_distance_m: float,
        slow_distance_m: float,
        traffic_light_distance_m: float,
        pedestrian_distance_m: float,
        depth_timeout_sec: float,
        depth_min_m: float,
        depth_max_m: float,
        no_target_state: str,
        traffic_light_only: bool,
    ):
        super().__init__("traffic_light_debug_node")
        self.model = create_detector(model_path, conf)
        self.conf = conf
        self.green_only = green_only
        self.last_colors = deque(maxlen=5)
        self.green_votes = deque(maxlen=5)
        self.last_stable_state = "STOP"
        self.last_green_time = None
        self.green_hold_sec = green_hold_sec
        self.use_depth = use_depth
        self.stop_distance_m = stop_distance_m
        self.slow_distance_m = slow_distance_m
        self.traffic_light_distance_m = traffic_light_distance_m
        self.pedestrian_distance_m = pedestrian_distance_m
        self.depth_timeout_sec = depth_timeout_sec
        self.depth_min_m = depth_min_m
        self.depth_max_m = depth_max_m
        self.no_target_state = no_target_state.strip().upper()
        self.traffic_light_only = traffic_light_only
        if self.no_target_state not in ("STOP", "GO", "SLOW"):
            self.no_target_state = "STOP"
        self.last_depth_m = None
        self.last_depth_time = None

        self.sub = self.create_subscription(
            Image, image_topic, self.on_image, qos_profile_sensor_data
        )
        self.depth_sub = None
        if self.use_depth:
            self.depth_sub = self.create_subscription(
                Image, depth_topic, self.on_depth, qos_profile_sensor_data
            )
        self.debug_pub = self.create_publisher(
            Image, "/traffic_light/debug_image", 10
        )
        self.color_pub = self.create_publisher(String, "/traffic_light/color", 10)
        self.state_pub = self.create_publisher(String, "/traffic_light/state", 10)
        self.get_logger().info(
            f"loaded model: {model_path}, conf={conf}, green_only={green_only}, "
            f"image_topic={image_topic}, use_depth={use_depth}, depth_topic={depth_topic}, "
            f"stop<={stop_distance_m:.2f}m, slow<={slow_distance_m:.2f}m, "
            f"traffic_light<={traffic_light_distance_m:.2f}m, "
            f"pedestrian<={pedestrian_distance_m:.2f}m, no_target={self.no_target_state}, "
            f"traffic_light_only={traffic_light_only}"
        )

    def destroy_node(self):
        if hasattr(self.model, "close"):
            self.model.close()
        super().destroy_node()

    def stable_color(self, color: str) -> str:
        if color == "UNKNOWN":
            return color
        self.last_colors.append(color)
        votes = {c: self.last_colors.count(c) for c in set(self.last_colors)}
        best, n = max(votes.items(), key=lambda kv: kv[1])
        return best if n >= 4 and len(self.last_colors) >= 5 else color

    def green_state(self, raw_green: bool) -> str:
        now = self.get_clock().now()
        self.green_votes.append(raw_green)
        if raw_green:
            self.last_green_time = now

        green_count = sum(self.green_votes)
        no_green_count = len(self.green_votes) - green_count

        if len(self.green_votes) >= 3 and green_count >= 2:
            self.last_stable_state = "GO"
            return "GO"

        if self.last_stable_state == "GO" and self.last_green_time is not None:
            elapsed = (now - self.last_green_time).nanoseconds / 1e9
            if elapsed <= self.green_hold_sec:
                return "GO_HOLD"

        if len(self.green_votes) >= 5 and no_green_count >= 4:
            self.last_stable_state = "STOP"
            return "STOP"

        return self.last_stable_state

    def on_depth(self, msg: Image):
        try:
            self.last_depth_m = ros_depth_to_meters(msg)
            self.last_depth_time = self.get_clock().now()
        except Exception as exc:
            self.get_logger().warn(str(exc))

    def current_depth(self) -> np.ndarray | None:
        if not self.use_depth or self.last_depth_m is None:
            return None
        if self.last_depth_time is None:
            return None
        age = (self.get_clock().now() - self.last_depth_time).nanoseconds / 1e9
        if age > self.depth_timeout_sec:
            return None
        return self.last_depth_m

    def distance_for_detection(
        self, det: Detection, frame_shape: tuple[int, int]
    ) -> float | None:
        depth = self.current_depth()
        if depth is None:
            return None
        return estimate_box_distance_m(
            depth,
            frame_shape,
            det.xyxy,
            self.depth_min_m,
            self.depth_max_m,
        )

    def detection_is_near(self, det: Detection, distance_m: float | None) -> bool:
        if not self.use_depth:
            return True
        if distance_m is None:
            return False
        if det.cls_id == 1:
            return distance_m <= self.stop_distance_m
        if det.cls_id == 2:
            return distance_m <= self.slow_distance_m
        if det.cls_id == 0:
            return distance_m <= self.traffic_light_distance_m
        if det.cls_id == 3:
            return distance_m <= self.pedestrian_distance_m
        return True

    def traffic_light_state(self, frame: np.ndarray, det: Detection) -> tuple[str, str, dict]:
        x1, y1, x2, y2 = map(int, det.xyxy)
        h, w = frame.shape[:2]
        x1, x2 = max(0, x1), min(w - 1, x2)
        y1, y2 = max(0, y1), min(h - 1, y2)
        roi = frame[y1:y2, x1:x2]
        if self.green_only:
            raw_green, green_info = detect_lit_green(roi)
            state = self.green_state(raw_green)
            color = "GREEN" if state in ("GO", "GO_HOLD") else "RED"
            info = {
                "green": int(green_info["green_pixels"]),
                "bright_green": int(green_info["bright_green_pixels"]),
                "bright_red": int(green_info["bright_red_pixels"]),
                "bright_yellow": int(green_info["bright_yellow_pixels"]),
            }
            return state, color, info

        color, counts = classify_light_hsv(roi)
        if not self.traffic_light_only:
            color = self.stable_color(color)
        return ("GO" if color == "GREEN" else "STOP"), color, counts

    def traffic_light_display_label(
        self, frame: np.ndarray, det: Detection
    ) -> tuple[str, tuple[int, int, int]]:
        x1, y1, x2, y2 = map(int, det.xyxy)
        h, w = frame.shape[:2]
        x1, x2 = max(0, x1), min(w - 1, x2)
        y1, y2 = max(0, y1), min(h - 1, y2)
        roi = frame[y1:y2, x1:x2]

        if self.green_only:
            raw_green, _info = detect_lit_green(roi)
            if raw_green:
                return "green_light", (0, 255, 0)
            return "red_light", (0, 0, 255)

        color, _counts = classify_light_hsv(roi)
        if color == "GREEN":
            return "green_light", (0, 255, 0)
        if color == "YELLOW":
            return "yellow_light", (0, 255, 255)
        if color == "RED":
            return "red_light", (0, 0, 255)
        return "unknown_light", (255, 255, 255)

    def put_label(self, frame: np.ndarray, x1: int, y1: int, text: str, color):
        cv2.putText(
            frame,
            text,
            (x1, max(20, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            color,
            2,
            cv2.LINE_AA,
        )

    def on_image(self, msg: Image):
        try:
            frame = ros_image_to_bgr(msg).copy()
        except Exception as exc:
            self.get_logger().warn(str(exc))
            return

        detections = []
        for det in self.model.predict(frame):
            if det.cls_id not in CLASS_NAMES:
                continue
            if self.traffic_light_only and det.cls_id != 0:
                continue
            detections.append(det)
        frame_shape = frame.shape[:2]
        enriched = []
        for det in detections:
            distance_m = self.distance_for_detection(det, frame_shape)
            near = self.detection_is_near(det, distance_m)
            enriched.append((det, distance_m, near))

        state = self.no_target_state
        color = "NONE"
        reason = "NO_TARGET"

        near_pedestrians = [
            item for item in enriched if item[0].cls_id == 3 and item[2]
        ]
        near_stops = [item for item in enriched if item[0].cls_id == 1 and item[2]]
        near_slows = [item for item in enriched if item[0].cls_id == 2 and item[2]]
        near_traffic_lights = [
            item for item in enriched if item[0].cls_id == 0 and item[2]
        ]

        if near_pedestrians:
            state = "STOP"
            color = "PEDESTRIAN"
            reason = "NEAR_PEDESTRIAN"
        elif near_stops:
            state = "STOP"
            color = "STOP_SIGN"
            reason = "NEAR_STOP_SIGN"
        elif near_slows:
            state = "SLOW"
            color = "SLOW_SIGN"
            reason = "NEAR_SLOW_SIGN"
        elif near_traffic_lights:
            best_light, _distance_m, _near = max(
                near_traffic_lights, key=lambda item: item[0].conf
            )
            state, color, _info = self.traffic_light_state(frame, best_light)
            reason = "TRAFFIC_LIGHT"

        for det, distance_m, near in enriched:
            x1, y1, x2, y2 = map(int, det.xyxy)
            h, w = frame.shape[:2]
            x1, x2 = max(0, x1), min(w - 1, x2)
            y1, y2 = max(0, y1), min(h - 1, y2)
            box_color = CLASS_BOX_COLORS.get(det.cls_id, (255, 255, 255))
            active_text = ""
            if det.cls_id in (0, 1, 2, 3):
                active_text = "near" if near else "far"
                if self.use_depth and distance_m is None:
                    active_text = "no_depth"
            if det.cls_id == 0 and reason == "TRAFFIC_LIGHT":
                box_color = {
                    "GREEN": (0, 255, 0),
                    "RED": (0, 0, 255),
                    "YELLOW": (0, 255, 255),
                    "UNKNOWN": (255, 255, 255),
                }.get(color, box_color)
            cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)

            dist_text = "--m" if distance_m is None else f"{distance_m:.2f}m"
            if det.cls_id == 0:
                light_text, light_color = self.traffic_light_display_label(frame, det)
                box_color = light_color
                cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)
                label = f"{light_text} {det.conf:.2f} {dist_text} {active_text}".strip()
            else:
                label = (
                    f"{CLASS_NAMES[det.cls_id]} {det.conf:.2f} {dist_text} {active_text}"
                ).strip()
            self.put_label(frame, x1, y1, label, box_color)

        cv2.putText(
            frame,
            f"STATE {state} {reason}",
            (8, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255) if state == "SLOW" else (0, 255, 0) if state in ("GO", "GO_HOLD") else (0, 0, 255),
            2,
            cv2.LINE_AA,
        )

        self.color_pub.publish(String(data=color))
        self.state_pub.publish(String(data=state))
        self.debug_pub.publish(bgr_to_ros_image(frame, msg.header))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model",
        default="/home/wheeltec/traffic_light_model/best.pt",
        help="Path to trained YOLOv8 traffic_light model.",
    )
    parser.add_argument("--conf", type=float, default=0.4)
    parser.add_argument(
        "--green-only",
        action="store_true",
        help="Only treat bright lit green as GO; all other states become STOP.",
    )
    parser.add_argument(
        "--green-hold-sec",
        type=float,
        default=0.8,
        help="Keep GO for this many seconds after a short green dropout.",
    )
    parser.add_argument("--image-topic", default="/image_raw")
    parser.add_argument("--depth-topic", default="/camera/depth/image_raw")
    parser.add_argument(
        "--use-depth",
        default="true",
        help="Use depth gating for traffic_light, stop_sign, slow_sign, and pedestrian.",
    )
    parser.add_argument("--stop-distance-m", type=float, default=0.53)
    parser.add_argument("--slow-distance-m", type=float, default=0.53)
    parser.add_argument("--traffic-light-distance-m", type=float, default=0.53)
    parser.add_argument("--pedestrian-distance-m", type=float, default=0.53)
    parser.add_argument("--depth-timeout-sec", type=float, default=1.0)
    parser.add_argument("--depth-min-m", type=float, default=0.15)
    parser.add_argument("--depth-max-m", type=float, default=6.0)
    parser.add_argument(
        "--no-target-state",
        default="STOP",
        help="State to publish when no actionable target is visible: STOP, GO, or SLOW.",
    )
    parser.add_argument(
        "--traffic-light-only",
        action="store_true",
        help="Ignore stop_sign, slow_sign, and pedestrian detections; only show traffic light color.",
    )
    args, _ros_args = parser.parse_known_args()

    rclpy.init()
    node = TrafficLightDebugNode(
        args.model,
        args.conf,
        args.green_only,
        args.green_hold_sec,
        args.image_topic,
        args.depth_topic,
        str_to_bool(args.use_depth),
        args.stop_distance_m,
        args.slow_distance_m,
        args.traffic_light_distance_m,
        args.pedestrian_distance_m,
        args.depth_timeout_sec,
        args.depth_min_m,
        args.depth_max_m,
        args.no_target_state,
        args.traffic_light_only,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, Exception):
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
