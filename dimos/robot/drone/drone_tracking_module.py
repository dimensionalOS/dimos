#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Drone tracking module with visual servoing for object following."""

import json
import os
import threading
import time
from typing import Any

import cv2
from dimos_lcm.std_msgs import String
import numpy as np
from numpy.typing import NDArray

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.models.qwen.video_query import get_bbox_from_qwen_frame
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.robot.drone.drone_visual_servoing_controller import (
    DroneVisualServoingController,
    PIDParams,
)
from dimos.utils.data import get_data
from dimos.utils.logging_config import setup_logger
from dimos.utils.simple_controller import PIDController

logger = setup_logger()

INDOOR_PID_PARAMS: PIDParams = (0.001, 0.0, 0.0001, (-1.0, 1.0), None, 30)
OUTDOOR_PID_PARAMS: PIDParams = (0.05, 0.0, 0.0003, (-5.0, 5.0), None, 10)
INDOOR_MAX_VELOCITY = 1.0  # m/s safety cap for indoor mode
INDOOR_MAX_FORWARD_VELOCITY = 0.25
INDOOR_MAX_LATERAL_VELOCITY = 0.35
PERSON_ALIASES = {"person", "people", "human", "man", "woman"}
PERSON_CONFIRM_FRAMES = 2
PERSON_CONFIRM_IOU = 0.12
YOLO_PERSON_CONF_MIN = 0.18
YOLO_PERSON_MIN_KEYPOINTS = 4
YOLO_MODEL_NAME = "yolo11s-pose.pt"
YOLO_MODEL_NAME_CPU = "yolo11n-pose.pt"
YOLO_DEFAULT_IMGSZ = 416
YOLO_DEFAULT_MAX_DET = 5
PASSIVE_DETECT_INTERVAL_SEC = 0.3
YAW_ONLY_DEADBAND_RATIO = 0.028
YAW_ONLY_PID_KP = 1.0
YAW_ONLY_PID_KI = 0.1
YAW_ONLY_PID_KD = 0.62
YAW_ONLY_MAX_RATE = 0.85
YAW_ONLY_INTEGRAL_LIMIT = 0.2
YAW_ONLY_CMD_SMOOTH_ALPHA = 0.40
YAW_ONLY_DT = 0.05
YAW_ONLY_ZERO_RATE_EPS = 0.015
YAW_ONLY_MIN_RATE = 0.14
YAW_ONLY_MIN_RATE_ERR_RATIO = 0.25
YAW_ONLY_RATE_SLEW_PER_SEC = 7.0
YAW_ONLY_LOCK_ENTER_RATIO = 0.03
YAW_ONLY_LOCK_EXIT_RATIO = 0.075
YAW_ONLY_LOCK_CMD_EPS = 0.08
YAW_ONLY_ERROR_LPF_ALPHA = 0.6
YAW_ONLY_DECEL_ERR_RATIO = 0.24
YAW_ONLY_DECEL_MIN_RATE = 0.08
YAW_ONLY_CROSS_BRAKE_ERR_RATIO = 0.24
YAW_ONLY_CROSS_BRAKE_SEC = 0.12
FOLLOW_TARGET_MIN_M = 0.6
FOLLOW_TARGET_MAX_M = 2.5
FOLLOW_APPROACH_SPEED = 0.15
FOLLOW_ALIGN_SOFT_ERR = 0.15
FOLLOW_ALIGN_STOP_ERR = 0.30
FOLLOW_EDGE_MARGIN_RATIO = 0.06


class DroneTrackingModule(Module):
    """Module for drone object tracking with visual servoing control."""

    # Inputs
    video_input: In[Image]
    follow_object_cmd: In[Any]

    # Outputs
    tracking_overlay: Out[Image]  # Visualization with bbox and crosshairs
    tracking_status: Out[Any]  # JSON status updates
    cmd_vel: Out[Twist]  # Velocity commands for drone control

    def __init__(
        self,
        outdoor: bool = False,
        x_pid_params: PIDParams | None = None,
        y_pid_params: PIDParams | None = None,
        z_pid_params: PIDParams | None = None,
        enable_passive_overlay: bool = False,
        use_local_person_detector: bool = False,
        force_detection_servoing_for_person: bool = False,
        person_follow_policy: str = "legacy_pid",
    ) -> None:
        """Initialize the drone tracking module.

        Args:
            outdoor: If True, use aggressive outdoor PID params (5 m/s max).
                     If False (default), use conservative indoor params (1 m/s max).
            x_pid_params: PID parameters for forward/backward control.
                          If None, uses preset based on outdoor flag.
            y_pid_params: PID parameters for left/right strafe control.
                          If None, uses preset based on outdoor flag.
            z_pid_params: Optional PID parameters for altitude control.
            enable_passive_overlay: If True, continuously publish person-detection overlays.
            use_local_person_detector: If True, allow local YOLO person detection fallback when Qwen is unavailable.
            force_detection_servoing_for_person: If True, bypass OpenCV tracker for person targets and use detector loop.
            person_follow_policy: Person follow behavior in detector loop.
                Supported: "legacy_pid" (original vx/vy PID), "yaw_forward_constant" (yaw-first + constant forward).
        """
        super().__init__()

        default_params = OUTDOOR_PID_PARAMS if outdoor else INDOOR_PID_PARAMS
        x_pid_params = x_pid_params if x_pid_params is not None else default_params
        y_pid_params = y_pid_params if y_pid_params is not None else default_params

        self._outdoor = outdoor
        self._max_velocity = None if outdoor else INDOOR_MAX_VELOCITY
        self._max_forward_velocity = None if outdoor else INDOOR_MAX_FORWARD_VELOCITY
        self._max_lateral_velocity = None if outdoor else INDOOR_MAX_LATERAL_VELOCITY

        self.servoing_controller = DroneVisualServoingController(
            x_pid_params=x_pid_params, y_pid_params=y_pid_params, z_pid_params=z_pid_params
        )

        # Tracking state
        self._tracking_active = False
        self._tracking_thread: threading.Thread | None = None
        self._passive_overlay_active = False
        self._passive_overlay_thread: threading.Thread | None = None
        self._enable_passive_overlay = enable_passive_overlay
        self._use_local_person_detector = use_local_person_detector
        self._force_detection_servoing_for_person = force_detection_servoing_for_person
        self._control_mode = "full"
        normalized_policy = person_follow_policy.strip().lower()
        if normalized_policy not in {"legacy_pid", "yaw_forward_constant"}:
            normalized_policy = "legacy_pid"
        self._person_follow_policy = normalized_policy
        self._yaw_rate_ema = 0.0
        self._yaw_error_lpf = 0.0
        self._yaw_lock_active = False
        self._yaw_prev_raw_error = 0.0
        self._yaw_cross_brake_until = 0.0
        self._yaw_pid = PIDController(
            YAW_ONLY_PID_KP,
            YAW_ONLY_PID_KI,
            YAW_ONLY_PID_KD,
            output_limits=(-YAW_ONLY_MAX_RATE, YAW_ONLY_MAX_RATE),
            integral_limit=YAW_ONLY_INTEGRAL_LIMIT,
            deadband=YAW_ONLY_DEADBAND_RATIO,
            inverse_output=True,
        )
        self._current_object: str | None = None
        self._latest_frame: Image | None = None
        self._frame_lock = threading.Lock()
        self._yolo: Any | None = None
        self._yolo_device = "cpu"
        self._yolo_half = False
        self._yolo_model_name = YOLO_MODEL_NAME
        self._yolo_imgsz = self._env_int("DIMOS_DRONE_YOLO_IMGSZ", YOLO_DEFAULT_IMGSZ, min_value=192)
        self._yolo_max_det = self._env_int("DIMOS_DRONE_YOLO_MAX_DET", YOLO_DEFAULT_MAX_DET, min_value=1)
        self._target_distance_m = 1.0
        self._person_lock_bbox: tuple[int, int, int, int] | None = None
        self._local_detector_enabled = (
            self._enable_passive_overlay
            or self._use_local_person_detector
            or self._force_detection_servoing_for_person
        )
        if self._local_detector_enabled:
            self._init_yolo_person_detector()

        # Subscribe to video input when transport is set
        # (will be done by connection module)

    def _on_new_frame(self, frame: Image) -> None:
        """Handle new video frame."""
        with self._frame_lock:
            self._latest_frame = frame

    def _on_follow_object_cmd(self, cmd: String) -> None:
        msg = json.loads(cmd.data)
        self.track_object(
            object_name=msg.get("object_description"),
            duration=float(msg.get("duration", 120.0)),
            distance_m=float(msg.get("distance_m", 1.0)),
            control_mode=str(msg.get("control_mode", "full")),
        )

    def _create_tracker(self) -> Any | None:
        """Create the best available OpenCV tracker for this environment."""
        candidates: list[tuple[Any, str]] = []

        if hasattr(cv2, "legacy"):
            candidates.extend(
                [
                    (getattr(cv2.legacy, "TrackerCSRT_create", None), "legacy.CSRT"),
                    (getattr(cv2.legacy, "TrackerKCF_create", None), "legacy.KCF"),
                    (getattr(cv2.legacy, "TrackerMIL_create", None), "legacy.MIL"),
                ]
            )

        candidates.extend(
            [
                (getattr(cv2, "TrackerCSRT_create", None), "CSRT"),
                (getattr(cv2, "TrackerKCF_create", None), "KCF"),
                (getattr(cv2, "TrackerMIL_create", None), "MIL"),
            ]
        )

        for factory, name in candidates:
            if callable(factory):
                try:
                    tracker = factory()
                    logger.info(f"Using OpenCV tracker backend: {name}")
                    return tracker
                except Exception as e:
                    logger.warning(f"Tracker backend {name} unavailable: {e}")

        logger.error("No supported OpenCV tracker backend available (CSRT/KCF/MIL)")
        return None

    def _bbox_to_tracker_rect(
        self,
        frame: np.ndarray[Any, np.dtype[Any]],
        bbox: tuple[int, int, int, int],
    ) -> tuple[int, int, int, int] | None:
        """Convert x1,y1,x2,y2 bbox into a safe x,y,w,h tracker rectangle."""
        frame_h, frame_w = frame.shape[:2]
        x1, y1, x2, y2 = bbox

        x1 = max(0, min(int(x1), frame_w - 2))
        y1 = max(0, min(int(y1), frame_h - 2))
        x2 = max(x1 + 2, min(int(x2), frame_w - 1))
        y2 = max(y1 + 2, min(int(y2), frame_h - 1))

        w = x2 - x1
        h = y2 - y1
        if w < 2 or h < 2:
            return None

        # Expand box slightly to make initialization less brittle.
        scale = 1.15
        cx = x1 + (w / 2.0)
        cy = y1 + (h / 2.0)
        w2 = max(8, round(w * scale))
        h2 = max(8, round(h * scale))

        tx = max(0, round(cx - w2 / 2.0))
        ty = max(0, round(cy - h2 / 2.0))
        tw = min(w2, frame_w - tx)
        th = min(h2, frame_h - ty)

        if tw < 8 or th < 8:
            return None

        return (tx, ty, tw, th)

    def _prepare_tracker_frame(
        self,
        frame: np.ndarray[Any, np.dtype[Any]],
    ) -> np.ndarray[Any, np.dtype[Any]]:
        """Normalize frame layout/type for OpenCV tracker APIs."""
        prepared = frame
        if prepared.dtype != np.uint8:
            prepared = prepared.astype(np.uint8, copy=False)
        return np.ascontiguousarray(prepared)

    def _init_tracker(
        self,
        tracker: Any,
        frame: np.ndarray[Any, np.dtype[Any]],
        tracker_rect: tuple[int, int, int, int],
    ) -> tuple[bool, str]:
        """Initialize tracker with conservative argument fallbacks."""
        init_frame = self._prepare_tracker_frame(frame)
        rect_float = (
            float(tracker_rect[0]),
            float(tracker_rect[1]),
            float(tracker_rect[2]),
            float(tracker_rect[3]),
        )
        rect_variants: list[tuple[float, float, float, float] | tuple[int, int, int, int]] = [
            rect_float,
            tracker_rect,
        ]

        last_error: Exception | None = None
        for rect in rect_variants:
            try:
                init_result = tracker.init(init_frame, rect)
                init_failed = isinstance(init_result, bool) and not init_result
                if init_failed:
                    continue
                return True, "ok"
            except Exception as exc:
                last_error = exc

        if last_error is not None:
            return False, f"{last_error!s}"
        return False, "tracker init returned false"

    def _get_latest_frame(self) -> np.ndarray[Any, np.dtype[Any]] | None:
        """Get the latest video frame as numpy array."""
        with self._frame_lock:
            if self._latest_frame is None:
                return None
            # Convert Image to numpy array
            data: np.ndarray[Any, np.dtype[Any]] = self._latest_frame.data
            return data

    @rpc
    def start(self) -> None:
        """Start the tracking module and subscribe to video input."""
        if self.video_input.transport:
            self.video_input.subscribe(self._on_new_frame)
            logger.info("DroneTrackingModule started - subscribed to video input")
        else:
            logger.warning("DroneTrackingModule: No video input transport configured")

        if self.follow_object_cmd.transport:
            self.follow_object_cmd.subscribe(self._on_follow_object_cmd)

        if self._enable_passive_overlay:
            self._start_passive_overlay_loop()
        return

    @rpc
    def stop(self) -> None:
        self._stop_passive_overlay_loop()
        self._stop_tracking()
        super().stop()

    def _start_passive_overlay_loop(self) -> None:
        if self._passive_overlay_active:
            return
        self._passive_overlay_active = True
        self._passive_overlay_thread = threading.Thread(
            target=self._passive_detection_overlay_loop,
            daemon=True,
        )
        self._passive_overlay_thread.start()

    def _stop_passive_overlay_loop(self) -> None:
        self._passive_overlay_active = False
        if self._passive_overlay_thread and self._passive_overlay_thread.is_alive():
            self._passive_overlay_thread.join(timeout=1.0)
        self._passive_overlay_thread = None

    @rpc
    def track_object(
        self,
        object_name: str | None = None,
        duration: float = 120.0,
        distance_m: float = 1.0,
        control_mode: str = "full",
    ) -> str:
        """Track and follow an object using visual servoing.

        Args:
            object_name: Name of object to track, or None for most prominent
            duration: Maximum tracking duration in seconds
            distance_m: Reserved follow-distance hint (person follow currently uses fixed approach speed)
            control_mode: "full" (translate/strafe) or "yaw_only" (rotate in place)

        Returns:
            String status message
        """
        if self._tracking_active:
            return "Already tracking an object"

        # Get current frame
        frame = self._get_latest_frame()
        if frame is None:
            return "Error: No video frame available"

        logger.info(f"Starting track_object for {object_name or 'any object'}")

        try:
            normalized_mode = control_mode.strip().lower()
            if normalized_mode not in {"full", "yaw_only"}:
                normalized_mode = "full"
            self._control_mode = normalized_mode
            self._reset_yaw_controller()
            self._target_distance_m = max(FOLLOW_TARGET_MIN_M, min(FOLLOW_TARGET_MAX_M, distance_m))
            self._person_lock_bbox = None

            bbox = self._detect_initial_bbox(frame, object_name)

            if bbox is None:
                msg = f"No object detected{' for: ' + object_name if object_name else ''}"
                logger.warning(msg)
                if self.tracking_overlay.transport:
                    overlay = self._draw_search_overlay(
                        frame, f"Searching for {object_name or 'object'} (not found)"
                    )
                    self.tracking_overlay.publish(Image.from_numpy(overlay, format=ImageFormat.BGR))
                self._publish_status({"status": "not_found", "object": self._current_object})
                return msg

            logger.info(f"Object detected at bbox: {bbox}")

            if (
                object_name is None or object_name.lower().strip() in PERSON_ALIASES
            ) and self._local_detector_enabled:
                confirmed_bbox = self._confirm_person_detection(bbox)
                if confirmed_bbox is None:
                    self._publish_status(
                        {
                            "status": "not_found",
                            "object": object_name or "person",
                            "error": "person confirmation failed",
                        }
                    )
                    return "Detected candidate was not a stable person target"
                bbox = confirmed_bbox

            person_target = object_name is None or object_name.lower().strip() in PERSON_ALIASES
            detector_person_mode = person_target and (
                self._control_mode == "yaw_only" or self._force_detection_servoing_for_person
            )

            if detector_person_mode:
                self._current_object = object_name or "person"
                self._tracking_active = True
                self._publish_status(
                    {
                        "status": "tracking",
                        "object": self._current_object,
                        "mode": self._control_mode,
                        "backend": "yolo_detection",
                    }
                )
                self._tracking_thread = threading.Thread(
                    target=self._detection_servoing_loop,
                    args=(object_name, duration, self._target_distance_m),
                    daemon=True,
                )
                self._tracking_thread.start()
                if self._control_mode == "yaw_only":
                    return f"Yaw-only tracking started for {self._current_object}"
                return f"Tracking started for {self._current_object} (YOLO detection mode)"

            if self._control_mode == "yaw_only":
                self._current_object = object_name or "object"
                self._tracking_active = True
                self._publish_status(
                    {
                        "status": "tracking",
                        "object": self._current_object,
                        "mode": "yaw_only",
                    }
                )
                self._tracking_thread = threading.Thread(
                    target=self._detection_servoing_loop,
                    args=(object_name, duration, self._target_distance_m),
                    daemon=True,
                )
                self._tracking_thread.start()
                return f"Yaw-only tracking started for {self._current_object}"

            tracker = self._create_tracker()
            if tracker is None:
                self._publish_status(
                    {
                        "status": "failed",
                        "object": self._current_object,
                        "error": "No supported OpenCV tracker backend",
                    }
                )
                return "Failed: no supported OpenCV tracker backend (need at least MIL/KCF/CSRT)"

            tracker_rect = self._bbox_to_tracker_rect(frame, bbox)
            if tracker_rect is None:
                self._publish_status(
                    {
                        "status": "failed",
                        "object": self._current_object,
                        "error": "Invalid detection bbox for tracker init",
                    }
                )
                return "Failed: invalid detection bbox"

            # Initialize tracker
            init_ok, init_reason = self._init_tracker(tracker, frame, tracker_rect)
            if not init_ok:
                if not self._local_detector_enabled:
                    self._publish_status(
                        {
                            "status": "failed",
                            "object": self._current_object,
                            "error": f"Tracker init failed: {init_reason}",
                        }
                    )
                    return f"Failed to initialize tracker: {init_reason}"
                # Fall back to detector-only loop so follow behavior still works
                # in OpenCV builds where tracker init is unreliable.
                logger.warning(f"Tracker init failed ({init_reason}); using detection fallback loop")
                self._current_object = object_name or "object"
                self._tracking_active = True
                self._publish_status(
                    {
                        "status": "tracking",
                        "object": self._current_object,
                        "mode": "detection_fallback",
                        "reason": init_reason,
                    }
                )
                self._tracking_thread = threading.Thread(
                    target=self._detection_servoing_loop,
                    args=(object_name, duration, self._target_distance_m),
                    daemon=True,
                )
                self._tracking_thread.start()
                return (
                    f"Tracking started for {self._current_object} "
                    "(detection fallback mode)"
                )

            self._current_object = object_name or "object"
            self._tracking_active = True

            # Start tracking in thread (non-blocking - caller should poll get_status())
            self._tracking_thread = threading.Thread(
                target=self._visual_servoing_loop, args=(tracker, duration), daemon=True
            )
            self._tracking_thread.start()

            return f"Tracking started for {self._current_object}. Poll get_status() for updates."

        except Exception as e:
            logger.error(f"Tracking error: {e}")
            self._stop_tracking()
            return f"Tracking failed: {e!s}"

    def _detect_initial_bbox(
        self,
        frame: np.ndarray[Any, np.dtype[Any]],
        object_name: str | None,
    ) -> tuple[int, int, int, int] | None:
        """Detect initial bbox using Qwen when available, with local fallback for person."""
        qwen_available = bool(os.getenv("ALIBABA_API_KEY"))
        if qwen_available:
            try:
                logger.info("Detecting object with Qwen...")
                # Qwen helper expects RGB input.
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                bbox = get_bbox_from_qwen_frame(rgb_frame, object_name)
                if bbox is not None:
                    return (int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3]))
            except Exception as e:
                logger.warning(f"Qwen detection failed: {e}")
        else:
            if not self._use_local_person_detector:
                logger.warning(
                    "ALIBABA_API_KEY is not set and local person detector fallback is disabled"
                )
                return None
            logger.warning("ALIBABA_API_KEY is not set; using local YOLO detector")

        if object_name is not None and object_name.lower().strip() not in PERSON_ALIASES:
            return None

        return self._detect_person_local(frame)

    def _bbox_iou(
        self,
        a: tuple[int, int, int, int],
        b: tuple[int, int, int, int],
    ) -> float:
        ax1, ay1, ax2, ay2 = a
        bx1, by1, bx2, by2 = b

        inter_x1 = max(ax1, bx1)
        inter_y1 = max(ay1, by1)
        inter_x2 = min(ax2, bx2)
        inter_y2 = min(ay2, by2)

        inter_w = max(0, inter_x2 - inter_x1)
        inter_h = max(0, inter_y2 - inter_y1)
        inter_area = inter_w * inter_h

        area_a = max(1, (ax2 - ax1) * (ay2 - ay1))
        area_b = max(1, (bx2 - bx1) * (by2 - by1))
        union = area_a + area_b - inter_area
        if union <= 0:
            return 0.0
        return inter_area / union

    def _is_plausible_person_bbox(
        self,
        frame: np.ndarray[Any, np.dtype[Any]],
        bbox: tuple[int, int, int, int],
        score: float,
    ) -> bool:
        frame_h, frame_w = frame.shape[:2]
        x1, y1, x2, y2 = bbox
        w = max(1, x2 - x1)
        h = max(1, y2 - y1)

        aspect = h / float(w)
        area_ratio = (w * h) / float(frame_w * frame_h)
        height_ratio = h / float(frame_h)

        if score < YOLO_PERSON_CONF_MIN:
            return False
        if aspect < 1.15 or aspect > 4.8:
            return False
        if height_ratio < 0.08 or height_ratio > 0.98:
            return False
        if area_ratio < 0.003 or area_ratio > 0.55:
            return False

        # Tiny edge-hugging boxes are usually false positives.
        edge_margin = 2
        touches_edge = (
            x1 <= edge_margin
            or y1 <= edge_margin
            or x2 >= frame_w - edge_margin
            or y2 >= frame_h - edge_margin
        )
        if touches_edge and area_ratio < 0.02:
            return False

        return True

    def _init_yolo_person_detector(self) -> None:
        """Initialize local YOLO pose model for person detection."""
        try:
            from ultralytics import YOLO  # type: ignore[attr-defined, import-not-found]

            device, cuda_ok = self._select_yolo_device()
            self._yolo_device = device
            self._yolo_half = cuda_ok
            configured_model = os.getenv("DIMOS_DRONE_YOLO_MODEL")
            self._yolo_model_name = (
                configured_model if configured_model else (YOLO_MODEL_NAME if cuda_ok else YOLO_MODEL_NAME_CPU)
            )

            model_path = get_data("models_yolo") / self._yolo_model_name
            self._yolo = YOLO(model_path, task="pose")
            warmup_ok = self._warmup_yolo()
            if not warmup_ok and self._yolo_device.startswith("cuda"):
                logger.warning("YOLO CUDA warmup failed; falling back to CPU")
                self._yolo_device = "cpu"
                self._yolo_half = False
                if configured_model is None and self._yolo_model_name != YOLO_MODEL_NAME_CPU:
                    self._yolo_model_name = YOLO_MODEL_NAME_CPU
                    cpu_model_path = get_data("models_yolo") / self._yolo_model_name
                    self._yolo = YOLO(cpu_model_path, task="pose")
                _ = self._warmup_yolo()
            logger.info(
                "Initialized YOLO person detector "
                f"model={self._yolo_model_name} device={self._yolo_device} "
                f"half={self._yolo_half} imgsz={self._yolo_imgsz} max_det={self._yolo_max_det}"
            )
        except Exception as e:
            self._yolo = None
            logger.warning(f"YOLO person detector unavailable: {e}")

    def _env_int(self, name: str, default: int, min_value: int = 1) -> int:
        raw = os.getenv(name)
        if raw is None:
            return default
        try:
            return max(min_value, int(raw))
        except ValueError:
            logger.warning(f"Invalid {name}={raw!r}; using default {default}")
            return default

    def _select_yolo_device(self) -> tuple[str, bool]:
        override = os.getenv("DIMOS_DRONE_YOLO_DEVICE")
        if override:
            normalized = override.strip().lower()
            if normalized == "cpu":
                return "cpu", False
            if normalized.startswith("cuda"):
                return normalized, True

        try:
            import torch

            if torch.cuda.is_available():
                # Force lazy init early so we can fail fast and avoid inference-time stalls.
                _ = torch.cuda.get_device_name(0)
                return "cuda:0", True

            if torch.cuda.device_count() > 0:
                logger.warning(
                    "Torch reports CUDA devices but initialization failed; using CPU. "
                    "Run `uv run python -c \"import torch; print(torch.cuda.is_available()); "
                    "print(torch.cuda.get_device_name(0))\"` to diagnose."
                )
        except Exception as e:
            logger.warning(f"Torch CUDA check failed; using CPU: {e}")

        return "cpu", False

    def _warmup_yolo(self) -> bool:
        if self._yolo is None:
            return False
        try:
            warmup = np.zeros((self._yolo_imgsz, self._yolo_imgsz, 3), dtype=np.uint8)
            _ = self._yolo.predict(
                source=warmup,
                conf=YOLO_PERSON_CONF_MIN,
                iou=0.5,
                classes=[0],
                imgsz=self._yolo_imgsz,
                max_det=1,
                verbose=False,
                device=self._yolo_device,
                half=self._yolo_half,
            )
            return True
        except Exception as e:
            logger.warning(f"YOLO warmup failed: {e}")
            return False

    def _detect_person_yolo_candidates(
        self,
        frame: np.ndarray[Any, np.dtype[Any]],
    ) -> list[tuple[tuple[int, int, int, int], float, int]]:
        """Detect person candidates using local YOLO pose model."""
        if self._yolo is None:
            return []

        try:
            results = self._yolo.predict(
                source=frame,
                conf=YOLO_PERSON_CONF_MIN,
                iou=0.5,
                classes=[0],  # person class
                imgsz=self._yolo_imgsz,
                max_det=self._yolo_max_det,
                verbose=False,
                device=self._yolo_device,
                half=self._yolo_half,
            )
        except Exception as e:
            logger.warning(f"YOLO detection inference failed: {e}")
            return []

        candidates: list[tuple[tuple[int, int, int, int], float, int]] = []
        frame_h, frame_w = frame.shape[:2]

        for result in results:
            boxes = getattr(result, "boxes", None)
            if boxes is None or boxes.xyxy is None:
                continue

            kp_conf = None
            keypoints = getattr(result, "keypoints", None)
            if keypoints is not None and getattr(keypoints, "conf", None) is not None:
                kp_conf = keypoints.conf

            num_dets = len(boxes.xyxy)
            for i in range(num_dets):
                try:
                    class_id = int(boxes.cls[i].item())
                    if class_id != 0:
                        continue
                    conf = float(boxes.conf[i].item())
                    xyxy = boxes.xyxy[i].cpu().numpy()
                    bbox = (int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3]))
                except Exception:
                    continue

                if not self._is_plausible_person_bbox(frame, bbox, conf):
                    continue

                visible_keypoints = 0
                if kp_conf is not None:
                    try:
                        kp = kp_conf[i].cpu().numpy()
                        visible_keypoints = int((kp > 0.35).sum())
                    except Exception:
                        visible_keypoints = 0

                if visible_keypoints < YOLO_PERSON_MIN_KEYPOINTS:
                    continue

                cx = (bbox[0] + bbox[2]) / 2.0
                cy = (bbox[1] + bbox[3]) / 2.0
                nx = abs((cx / frame_w) - 0.5)
                ny = abs((cy / frame_h) - 0.55)
                center_bonus = max(0.0, 1.0 - (nx + ny))

                score = conf + (0.2 * center_bonus) + (0.01 * visible_keypoints)
                candidates.append((bbox, score, visible_keypoints))

        return candidates

    def _detect_person_yolo(
        self,
        frame: np.ndarray[Any, np.dtype[Any]],
        log_miss: bool = True,
        log_hit: bool = True,
        anchor_bbox: tuple[int, int, int, int] | None = None,
    ) -> tuple[int, int, int, int] | None:
        """Detect best person using local YOLO pose model."""
        candidates = self._detect_person_yolo_candidates(frame)
        if not candidates:
            if log_miss:
                logger.info("YOLO detector found no person")
            return None

        best_bbox, best_score, best_kp = self._select_person_candidate(candidates, anchor_bbox)
        if log_hit:
            logger.info(
                f"YOLO person detection bbox={best_bbox} score={best_score:.3f} keypoints={best_kp}"
            )
        return best_bbox

    def _detect_person_local(
        self,
        frame: np.ndarray[Any, np.dtype[Any]],
        log_miss: bool = True,
        log_hit: bool = True,
        anchor_bbox: tuple[int, int, int, int] | None = None,
    ) -> tuple[int, int, int, int] | None:
        """Local detector path: YOLO person pose only."""
        return self._detect_person_yolo(
            frame,
            log_miss=log_miss,
            log_hit=log_hit,
            anchor_bbox=anchor_bbox,
        )

    def _select_person_candidate(
        self,
        candidates: list[tuple[tuple[int, int, int, int], float, int]],
        anchor_bbox: tuple[int, int, int, int] | None,
    ) -> tuple[tuple[int, int, int, int], float, int]:
        if anchor_bbox is None:
            return max(candidates, key=lambda x: x[1])

        def _rank(item: tuple[tuple[int, int, int, int], float, int]) -> float:
            bbox, score, _ = item
            iou_bonus = 0.35 * self._bbox_iou(anchor_bbox, bbox)
            return score + iou_bonus

        return max(candidates, key=_rank)

    def _confirm_person_detection(
        self,
        seed_bbox: tuple[int, int, int, int],
        timeout_s: float = 2.5,
    ) -> tuple[int, int, int, int] | None:
        """Require consistent person detections before enabling follow."""
        hits = 1
        misses = 0
        max_misses = 3
        last_bbox = seed_bbox
        deadline = time.time() + timeout_s

        while time.time() < deadline and hits < PERSON_CONFIRM_FRAMES:
            frame = self._get_latest_frame()
            if frame is None:
                time.sleep(0.03)
                continue

            candidate = self._detect_person_local(
                self._prepare_tracker_frame(frame),
                log_miss=False,
                log_hit=False,
                anchor_bbox=last_bbox,
            )
            if candidate is None:
                misses += 1
                if misses > max_misses:
                    return None
                time.sleep(0.05)
                continue

            iou = self._bbox_iou(last_bbox, candidate)
            if iou >= PERSON_CONFIRM_IOU:
                hits += 1
                last_bbox = candidate
            else:
                hits = 1
                last_bbox = candidate
            time.sleep(0.05)

        if hits >= PERSON_CONFIRM_FRAMES:
            return last_bbox
        return None

    def _draw_labeled_bbox(
        self,
        image: NDArray[np.uint8],
        bbox: tuple[int, int, int, int],
        label: str,
        color: tuple[int, int, int] = (0, 255, 0),
    ) -> NDArray[np.uint8]:  # type: ignore[type-arg]
        """Draw a bbox with text label."""
        overlay: NDArray[np.uint8] = image.copy()  # type: ignore[type-arg]
        x1, y1, x2, y2 = bbox
        cv2.rectangle(overlay, (x1, y1), (x2, y2), color, 2)
        text_y = max(18, y1 - 8)
        cv2.putText(
            overlay,
            label,
            (x1, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
        )
        return overlay

    def _passive_detection_overlay_loop(self) -> None:
        """Continuously publish detection overlay even when not actively following."""
        while self._passive_overlay_active:
            if self._tracking_active:
                time.sleep(PASSIVE_DETECT_INTERVAL_SEC)
                continue

            frame = self._get_latest_frame()
            if frame is None:
                time.sleep(PASSIVE_DETECT_INTERVAL_SEC)
                continue

            detect_frame = self._prepare_tracker_frame(frame)
            overlay = detect_frame.copy()

            yolo_candidates = self._detect_person_yolo_candidates(detect_frame)
            if yolo_candidates:
                sorted_candidates = sorted(yolo_candidates, key=lambda x: x[1], reverse=True)
                for bbox, score, kps in sorted_candidates[:3]:
                    overlay = self._draw_labeled_bbox(
                        overlay,
                        bbox,
                        f"person {score:.2f} kp={kps}",
                        color=(0, 255, 0),
                    )
                overlay = self._draw_search_overlay(
                    overlay,
                    f"Passive detect: {len(yolo_candidates)} person(s) [YOLO]",
                )
            else:
                overlay = self._draw_search_overlay(
                    overlay,
                    "Passive detect: no person",
                )

            if self.tracking_overlay.transport:
                self.tracking_overlay.publish(Image.from_numpy(overlay, format=ImageFormat.BGR))

            time.sleep(PASSIVE_DETECT_INTERVAL_SEC)

    def _compute_yaw_rate(self, current_x: float, center_x: float) -> float:
        """Compute yaw-rate with PID control on normalized horizontal image error."""
        now = time.time()
        raw_error = (current_x - center_x) / max(center_x, 1.0)

        # If we just crossed centerline, briefly brake yaw to avoid ping-pong from latency.
        crossed_center = (self._yaw_prev_raw_error * raw_error) < 0.0
        if crossed_center and max(abs(self._yaw_prev_raw_error), abs(raw_error)) <= YAW_ONLY_CROSS_BRAKE_ERR_RATIO:
            self._yaw_cross_brake_until = now + YAW_ONLY_CROSS_BRAKE_SEC
            self._yaw_rate_ema = 0.0
            self._yaw_pid.integral = 0.0
            self._yaw_pid.prev_error = 0.0
            self._yaw_lock_active = True

        if now < self._yaw_cross_brake_until:
            self._yaw_prev_raw_error = raw_error
            return 0.0

        self._yaw_error_lpf = (YAW_ONLY_ERROR_LPF_ALPHA * self._yaw_error_lpf) + (
            (1.0 - YAW_ONLY_ERROR_LPF_ALPHA) * raw_error
        )
        error = self._yaw_error_lpf

        # Hysteresis lock around centerline prevents sign-flip oscillation near zero error.
        if self._yaw_lock_active:
            if abs(raw_error) <= YAW_ONLY_LOCK_EXIT_RATIO:
                self._yaw_rate_ema = 0.0
                self._yaw_pid.integral = 0.0
                self._yaw_pid.prev_error = 0.0
                self._yaw_prev_raw_error = raw_error
                return 0.0
            self._yaw_lock_active = False

        target_rate = float(self._yaw_pid.update(error, YAW_ONLY_DT))  # type: ignore[no-untyped-call]

        # Keep enough turning authority once the target is meaningfully off-center.
        if abs(raw_error) >= YAW_ONLY_MIN_RATE_ERR_RATIO and abs(target_rate) < YAW_ONLY_MIN_RATE:
            sign = 1.0 if error > 0 else -1.0
            target_rate = sign * YAW_ONLY_MIN_RATE

        # Taper max yaw rate as target approaches center to avoid overshoot in large turns.
        abs_error = abs(raw_error)
        if abs_error < YAW_ONLY_DECEL_ERR_RATIO:
            frac = abs_error / max(YAW_ONLY_DECEL_ERR_RATIO, 1e-6)
            adaptive_limit = YAW_ONLY_DECEL_MIN_RATE + (
                (YAW_ONLY_MAX_RATE - YAW_ONLY_DECEL_MIN_RATE) * frac
            )
            target_rate = max(-adaptive_limit, min(adaptive_limit, target_rate))

        prev_rate = self._yaw_rate_ema
        filtered_rate = (YAW_ONLY_CMD_SMOOTH_ALPHA * prev_rate) + (
            (1.0 - YAW_ONLY_CMD_SMOOTH_ALPHA) * target_rate
        )

        # Limit per-step command delta to reduce oscillation and overshoot near lock.
        max_step = YAW_ONLY_RATE_SLEW_PER_SEC * YAW_ONLY_DT
        delta = filtered_rate - prev_rate
        if delta > max_step:
            filtered_rate = prev_rate + max_step
        elif delta < -max_step:
            filtered_rate = prev_rate - max_step
        self._yaw_rate_ema = filtered_rate

        if abs(target_rate) <= YAW_ONLY_ZERO_RATE_EPS and abs(self._yaw_rate_ema) <= YAW_ONLY_ZERO_RATE_EPS:
            self._yaw_rate_ema = 0.0

        if abs(raw_error) <= YAW_ONLY_LOCK_ENTER_RATIO and abs(self._yaw_rate_ema) <= YAW_ONLY_LOCK_CMD_EPS:
            self._yaw_lock_active = True
            self._yaw_rate_ema = 0.0
            self._yaw_pid.integral = 0.0
            self._yaw_pid.prev_error = 0.0
            self._yaw_prev_raw_error = raw_error
            return 0.0

        self._yaw_prev_raw_error = raw_error
        return float(self._yaw_rate_ema)

    def _reset_yaw_controller(self) -> None:
        self._yaw_rate_ema = 0.0
        self._yaw_error_lpf = 0.0
        self._yaw_lock_active = False
        self._yaw_prev_raw_error = 0.0
        self._yaw_cross_brake_until = 0.0
        self._yaw_pid.integral = 0.0
        self._yaw_pid.prev_error = 0.0

    def _compute_person_follow_command(
        self,
        bbox: tuple[int, int, int, int],
        frame_width: int,
        _frame_height: int,
        current_x: float,
    ) -> tuple[float, float, float, float]:
        yaw_rate = self._compute_yaw_rate(current_x, frame_width / 2.0)
        norm_yaw_error = abs((current_x - (frame_width / 2.0)) / max(frame_width / 2.0, 1.0))

        x1, _y1, x2, _y2 = bbox
        vx = FOLLOW_APPROACH_SPEED

        if norm_yaw_error >= FOLLOW_ALIGN_STOP_ERR:
            vx = 0.0
        elif norm_yaw_error > FOLLOW_ALIGN_SOFT_ERR:
            soft = 1.0 - (
                (norm_yaw_error - FOLLOW_ALIGN_SOFT_ERR)
                / max(FOLLOW_ALIGN_STOP_ERR - FOLLOW_ALIGN_SOFT_ERR, 1e-6)
            )
            vx *= max(0.25, min(1.0, soft))

        edge_margin = int(frame_width * FOLLOW_EDGE_MARGIN_RATIO)
        if (x1 < edge_margin or x2 > (frame_width - edge_margin)) and vx > 0.0:
            vx = 0.0

        return float(vx), 0.0, 0.0, float(yaw_rate)

    def _visual_servoing_loop(self, tracker: Any, duration: float) -> None:
        """Main visual servoing control loop.

        Args:
            tracker: OpenCV tracker instance
            duration: Maximum duration in seconds
        """
        start_time = time.time()
        frame_count = 0
        lost_track_count = 0
        max_lost_frames = 100

        logger.info("Starting visual servoing loop")

        try:
            while self._tracking_active and (time.time() - start_time < duration):
                # Get latest frame
                frame = self._get_latest_frame()
                if frame is None:
                    time.sleep(0.01)
                    continue

                frame_count += 1

                # Update tracker
                update_frame = self._prepare_tracker_frame(frame)
                update_result = tracker.update(update_frame)
                success = False
                bbox: Any = None
                if isinstance(update_result, tuple) and len(update_result) == 2:
                    success = bool(update_result[0])
                    bbox = update_result[1]

                if not success or bbox is None:
                    lost_track_count += 1
                    logger.warning(f"Lost track (count: {lost_track_count})")

                    if lost_track_count >= max_lost_frames:
                        logger.error("Lost track of object")
                        self._publish_status(
                            {"status": "lost", "object": self._current_object, "frame": frame_count}
                        )
                        break
                    continue
                else:
                    lost_track_count = 0

                # Calculate object center
                x, y, w, h = bbox
                current_x = x + w / 2
                current_y = y + h / 2

                # Get frame dimensions
                frame_height, frame_width = frame.shape[:2]
                center_x = frame_width / 2
                center_y = frame_height / 2

                yaw_rate = 0.0
                if self._control_mode == "yaw_only":
                    vx, vy, vz = 0.0, 0.0, 0.0
                    yaw_rate = self._compute_yaw_rate(current_x, center_x)
                else:
                    # Compute velocity commands
                    vx, vy, vz = self.servoing_controller.compute_velocity_control(
                        target_x=current_x,
                        target_y=current_y,
                        center_x=center_x,
                        center_y=center_y,
                        dt=0.033,  # ~30Hz
                        lock_altitude=True,
                    )

                    # Clamp velocity for indoor safety
                    if self._max_velocity is not None:
                        vx = max(-self._max_velocity, min(self._max_velocity, vx))
                        vy = max(-self._max_velocity, min(self._max_velocity, vy))
                    if self._max_forward_velocity is not None:
                        vx = max(-self._max_forward_velocity, min(self._max_forward_velocity, vx))
                    if self._max_lateral_velocity is not None:
                        vy = max(-self._max_lateral_velocity, min(self._max_lateral_velocity, vy))

                # Publish velocity command via LCM
                if self.cmd_vel.transport:
                    twist = Twist()
                    twist.linear = Vector3(vx, vy, 0)
                    twist.angular = Vector3(0, 0, yaw_rate)
                    self.cmd_vel.publish(twist)

                # Publish visualization if transport is set
                if self.tracking_overlay.transport:
                    overlay = self._draw_tracking_overlay(
                        frame, (int(x), int(y), int(w), int(h)), (int(current_x), int(current_y))
                    )
                    overlay_msg = Image.from_numpy(overlay, format=ImageFormat.BGR)
                    self.tracking_overlay.publish(overlay_msg)

                # Publish status
                self._publish_status(
                    {
                        "status": "tracking",
                        "object": self._current_object,
                        "bbox": [int(x), int(y), int(w), int(h)],
                        "center": [int(current_x), int(current_y)],
                        "error": [int(current_x - center_x), int(current_y - center_y)],
                        "velocity": [float(vx), float(vy), float(vz)],
                        "yaw_rate": float(yaw_rate),
                        "mode": self._control_mode,
                        "frame": frame_count,
                    }
                )

                # Control loop rate
                time.sleep(0.033)  # ~30Hz

        except Exception as e:
            logger.error(f"Error in servoing loop: {e}")
        finally:
            # Stop movement by publishing zero velocity
            if self.cmd_vel.transport:
                stop_twist = Twist()
                stop_twist.linear = Vector3(0, 0, 0)
                stop_twist.angular = Vector3(0, 0, 0)
                self.cmd_vel.publish(stop_twist)
            self._tracking_active = False
            logger.info(f"Visual servoing loop ended after {frame_count} frames")

    def _detection_servoing_loop(
        self, object_name: str | None, duration: float, distance_m: float
    ) -> None:
        """Detector-driven servoing loop that re-detects the target every frame."""
        start_time = time.time()
        frame_count = 0
        lost_track_count = 0
        max_lost_frames = 60
        confirm_hits = 0
        confirmed = False
        last_candidate_bbox: tuple[int, int, int, int] | None = None
        person_target = object_name is None or object_name.lower().strip() in PERSON_ALIASES

        logger.info("Starting detector-driven servoing loop")

        try:
            while self._tracking_active and (time.time() - start_time < duration):
                frame = self._get_latest_frame()
                if frame is None:
                    time.sleep(0.02)
                    continue

                frame_count += 1
                detect_frame = self._prepare_tracker_frame(frame)

                bbox_xyxy: tuple[int, int, int, int] | None
                if person_target:
                    bbox_xyxy = self._detect_person_local(
                        detect_frame,
                        log_miss=False,
                        log_hit=False,
                        anchor_bbox=self._person_lock_bbox,
                    )
                else:
                    bbox_xyxy = self._detect_initial_bbox(detect_frame, object_name)

                if bbox_xyxy is None:
                    lost_track_count += 1
                    if not confirmed:
                        confirm_hits = max(0, confirm_hits - 1)
                    if person_target and lost_track_count > 8:
                        self._person_lock_bbox = None
                    if lost_track_count % 10 == 0:
                        logger.warning(f"Detector loop has no target (count: {lost_track_count})")
                    if self.tracking_overlay.transport and lost_track_count % 5 == 0:
                        overlay = self._draw_search_overlay(
                            detect_frame, f"Searching for {object_name or 'object'}"
                        )
                        self.tracking_overlay.publish(Image.from_numpy(overlay, format=ImageFormat.BGR))
                    if lost_track_count >= max_lost_frames:
                        self._publish_status(
                            {"status": "lost", "object": self._current_object, "frame": frame_count}
                        )
                        break
                    time.sleep(0.05)
                    continue

                lost_track_count = 0
                if person_target:
                    self._person_lock_bbox = bbox_xyxy

                if not confirmed:
                    if (
                        last_candidate_bbox is None
                        or self._bbox_iou(last_candidate_bbox, bbox_xyxy) >= PERSON_CONFIRM_IOU
                    ):
                        confirm_hits += 1
                    else:
                        confirm_hits = 1
                    last_candidate_bbox = bbox_xyxy

                    if confirm_hits < PERSON_CONFIRM_FRAMES:
                        if self.cmd_vel.transport:
                            hold_twist = Twist()
                            hold_twist.linear = Vector3(0, 0, 0)
                            hold_twist.angular = Vector3(0, 0, 0)
                            self.cmd_vel.publish(hold_twist)
                        if self.tracking_overlay.transport:
                            cx = int((bbox_xyxy[0] + bbox_xyxy[2]) / 2)
                            cy = int((bbox_xyxy[1] + bbox_xyxy[3]) / 2)
                            overlay = self._draw_tracking_overlay(
                                detect_frame,
                                (
                                    int(bbox_xyxy[0]),
                                    int(bbox_xyxy[1]),
                                    int(max(1, bbox_xyxy[2] - bbox_xyxy[0])),
                                    int(max(1, bbox_xyxy[3] - bbox_xyxy[1])),
                                ),
                                (cx, cy),
                            )
                            overlay = self._draw_search_overlay(
                                overlay,
                                f"Candidate person {confirm_hits}/{PERSON_CONFIRM_FRAMES}",
                            )
                            self.tracking_overlay.publish(Image.from_numpy(overlay, format=ImageFormat.BGR))
                        time.sleep(0.05)
                        continue

                    confirmed = True
                    logger.info("Detector loop person target confirmed")

                x1, y1, x2, y2 = bbox_xyxy
                x = int(x1)
                y = int(y1)
                w = int(max(1, x2 - x1))
                h = int(max(1, y2 - y1))
                current_x = x + (w / 2.0)
                current_y = y + (h / 2.0)

                frame_height, frame_width = detect_frame.shape[:2]
                center_x = frame_width / 2
                center_y = frame_height / 2

                yaw_rate = 0.0
                bbox_height_ratio = h / float(max(frame_height, 1))
                if self._control_mode == "yaw_only":
                    vx, vy, vz = 0.0, 0.0, 0.0
                    yaw_rate = self._compute_yaw_rate(current_x, center_x)
                elif person_target and self._person_follow_policy == "yaw_forward_constant":
                    self._target_distance_m = max(
                        FOLLOW_TARGET_MIN_M, min(FOLLOW_TARGET_MAX_M, distance_m)
                    )
                    vx, vy, vz, yaw_rate = self._compute_person_follow_command(
                        (x, y, x + w, y + h),
                        frame_width,
                        frame_height,
                        current_x,
                    )
                else:
                    vx, vy, vz = self.servoing_controller.compute_velocity_control(
                        target_x=current_x,
                        target_y=current_y,
                        center_x=center_x,
                        center_y=center_y,
                        dt=0.05,
                        lock_altitude=True,
                    )
                    yaw_rate = 0.0

                    if self._max_velocity is not None:
                        vx = max(-self._max_velocity, min(self._max_velocity, vx))
                        vy = max(-self._max_velocity, min(self._max_velocity, vy))
                    if self._max_forward_velocity is not None:
                        vx = max(-self._max_forward_velocity, min(self._max_forward_velocity, vx))
                    if self._max_lateral_velocity is not None:
                        vy = max(-self._max_lateral_velocity, min(self._max_lateral_velocity, vy))

                if self.cmd_vel.transport:
                    twist = Twist()
                    twist.linear = Vector3(vx, vy, 0)
                    twist.angular = Vector3(0, 0, yaw_rate)
                    self.cmd_vel.publish(twist)

                if self.tracking_overlay.transport:
                    overlay = self._draw_tracking_overlay(
                        detect_frame, (x, y, w, h), (int(current_x), int(current_y))
                    )
                    overlay_msg = Image.from_numpy(overlay, format=ImageFormat.BGR)
                    self.tracking_overlay.publish(overlay_msg)

                self._publish_status(
                    {
                        "status": "tracking",
                        "object": self._current_object,
                        "backend": "yolo_detection",
                        "bbox": [x, y, w, h],
                        "center": [int(current_x), int(current_y)],
                        "error": [int(current_x - center_x), int(current_y - center_y)],
                        "velocity": [float(vx), float(vy), float(vz)],
                        "yaw_rate": float(yaw_rate),
                        "mode": self._control_mode,
                        "distance_target_m": float(self._target_distance_m),
                        "bbox_height_ratio": float(bbox_height_ratio),
                        "frame": frame_count,
                    }
                )

                time.sleep(0.05)

        except Exception as e:
            logger.error(f"Error in detection fallback loop: {e}")
        finally:
            if self.cmd_vel.transport:
                stop_twist = Twist()
                stop_twist.linear = Vector3(0, 0, 0)
                stop_twist.angular = Vector3(0, 0, 0)
                self.cmd_vel.publish(stop_twist)
            self._tracking_active = False
            logger.info(f"Detector loop ended after {frame_count} frames")

    def _draw_tracking_overlay(
        self,
        frame: NDArray[np.uint8],
        bbox: tuple[int, int, int, int],
        center: tuple[int, int],
    ) -> NDArray[np.uint8]:  # type: ignore[type-arg]
        """Draw tracking visualization overlay.

        Args:
            frame: Current video frame
            bbox: Bounding box (x, y, w, h)
            center: Object center (x, y)

        Returns:
            Frame with overlay drawn
        """
        overlay: NDArray[np.uint8] = frame.copy()  # type: ignore[type-arg]
        x, y, w, h = bbox

        # Draw tracking box (green)
        cv2.rectangle(overlay, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Draw object center (red crosshair)
        cv2.drawMarker(overlay, center, (0, 0, 255), cv2.MARKER_CROSS, 20, 2)

        # Draw desired center (blue crosshair)
        frame_h, frame_w = frame.shape[:2]
        frame_center = (frame_w // 2, frame_h // 2)
        cv2.drawMarker(overlay, frame_center, (255, 0, 0), cv2.MARKER_CROSS, 20, 2)

        # Draw line from object to desired center
        cv2.line(overlay, center, frame_center, (255, 255, 0), 1)

        # Add status text
        status_text = f"Tracking: {self._current_object}"
        cv2.putText(overlay, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Add error text
        error_x = center[0] - frame_center[0]
        error_y = center[1] - frame_center[1]
        error_text = f"Error: ({error_x}, {error_y})"
        cv2.putText(
            overlay, error_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1
        )

        return overlay

    def _draw_search_overlay(
        self,
        frame: NDArray[np.uint8],
        text: str,
    ) -> NDArray[np.uint8]:  # type: ignore[type-arg]
        """Draw lightweight debug overlay while target is not detected."""
        overlay: NDArray[np.uint8] = frame.copy()  # type: ignore[type-arg]
        frame_h, frame_w = overlay.shape[:2]
        center = (frame_w // 2, frame_h // 2)
        cv2.drawMarker(overlay, center, (255, 255, 0), cv2.MARKER_CROSS, 30, 2)
        cv2.putText(
            overlay,
            text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 200, 255),
            2,
        )
        return overlay

    def _publish_status(self, status: dict[str, Any]) -> None:
        """Publish tracking status as JSON.

        Args:
            status: Status dictionary
        """
        if self.tracking_status.transport:
            status_msg = String(json.dumps(status))
            self.tracking_status.publish(status_msg)

    def _stop_tracking(self) -> None:
        """Stop tracking and clean up."""
        self._tracking_active = False
        if self._tracking_thread and self._tracking_thread.is_alive():
            self._tracking_thread.join(timeout=1)

        # Send stop command via LCM
        if self.cmd_vel.transport:
            stop_twist = Twist()
            stop_twist.linear = Vector3(0, 0, 0)
            stop_twist.angular = Vector3(0, 0, 0)
            self.cmd_vel.publish(stop_twist)

        self._publish_status({"status": "stopped", "object": self._current_object})

        self._current_object = None
        self._control_mode = "full"
        self._person_lock_bbox = None
        self._target_distance_m = 1.0
        self._reset_yaw_controller()
        logger.info("Tracking stopped")

    @rpc
    def stop_tracking(self) -> str:
        """Stop current tracking operation."""
        self._stop_tracking()
        return "Tracking stopped"

    @rpc
    def get_status(self) -> dict[str, Any]:
        """Get current tracking status.

        Returns:
            Status dictionary
        """
        return {
            "active": self._tracking_active,
            "object": self._current_object,
            "has_frame": self._latest_frame is not None,
        }
