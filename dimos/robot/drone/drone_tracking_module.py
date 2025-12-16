#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
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
import threading
import time
from typing import Any

import cv2
from dimos_lcm.std_msgs import String  # type: ignore[import-untyped]
import numpy as np

from dimos.core import In, Module, Out, rpc
from dimos.models.qwen.video_query import get_bbox_from_qwen_frame
from dimos.msgs.geometry_msgs import Twist, Vector3
from dimos.msgs.sensor_msgs import Image, ImageFormat
from dimos.robot.drone.drone_visual_servoing_controller import (
    DroneVisualServoingController,
    PIDParams,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

INDOOR_PID_PARAMS: PIDParams = (0.001, 0.0, 0.0001, (-1.0, 1.0), None, 30)
OUTDOOR_PID_PARAMS: PIDParams = (0.05, 0.0, 0.0003, (-5.0, 5.0), None, 10)
INDOOR_MAX_VELOCITY = 1.0  # m/s safety cap for indoor mode


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
        """
        super().__init__()

        default_params = OUTDOOR_PID_PARAMS if outdoor else INDOOR_PID_PARAMS
        x_pid_params = x_pid_params if x_pid_params is not None else default_params
        y_pid_params = y_pid_params if y_pid_params is not None else default_params

        self._outdoor = outdoor
        self._max_velocity = None if outdoor else INDOOR_MAX_VELOCITY

        self.servoing_controller = DroneVisualServoingController(
            x_pid_params=x_pid_params, y_pid_params=y_pid_params, z_pid_params=z_pid_params
        )

        # Tracking state
        self._tracking_active = False
        self._tracking_thread: threading.Thread | None = None
        self._current_object: str | None = None
        self._latest_frame: Image | None = None
        self._frame_lock = threading.Lock()

        # Subscribe to video input when transport is set
        # (will be done by connection module)

    def _on_new_frame(self, frame: Image) -> None:
        """Handle new video frame."""
        with self._frame_lock:
            self._latest_frame = frame

    def _on_follow_object_cmd(self, cmd: String) -> None:
        msg = json.loads(cmd.data)
        self.track_object(msg["object_description"], msg["duration"])

    def _get_latest_frame(self) -> np.ndarray[Any, np.dtype[Any]] | None:
        """Get the latest video frame as numpy array."""
        with self._frame_lock:
            if self._latest_frame is None:
                return None
            # Convert Image to numpy array
            data: np.ndarray[Any, np.dtype[Any]] = self._latest_frame.data
            return data

    @rpc
    def start(self) -> bool:
        """Start the tracking module and subscribe to video input."""
        if self.video_input.transport:
            self.video_input.subscribe(self._on_new_frame)
            logger.info("DroneTrackingModule started - subscribed to video input")
        else:
            logger.warning("DroneTrackingModule: No video input transport configured")

        if self.follow_object_cmd.transport:
            self.follow_object_cmd.subscribe(self._on_follow_object_cmd)

        return True

    @rpc
    def stop(self) -> None:
        self._stop_tracking()
        super().stop()

    @rpc
    def track_object(self, object_name: str | None = None, duration: float = 120.0) -> str:
        """Track and follow an object using visual servoing.

        Args:
            object_name: Name of object to track, or None for most prominent
            duration: Maximum tracking duration in seconds

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
            # Detect object with Qwen
            logger.info("Detecting object with Qwen...")
            bbox = get_bbox_from_qwen_frame(frame, object_name)

            if bbox is None:
                msg = f"No object detected{' for: ' + object_name if object_name else ''}"
                logger.warning(msg)
                self._publish_status({"status": "not_found", "object": self._current_object})
                return msg

            logger.info(f"Object detected at bbox: {bbox}")

            # Initialize CSRT tracker (use legacy for OpenCV 4)
            try:
                tracker = cv2.legacy.TrackerCSRT_create()  # type: ignore[attr-defined]
            except AttributeError:
                tracker = cv2.TrackerCSRT_create()  # type: ignore[attr-defined]

            # Convert bbox format from [x1, y1, x2, y2] to [x, y, w, h]
            x1, y1, x2, y2 = bbox
            x, y, w, h = x1, y1, x2 - x1, y2 - y1

            # Initialize tracker
            success = tracker.init(frame, (x, y, w, h))
            if not success:
                self._publish_status({"status": "failed", "object": self._current_object})
                return "Failed to initialize tracker"

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
                success, bbox = tracker.update(frame)

                if not success:
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

                # Publish velocity command via LCM
                if self.cmd_vel.transport:
                    twist = Twist()
                    twist.linear = Vector3(vx, vy, 0)
                    twist.angular = Vector3(0, 0, 0)  # No rotation for now
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

    def _draw_tracking_overlay(
        self,
        frame: np.ndarray[Any, np.dtype[Any]],
        bbox: tuple[int, int, int, int],
        center: tuple[int, int],
    ) -> np.ndarray[Any, np.dtype[Any]]:
        """Draw tracking visualization overlay.

        Args:
            frame: Current video frame
            bbox: Bounding box (x, y, w, h)
            center: Object center (x, y)

        Returns:
            Frame with overlay drawn
        """
        overlay = frame.copy()
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
