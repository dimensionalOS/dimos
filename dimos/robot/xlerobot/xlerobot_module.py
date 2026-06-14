"""DimOS module for XLeRobot — full mobile manipulator.

Exposes the XLeRobot as a DimOS Module with:
  - cmd_vel input for velocity commands (Twist)
  - color_image output for camera frames
  - odom output for dead-reckoning odometry
  - joint_states output for arm/head joint positions
  - Skills: move_base, stop_base, observe, move_arm, move_head,
            open_gripper, close_gripper, get_joint_positions, home_arms
"""

from __future__ import annotations

import json
import logging
import math
import threading
import time
from typing import Any

import numpy as np
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.xlerobot.config import XLeRobotConfig
from dimos.robot.xlerobot.connection import XLeRobotDriver
from dimos.spec.perception import Camera

logger = logging.getLogger(__name__)


class XLeRobotConnection(Module):
    """DimOS module wrapping the full XLeRobot: omni base, dual arms, head, camera."""

    config: XLeRobotConfig

    # Inputs
    cmd_vel: In[Twist]

    # Outputs
    color_image: Out[Image]
    odom: Out[PoseStamped]
    joint_states: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._driver: XLeRobotDriver | None = None
        self._latest_image: Image | None = None
        self._running = False
        self._sensor_thread: threading.Thread | None = None
        self._recording: bool = False
        self._record_path: str | None = None
        self._record_data: list[dict[str, Any]] = []

        # Dead-reckoning state
        self._pose_x = 0.0
        self._pose_y = 0.0
        self._pose_theta = 0.0
        self._last_odom_time: float | None = None

    @rpc
    def start(self) -> None:
        """Connect to XLeRobot hardware and start sensor streaming."""
        super().start()
        self._driver = XLeRobotDriver(self.config)

        try:
            self._driver.connect()
        except Exception as e:
            logger.error(f"Failed to connect to XLeRobot hardware: {e}")
            logger.error(
                "Check that serial ports are correct and the robot is powered on. "
                f"Bus 1: {self.config.port_bus1}, Bus 2: {self.config.port_bus2}"
            )
            self._driver = None
            return

        # Subscribe cmd_vel → move
        if self.cmd_vel.transport:
            self.register_disposable(Disposable(self.cmd_vel.subscribe(self._on_cmd_vel)))

        # Start sensor polling thread
        self._running = True
        self._sensor_thread = threading.Thread(target=self._sensor_loop, daemon=True)
        self._sensor_thread.start()

        logger.info("XLeRobotConnection started")

    def _on_cmd_vel(self, twist: Twist) -> None:
        if self._driver and self._driver.connected:
            self._driver.move(
                x=twist.linear.x,
                y=twist.linear.y,
                theta=math.degrees(twist.angular.z),
            )

    def _sensor_loop(self) -> None:
        """Poll camera, wheel encoders, and joint states at ~20 Hz."""
        while self._running:
            try:
                if self._driver and self._driver.connected:
                    self._update_camera()
                    self._update_odom()
                    self._update_joint_states()
                time.sleep(0.05)
            except Exception as e:
                logger.debug(f"Sensor loop error: {e}")
                time.sleep(0.1)

    def _update_camera(self) -> None:
        assert self._driver is not None
        frame = self._driver.read_camera()
        if frame is not None:
            img = Image(
                data=frame,
                height=frame.shape[0],
                width=frame.shape[1],
                encoding="bgr8",
            )
            self._latest_image = img
            self.color_image.publish(img)

    def _update_odom(self) -> None:
        assert self._driver is not None
        now = time.time()
        if self._last_odom_time is None:
            self._last_odom_time = now
            return

        dt = now - self._last_odom_time
        self._last_odom_time = now

        vels = self._driver.read_wheel_velocities()
        vx = vels["x_vel"]
        vy = vels["y_vel"]
        vtheta = math.radians(vels["theta_vel"])

        cos_t = math.cos(self._pose_theta)
        sin_t = math.sin(self._pose_theta)
        self._pose_x += (vx * cos_t - vy * sin_t) * dt
        self._pose_y += (vx * sin_t + vy * cos_t) * dt
        self._pose_theta += vtheta * dt

        half = self._pose_theta / 2.0
        qz = math.sin(half)
        qw = math.cos(half)

        pose = PoseStamped(
            position=Vector3(self._pose_x, self._pose_y, 0.0),
            orientation=Quaternion(0.0, 0.0, qz, qw),
            frame_id="odom",
            ts=now,
        )
        self.odom.publish(pose)

        self.tf.publish(
            Transform(
                translation=Vector3(self._pose_x, self._pose_y, 0.0),
                rotation=Quaternion(0.0, 0.0, qz, qw),
                frame_id="odom",
                child_frame_id="base_link",
                ts=now,
            )
        )

    def _update_joint_states(self) -> None:
        """Publish arm + head joint positions as a JointState message."""
        assert self._driver is not None
        if not self._driver.arms_enabled and not self._driver.head_enabled:
            return

        try:
            positions = self._driver.read_joint_positions()
        except Exception:
            return

        if not positions:
            return

        names = list(positions.keys())
        pos_values = [float(positions[n]) for n in names]

        js = JointState(
            ts=time.time(),
            frame_id="base_link",
            name=names,
            position=pos_values,
            velocity=[0.0] * len(names),
            effort=[0.0] * len(names),
        )
        self.joint_states.publish(js)

        if self._recording:
            self._record_data.append({
                "ts": js.ts,
                "joint_positions": dict(zip(names, pos_values)),
            })

    # ---- Skills (exposed to MCP / agent) ----

    @skill
    def observe(self) -> Image | None:
        """Return the latest camera frame from the XLeRobot's head camera."""
        return self._latest_image

    @skill
    def move_base(
        self,
        x: float = 0.0,
        y: float = 0.0,
        theta: float = 0.0,
        duration: float = 0.0,
    ) -> str:
        """Drive the XLeRobot base.

        Args:
            x: forward speed in m/s (positive = forward)
            y: lateral speed in m/s (positive = left)
            theta: rotation speed in deg/s (positive = counter-clockwise)
            duration: seconds to move (0 = send single command)
        """
        if not self._driver or not self._driver.connected:
            return "Not connected"

        self._driver.move(x, y, theta)
        if duration > 0:
            time.sleep(duration)
            self._driver.stop_base()
            return f"Moved for {duration:.1f}s then stopped"
        return "Velocity command sent"

    @skill
    def stop_base(self) -> str:
        """Emergency stop — zero all wheel velocities."""
        if self._driver and self._driver.connected:
            self._driver.stop_base()
            return "Base stopped"
        return "Not connected"

    @skill
    def move_arm(self, arm: str, joint_positions: str, duration: float = 1.0) -> str:
        """Move an arm to target joint positions.

        Args:
            arm: which arm to move, either "left" or "right"
            joint_positions: JSON string mapping joint names to target values, e.g. '{"left_arm_shoulder_pan": 45.0}'
            duration: time to wait for motion to complete in seconds
        """
        if not self._driver or not self._driver.connected:
            return "Not connected"
        if not self._driver.arms_enabled:
            return "Arms are disabled in config"

        try:
            positions = json.loads(joint_positions) if isinstance(joint_positions, str) else joint_positions
        except json.JSONDecodeError as e:
            return f"Invalid joint_positions JSON: {e}"

        try:
            self._driver.move_arm(arm, positions)
            if duration > 0:
                time.sleep(duration)
            return f"Moved {arm} arm to target positions"
        except Exception as e:
            return f"Failed to move {arm} arm: {e}"

    @skill
    def move_head(self, pan: float, tilt: float) -> str:
        """Move the head to look in a direction.

        Args:
            pan: head_motor_1 target position (horizontal rotation)
            tilt: head_motor_2 target position (vertical rotation)
        """
        if not self._driver or not self._driver.connected:
            return "Not connected"
        if not self._driver.head_enabled:
            return "Head is disabled in config"

        try:
            self._driver.move_head(pan, tilt)
            return f"Head moved to pan={pan:.1f}, tilt={tilt:.1f}"
        except Exception as e:
            return f"Failed to move head: {e}"

    @skill
    def open_gripper(self, arm: str) -> str:
        """Open the gripper on the specified arm.

        Args:
            arm: which gripper to open, either "left" or "right"
        """
        if not self._driver or not self._driver.connected:
            return "Not connected"
        if not self._driver.arms_enabled:
            return "Arms are disabled in config"

        try:
            self._driver.open_gripper(arm)
            return f"{arm.capitalize()} gripper opened"
        except Exception as e:
            return f"Failed to open {arm} gripper: {e}"

    @skill
    def close_gripper(self, arm: str) -> str:
        """Close the gripper on the specified arm.

        Args:
            arm: which gripper to close, either "left" or "right"
        """
        if not self._driver or not self._driver.connected:
            return "Not connected"
        if not self._driver.arms_enabled:
            return "Arms are disabled in config"

        try:
            self._driver.close_gripper(arm)
            return f"{arm.capitalize()} gripper closed"
        except Exception as e:
            return f"Failed to close {arm} gripper: {e}"

    @skill
    def get_joint_positions(self) -> str:
        """Return all arm and head joint positions as a JSON string."""
        if not self._driver or not self._driver.connected:
            return "Not connected"

        try:
            positions = self._driver.read_joint_positions()
            return json.dumps(positions, indent=2)
        except Exception as e:
            return f"Failed to read joints: {e}"

    @skill
    def home_arms(self) -> str:
        """Move both arms and head to their zero (home) position."""
        if not self._driver or not self._driver.connected:
            return "Not connected"
        if not self._driver.arms_enabled:
            return "Arms are disabled in config"

        try:
            self._driver.home_arms()
            time.sleep(2.0)
            return "Arms and head moved to home position"
        except Exception as e:
            return f"Failed to home arms: {e}"

    # ---- @rpc methods ----

    @rpc
    def record(self, name: str) -> str:
        """Start recording camera + joint states to disk.

        Args:
            name: recording session name (used as subdirectory)
        """
        from pathlib import Path

        record_dir = Path.home() / ".cache" / "xlerobot" / "recordings" / name
        record_dir.mkdir(parents=True, exist_ok=True)
        self._record_path = str(record_dir)
        self._record_data = []
        self._recording = True
        logger.info(f"Recording started: {self._record_path}")
        return f"Recording to {self._record_path}"

    @rpc
    def stop_recording(self) -> str:
        """Stop recording and save data to disk."""
        if not self._recording:
            return "Not recording"

        self._recording = False
        if self._record_path and self._record_data:
            from pathlib import Path

            out = Path(self._record_path) / "joint_states.json"
            with open(out, "w") as f:
                json.dump(self._record_data, f, indent=2)
            count = len(self._record_data)
            self._record_data = []
            return f"Saved {count} frames to {out}"
        return "No data recorded"

    @rpc
    def stop(self) -> None:
        """Stop the module and disconnect hardware."""
        self._running = False
        if self._sensor_thread and self._sensor_thread.is_alive():
            self._sensor_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)

        if self._recording:
            self.stop_recording()

        if self._driver and self._driver.connected:
            self._driver.disconnect()

        logger.info("XLeRobotConnection stopped")
        super().stop()
