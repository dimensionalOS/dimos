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

"""One ordinary gripper joint backed by the vendor R1Pro position controller.

Native position is millimetres. Lifecycle calls do not enable motors or move jaws.
The already-running vendor controller owns interpolation, speed and gains.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any

from dimos.hardware.manipulators.spec import ControlMode, ManipulatorInfo
from dimos.hardware.spec import JointLimits
from dimos.protocol.pubsub.impl.rospubsub import RawROS, RawROSTopic

_ROS_IMPORT_ERROR: ImportError | None = None
try:
    from hdas_msg.msg import FeedbackStatus  # type: ignore[import-not-found, import-untyped]
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState as RosJointState
except ImportError as error:
    _ROS_IMPORT_ERROR = error
    FeedbackStatus = None
    RosJointState = None


def _make_qos() -> Any:
    return QoSProfile(
        depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE
    )


_MAX_OPENING_MM = 100.0
_VENDOR_MAX_VELOCITY_MM_S = 30.0


class R1ProGripperAdapter:
    """ManipulatorAdapter for exactly one side; commands require live fault-free feedback."""

    def __init__(self, *, side: str, dof: int = 1, feedback_timeout: float = 2.0, **_: Any) -> None:
        if side not in ("left", "right") or dof != 1:
            raise ValueError("R1Pro gripper requires side left/right and exactly one joint")
        if not math.isfinite(feedback_timeout) or feedback_timeout <= 0:
            raise ValueError("feedback_timeout must be finite and positive")
        self._side = side
        self._timeout = feedback_timeout
        self._lock = threading.RLock()
        self._ros: RawROS | None = None
        self._publisher: Any = None
        self._active = False
        self._mode = ControlMode.SERVO_POSITION
        self._sample: tuple[float, float] | None = None
        self._faults: dict[str, tuple[list[str], float]] = {}
        self._last_target: float | None = None
        self._state_invalidated = False
        self._was_ready = False

    @property
    def command_topic(self) -> str:
        return f"/motion_target/target_position_gripper_{self._side}"

    def connect(self) -> bool:
        if RosJointState is None or FeedbackStatus is None:
            raise ImportError(
                "R1Pro gripper requires ROS Humble Python 3.10 and vendor messages"
            ) from _ROS_IMPORT_ERROR
        with self._lock:
            if self._ros is not None:
                return True
            ros = RawROS(node_name=f"dimos_gripper_{self._side}")
            ros.start()
            self._ros = ros
        try:
            node = ros._node
            assert node is not None
            self._publisher = node.create_publisher(RosJointState, self.command_topic, _make_qos())
            ros.subscribe(
                RawROSTopic(f"/hdas/feedback_gripper_{self._side}", RosJointState, _make_qos()),
                self._on_feedback,
            )
            for segment in ("torso", f"arm_{self._side}", f"gripper_{self._side}"):
                ros.subscribe(
                    RawROSTopic(f"/hdas/feedback_status_{segment}", FeedbackStatus, _make_qos()),
                    self._on_status,
                )
            return True
        except BaseException:
            self.disconnect()
            raise

    def disconnect(self) -> None:
        with self._lock:
            ros, self._ros = self._ros, None
            self._active = False
            self._publisher = None
            self._sample = None
            self._faults.clear()
            self._last_target = None
            self._state_invalidated = True
            self._was_ready = False
        if ros is not None:
            ros.stop()

    def is_connected(self) -> bool:
        return self._ros is not None

    def activate(self) -> bool:
        """Permit explicit writes locally; never enable physical motors or publish a target."""
        with self._lock:
            self._active = self.is_connected()
            return self._active

    def deactivate(self) -> bool:
        """Drop local write permission without releasing an object or sending a target."""
        with self._lock:
            self._active = False
            self._last_target = None
        return True

    def _on_feedback(self, message: Any, _topic: RawROSTopic) -> None:
        with self._lock:
            self._sample = None
            if len(message.position) != 1 or not math.isfinite(message.position[0]):
                self._state_invalidated = True
                self._last_target = None
                return
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            age = max(0.0, time.time() - stamp) if stamp else 0.0
            self._sample = (float(message.position[0]), time.monotonic() - age)

    def _on_status(self, message: Any, topic: RawROSTopic) -> None:
        segment = topic.topic.removeprefix("/hdas/feedback_status_")
        errors = [
            f"{e.name}: {e.error_code} {e.error_description}"
            for e in message.errors
            if e.error_code or e.error_description
        ]
        if not message.errors:
            errors = ["empty status report"]
        with self._lock:
            self._faults[segment] = (errors, time.monotonic())
            if errors:
                self._state_invalidated = True
                self._last_target = None

    def get_diagnostics(self) -> dict[str, Any]:
        """Report readiness, including whether dedicated gripper fault feedback exists."""
        with self._lock:
            now = time.monotonic()
            reasons = []
            age = now - self._sample[1] if self._sample else None
            if age is None or age > self._timeout:
                reasons.append("missing or stale gripper position")
            for segment in ("torso", f"arm_{self._side}"):
                status = self._faults.get(segment)
                if status is None or now - status[1] > self._timeout:
                    reasons.append(f"missing or stale {segment} fault report")
                elif status[0]:
                    reasons.extend(status[0])
            gripper_status = self._faults.get(f"gripper_{self._side}")
            if gripper_status is not None:
                if now - gripper_status[1] > self._timeout:
                    reasons.append("stale gripper fault report")
                reasons.extend(gripper_status[0])
            node = self._ros._node if self._ros is not None else None
            if node is None:
                reasons.append("ROS bridge not connected")
            else:
                if node.count_subscribers(self.command_topic) != 1:
                    reasons.append("expected exactly one vendor target subscriber")
                if node.count_publishers(self.command_topic) != 1:
                    reasons.append("another target publisher exists or discovery is incomplete")
            if reasons and self._was_ready:
                self._state_invalidated = True
            if self._state_invalidated:
                reasons.append("waiting for a control read tick to discard old intent")
            self._was_ready = not reasons
            if reasons:
                self._last_target = None
            return {
                "side": self._side,
                "ready": not reasons,
                "reasons": reasons,
                "position_mm": self._sample[0] if self._sample else None,
                "feedback_age_s": age,
                "gripper_fault_status_available": gripper_status is not None,
            }

    def acknowledge_unavailable(self) -> None:
        """Called only after the control read tick omits this device's joints."""
        with self._lock:
            self._state_invalidated = False

    def has_live_state(self) -> bool:
        return bool(self.get_diagnostics()["ready"])

    def get_info(self) -> ManipulatorInfo:
        return ManipulatorInfo(vendor="Galaxea", model=f"R1Pro {self._side} gripper", dof=1)

    def get_dof(self) -> int:
        return 1

    def get_limits(self) -> JointLimits:
        # Vendor gripper_controller_max_vel is 30 mm/s; writes do not override it.
        return JointLimits(
            position_lower=[0.0],
            position_upper=[_MAX_OPENING_MM],
            velocity_max=[_VENDOR_MAX_VELOCITY_MM_S],
        )

    def set_control_mode(self, mode: ControlMode) -> bool:
        if mode not in (ControlMode.POSITION, ControlMode.SERVO_POSITION):
            return False
        self._mode = mode
        return True

    def get_control_mode(self) -> ControlMode:
        return self._mode

    def read_joint_positions(self) -> list[float]:
        with self._lock:
            if not self.has_live_state() or self._sample is None:
                raise RuntimeError("No live R1Pro gripper state")
            return [self._sample[0]]

    def read_joint_velocities(self) -> list[float]:
        return [0.0]  # Vendor documents this feedback field as unused.

    def read_joint_efforts(self) -> list[float]:
        return [0.0]  # No verified force/torque measurement is available.

    def read_state(self) -> dict[str, int]:
        diagnostics = self.get_diagnostics()
        return {
            "connected": int(self.is_connected()),
            "active": int(self._active),
            "gripper_fault_status_available": int(diagnostics["gripper_fault_status_available"]),
        }

    def read_error(self) -> tuple[int, str]:
        reasons = self.get_diagnostics()["reasons"]
        return (1, "; ".join(reasons)) if reasons else (0, "")

    def write_joint_positions(self, positions: list[float], velocity: float = 1.0) -> bool:
        """Accept mm targets; vendor controller controls speed. Do not reset it every tick."""
        if (
            len(positions) != 1
            or not math.isfinite(positions[0])
            or not 0 <= positions[0] <= _MAX_OPENING_MM
        ):
            return False
        with self._lock:
            if not self._active or not self.has_live_state() or self._publisher is None:
                return False
            target = float(positions[0])
            if self._last_target == target:
                return True
            assert self._ros is not None and self._ros._node is not None
            message = RosJointState()
            message.header.stamp = self._ros._node.get_clock().now().to_msg()
            message.name = [f"{self._side}_gripper"]
            message.position = [target]
            self._publisher.publish(message)
            self._last_target = target
            return True

    def write_joint_velocities(self, velocities: list[float]) -> bool:
        return False

    def write_stop(self) -> bool:
        """No verified vendor cancel exists; never claim a physical emergency stop."""
        with self._lock:
            self._last_target = None
        return False

    def write_enable(self, enable: bool) -> bool:
        return False  # Servo enable/disable belongs to the external vendor stack.

    def read_enabled(self) -> bool:
        return self._active and self.has_live_state()

    def write_clear_errors(self) -> bool:
        return False

    def read_cartesian_position(self) -> dict[str, float] | None:
        return None

    def write_cartesian_position(self, pose: dict[str, float], velocity: float = 1.0) -> bool:
        return False

    def read_force_torque(self) -> list[float] | None:
        return None
