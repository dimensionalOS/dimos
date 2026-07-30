# Copyright 2026 Dimensional Inc.
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

from __future__ import annotations

from dataclasses import dataclass
import math
from threading import Condition
import time
from typing import Any, Literal

from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.unitree.unitree_skill_container import UnitreeSkillContainer

_CONTROL_PERIOD_SEC = 0.1
_STOP_PUBLISH_PERIOD_SEC = 0.05
_STOP_PUBLISH_COUNT = 5
_ODOM_READY_TIMEOUT_SEC = 1.0
_ODOM_STALE_SEC = 1.0
_MAX_TRANSLATION_M = 1.0
_MAX_ROTATION_DEG = 60.0
_LINEAR_COMMAND_MPS = 0.2
_ANGULAR_COMMAND_RAD_PER_SEC = 0.35
_ANGULAR_CORRECTION_RAD_PER_SEC = 0.08
_TRANSLATION_TOLERANCE_M = 0.04
_ROTATION_TOLERANCE_RAD = math.radians(2.0)
_SETTLED_POSITION_TOLERANCE_M = 0.003
_SETTLED_YAW_TOLERANCE_RAD = math.radians(0.3)
_SETTLED_SAMPLE_COUNT = 3
_SETTLE_TIMEOUT_SEC = 2.0
_MAX_ROTATION_CORRECTIONS = 3
_MIN_PROGRESS_M = 0.01
_MIN_ROTATION_PROGRESS_RAD = math.radians(0.5)
_MAX_STAGNANT_SAMPLES = 25

_SegmentStatus = Literal["completed", "blocked", "odometry-timeout"]


@dataclass(frozen=True)
class _TranslationResult:
    status: _SegmentStatus
    along_m: float
    lateral_m: float


@dataclass(frozen=True)
class _RotationResult:
    status: _SegmentStatus
    radians: float


def _normalize_angle(radians: float) -> float:
    return math.atan2(math.sin(radians), math.cos(radians))


def _yaw(pose: PoseStamped) -> float:
    return pose.orientation.to_euler().yaw


class DimSimUnitreeSkillContainer(UnitreeSkillContainer):
    """Unitree skills with bounded, collision-aware local motion for DimSim."""

    odom: In[PoseStamped]
    tele_cmd_vel: Out[Twist]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._odom_condition = Condition()
        self._latest_odom: PoseStamped | None = None
        self._latest_odom_received_at = float("-inf")

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            Disposable(self.odom.subscribe(self._on_odom)),
        )

    def _on_odom(self, pose: PoseStamped) -> None:
        with self._odom_condition:
            if self._latest_odom is not None and pose.ts <= self._latest_odom.ts:
                return
            self._latest_odom = pose
            self._latest_odom_received_at = time.monotonic()
            self._odom_condition.notify_all()

    def _fresh_odom(self, timeout: float = _ODOM_READY_TIMEOUT_SEC) -> PoseStamped | None:
        deadline = time.monotonic() + timeout
        with self._odom_condition:
            while True:
                now = time.monotonic()
                if (
                    self._latest_odom is not None
                    and now - self._latest_odom_received_at <= _ODOM_STALE_SEC
                ):
                    return self._latest_odom
                remaining = deadline - now
                if remaining <= 0:
                    return None
                self._odom_condition.wait(remaining)

    def _next_odom(self, after_ts: float, timeout: float) -> PoseStamped | None:
        deadline = time.monotonic() + timeout
        with self._odom_condition:
            while self._latest_odom is None or self._latest_odom.ts <= after_ts:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._odom_condition.wait(remaining)
            return self._latest_odom

    @skill(uses=["movement"])
    def relative_move(
        self,
        forward: float = 0.0,
        left: float = 0.0,
        degrees: float = 0.0,
    ) -> str:
        """Move the simulated robot relative to its current camera heading.

        Rapier remains authoritative for collision handling. Translation is
        limited to 1 metre and rotation to 60 degrees per call. The result
        reports measured progress, allowing blocked motion to be distinguished
        from a completed move.

        The camera image is not mirrored: an object on the left side of the
        image requires a positive `left` value, and an object on the right side
        requires a negative `left` value. For example,
        `relative_move(left=0.5)` moves toward screen-left and
        `relative_move(left=-0.5)` moves toward screen-right.

        Args:
            forward: Forward distance in metres; negative moves backward.
            left: Screen-left distance in metres; negative moves screen-right.
            degrees: Final relative yaw in degrees; positive turns left.
        """
        forward, left, degrees = float(forward), float(left), float(degrees)
        if not all(math.isfinite(value) for value in (forward, left, degrees)):
            raise ValueError("relative movement values must be finite")

        requested_distance = math.hypot(forward, left)
        if requested_distance > _MAX_TRANSLATION_M:
            return (
                f"Movement rejected: request {_MAX_TRANSLATION_M:.2f}m or less per call, "
                "then observe."
            )
        if abs(degrees) > _MAX_ROTATION_DEG:
            return (
                f"Movement rejected: request at most {_MAX_ROTATION_DEG:.0f} degrees "
                "per call, then observe."
            )
        if requested_distance == 0 and degrees == 0:
            return "No movement requested."

        start_pose = self._fresh_odom()
        if start_pose is None:
            return "Movement not started: no fresh DimSim odometry is available."

        translation: _TranslationResult | None = None
        rotation: _RotationResult | None = None
        try:
            if requested_distance:
                translation = self._run_translation(
                    start_pose,
                    forward,
                    left,
                    requested_distance,
                )
            if degrees:
                rotation_start = self._fresh_odom()
                if rotation_start is None:
                    rotation = _RotationResult("odometry-timeout", 0.0)
                else:
                    rotation = self._run_precise_rotation(
                        rotation_start,
                        math.radians(degrees),
                    )
        finally:
            self._clear_velocity()

        return self._format_result(
            requested_distance=requested_distance,
            requested_degrees=degrees,
            translation=translation,
            rotation=rotation,
        )

    def _clear_velocity(self) -> None:
        """Hold zero velocity long enough to cross the process/LCM boundary.

        A single final publication can be overtaken by odometry and lidar work
        in the live DimSim stack. Repeating zero over several control periods
        makes the stop authoritative before a later tool call starts.
        """
        zero = Twist.zero()
        for index in range(_STOP_PUBLISH_COUNT):
            self.tele_cmd_vel.publish(zero)
            if index + 1 < _STOP_PUBLISH_COUNT:
                time.sleep(_STOP_PUBLISH_PERIOD_SEC)

    def _settled_odom(self) -> PoseStamped | None:
        """Return odometry after consecutive stationary server poses."""
        current = self._fresh_odom()
        if current is None:
            return None

        deadline = time.monotonic() + _SETTLE_TIMEOUT_SEC
        stable_samples = 0
        while time.monotonic() < deadline:
            next_pose = self._next_odom(
                current.ts,
                min(_CONTROL_PERIOD_SEC, deadline - time.monotonic()),
            )
            if next_pose is None:
                continue

            position_delta = (next_pose.position - current.position).magnitude()
            yaw_delta = abs(_normalize_angle(_yaw(next_pose) - _yaw(current)))
            if (
                position_delta <= _SETTLED_POSITION_TOLERANCE_M
                and yaw_delta <= _SETTLED_YAW_TOLERANCE_RAD
            ):
                stable_samples += 1
                if stable_samples >= _SETTLED_SAMPLE_COUNT:
                    return next_pose
            else:
                stable_samples = 0
            current = next_pose

        return current

    def _run_precise_rotation(
        self,
        start_pose: PoseStamped,
        requested_radians: float,
    ) -> _RotationResult:
        """Correct delayed-odometry overshoot against an absolute yaw target."""
        target_yaw = _normalize_angle(_yaw(start_pose) + requested_radians)
        current = start_pose
        command_radians = requested_radians
        speed = _ANGULAR_COMMAND_RAD_PER_SEC

        for attempt in range(_MAX_ROTATION_CORRECTIONS + 1):
            self._run_rotation(current, command_radians, speed)
            self._clear_velocity()
            settled = self._settled_odom()
            if settled is None:
                return _RotationResult("odometry-timeout", 0.0)

            actual = _normalize_angle(_yaw(settled) - _yaw(start_pose))
            error = _normalize_angle(target_yaw - _yaw(settled))
            if abs(error) <= _ROTATION_TOLERANCE_RAD:
                return _RotationResult("completed", actual)
            if attempt == _MAX_ROTATION_CORRECTIONS:
                return _RotationResult("blocked", actual)

            current = settled
            command_radians = error
            speed = _ANGULAR_CORRECTION_RAD_PER_SEC

        raise AssertionError("unreachable")

    def _run_translation(
        self,
        start_pose: PoseStamped,
        forward: float,
        left: float,
        requested_distance: float,
    ) -> _TranslationResult:
        local_direction = Vector3(forward / requested_distance, left / requested_distance, 0)
        world_direction = start_pose.orientation.rotate_vector(local_direction)
        perpendicular = Vector3(-world_direction.y, world_direction.x, 0)
        command = Twist(
            linear=Vector3(
                _LINEAR_COMMAND_MPS * forward / requested_distance,
                _LINEAR_COMMAND_MPS * left / requested_distance,
                0,
            ),
            angular=Vector3(),
        )

        deadline = time.monotonic() + min(8.0, max(2.0, requested_distance * 4.0 + 2.0))
        last_pose = start_pose
        best_progress = 0.0
        stagnant_samples = 0
        along = 0.0
        lateral = 0.0
        last_command_at = float("-inf")

        while time.monotonic() < deadline:
            now = time.monotonic()
            if now - last_command_at >= _CONTROL_PERIOD_SEC:
                self.tele_cmd_vel.publish(command)
                last_command_at = now
            next_pose = self._next_odom(last_pose.ts, _CONTROL_PERIOD_SEC)
            if next_pose is None:
                continue
            last_pose = next_pose

            delta = next_pose.position - start_pose.position
            along = delta.dot(world_direction)
            lateral = abs(delta.dot(perpendicular))
            if along >= requested_distance - _TRANSLATION_TOLERANCE_M:
                return _TranslationResult("completed", along, lateral)

            if along >= best_progress + _MIN_PROGRESS_M:
                best_progress = along
                stagnant_samples = 0
            else:
                stagnant_samples += 1
                if stagnant_samples >= _MAX_STAGNANT_SAMPLES:
                    return _TranslationResult("blocked", max(0.0, along), lateral)

        status: _SegmentStatus = "blocked" if last_pose.ts > start_pose.ts else "odometry-timeout"
        return _TranslationResult(status, max(0.0, along), lateral)

    def _run_rotation(
        self,
        start_pose: PoseStamped,
        requested_radians: float,
        command_speed: float = _ANGULAR_COMMAND_RAD_PER_SEC,
    ) -> _RotationResult:
        direction = math.copysign(1.0, requested_radians)
        command = Twist(
            linear=Vector3(),
            angular=Vector3(0, 0, direction * command_speed),
        )
        deadline = time.monotonic() + min(
            6.0,
            max(2.0, abs(requested_radians) * 3.0 + 2.0),
        )
        last_pose = start_pose
        last_yaw = _yaw(start_pose)
        accumulated = 0.0
        best_progress = 0.0
        stagnant_samples = 0
        last_command_at = float("-inf")

        while time.monotonic() < deadline:
            now = time.monotonic()
            if now - last_command_at >= _CONTROL_PERIOD_SEC:
                self.tele_cmd_vel.publish(command)
                last_command_at = now
            next_pose = self._next_odom(last_pose.ts, _CONTROL_PERIOD_SEC)
            if next_pose is None:
                continue
            last_pose = next_pose

            next_yaw = _yaw(next_pose)
            accumulated += _normalize_angle(next_yaw - last_yaw)
            last_yaw = next_yaw
            progress = direction * accumulated
            if progress >= abs(requested_radians) - _ROTATION_TOLERANCE_RAD:
                return _RotationResult("completed", accumulated)

            if progress >= best_progress + _MIN_ROTATION_PROGRESS_RAD:
                best_progress = progress
                stagnant_samples = 0
            else:
                stagnant_samples += 1
                if stagnant_samples >= _MAX_STAGNANT_SAMPLES:
                    return _RotationResult("blocked", accumulated)

        status: _SegmentStatus = "blocked" if last_pose.ts > start_pose.ts else "odometry-timeout"
        return _RotationResult(status, accumulated)

    @staticmethod
    def _format_result(
        *,
        requested_distance: float,
        requested_degrees: float,
        translation: _TranslationResult | None,
        rotation: _RotationResult | None,
    ) -> str:
        segments: list[str] = []
        statuses: list[_SegmentStatus] = []
        if translation is not None:
            statuses.append(translation.status)
            segments.append(
                f"travelled {translation.along_m:.2f}m of the requested "
                f"{requested_distance:.2f}m along the requested direction "
                f"(lateral drift {translation.lateral_m:.2f}m)"
            )
        if rotation is not None:
            statuses.append(rotation.status)
            segments.append(
                f"turned {math.degrees(rotation.radians):.1f} degrees of the requested "
                f"{requested_degrees:.1f} degrees"
            )

        detail = "; ".join(segments)
        if "odometry-timeout" in statuses:
            return f"Movement stopped because DimSim odometry became unavailable: {detail}."
        if "blocked" in statuses:
            return (
                f"Movement was blocked or only partially completed: {detail}. "
                "Rotate toward another open route, then observe."
            )
        return f"Movement completed: {detail}. Observe to verify the camera view."
