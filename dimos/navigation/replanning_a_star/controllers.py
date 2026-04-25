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

import math
from typing import Protocol

import numpy as np
from numpy.typing import NDArray

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.trajectory_command_limits import (
    HolonomicCommandLimits,
    clamp_holonomic_cmd_vel,
)
from dimos.navigation.trajectory_holonomic_tracking_controller import HolonomicTrackingController
from dimos.navigation.trajectory_types import TrajectoryMeasuredSample, TrajectoryReferenceSample
from dimos.utils.trigonometry import angle_diff


def _pose_from_xy_yaw(x: float, y: float, yaw: float) -> Pose:
    return Pose(
        position=Vector3(x, y, 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, float(yaw))),
    )


def _pose_from_pose_stamped(odom: PoseStamped) -> Pose:
    return Pose(odom.position, odom.orientation)


class Controller(Protocol):
    def advance(self, lookahead_point: NDArray[np.float64], current_odom: PoseStamped) -> Twist: ...

    def rotate(
        self, yaw_error: float, current_odom: PoseStamped | None = None
    ) -> Twist: ...

    def set_speed(self, speed_m_s: float) -> None: ...

    def reset_errors(self) -> None: ...

    def reset_yaw_error(self, value: float) -> None: ...


class PController:
    _global_config: GlobalConfig
    _speed: float
    _control_frequency: float

    _min_linear_velocity: float = 0.2
    _min_angular_velocity: float = 0.2
    _k_angular: float = 0.5
    _max_angular_accel: float = 2.0
    _rotation_threshold: float = 90 * (math.pi / 180)

    def __init__(self, global_config: GlobalConfig, speed: float, control_frequency: float) -> None:
        self._global_config = global_config
        self._speed = speed
        self._control_frequency = control_frequency

    def set_speed(self, speed_m_s: float) -> None:
        self._speed = float(speed_m_s)

    def advance(self, lookahead_point: NDArray[np.float64], current_odom: PoseStamped) -> Twist:
        current_pos = np.array([current_odom.position.x, current_odom.position.y])
        direction = lookahead_point - current_pos
        distance = np.linalg.norm(direction)

        if distance < 1e-6:
            # Robot is coincidentally at the lookahead point; skip this cycle.
            return Twist()

        robot_yaw = current_odom.orientation.euler[2]
        desired_yaw = np.arctan2(direction[1], direction[0])
        yaw_error = angle_diff(desired_yaw, robot_yaw)

        angular_velocity = self._compute_angular_velocity(yaw_error)

        # Rotate-then-drive: if heading error is large, rotate in place first
        if abs(yaw_error) > self._rotation_threshold:
            return self._angular_twist(angular_velocity)

        # When aligned, drive forward with proportional angular correction
        linear_velocity = self._speed * (1.0 - abs(yaw_error) / self._rotation_threshold)
        linear_velocity = self._apply_min_velocity(linear_velocity, self._min_linear_velocity)

        return Twist(
            linear=Vector3(linear_velocity, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, angular_velocity),
        )

    def rotate(
        self, yaw_error: float, current_odom: PoseStamped | None = None
    ) -> Twist:
        del current_odom
        angular_velocity = self._compute_angular_velocity(yaw_error)
        return self._angular_twist(angular_velocity)

    def _compute_angular_velocity(self, yaw_error: float) -> float:
        angular_velocity = self._k_angular * yaw_error
        angular_velocity = np.clip(angular_velocity, -self._speed, self._speed)
        angular_velocity = self._apply_min_velocity(angular_velocity, self._min_angular_velocity)
        return float(angular_velocity)

    def reset_errors(self) -> None:
        pass

    def reset_yaw_error(self, value: float) -> None:
        pass

    def _apply_min_velocity(self, velocity: float, min_velocity: float) -> float:
        """Apply minimum velocity threshold, preserving sign. Returns 0 if velocity is 0."""
        if velocity == 0.0:
            return 0.0
        if abs(velocity) < min_velocity:
            return min_velocity if velocity > 0 else -min_velocity
        return velocity

    def _angular_twist(self, angular_velocity: float) -> Twist:
        # In simulation, we need stroger values
        if self._global_config.simulation and abs(angular_velocity) < 0.8:
            angular_velocity = 0.8 * np.sign(angular_velocity)

        return Twist(
            linear=Vector3(0.0, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, angular_velocity),
        )


class PdController(PController):
    _k_derivative: float = 0.15

    _prev_yaw_error: float
    _prev_angular_velocity: float

    def __init__(self, global_config: GlobalConfig, speed: float, control_frequency: float) -> None:
        super().__init__(global_config, speed, control_frequency)

        self._prev_yaw_error = 0.0
        self._prev_angular_velocity = 0.0

    def reset_errors(self) -> None:
        self._prev_yaw_error = 0.0
        self._prev_angular_velocity = 0.0

    def reset_yaw_error(self, value: float) -> None:
        self._prev_yaw_error = value

    def _compute_angular_velocity(self, yaw_error: float) -> float:
        dt = 1.0 / self._control_frequency

        # PD control: proportional + derivative damping
        yaw_error_derivative = (yaw_error - self._prev_yaw_error) / dt
        angular_velocity = self._k_angular * yaw_error - self._k_derivative * yaw_error_derivative

        # Rate limiting: limit angular acceleration to prevent jerky corrections
        max_delta = self._max_angular_accel * dt
        angular_velocity = np.clip(
            angular_velocity,
            self._prev_angular_velocity - max_delta,
            self._prev_angular_velocity + max_delta,
        )

        angular_velocity = np.clip(angular_velocity, -self._speed, self._speed)
        angular_velocity = self._apply_min_velocity(angular_velocity, self._min_angular_velocity)

        self._prev_yaw_error = yaw_error
        self._prev_angular_velocity = angular_velocity

        return float(angular_velocity)


class HolonomicPathController:
    """Follow path segments using the holonomic tracking law (P3-3, issue 921).

    Wraps :class:`HolonomicTrackingController` in the :class:`Controller` seam
    (lookahead + odom). Rotations in place use the same law with a fixed
    position reference. Not a car-style or Pure Pursuit path law.
    """

    def __init__(
        self,
        global_config: GlobalConfig,
        speed: float,
        control_frequency: float,
        k_position_per_s: float,
        k_yaw_per_s: float,
    ) -> None:
        self._global_config = global_config
        self._speed = float(speed)
        self._control_frequency = float(control_frequency)
        self._inner = HolonomicTrackingController(
            k_position_per_s=k_position_per_s,
            k_yaw_per_s=k_yaw_per_s,
        )
        self._limits = HolonomicCommandLimits(
            max_planar_speed_m_s=self._speed,
            max_yaw_rate_rad_s=self._speed,
            max_planar_linear_accel_m_s2=5.0,
            max_yaw_accel_rad_s2=5.0,
        )
        self._inner.configure(self._limits)
        self._previous_cmd = Twist()

    def set_speed(self, speed_m_s: float) -> None:
        self._speed = float(speed_m_s)
        self._limits = HolonomicCommandLimits(
            max_planar_speed_m_s=self._speed,
            max_yaw_rate_rad_s=self._speed,
            max_planar_linear_accel_m_s2=5.0,
            max_yaw_accel_rad_s2=5.0,
        )
        self._inner.configure(self._limits)

    def advance(self, lookahead_point: NDArray[np.float64], current_odom: PoseStamped) -> Twist:
        current_pos = np.array(
            [float(current_odom.position.x), float(current_odom.position.y)]
        )
        direction = np.asarray(lookahead_point, dtype=np.float64) - current_pos
        distance = float(np.linalg.norm(direction))

        if distance < 1e-6 or not np.isfinite(distance):
            return Twist()

        ref_yaw = float(np.arctan2(direction[1], direction[0]))
        ref_pose = _pose_from_xy_yaw(
            float(lookahead_point[0]), float(lookahead_point[1]), ref_yaw
        )
        # Feedforward along the reference heading in the body frame of the target pose.
        ref_ff = Twist(
            linear=Vector3(self._speed, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, 0.0),
        )
        ref = TrajectoryReferenceSample(0.0, ref_pose, ref_ff)
        meas = TrajectoryMeasuredSample(0.0, _pose_from_pose_stamped(current_odom), Twist())
        return self._limit_output(self._inner.control(ref, meas))

    def rotate(
        self, yaw_error: float, current_odom: PoseStamped | None = None
    ) -> Twist:
        if current_odom is None:
            # ``LocalPlanner`` should always pass odom; keep a safe fallback.
            wz = float(0.5 * yaw_error)
            wz = float(np.clip(wz, -self._speed, self._speed))
            if wz != 0.0 and abs(wz) < 0.2:
                wz = 0.2 * (1.0 if wz > 0 else -1.0)
            t = Twist(
                linear=Vector3(0.0, 0.0, 0.0),
                angular=Vector3(0.0, 0.0, wz),
            )
            return self._limit_output(self._apply_sim_angular(t))

        robot_yaw = float(current_odom.orientation.euler[2])
        target_yaw = float(np.arctan2(np.sin(robot_yaw + yaw_error), np.cos(robot_yaw + yaw_error)))
        p = _pose_from_xy_yaw(
            float(current_odom.position.x),
            float(current_odom.position.y),
            target_yaw,
        )
        ref = TrajectoryReferenceSample(0.0, p, Twist())
        meas = TrajectoryMeasuredSample(0.0, _pose_from_pose_stamped(current_odom), Twist())
        out = self._inner.control(ref, meas)
        return self._limit_output(self._apply_sim_angular(out))

    def reset_errors(self) -> None:
        self._inner.reset()
        self._previous_cmd = Twist()

    def reset_yaw_error(self, value: float) -> None:
        del value

    def _apply_sim_angular(self, t: Twist) -> Twist:
        wz = float(t.angular.z)
        if self._global_config.simulation and 1e-9 < abs(wz) < 0.8:
            wz = 0.8 * (1.0 if wz > 0 else -1.0)
        return Twist(
            linear=Vector3(
                float(t.linear.x), float(t.linear.y), float(t.linear.z)
            ),
            angular=Vector3(0.0, 0.0, wz),
        )

    def _limit_output(self, raw: Twist) -> Twist:
        out = clamp_holonomic_cmd_vel(
            self._previous_cmd,
            raw,
            self._limits,
            1.0 / self._control_frequency,
        )
        self._previous_cmd = Twist(out)
        return out


def make_local_path_controller(
    global_config: GlobalConfig,
    speed: float,
    control_frequency: float,
) -> Controller:
    if global_config.local_planner_path_controller == "holonomic":
        return HolonomicPathController(
            global_config,
            speed,
            control_frequency,
            k_position_per_s=global_config.local_planner_holonomic_kp,
            k_yaw_per_s=global_config.local_planner_holonomic_ky,
        )
    return PController(global_config, speed, control_frequency)
