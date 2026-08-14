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

import numpy as np

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.dannav.holonomic_tc.command_limits import (
    HolonomicCommandLimits,
    clamp_holonomic_cmd_vel,
)
from dimos.navigation.dannav.holonomic_tc.holonomic_tracking_controller import (
    HolonomicTrackingController,
)
from dimos.navigation.dannav.holonomic_tc.run_profiles import RunProfile
from dimos.navigation.dannav.holonomic_tc.types import (
    TrajectoryMeasuredSample,
    TrajectoryReferenceSample,
)
from dimos.utils.transform_utils import normalize_angle


def _pose_from_xy_yaw(x: float, y: float, yaw: float) -> Pose:
    return Pose(
        position=Vector3(x, y, 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, float(yaw))),
    )


def _pose_from_pose_stamped(odom: PoseStamped) -> Pose:
    return Pose(odom.position, odom.orientation)


class HolonomicPathController:
    """Follow path segments using the holonomic tracking law.

    Wraps :class:`HolonomicTrackingController` in the :class:`Controller` seam
    (lookahead + odom). Rotations in place use the same law with a fixed
    position reference. Not a car-style or Pure Pursuit path law.
    """

    def __init__(
        self,
        global_config: GlobalConfig,
        profile: RunProfile,
        speed: float,
        control_frequency: float,
        k_position_per_s: float,
        k_yaw_per_s: float,
        k_velocity_per_s: float = 0.0,
        k_yaw_rate_per_s: float = 0.0,
    ) -> None:
        self._global_config = global_config
        self._profile = profile
        self._speed = float(speed)
        self._control_frequency = float(control_frequency)
        self._inner = HolonomicTrackingController(
            k_position_per_s=k_position_per_s,
            k_yaw_per_s=k_yaw_per_s,
            k_velocity_per_s=k_velocity_per_s,
            k_yaw_rate_per_s=k_yaw_rate_per_s,
        )
        self._limits = self._make_limits()
        self._inner.configure(self._limits)
        self._previous_cmd = Twist()

    def set_speed(self, speed_m_s: float) -> None:
        self._speed = float(speed_m_s)
        self._limits = self._make_limits()
        self._inner.configure(self._limits)

    def set_profile(self, profile: RunProfile) -> None:
        """Apply a run profile's command saturation caps."""
        self._profile = profile
        self._limits = self._make_limits()
        self._inner.configure(self._limits)

    def _make_limits(self) -> HolonomicCommandLimits:
        profile = self._profile
        return HolonomicCommandLimits(
            max_planar_speed_m_s=self._speed,
            max_yaw_rate_rad_s=profile.max_yaw_rate_rad_s,
            max_planar_linear_accel_m_s2=profile.max_planar_cmd_accel_m_s2,
            max_yaw_accel_rad_s2=profile.max_yaw_accel_rad_s2,
        )

    def advance_reference(
        self,
        reference: TrajectoryReferenceSample,
        current_odom: PoseStamped,
        measured_body_twist: Twist | None = None,
    ) -> Twist:
        twist = Twist() if measured_body_twist is None else measured_body_twist
        meas = TrajectoryMeasuredSample(0.0, _pose_from_pose_stamped(current_odom), twist)
        return self._limit_output(self._inner.control(reference, meas))

    def rotate(
        self,
        yaw_error: float,
        current_odom: PoseStamped | None = None,
        measured_body_twist: Twist | None = None,
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
        target_yaw = float(normalize_angle(robot_yaw + yaw_error))
        p = _pose_from_xy_yaw(
            float(current_odom.position.x),
            float(current_odom.position.y),
            target_yaw,
        )
        ref = TrajectoryReferenceSample(0.0, p, Twist())
        twist = Twist() if measured_body_twist is None else measured_body_twist
        meas = TrajectoryMeasuredSample(0.0, _pose_from_pose_stamped(current_odom), twist)
        out = self._inner.control(ref, meas)
        return self._limit_output(self._apply_sim_angular(out))

    def reset_errors(self) -> None:
        self._inner.reset()
        self._previous_cmd = Twist()

    def _apply_sim_angular(self, t: Twist) -> Twist:
        wz = float(t.angular.z)
        if self._global_config.simulation and 1e-9 < abs(wz) < 0.8:
            wz = 0.8 * (1.0 if wz > 0 else -1.0)
        return Twist(
            linear=Vector3(float(t.linear.x), float(t.linear.y), float(t.linear.z)),
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
