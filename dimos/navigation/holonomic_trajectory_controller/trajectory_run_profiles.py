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

"""Named Go2 movement envelopes (speed and limit caps).

Data and validation only. Live wiring: ``GlobalConfig.go2_run_profile``,
``LocalPlanner._resolve_run_envelope``, and ``go2.connection`` locomotion mode.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import math

from dimos.navigation.holonomic_trajectory_controller.trajectory_command_limits import HolonomicCommandLimits
from dimos.navigation.holonomic_trajectory_controller.trajectory_path_speed_profile import PathSpeedProfileLimits

_POSITIVE_FIELDS: tuple[str, ...] = (
    "requested_planner_speed_m_s",
    "max_tangent_accel_m_s2",
    "max_normal_accel_m_s2",
    "goal_decel_m_s2",
    "max_planar_cmd_accel_m_s2",
    "max_yaw_rate_rad_s",
    "max_yaw_accel_rad_s2",
)


class RunProfileError(ValueError):
    """Invalid run-profile definition (bad units or unknown name)."""


@dataclass(frozen=True)
class RunProfile:
    """One named movement envelope an operator may request."""

    name: str
    requested_planner_speed_m_s: float
    max_tangent_accel_m_s2: float
    max_normal_accel_m_s2: float
    goal_decel_m_s2: float
    max_planar_cmd_accel_m_s2: float
    max_yaw_rate_rad_s: float
    max_yaw_accel_rad_s2: float
    required_locomotion_mode: str
    description: str = ""

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise RunProfileError("run profile name must be non-empty")
        for field_name in _POSITIVE_FIELDS:
            value = getattr(self, field_name)
            if not math.isfinite(value) or value <= 0.0:
                raise RunProfileError(
                    f"{self.name!r}.{field_name} must be a positive finite float, got {value!r}"
                )
        if not self.required_locomotion_mode.strip():
            raise RunProfileError(f"{self.name!r}.required_locomotion_mode must be non-empty")

    def command_limits(self) -> HolonomicCommandLimits:
        """Body-frame command saturation envelope for this profile."""
        return HolonomicCommandLimits(
            max_planar_speed_m_s=self.requested_planner_speed_m_s,
            max_yaw_rate_rad_s=self.max_yaw_rate_rad_s,
            max_planar_linear_accel_m_s2=self.max_planar_cmd_accel_m_s2,
            max_yaw_accel_rad_s2=self.max_yaw_accel_rad_s2,
        )

    def path_speed_profile_limits_at(self, max_speed_m_s: float) -> PathSpeedProfileLimits:
        """Geometry-aware path speed limits at the given cruise cap (m/s)."""
        return PathSpeedProfileLimits(
            max_speed_m_s=max_speed_m_s,
            max_tangent_accel_m_s2=self.max_tangent_accel_m_s2,
            max_normal_accel_m_s2=self.max_normal_accel_m_s2,
        )


@dataclass(frozen=True)
class RunProfileRegistry:
    """Named run profiles with a declared default profile name."""

    profiles: Mapping[str, RunProfile]
    default_profile_name: str

    def __post_init__(self) -> None:
        if not self.profiles:
            raise RunProfileError("registry must define at least one profile")
        for key, profile in self.profiles.items():
            if key != profile.name:
                raise RunProfileError(
                    f"registry key {key!r} does not match profile name {profile.name!r}"
                )
        if self.default_profile_name not in self.profiles:
            raise RunProfileError(
                f"default profile {self.default_profile_name!r} is not in the registry"
            )
        object.__setattr__(self, "profiles", dict(self.profiles))

    def get(self, name: str) -> RunProfile:
        """Look up a profile by name; unknown names list the known profiles."""
        try:
            return self.profiles[name]
        except KeyError as exc:
            known = ", ".join(sorted(self.profiles))
            raise RunProfileError(f"unknown run profile {name!r}; known profiles: {known}") from exc

    def names(self) -> tuple[str, ...]:
        """Profile names in registration order."""
        return tuple(self.profiles)


GO2_RUN_PROFILES = RunProfileRegistry(
    default_profile_name="walk",
    profiles={
        "walk": RunProfile(
            name="walk",
            requested_planner_speed_m_s=0.55,
            max_tangent_accel_m_s2=1.0,
            max_normal_accel_m_s2=0.6,
            goal_decel_m_s2=1.0,
            max_planar_cmd_accel_m_s2=5.0,
            max_yaw_rate_rad_s=1.0,
            max_yaw_accel_rad_s2=5.0,
            required_locomotion_mode="default",
            description="Default cautious navigation; matches today's LocalPlanner walking behavior.",
        ),
        "trot": RunProfile(
            name="trot",
            requested_planner_speed_m_s=1.0,
            max_tangent_accel_m_s2=1.5,
            max_normal_accel_m_s2=0.8,
            goal_decel_m_s2=1.2,
            max_planar_cmd_accel_m_s2=5.0,
            max_yaw_rate_rad_s=1.2,
            max_yaw_accel_rad_s2=5.0,
            required_locomotion_mode="default",
            description="Faster trot within the default locomotion mode.",
        ),
        "run_conservative": RunProfile(
            name="run_conservative",
            requested_planner_speed_m_s=1.5,
            max_tangent_accel_m_s2=2.0,
            max_normal_accel_m_s2=1.0,
            goal_decel_m_s2=1.5,
            max_planar_cmd_accel_m_s2=6.0,
            max_yaw_rate_rad_s=1.0,
            max_yaw_accel_rad_s2=4.0,
            required_locomotion_mode="default",
            description=(
                "Conservative run envelope at planner speed caps; stays in default "
                "locomotion mode so onboard lidar and obstacle avoidance remain active."
            ),
        ),
        "run_verified": RunProfile(
            name="run_verified",
            requested_planner_speed_m_s=2.5,
            max_tangent_accel_m_s2=2.5,
            max_normal_accel_m_s2=1.2,
            goal_decel_m_s2=2.0,
            max_planar_cmd_accel_m_s2=6.0,
            max_yaw_rate_rad_s=0.8,
            max_yaw_accel_rad_s2=3.0,
            required_locomotion_mode="rage",
            description="Highest envelope; switches Go2 to rage locomotion mode.",
        ),
    },
)


__all__ = [
    "GO2_RUN_PROFILES",
    "RunProfile",
    "RunProfileError",
    "RunProfileRegistry",
]
