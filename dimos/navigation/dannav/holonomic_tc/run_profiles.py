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

Data and validation only. Live wiring: ``DanHolonomicTCConfig.run_profile`` and
``_HolonomicPathFollower._resolve_run_envelope``.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import math

from dimos.navigation.dannav.geometry.path_speed_profile import PathSpeedProfileLimits

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

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise RunProfileError("run profile name must be non-empty")
        for field_name in _POSITIVE_FIELDS:
            value = getattr(self, field_name)
            if not math.isfinite(value) or value <= 0.0:
                raise RunProfileError(
                    f"{self.name!r}.{field_name} must be a positive finite float, got {value!r}"
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
    """Named run profiles."""

    profiles: Mapping[str, RunProfile]

    def __post_init__(self) -> None:
        if not self.profiles:
            raise RunProfileError("registry must define at least one profile")
        for key, profile in self.profiles.items():
            if key != profile.name:
                raise RunProfileError(
                    f"registry key {key!r} does not match profile name {profile.name!r}"
                )
        object.__setattr__(self, "profiles", dict(self.profiles))

    def get(self, name: str) -> RunProfile:
        """Look up a profile by name; unknown names list the known profiles."""
        try:
            return self.profiles[name]
        except KeyError as exc:
            known = ", ".join(sorted(self.profiles))
            raise RunProfileError(f"unknown run profile {name!r}; known profiles: {known}") from exc


GO2_RUN_PROFILES = RunProfileRegistry(
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
        ),
    },
)
