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

Data and validation only. Live wiring: ``DanHolonomicTCConfig.run_profile``, resolved
by ``DanHolonomicTC`` into ``HolonomicPathController.configure``.
"""

from __future__ import annotations

from dataclasses import dataclass, fields
import math

from dimos.navigation.experimental.dannav.geometry.path_speed_profile import PathSpeedProfileLimits


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
    max_yaw_rate_rad_s: float

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise RunProfileError("run profile name must be non-empty")
        for field in fields(self)[1:]:
            value = getattr(self, field.name)
            if not math.isfinite(value) or value <= 0.0:
                raise RunProfileError(
                    f"{self.name!r}.{field.name} must be a positive finite float, got {value!r}"
                )

    def path_speed_profile_limits_at(self, max_speed_m_s: float) -> PathSpeedProfileLimits:
        """Geometry-aware path speed limits at the given cruise cap (m/s)."""
        return PathSpeedProfileLimits(
            max_speed_m_s=max_speed_m_s,
            max_tangent_accel_m_s2=self.max_tangent_accel_m_s2,
            max_normal_accel_m_s2=self.max_normal_accel_m_s2,
            max_yaw_rate_rad_s=self.max_yaw_rate_rad_s,
        )


GO2_RUN_PROFILES: dict[str, RunProfile] = {
    profile.name: profile
    for profile in (
        RunProfile(
            name="walk",
            requested_planner_speed_m_s=0.55,
            max_tangent_accel_m_s2=1.0,
            max_normal_accel_m_s2=0.6,
            goal_decel_m_s2=0.5,
            max_yaw_rate_rad_s=1.0,
        ),
        RunProfile(
            name="trot",
            requested_planner_speed_m_s=1.0,
            max_tangent_accel_m_s2=1.5,
            max_normal_accel_m_s2=0.8,
            goal_decel_m_s2=1.2,
            max_yaw_rate_rad_s=1.2,
        ),
        RunProfile(
            name="run_conservative",
            requested_planner_speed_m_s=1.5,
            max_tangent_accel_m_s2=2.0,
            max_normal_accel_m_s2=1.0,
            goal_decel_m_s2=1.5,
            max_yaw_rate_rad_s=1.0,
        ),
    )
}


def get_run_profile(name: str) -> RunProfile:
    """Look up a profile by name; unknown names list the known profiles."""
    try:
        return GO2_RUN_PROFILES[name]
    except KeyError as exc:
        known = ", ".join(sorted(GO2_RUN_PROFILES))
        raise RunProfileError(f"unknown run profile {name!r}; known profiles: {known}") from exc
