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

"""Frenet-frame Lyapunov path-following controller for holonomic mobile bases.

Computes ALL velocity components (vx, vy, wz) reactively from the robot's
current error state relative to the path.  No precomputed velocity profiles.

Error decomposition (Frenet frame of closest path point):
    e_s  — along-track error  (ahead/behind)
    e_d  — cross-track error  (lateral offset, signed)
    e_θ  — heading error      (robot heading vs path tangent)

Control law:
    v_ref = v_max · s(e_d, e_θ, κ, d_goal)      — reactive reference speed
    vx    = v_ref · cos(e_θ) + k_s · e_s         — forward (+ along-track catch-up)
    vy    = -k_d · e_d                            — lateral correction (holonomic)
    wz    = v_ref · κ + k_θ · sin(e_θ)           — feedforward curvature + feedback heading

Stability: Lyapunov candidate V = ½(e_s² + e_d² + e_θ²) yields dV/dt < 0
for appropriate gain selection → exponential error convergence.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import TYPE_CHECKING

import numpy as np

from dimos.utils.trigonometry import angle_diff

if TYPE_CHECKING:
    from dimos.control.tasks.path_distancer import PathDistancer
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


@dataclass
class LyapunovPathControllerConfig:
    """Gains and limits for the Lyapunov path controller.

    All gains have physical meaning — see inline comments.
    """

    # Speed limits
    v_max: float = 0.6  # max forward speed (m/s)
    vy_max: float = 0.3  # max lateral speed (m/s)
    wz_max: float = 1.2  # max angular rate (rad/s)

    # Lyapunov feedback gains
    k_s: float = 0.8  # along-track: how aggressively to catch up/slow down
    k_d: float = 1.0  # cross-track: lateral correction strength (vy gain)
    k_theta: float = 1.5  # heading: angular correction strength

    # Reactive speed modulation
    k_cte: float = 4.0  # CTE penalty on vx — higher = slower when off-path
    k_heading: float = 2.0  # heading error penalty on vx
    k_curv: float = 1.0  # curvature penalty on vx

    # Goal approach
    decel_radius: float = 0.5  # start decelerating this far from goal (m)

    # Minimum forward crawl (avoids stalling when errors are tiny)
    min_vx: float = 0.05


@dataclass(frozen=True)
class ControlOutput:
    """Reactive velocity command from the controller."""

    vx: float
    vy: float
    wz: float

    # Diagnostic info (for logging / benchmarks)
    e_s: float = 0.0
    e_d: float = 0.0
    e_theta: float = 0.0
    v_ref: float = 0.0


class LyapunovPathController:
    """Fully reactive holonomic path-following controller.

    Call ``compute()`` each control tick with the current odom and path.
    All velocity components are derived from the instantaneous error state.
    """

    def __init__(self, config: LyapunovPathControllerConfig | None = None) -> None:
        self._cfg = config or LyapunovPathControllerConfig()

    @property
    def config(self) -> LyapunovPathControllerConfig:
        return self._cfg

    def compute(
        self,
        odom: PoseStamped,
        distancer: PathDistancer,
        distance_to_goal: float,
    ) -> ControlOutput:
        """Compute reactive (vx, vy, wz) from current error state.

        Args:
            odom: Current robot pose.
            distancer: PathDistancer wrapping the target path.
            distance_to_goal: Euclidean distance to final path point (m).

        Returns:
            ControlOutput with clamped velocities and diagnostic errors.
        """
        cfg = self._cfg
        pos = np.array([odom.position.x, odom.position.y])
        robot_yaw = odom.orientation.euler[2]

        # ---- Error decomposition in Frenet frame ----
        closest_idx = distancer.find_closest_point_index(pos)
        e_d = distancer.get_signed_cross_track_error(pos)
        curvature = distancer.get_curvature_at_index(closest_idx)
        tangent_yaw = self._path_tangent_yaw(distancer, closest_idx)
        e_theta = angle_diff(tangent_yaw, robot_yaw)

        # Along-track error: project position error onto tangent direction
        closest_pt = distancer._path[closest_idx]
        diff = pos - closest_pt
        tangent_vec = np.array([math.cos(tangent_yaw), math.sin(tangent_yaw)])
        e_s = float(np.dot(diff, tangent_vec))

        # ---- Reactive reference speed ----
        cte_factor = 1.0 / (1.0 + cfg.k_cte * e_d * e_d)
        heading_factor = 1.0 / (1.0 + cfg.k_heading * e_theta * e_theta)
        curv_factor = 1.0 / math.sqrt(1.0 + cfg.k_curv * curvature * curvature)
        approach = min(1.0, distance_to_goal / cfg.decel_radius) if cfg.decel_radius > 0 else 1.0

        v_ref = cfg.v_max * cte_factor * heading_factor * curv_factor * approach

        # ---- Control law ----
        vx = v_ref * math.cos(e_theta) + cfg.k_s * e_s
        vy = -cfg.k_d * e_d
        wz = v_ref * curvature + cfg.k_theta * math.sin(e_theta)

        # ---- Clamp outputs ----
        vx = float(np.clip(vx, 0.0, cfg.v_max))
        if 0.0 < vx < cfg.min_vx:
            vx = cfg.min_vx
        vy = float(np.clip(vy, -cfg.vy_max, cfg.vy_max))
        wz = float(np.clip(wz, -cfg.wz_max, cfg.wz_max))

        return ControlOutput(
            vx=vx,
            vy=vy,
            wz=wz,
            e_s=e_s,
            e_d=e_d,
            e_theta=e_theta,
            v_ref=v_ref,
        )

    @staticmethod
    def _path_tangent_yaw(distancer: PathDistancer, index: int) -> float:
        """Compute tangent heading at a path index."""
        path = distancer._path
        if index < len(path) - 1:
            dx = path[index + 1][0] - path[index][0]
            dy = path[index + 1][1] - path[index][1]
        elif index > 0:
            dx = path[index][0] - path[index - 1][0]
            dy = path[index][1] - path[index - 1][1]
        else:
            return 0.0
        return float(np.arctan2(dy, dx))


__all__ = [
    "ControlOutput",
    "LyapunovPathController",
    "LyapunovPathControllerConfig",
]
