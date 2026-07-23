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

"""Static feedforward gain compensator (Strategy B).

Sits between any path-following controller and the platform. Rather than
closing a velocity loop with a PID (which requires actual_velocity feedback
and is fragile when cascaded over a firmware that already tracks velocity),
this compensator just **inverts the steady-state plant gain** so the
controller's "I want vx=X" command actually produces vx=X at the wheels:

    cmd_to_robot = controller_cmd / K_plant

Stateless, no actual feedback needed, no phase-margin issues. Works as
long as K is reasonably accurate. Trade: doesn't compensate for plant
dynamics (tau, L) - controller's own outer loop handles those via pose
feedback.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


@dataclass
class FeedforwardGainConfig:
    """Steady-state plant gains. Default = unity (passthrough).

    For Go2, do not hardcode: read the vendored fit
    ``dimos.control.benchmarking.plant_models.GO2_PLANT_FITTED`` (currently
    ``K_vx≈0.92``, ``K_wz≈2.45``). A stale hardcoded ``K_wz=2.175`` copy
    silently mis-calibrated every FF controller; the single source of
    truth is plant_models.
    """

    K_vx: float = 1.0
    K_vy: float = 1.0
    K_wz: float = 1.0
    output_min_vx: float = -1.0
    output_max_vx: float = 1.0
    output_min_vy: float = -1.0
    output_max_vy: float = 1.0
    output_min_wz: float = -1.5
    output_max_wz: float = 1.5


def validate_plant_gains(K_vx: float, K_vy: float, K_wz: float) -> None:
    """Raise if any gain is non-finite or non-positive. Callers divide by these
    (u/K, envelope/K limits): ~0 blows up, and a negative gain reverses the
    command (and inverts the envelope bounds). Validate before that division,
    not only at construction."""
    for axis, k in (("vx", K_vx), ("vy", K_vy), ("wz", K_wz)):
        if not math.isfinite(k) or k <= 1e-6:
            raise ValueError(
                f"plant gain K_{axis}={k} is not usable (must be finite and "
                f"positive); invalid calibration artifact."
            )


class FeedforwardGainCompensator:
    """Divide controller-output velocities by plant gains; clamp to limits.

    API mirrors :class:`VelocityTrackingPID.compute` so it slots into the
    same place in the path-follower task pipeline. ``actual_*`` arguments
    are accepted but ignored - this is pure feedforward.
    """

    def __init__(self, config: FeedforwardGainConfig | None = None) -> None:
        self.cfg = config or FeedforwardGainConfig()
        validate_plant_gains(self.cfg.K_vx, self.cfg.K_vy, self.cfg.K_wz)

    def compute(
        self,
        desired_vx: float,
        desired_vy: float,
        desired_wz: float,
        actual_vx: float = 0.0,
        actual_vy: float = 0.0,
        actual_wz: float = 0.0,
    ) -> tuple[float, float, float]:
        return (
            _clamp(desired_vx / self.cfg.K_vx, self.cfg.output_min_vx, self.cfg.output_max_vx),
            _clamp(desired_vy / self.cfg.K_vy, self.cfg.output_min_vy, self.cfg.output_max_vy),
            _clamp(desired_wz / self.cfg.K_wz, self.cfg.output_min_wz, self.cfg.output_max_wz),
        )

    def reset(self) -> None:
        # Stateless. Method exists so it's drop-in for VelocityTrackingPID.
        pass
