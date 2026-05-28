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

"""Precision path-follower: PathFollowerTask + live e_max corridor.

Subclass of :class:`PathFollowerTask` that adds a reactive precision
input: ``set_e_max(value)`` updates the RG corridor half-width and
recomputes the per-waypoint speed profile in place. The parent's tick
loop / ``compute()`` consumes the swapped ``_velocity_profile`` array
unchanged — no override of the control law needed.

The plant model and velocity-profile constants come from a tuning
artifact loaded once on the first ``start_path()`` call.
"""

from __future__ import annotations

from pathlib import Path as _Path
from typing import TYPE_CHECKING, Any

import numpy as np
from numpy.typing import NDArray

from dimos.control.tasks.path_follower_task.path_follower_task import (
    PathFollowerTask,
    PathFollowerTaskConfig,
)
from dimos.core.global_config import global_config as _gc
from dimos.navigation.reference_governor.reference_governor import (
    GeometricMVC,
    LateralMVC,
    PrecisionMVC,
    SaturationMVC,
    solve_profile,
)
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.benchmarking.tuning import TuningConfig
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.global_config import GlobalConfig
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.nav_msgs.Path import Path

logger = setup_logger()


class PrecisionPathFollowerTask(PathFollowerTask):
    """Path follower whose speed profile reacts to a live corridor
    half-width (e_max). The ControlCoordinator broadcasts e_max updates
    via :meth:`set_e_max`; each update triggers ``solve_profile()`` and
    atomically swaps the parent's ``_velocity_profile`` so the per-tick
    indexing inherits the new values immediately."""

    def __init__(
        self,
        name: str,
        config: PathFollowerTaskConfig,
        global_config: GlobalConfig,
        artifact_path: str,
        e_max_default: float = 0.05,
    ) -> None:
        super().__init__(name, config, global_config=global_config)
        self._artifact_path = artifact_path
        self._e_max: float = float(e_max_default)
        # Plant + vp_spec lazy-load on first start_path().
        self._plant: Any = None
        self._vp_spec: Any = None
        self._constraints: list[Any] | None = None
        # Cached path geometry — solve_profile() accepts these as kwargs
        # so e_max updates skip the re-computation.
        self._cached_pts: NDArray[np.float64] | None = None
        self._cached_curvatures: NDArray[np.float64] | None = None

    # ------------------------------------------------------------------
    # ControlCoordinator stream hook
    # ------------------------------------------------------------------

    def set_e_max(self, value: float) -> None:
        """Coordinator broadcast hook. Update e_max; recompute profile if
        a path is loaded."""
        self._e_max = float(value)
        if self._path is not None:
            self._recompute_profile()

    # ------------------------------------------------------------------
    # Path lifecycle
    # ------------------------------------------------------------------

    def start_path(self, path: Path, current_odom: PoseStamped) -> bool:
        ok = super().start_path(path, current_odom)
        if not ok:
            return False

        # Lazy-load the artifact + build constraints the first time we run.
        if self._plant is None or self._vp_spec is None:
            self._load_artifact()

        # Per-path geometry cache (skip on every e_max update).
        self._cached_pts = np.array([[p.position.x, p.position.y] for p in path.poses], dtype=float)
        self._cached_curvatures = None  # solve_profile recomputes once on first call

        self._recompute_profile()
        return True

    def _load_artifact(self) -> None:
        if not self._artifact_path:
            raise RuntimeError(
                f"PrecisionPathFollowerTask '{self._name}': artifact_path is empty; "
                f"pass via params.artifact_path on the TaskConfig."
            )
        if not _Path(self._artifact_path).exists():
            raise RuntimeError(
                f"PrecisionPathFollowerTask '{self._name}': artifact not found at "
                f"{self._artifact_path}"
            )

        art = TuningConfig.from_json(self._artifact_path)
        self._plant = art.plant
        self._vp_spec = art.velocity_profile
        vp = self._vp_spec
        # PrecisionMVC reads e_max via a closure so live updates don't
        # require rebuilding the constraint list.
        self._constraints = [
            GeometricMVC(v_max=vp.max_linear_speed),
            SaturationMVC(omega_max=vp.max_angular_speed),
            LateralMVC(a_lat_max=vp.max_centripetal_accel),
            PrecisionMVC(e_max_provider=lambda: self._e_max),
        ]
        logger.info(
            f"PrecisionPathFollowerTask '{self._name}': loaded artifact "
            f"{self._artifact_path} (plant + vp_spec ready, e_max={self._e_max:.3f})"
        )

    def _recompute_profile(self) -> None:
        """Run solve_profile() against current e_max + cached geometry.
        Atomic swap of self._velocity_profile / _velocity_profile_pts so
        the parent's compute() picks up the new array on the next tick."""
        if (
            self._path is None
            or self._plant is None
            or self._vp_spec is None
            or self._constraints is None
            or self._cached_pts is None
        ):
            return

        vp = self._vp_spec
        arr = solve_profile(
            self._path,
            self._plant,
            self._constraints,
            accel_max=vp.max_linear_accel,
            decel_max=vp.max_linear_decel,
            min_speed=vp.min_speed,
            pts=self._cached_pts,
            curvatures=self._cached_curvatures,
        )
        # Single-attribute assignment is atomic under the GIL — safe race
        # vs. the tick thread reading the prior array.
        self._velocity_profile = arr
        self._velocity_profile_pts = self._cached_pts
        logger.info(
            f"PrecisionPathFollowerTask '{self._name}': recomputed profile "
            f"(e_max={self._e_max:.3f}, n={len(arr)}, "
            f"v_min={float(np.min(arr)):.3f}, v_max={float(np.max(arr)):.3f})"
        )


class PrecisionPathFollowerTaskParams(BaseConfig):
    artifact_path: str
    speed: float = 0.55
    control_frequency: float = 10.0
    goal_tolerance: float = 0.05
    orientation_tolerance: float = 0.1
    k_angular: float = 0.5
    e_max_default: float = 0.2


def create_task(cfg: Any, hardware: Any) -> PrecisionPathFollowerTask:
    params = PrecisionPathFollowerTaskParams.model_validate(cfg.params)
    return PrecisionPathFollowerTask(
        cfg.name,
        PathFollowerTaskConfig(
            joint_names=cfg.joint_names,
            priority=cfg.priority,
            speed=params.speed,
            control_frequency=params.control_frequency,
            goal_tolerance=params.goal_tolerance,
            orientation_tolerance=params.orientation_tolerance,
            k_angular=params.k_angular,
        ),
        global_config=_gc,
        artifact_path=params.artifact_path,
        e_max_default=params.e_max_default,
    )


__all__ = [
    "PrecisionPathFollowerTask",
    "PrecisionPathFollowerTaskParams",
    "create_task",
]
