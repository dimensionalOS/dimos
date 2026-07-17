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

"""Regulated-pure-pursuit nav path-follower: PathFollowerTask self-calibrated
from a characterization artifact.

A thin subclass of :class:`PathFollowerTask` for the navigation stack. The
benchmark configures the bare path_follower over RPC (feedforward + curvature
profile built by the Benchmarker); in nav there is no Benchmarker, so this task
loads the artifact itself — lazily on the first ``start_path()`` so a missing
default never blocks startup — and builds the same calibration:

* feedforward gain compensation with ``FF.K = plant.K`` (commanded == achieved;
  the same convention the trajectory tracker uses),
* curvature speed regulation (``PathSpeedCap`` from the artifact's
  ``velocity_profile`` — a_lat / turn-rate / min-speed),
* the runtime yaw-rate clamp from the artifact's measured turn-rate ceiling.

The pursuit knobs (adaptive lookahead, ``k_angular``, ``forward_only``) come from
the task config. ``set_path`` (inherited from the base) arms it on a clicked nav
goal or a replan; each path resets the pursuit cleanly — no clock to re-sync.
"""

from __future__ import annotations

import math
from pathlib import Path as _Path
from typing import TYPE_CHECKING, Any

from dimos.control.benchmarking.tuning import TuningConfig
from dimos.control.benchmarking.velocity_profile import PathSpeedCap, VelocityProfileConfig
from dimos.control.tasks.feedforward_gain_compensator import (
    FeedforwardGainCompensator,
    FeedforwardGainConfig,
)
from dimos.control.tasks.path_follower_task.path_follower_task import (
    PathFollowerTask,
    PathFollowerTaskConfig,
)
from dimos.core.global_config import global_config as _gc
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.global_config import GlobalConfig

logger = setup_logger()

# Pose-domain Go2 tuning artifact, vendored next to this task so the controller
# is self-contained (no gitignored data/ dependency). Blueprints and the task
# params default to it; override per run with params.artifact_path.
DEFAULT_ARTIFACT_PATH = str(_Path(__file__).parent / "artifacts" / "go2_posedomain.json")

# Heading-degeneracy threshold (rad): if every path pose carries the same yaw
# (e.g. the MLS planner stamps identity orientation on all poses), the spread is
# ~0 and we synthesize per-pose headings from the path tangent. A planner that
# emits real per-pose orientations has a spread above this and is left untouched.
_HEADING_DEGENERATE_EPS = 1e-3


def _with_tangent_headings(path: Path) -> Path:
    """Return ``path`` with each pose's yaw set to its forward path tangent when
    the path's orientations are degenerate (all equal — e.g. the MLS planner
    stamps identity on every pose). RPP's rotate-then-drive + final-settle states
    read ``poses[0]``/``poses[-1]`` orientation, so a heading-less path would spin
    the robot to face world-yaw 0 at the start and goal. For a forward-only
    follower the segment tangent IS the desired heading, so this restores
    align-then-drive (first segment) and a settle to the approach heading (last
    segment). Paths that already carry per-pose headings are returned unchanged.
    """
    poses = path.poses
    if poses is None or len(poses) < 2:
        return path

    yaws = [p.orientation.euler[2] for p in poses]
    if max(yaws) - min(yaws) > _HEADING_DEGENERATE_EPS:
        return path  # planner provided real per-pose headings; leave them.

    new_poses: list[PoseStamped] = []
    n = len(poses)
    for i, p in enumerate(poses):
        if i < n - 1:
            dx = poses[i + 1].position.x - p.position.x
            dy = poses[i + 1].position.y - p.position.y
            yaw = (
                math.atan2(dy, dx)
                if (dx * dx + dy * dy) > 1e-12
                # Coincident waypoints: keep the previous heading rather than
                # snapping to 0.
                else (new_poses[-1].orientation.euler[2] if new_poses else 0.0)
            )
        else:
            # Last pose has no next segment — settle to the approach heading.
            yaw = new_poses[-1].orientation.euler[2] if new_poses else 0.0
        new_poses.append(
            PoseStamped(
                ts=p.ts,
                frame_id=p.frame_id,
                position=Vector3(p.position.x, p.position.y, p.position.z),
                orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
            )
        )
    return Path(ts=path.ts, frame_id=path.frame_id, poses=new_poses)


class RPPPathFollowerTask(PathFollowerTask):
    """Path follower that self-calibrates its feedforward + curvature velocity
    profile from a tuning artifact (loaded lazily on first ``start_path``)."""

    def __init__(
        self,
        name: str,
        config: PathFollowerTaskConfig,
        global_config: GlobalConfig,
        artifact_path: str,
        v_max_override: float | None = None,
    ) -> None:
        super().__init__(name, config, global_config=global_config)
        self._artifact_path = artifact_path
        self._v_max_override = float(v_max_override) if v_max_override is not None else None
        self._artifact_loaded = False

    def start_path(self, path: Path, current_odom: PoseStamped) -> bool:
        if not self._artifact_loaded:
            self._load_artifact()
        return super().start_path(_with_tangent_headings(path), current_odom)

    def _load_artifact(self) -> None:
        if not self._artifact_path:
            raise RuntimeError(
                f"RPPPathFollowerTask '{self._name}': artifact_path is empty; "
                f"pass via params.artifact_path on the TaskConfig."
            )
        if not _Path(self._artifact_path).exists():
            raise RuntimeError(
                f"RPPPathFollowerTask '{self._name}': artifact not found at {self._artifact_path}"
            )
        art = TuningConfig.from_json(self._artifact_path)

        # Feedforward: FF.K = plant.K so the command divides by the plant gain
        # and the robot achieves the commanded velocity (same convention as the
        # trajectory tracker; the artifact's `feedforward` block stores 1/K and
        # is inverted for the compensator, so build from `plant` directly).
        self._ff = FeedforwardGainCompensator(
            FeedforwardGainConfig(
                K_vx=art.plant.vx.K,
                K_vy=art.plant.vy.K,
                K_wz=art.plant.wz.K,
            )
        )

        # Curvature speed regulation from the artifact's velocity profile.
        # Stash the profile config + the measured top-speed ceiling on the base
        # so a runtime set_speed() (the coordinator's `speed` port) can rebuild
        # the cap to the new cruise speed, clamped to this ceiling.
        vp = art.velocity_profile
        v_max = self._v_max_override if self._v_max_override is not None else vp.max_linear_speed
        self._v_max_cap = v_max
        self._config.velocity_profile_config = VelocityProfileConfig(
            max_linear_speed=min(self._config.speed, v_max),
            max_angular_speed=vp.max_angular_speed,
            max_centripetal_accel=vp.max_centripetal_accel,
            max_linear_accel=vp.max_linear_accel,
            max_linear_decel=vp.max_linear_decel,
            min_speed=vp.min_speed,
        )
        self._profile_cap = PathSpeedCap(self._config.velocity_profile_config)

        # Runtime yaw-rate clamp = the measured turn-rate ceiling, unless the
        # config set it explicitly.
        if self._config.max_yaw_rate is None:
            self._config.max_yaw_rate = vp.max_angular_speed

        self._artifact_loaded = True
        logger.info(
            f"RPPPathFollowerTask '{self._name}': loaded artifact {self._artifact_path} "
            f"(FF.K=plant.K, v_max={min(self._config.speed, v_max):.3f}, "
            f"a_lat={vp.max_centripetal_accel:.2f}, yaw_cap={vp.max_angular_speed:.3f}, "
            f"min_speed={vp.min_speed:.3f})"
        )


class RPPPathFollowerTaskParams(BaseConfig):
    artifact_path: str = DEFAULT_ARTIFACT_PATH
    speed: float = 0.7
    control_frequency: float = 10.0
    goal_tolerance: float = 0.2
    orientation_tolerance: float = 0.35
    # CTE-tuned pursuit knobs (see the RPP benchmark report): high heading gain
    # + a near-fixed largish lookahead; forward-only for the forward-facing lidar.
    k_angular: float = 1.5
    lookahead_dist: float = 0.7
    lookahead_min: float = 0.5
    lookahead_max: float = 0.7
    lookahead_speed_scale: float = 2.0
    max_yaw_rate: float | None = None  # None ⟹ artifact's measured turn-rate ceiling
    forward_only: bool = True
    v_max_override: float | None = None


def create_task(cfg: Any, hardware: Any) -> RPPPathFollowerTask:
    params = RPPPathFollowerTaskParams.model_validate(cfg.params)
    return RPPPathFollowerTask(
        cfg.name,
        PathFollowerTaskConfig(
            joint_names=cfg.joint_names,
            priority=cfg.priority,
            speed=params.speed,
            control_frequency=params.control_frequency,
            goal_tolerance=params.goal_tolerance,
            orientation_tolerance=params.orientation_tolerance,
            k_angular=params.k_angular,
            lookahead_dist=params.lookahead_dist,
            lookahead_min=params.lookahead_min,
            lookahead_max=params.lookahead_max,
            lookahead_speed_scale=params.lookahead_speed_scale,
            max_yaw_rate=params.max_yaw_rate,
            forward_only=params.forward_only,
        ),
        global_config=_gc,
        artifact_path=params.artifact_path,
        v_max_override=params.v_max_override,
    )


__all__ = [
    "DEFAULT_ARTIFACT_PATH",
    "RPPPathFollowerTask",
    "RPPPathFollowerTaskParams",
    "create_task",
]
