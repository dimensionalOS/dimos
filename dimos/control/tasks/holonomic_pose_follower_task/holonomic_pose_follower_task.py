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

"""Holonomic full-pose path follower (progress-indexed).

Tracks a poses-only Path where every waypoint carries a COMMANDED orientation
that may be decoupled from the travel direction (strafe through a tunnel while
facing along it). This is the capability the pursuit followers structurally
lack: RPP faces the travel direction, so it cannot honor a commanded yaw.

Control stack, per coordinator tick:

    Path (position + yaw per waypoint, no timing)
      -> ProgressPathReference: project the robot onto the path -> s_robot;
         reference pose = path interpolated at s_robot + lookahead
      -> speed regulator: cruise speed capped by the yaw-rate/curvature
         envelope ahead and a decel ramp into the goal, slew-limited
      -> feedforward (optional): reference spatial rates x path speed,
         rotated into the body frame
      -> feedback: per-axis P on pose error (world error rotated into the
         body frame by the current yaw), clamped — the same decoupled-SISO
         law as the trajectory tracker, gains derived from the plant fit
      -> command calibration: u_cmd = u_phys / K_plant (artifact), clamped
         to the measured envelope
      -> JointCommandOutput (VELOCITY)

The commanded yaw comes straight from the path — never re-derived from the
tangent; choosing orientations is the planner's job. Progress indexing makes
the reference robust by construction: no clock to desync at corners, and a
replan re-projects with no re-ramp from rest.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path as _FsPath
from typing import Any, Literal

from dimos.control.benchmarking.tuning import TuningConfig
from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.control.tasks.feedforward_gain_compensator import (
    FeedforwardGainCompensator,
    FeedforwardGainConfig,
)
from dimos.control.tasks.holonomic_pose_follower_task.progress_reference import (
    ProgressPathReference,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Path import Path
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger
from dimos.utils.trigonometry import angle_diff

logger = setup_logger()

# Same vendored pose-domain Go2 artifact the RPP follower calibrates from.
DEFAULT_ARTIFACT_PATH = str(
    _FsPath(__file__).parent.parent / "rpp_path_follower_task" / "artifacts" / "go2_posedomain.json"
)

HolonomicPoseFollowerState = Literal["idle", "tracking", "settling", "arrived", "aborted"]

# P gain for a critically-damped outer loop on a first-order-lag plant
# (tau*s^2 + s + kp = 0 -> kp = 1/(4 zeta^2 tau)); same derivation the
# trajectory tracker certified against the FOPDT fit.
_ZETA = 1.0
# Feedback-contribution clamps (m/s, rad/s) — feedforward carries the path;
# feedback only trims. Same values the trajectory tracker shipped with.
_FB_CLAMP_LINEAR = 0.15
_FB_CLAMP_YAW = 0.4


def _kp_for_tau(tau: float) -> float:
    return 1.0 / (4.0 * _ZETA * _ZETA * max(tau, 1e-3))


def _clamp(v: float, limit: float) -> float:
    return max(-limit, min(limit, v))


@dataclass
class HolonomicPoseFollowerTaskConfig:
    joint_names: list[str] = field(default_factory=lambda: ["base/vx", "base/vy", "base/wz"])
    priority: int = 10
    # Cruise translational speed along the path (m/s); the envelope and the
    # yaw-rate/curvature regulation cap it. set_speed() updates it per run.
    speed: float = 0.5
    # Progress lookahead (m): the reference pose sits this far ahead of the
    # robot's projection. Small and bounded — it pulls the robot along the
    # path; it is NOT a pursuit carrot that synthesizes heading.
    lookahead: float = 0.25
    # Speed-regulation preview (m): slow down for yaw-rate/curvature demands
    # within this arc-length window ahead of the reference.
    regulate_horizon: float = 0.6
    goal_tolerance: float = 0.20
    orientation_tolerance: float = 0.25
    # Feedforward from the reference's spatial rates (tangent, d yaw/ds) times
    # the regulated path speed. Off falls back to carrot-only pursuit of the
    # previewed pose (slow; mainly for isolation tests).
    feedforward: bool = True
    # Calibration artifact (plant K per axis, envelope, accel limits). Loaded
    # lazily on the first start_path so a missing file never blocks startup.
    artifact_path: str = DEFAULT_ARTIFACT_PATH
    # If the pose in CoordinatorState goes stale for longer than this, command
    # zero rather than integrate against a frozen error.
    stale_pose_timeout: float = 0.3


class HolonomicPoseFollowerTask(BaseControlTask):
    """Progress-indexed holonomic full-pose tracker as a passive ControlTask."""

    def __init__(self, name: str, config: HolonomicPoseFollowerTaskConfig) -> None:
        if len(config.joint_names) != 3:
            raise ValueError(
                f"HolonomicPoseFollowerTask '{name}' needs 3 joints (vx, vy, wz), "
                f"got {len(config.joint_names)}"
            )
        self._name = name
        self._config = config
        self._joint_names_list = list(config.joint_names)
        self._joint_names = frozenset(config.joint_names)

        self._artifact_loaded = False
        # Calibration (rebuilt from the artifact on first start_path).
        self._kp = (1.0, 1.0, 1.0)  # per-axis P (x, y, yaw)
        self._ff_comp: FeedforwardGainCompensator | None = None
        self._v_max_lin = config.speed
        self._wz_max = 1.0
        self._a_acc = 1.0
        self._a_dec = 1.0
        self._a_lat = 1.0
        self._min_speed = 0.05

        self._state: HolonomicPoseFollowerState = "idle"
        self._reference: ProgressPathReference | None = None
        self._v_path = 0.0  # slew-limited path speed (m/s)
        self._last_t: float | None = None
        self._last_pose: tuple[float, float, float] | None = None
        self._last_pose_t: float | None = None

    # ControlTask protocol

    @property
    def name(self) -> str:
        return self._name

    def claim(self) -> ResourceClaim:
        return ResourceClaim(
            joints=self._joint_names,
            priority=self._config.priority,
            mode=ControlMode.VELOCITY,
        )

    def is_active(self) -> bool:
        return self._state in ("tracking", "settling")

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        if not self.is_active() or self._reference is None:
            return None

        pose = self._read_pose(state)
        if pose is not None:
            self._last_pose = pose
            self._last_pose_t = state.t_now
        elif (
            self._last_pose_t is not None
            and state.t_now - self._last_pose_t < self._config.stale_pose_timeout
        ):
            pose = self._last_pose
        if pose is None:
            # No usable pose: never track against a frozen error.
            return self._command(0.0, 0.0, 0.0, calibrate=False)

        dt = state.t_now - self._last_t if self._last_t is not None else 0.0
        self._last_t = state.t_now

        if self._state == "tracking":
            vx, vy, wz = self._tracking_command(pose, dt)
        else:
            vx, vy, wz = self._settling_command(pose)
        return self._command(vx, vy, wz, calibrate=True)

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        if joints & self._joint_names and self.is_active():
            logger.warning(f"HolonomicPoseFollowerTask '{self._name}' preempted by {by_task}")
            self._state = "aborted"

    # Control law

    def _read_pose(self, state: CoordinatorState) -> tuple[float, float, float] | None:
        # Twist-base ConnectedHardware routes adapter.read_odometry() ->
        # joint positions [x, y, yaw] (same convention as the other followers).
        positions = state.joints.joint_positions
        x = positions.get(self._joint_names_list[0])
        y = positions.get(self._joint_names_list[1])
        yaw = positions.get(self._joint_names_list[2])
        if x is None or y is None or yaw is None:
            return None
        return float(x), float(y), float(yaw)

    def _tracking_command(
        self, pose: tuple[float, float, float], dt: float
    ) -> tuple[float, float, float]:
        assert self._reference is not None
        ref = self._reference
        x, y, yaw = pose

        s_robot = ref.advance(x, y)
        remaining = ref.length - s_robot
        preview = ref.sample(s_robot + self._config.lookahead)

        # Near the end, hand over to the settle regulator (pure feedback onto
        # the final pose, including its commanded yaw).
        if remaining < max(self._config.goal_tolerance, 1e-6):
            self._state = "settling"
            return self._settling_command(pose)

        v_path = self._regulated_speed(preview.s, remaining, dt)

        if not self._config.feedforward:
            # Carrot-only degraded mode: pure per-axis P toward the previewed
            # pose. Functional but slow (feedback clamps bound the speed).
            return self._feedback((preview.x, preview.y, preview.yaw), pose)

        # Feedforward: the previewed spatial rates x the regulated path speed,
        # rotated into the body frame. Previewing by the lookahead lets the
        # command lead the plant's dead time + lag through corners.
        ff_vx_world = preview.tangent_x * v_path
        ff_vy_world = preview.tangent_y * v_path
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        vx = cos_yaw * ff_vx_world + sin_yaw * ff_vy_world
        vy = -sin_yaw * ff_vx_world + cos_yaw * ff_vy_world
        wz = preview.dyaw_ds * v_path

        # Feedback trims against the pose the robot should hold NOW — the
        # projection foot, not the preview point. Along-track error is ~0 at
        # the foot by construction (the robot defines its own progress), so
        # feedback carries no persistent along-path bias: cross-track and the
        # commanded yaw get the full correction, and the FF alone sets speed.
        foot = ref.sample(s_robot)
        fb_vx, fb_vy, fb_wz = self._feedback((foot.x, foot.y, foot.yaw), pose)
        return vx + fb_vx, vy + fb_vy, wz + fb_wz

    def _feedback(
        self, reference: tuple[float, float, float], pose: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        x, y, yaw = pose
        ex_world = reference[0] - x
        ey_world = reference[1] - y
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        ex_body = cos_yaw * ex_world + sin_yaw * ey_world
        ey_body = -sin_yaw * ex_world + cos_yaw * ey_world
        e_yaw = angle_diff(reference[2], yaw)
        return (
            _clamp(self._kp[0] * ex_body, _FB_CLAMP_LINEAR),
            _clamp(self._kp[1] * ey_body, _FB_CLAMP_LINEAR),
            _clamp(self._kp[2] * e_yaw, _FB_CLAMP_YAW),
        )

    def _regulated_speed(self, s_ref: float, remaining: float, dt: float) -> float:
        """Translational path speed: cruise, capped by the envelope demands
        ahead (yaw rate, centripetal), ramped down into the goal, slew-limited
        by the measured accel/decel."""
        assert self._reference is not None
        v_cruise = min(self._config.speed, self._v_max_lin)

        dyaw_ds_max, kappa_max = self._reference.max_rates_ahead(
            s_ref, self._config.regulate_horizon
        )
        v = v_cruise
        if dyaw_ds_max > 1e-6:
            # Commanded-yaw feasibility: |wz_ff| = |dyaw/ds| * v <= wz_max.
            v = min(v, self._wz_max / dyaw_ds_max)
        if kappa_max > 1e-6:
            v = min(v, math.sqrt(self._a_lat / kappa_max))
        # Yaw/curvature regulation never drags v below the plant's floor speed;
        # only the goal decel ramp may (it must reach 0).
        v = max(v, min(self._min_speed, v_cruise))
        v = min(v, math.sqrt(2.0 * self._a_dec * max(remaining, 0.0)))

        if dt > 0.0:
            dv_up = self._a_acc * dt
            dv_down = self._a_dec * dt
            v = min(max(v, self._v_path - dv_down), self._v_path + dv_up)
        self._v_path = max(0.0, v)
        return self._v_path

    def _settling_command(self, pose: tuple[float, float, float]) -> tuple[float, float, float]:
        assert self._reference is not None
        end = self._reference.end_pose()
        pos_err = math.hypot(end[0] - pose[0], end[1] - pose[1])
        yaw_err = abs(angle_diff(end[2], pose[2]))
        if pos_err < self._config.goal_tolerance and yaw_err < self._config.orientation_tolerance:
            self._state = "arrived"
            logger.info(f"HolonomicPoseFollowerTask '{self._name}' arrived")
            return 0.0, 0.0, 0.0
        self._v_path = 0.0
        return self._feedback(end, pose)

    def _command(self, vx: float, vy: float, wz: float, *, calibrate: bool) -> JointCommandOutput:
        if calibrate:
            # Envelope clamp on the DESIRED (physical) velocities first, then
            # the gain inversion so the robot actually achieves them.
            vx = _clamp(vx, self._v_max_lin)
            vy = _clamp(vy, self._v_max_lin)
            wz = _clamp(wz, self._wz_max)
            if self._ff_comp is not None:
                vx, vy, wz = self._ff_comp.compute(vx, vy, wz)
        return JointCommandOutput(
            joint_names=self._joint_names_list,
            velocities=[vx, vy, wz],
            mode=ControlMode.VELOCITY,
        )

    # Calibration

    def _ensure_artifact_loaded(self) -> None:
        if self._artifact_loaded:
            return
        path = self._config.artifact_path
        if not path or not _FsPath(path).exists():
            raise RuntimeError(
                f"HolonomicPoseFollowerTask '{self._name}': artifact not found at {path!r}"
            )
        art = TuningConfig.from_json(path)
        plant = art.plant
        vp = art.velocity_profile

        self._kp = (
            _kp_for_tau(plant.vx.tau),
            _kp_for_tau(plant.vy.tau),
            _kp_for_tau(plant.wz.tau),
        )
        # u_cmd = u_phys / K; the command that reaches the envelope speed is
        # envelope / K (same convention as the trajectory tracker).
        self._ff_comp = FeedforwardGainCompensator(
            FeedforwardGainConfig(
                K_vx=plant.vx.K,
                K_vy=plant.vy.K,
                K_wz=plant.wz.K,
                output_min_vx=-vp.max_linear_speed / plant.vx.K,
                output_max_vx=vp.max_linear_speed / plant.vx.K,
                output_min_vy=-vp.max_linear_speed / plant.vy.K,
                output_max_vy=vp.max_linear_speed / plant.vy.K,
                output_min_wz=-vp.max_angular_speed / plant.wz.K,
                output_max_wz=vp.max_angular_speed / plant.wz.K,
            )
        )
        self._v_max_lin = vp.max_linear_speed
        self._wz_max = vp.max_angular_speed
        self._a_acc = vp.max_linear_accel
        self._a_dec = vp.max_linear_decel
        self._a_lat = vp.max_centripetal_accel
        self._min_speed = vp.min_speed
        self._artifact_loaded = True
        logger.info(
            f"HolonomicPoseFollowerTask '{self._name}': loaded artifact {path} "
            f"(kp={tuple(round(k, 3) for k in self._kp)}, v_max={self._v_max_lin:.3f}, "
            f"wz_max={self._wz_max:.3f}, a_lat={self._a_lat:.2f})"
        )

    # Public API (coordinator broadcast hooks + RPC)

    def start_path(self, path: Path, current_odom: PoseStamped) -> bool:
        if path is None or len(path.poses) < 2:
            logger.warning(f"HolonomicPoseFollowerTask '{self._name}': invalid path")
            return False
        self._ensure_artifact_loaded()
        try:
            reference = ProgressPathReference(path)
        except ValueError as e:
            logger.warning(f"HolonomicPoseFollowerTask '{self._name}': {e}")
            return False
        self._reference = reference
        # Re-project the robot onto the (new) path — this is the whole replan
        # story: no clock to reset, and _v_path carries over so an in-motion
        # replan does not re-ramp from rest.
        reference.advance(float(current_odom.position.x), float(current_odom.position.y))
        self._last_t = None
        self._last_pose = None
        self._last_pose_t = None
        self._state = "tracking"
        logger.info(
            f"HolonomicPoseFollowerTask '{self._name}' started: {len(path.poses)} poses, "
            f"{reference.length:.2f} m, cruise {min(self._config.speed, self._v_max_lin):.2f} m/s"
        )
        return True

    def set_path(self, path: Path, odom: PoseStamped | None = None) -> None:
        """Coordinator broadcast hook (``ControlCoordinator._on_path``)."""
        if odom is None:
            logger.warning(
                f"HolonomicPoseFollowerTask '{self._name}': received path without odom; dropping."
            )
            return
        logger.info(
            f"HolonomicPoseFollowerTask '{self._name}': received path (n={len(path.poses)})"
        )
        self.start_path(path, odom)

    def set_speed(self, speed: float) -> None:
        """Coordinator broadcast hook (``ControlCoordinator._on_speed``)."""
        if self.is_active():
            logger.warning(
                f"HolonomicPoseFollowerTask '{self._name}': ignoring set_speed while active"
            )
            return
        self._config.speed = float(speed)
        logger.info(f"HolonomicPoseFollowerTask '{self._name}': set_speed({speed:.3f})")

    def configure(
        self,
        speed: float | None = None,
        lookahead: float | None = None,
        regulate_horizon: float | None = None,
        goal_tolerance: float | None = None,
        orientation_tolerance: float | None = None,
        feedforward: bool | None = None,
        **ignored: Any,
    ) -> bool:
        """Override per-run knobs before start_path. Unknown kwargs are
        accepted and logged so callers built for a sibling follower's
        configure signature work unchanged."""
        if self.is_active():
            logger.warning(
                f"HolonomicPoseFollowerTask '{self._name}': cannot configure while active"
            )
            return False
        if speed is not None:
            self._config.speed = speed
        if lookahead is not None:
            self._config.lookahead = lookahead
        if regulate_horizon is not None:
            self._config.regulate_horizon = regulate_horizon
        if goal_tolerance is not None:
            self._config.goal_tolerance = goal_tolerance
        if orientation_tolerance is not None:
            self._config.orientation_tolerance = orientation_tolerance
        if feedforward is not None:
            self._config.feedforward = feedforward
        if ignored:
            logger.info(
                f"HolonomicPoseFollowerTask '{self._name}': ignoring unknown configure "
                f"kwargs {sorted(ignored)}"
            )
        return True

    def cancel(self) -> bool:
        if not self.is_active():
            return False
        self._state = "aborted"
        return True

    def reset(self) -> bool:
        if self.is_active():
            return False
        self._state = "idle"
        self._reference = None
        self._v_path = 0.0
        self._last_t = None
        self._last_pose = None
        self._last_pose_t = None
        return True

    def get_state(self) -> HolonomicPoseFollowerState:
        return self._state


class HolonomicPoseFollowerTaskParams(BaseConfig):
    artifact_path: str = DEFAULT_ARTIFACT_PATH
    speed: float = 0.5
    lookahead: float = 0.25
    regulate_horizon: float = 0.6
    goal_tolerance: float = 0.20
    orientation_tolerance: float = 0.25
    feedforward: bool = True
    stale_pose_timeout: float = 0.3


def create_task(cfg: Any, hardware: Any) -> HolonomicPoseFollowerTask:
    params = HolonomicPoseFollowerTaskParams.model_validate(cfg.params)
    return HolonomicPoseFollowerTask(
        cfg.name,
        HolonomicPoseFollowerTaskConfig(
            joint_names=cfg.joint_names,
            priority=cfg.priority,
            speed=params.speed,
            lookahead=params.lookahead,
            regulate_horizon=params.regulate_horizon,
            goal_tolerance=params.goal_tolerance,
            orientation_tolerance=params.orientation_tolerance,
            feedforward=params.feedforward,
            artifact_path=params.artifact_path,
            stale_pose_timeout=params.stale_pose_timeout,
        ),
    )
