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

"""Holonomic full-pose path follower, indexed by progress (arc length) not time.

Tracks a Path whose waypoints carry a commanded yaw decoupled from the travel
direction, which the pursuit followers structurally cannot do. The commanded
yaw comes from the path and is never re-derived from the tangent.
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
    validate_plant_gains,
)
from dimos.control.tasks.holonomic_pose_follower_task.progress_reference import (
    ProgressPathReference,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger
from dimos.utils.trigonometry import angle_diff

logger = setup_logger()

DEFAULT_ARTIFACT_PATH = str(
    _FsPath(__file__).parent.parent / "rpp_path_follower_task" / "artifacts" / "go2_posedomain.json"
)

# "stopping" streams zeros until the base rests: one zero does not stop a
# legged base mid-glide.
HolonomicPoseFollowerState = Literal[
    "idle", "tracking", "settling", "stopping", "arrived", "aborted"
]

_ZETA = 1.0
# Feedback only trims; feedforward carries the path.
_FB_CLAMP_LINEAR = 0.15
_FB_CLAMP_YAW = 0.4


def _kp_for_tau(tau: float) -> float:
    return 1.0 / (4.0 * _ZETA * _ZETA * max(tau, 1e-3))


def _clamp(v: float, limit: float) -> float:
    return max(-limit, min(limit, v))


def _pose_stamped(pose: tuple[float, float, float]) -> PoseStamped:
    return PoseStamped(
        position=Vector3(pose[0], pose[1], 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, pose[2])),
    )


@dataclass
class HolonomicPoseFollowerTaskConfig:
    joint_names: list[str] = field(default_factory=lambda: ["base/vx", "base/vy", "base/wz"])
    priority: int = 10
    speed: float = 0.5
    lookahead: float = 0.25
    regulate_horizon: float = 0.6
    goal_tolerance: float = 0.20
    orientation_tolerance: float = 0.25
    feedforward: bool = True
    # Well below the artifact's max decel (~5.5): braking at the measured
    # maximum starts centimeters out, so the robot crosses the arrival ring at
    # cruise and glides past (hardware 2026-07-13: 0.29 m past at v=0.7).
    approach_decel: float = 1.0
    stop_hold_s: float = 1.0
    artifact_path: str = DEFAULT_ARTIFACT_PATH
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
        self._stop_started_t: float | None = None
        self._pending_path: Path | None = None

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
        return self._state in ("tracking", "settling", "stopping")

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        if self._pending_path is not None:
            # Arm a stream-delivered path on the first tick that carries a pose;
            # the card handler has no odom to hand us.
            armed = self._read_pose(state)
            if armed is not None:
                path, self._pending_path = self._pending_path, None
                self.start_path(path, _pose_stamped(armed))
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
            return self._command(0.0, 0.0, 0.0, calibrate=False)

        dt = state.t_now - self._last_t if self._last_t is not None else 0.0
        self._last_t = state.t_now

        if self._state == "stopping":
            vx, vy, wz = self._stopping_command(pose, state.t_now)
        elif self._state == "tracking":
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
        # Twist-base hardware routes read_odometry() into joint positions [x, y, yaw].
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

        if remaining < max(self._config.goal_tolerance, 1e-6):
            self._state = "settling"
            return self._settling_command(pose)

        v_path = self._regulated_speed(preview.s, remaining, dt)

        if not self._config.feedforward:
            return self._feedback((preview.x, preview.y, preview.yaw), pose)

        # Previewing by the lookahead lets the command lead the plant's dead
        # time + lag through corners.
        ff_vx_world = preview.tangent_x * v_path
        ff_vy_world = preview.tangent_y * v_path
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        vx = cos_yaw * ff_vx_world + sin_yaw * ff_vy_world
        vy = -sin_yaw * ff_vx_world + cos_yaw * ff_vy_world
        wz = preview.dyaw_ds * v_path

        # Trim against the projection foot, not the preview: along-track error
        # is ~0 at the foot, so feedback adds no along-path bias and the
        # feedforward alone sets speed.
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
        """Cruise speed capped by the yaw-rate/curvature demands ahead, ramped
        into the goal, slew-limited."""
        assert self._reference is not None
        v_cruise = min(self._config.speed, self._v_max_lin)

        dyaw_ds_max, kappa_max = self._reference.max_rates_ahead(
            s_ref, self._config.regulate_horizon
        )
        v = v_cruise
        if dyaw_ds_max > 1e-6:
            v = min(v, self._wz_max / dyaw_ds_max)
        if kappa_max > 1e-6:
            v = min(v, math.sqrt(self._a_lat / kappa_max))
        # Only the goal ramp may go below the plant's floor speed (it must reach 0).
        v = max(v, min(self._min_speed, v_cruise))
        a_app = min(self._a_dec, self._config.approach_decel)
        d_to_land = max(0.0, remaining - self._config.goal_tolerance)
        v_land = min(self._min_speed, v_cruise)
        v = min(v, math.sqrt(v_land * v_land + 2.0 * a_app * d_to_land))

        if dt > 0.0:
            dv_up = self._a_acc * dt
            dv_down = self._a_dec * dt
            v = min(max(v, self._v_path - dv_down), self._v_path + dv_up)
        else:
            # First tick: hold the previous speed rather than jump to cruise.
            v = min(v, self._v_path)
        self._v_path = max(0.0, v)
        return self._v_path

    def _goal_errors(self, pose: tuple[float, float, float]) -> tuple[float, float]:
        assert self._reference is not None
        end = self._reference.end_pose()
        return (
            math.hypot(end[0] - pose[0], end[1] - pose[1]),
            abs(angle_diff(end[2], pose[2])),
        )

    def _settling_command(self, pose: tuple[float, float, float]) -> tuple[float, float, float]:
        assert self._reference is not None
        pos_err, yaw_err = self._goal_errors(pose)
        # Arrival is declared only after the hold confirms REST in tolerance;
        # merely passing through the zone does not count.
        if pos_err < self._config.goal_tolerance and yaw_err < self._config.orientation_tolerance:
            self._state = "stopping"
            self._stop_started_t = None
            return 0.0, 0.0, 0.0
        self._v_path = 0.0
        return self._feedback(self._reference.end_pose(), pose)

    def _stopping_command(
        self, pose: tuple[float, float, float], t_now: float
    ) -> tuple[float, float, float]:
        if self._stop_started_t is None:
            self._stop_started_t = t_now
        if t_now - self._stop_started_t < self._config.stop_hold_s:
            return 0.0, 0.0, 0.0
        pos_err, yaw_err = self._goal_errors(pose)
        if pos_err < self._config.goal_tolerance and yaw_err < self._config.orientation_tolerance:
            self._state = "arrived"
            logger.info(
                f"HolonomicPoseFollowerTask '{self._name}' arrived "
                f"(rest pos_err={pos_err:.3f} m, yaw_err={yaw_err:.3f} rad)"
            )
            return 0.0, 0.0, 0.0
        # The glide carried the robot outside tolerance; pull onto the goal again.
        logger.info(
            f"HolonomicPoseFollowerTask '{self._name}': rest pose outside tolerance "
            f"(pos_err={pos_err:.3f} m, yaw_err={yaw_err:.3f} rad); re-settling"
        )
        self._state = "settling"
        self._stop_started_t = None
        return self._settling_command(pose)

    def _command(self, vx: float, vy: float, wz: float, *, calibrate: bool) -> JointCommandOutput:
        if calibrate:
            # Clamp the physical velocities BEFORE the gain inversion; the
            # order matters, the inverted command exceeds the envelope.
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

        # Reject a zero-gain artifact before the envelope/K divisions below.
        validate_plant_gains(plant.vx.K, plant.vy.K, plant.wz.K)

        self._kp = (
            _kp_for_tau(plant.vx.tau),
            _kp_for_tau(plant.vy.tau),
            _kp_for_tau(plant.wz.tau),
        )
        # u_cmd = u_phys / K, so the envelope in command units is envelope / K.
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
        # Re-project onto the new path. _v_path deliberately carries over, so an
        # in-motion replan does not re-ramp from rest.
        reference.advance(float(current_odom.position.x), float(current_odom.position.y))
        self._last_t = None
        self._last_pose = None
        self._last_pose_t = None
        self._stop_started_t = None
        self._state = "tracking"
        logger.info(
            f"HolonomicPoseFollowerTask '{self._name}' started: {len(path.poses)} poses, "
            f"{reference.length:.2f} m, cruise {min(self._config.speed, self._v_max_lin):.2f} m/s"
        )
        return True

    def on_path(self, msg: Path, t_now: float) -> None:
        """``path`` card handler. Latched, not armed here: the handler carries no
        odom, so compute() arms it on the first tick that has a pose."""
        logger.info(f"HolonomicPoseFollowerTask '{self._name}': received path (n={len(msg.poses)})")
        self._pending_path = msg

    def on_speed(self, msg: Float32, t_now: float) -> None:
        """``speed`` card handler."""
        self.set_speed(float(msg.data))

    def set_speed(self, speed: float) -> None:
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
        approach_decel: float | None = None,
        stop_hold_s: float | None = None,
        **ignored: Any,
    ) -> bool:
        """Override per-run knobs before start_path. Unknown kwargs are accepted
        so callers built for a sibling follower's signature work unchanged."""
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
        if approach_decel is not None:
            self._config.approach_decel = approach_decel
        if stop_hold_s is not None:
            self._config.stop_hold_s = stop_hold_s
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
        self._stop_started_t = None
        self._pending_path = None
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
    approach_decel: float = 1.0
    stop_hold_s: float = 1.0
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
            approach_decel=params.approach_decel,
            stop_hold_s=params.stop_hold_s,
            artifact_path=params.artifact_path,
            stale_pose_timeout=params.stale_pose_timeout,
        ),
    )
