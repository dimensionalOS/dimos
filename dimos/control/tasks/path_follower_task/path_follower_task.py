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

"""Path-follower ControlTask: production LocalPlanner algorithm,
unwrapped from its daemon thread and rebuilt as a passive ControlTask.

Algorithm is a faithful port of
:class:`dimos.navigation.replanning_a_star.local_planner.LocalPlanner`:
PController + 0.5 m fixed lookahead + rotate-then-drive heuristic +
state machine (initial_rotation → path_following → final_rotation → arrived).

Costmap / obstacle-clearance plumbing is intentionally omitted - the
benchmark battery is obstacle-free.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import TYPE_CHECKING, Any, Literal

import numpy as np

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
from dimos.control.tasks.velocity_tracking_pid import (
    VelocityTrackingConfig,
    VelocityTrackingPID,
)
from dimos.core.global_config import global_config as _gc
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.replanning_a_star.controllers import PController
from dimos.navigation.replanning_a_star.path_distancer import PathDistancer
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.benchmarking.velocity_profile import (
    PathSpeedCap,
    PathSpeedCapProtocol,
    VelocityProfileConfig,
)
from dimos.utils.logging_config import setup_logger
from dimos.utils.trigonometry import angle_diff

if TYPE_CHECKING:
    from dimos.core.global_config import GlobalConfig
    from dimos.msgs.nav_msgs.Path import Path

logger = setup_logger()

PathFollowerState = Literal[
    "idle", "initial_rotation", "path_following", "final_rotation", "arrived", "aborted"
]

# Sentinel so callers can pass ``None`` to configure() to explicitly
# clear ff/profile_config, distinct from "don't touch this field".
_UNSET: object = object()


@dataclass
class PathFollowerTaskConfig:
    joint_names: list[str] = field(default_factory=lambda: ["base/vx", "base/vy", "base/wz"])
    priority: int = 20
    speed: float = 0.55
    control_frequency: float = 10.0
    goal_tolerance: float = 0.2
    orientation_tolerance: float = 0.35
    # PController outer-loop angular gain. Default 0.5 matches production
    # LocalPlanner; sweep on circle_R1.0 found 1.0 gives ~9x lower CTE.
    k_angular: float = 0.5
    # Pure-pursuit lookahead distance (m). Default 0.5 matches production
    # LocalPlanner. Smaller → tighter curve tracking: the steady-state
    # inside-offset on a circle is ~lookahead²/(2·R), so dropping it cuts the
    # circle/corner error roughly with the square — but too small wobbles.
    # Used as the FIXED lookahead when lookahead_speed_scale == 0 and as the
    # initial value before the first adaptive update.
    lookahead_dist: float = 0.5
    # Regulated-pure-pursuit adaptive lookahead (RPP's own knob)
    # When lookahead_speed_scale > 0 the lookahead distance adapts to the
    # robot's current pursuit speed each tick:
    #     L = clip(lookahead_speed_scale * v, lookahead_min, lookahead_max)
    # Long lookahead at speed (smooth, stable on straights); short lookahead
    # in slow corners (tight tracking). 0.0 disables adaptation and the fixed
    # ``lookahead_dist`` above is used unchanged.
    lookahead_min: float = 0.3
    lookahead_max: float = 0.9
    lookahead_speed_scale: float = 0.0
    # Runtime yaw-rate (steering) saturation, rad/s, applied to the commanded
    # wz actually sent to the base. None ⟹ no extra clamp beyond the
    # PController's ±speed clip and the FF compensator's ±output_max_wz. Set
    # from the artifact's ``velocity_profile.max_angular_speed`` (~1.18 rad/s
    # for the Go2) so commanded wz never exceeds the measured turn-rate ceiling.
    max_yaw_rate: float | None = None
    # Forward-only (car-like / non-holonomic) contract. Pursuit structurally
    # commands vy == 0, so this ASSERTS that invariant and clamps vx >= 0
    # (never reverse). The Go2's lidar faces forward; strafing/reversing would
    # drive into unsensed space off the collision-free planned path.
    forward_only: bool = False
    # Optional inner-loop velocity-tracking PID. None ⟹ no closed loop.
    # Mutually exclusive with ff_config (PI takes precedence if both set).
    pid_config: VelocityTrackingConfig | None = None
    # Optional static feedforward plant-gain compensator (Strategy B).
    # cmd_to_robot = controller_cmd / K_plant. No actual feedback needed.
    ff_config: FeedforwardGainConfig | None = None
    # Optional curvature velocity-profile cap. None ⟹ off
    velocity_profile_config: VelocityProfileConfig | None = None


class PathFollowerTask(BaseControlTask):
    """Production LocalPlanner algorithm as a passive ControlTask."""

    def __init__(
        self,
        name: str,
        config: PathFollowerTaskConfig,
        global_config: GlobalConfig,
        external_profile_cap: PathSpeedCapProtocol | None = None,
    ) -> None:
        if len(config.joint_names) != 3:
            raise ValueError(
                f"PathFollowerTask '{name}' needs 3 joints (vx, vy, wz), "
                f"got {len(config.joint_names)}"
            )

        self._name = name
        self._config = config
        self._joint_names_list = list(config.joint_names)
        self._joint_names = frozenset(config.joint_names)

        self._controller = PController(global_config, config.speed, config.control_frequency)
        # Override the class-level _k_angular for this instance only.
        self._controller._k_angular = config.k_angular
        self._pid: VelocityTrackingPID | None = (
            VelocityTrackingPID(config.pid_config) if config.pid_config else None
        )
        self._ff: FeedforwardGainCompensator | None = (
            FeedforwardGainCompensator(config.ff_config) if config.ff_config else None
        )
        # external_profile_cap (e.g. a ReferenceGovernor) wins over the
        # auto-built PathSpeedCap from velocity_profile_config. Either
        # path produces a duck-typed PathSpeedCapProtocol object that
        # .compute() drives in the same way.
        self._profile_cap: PathSpeedCapProtocol | None = (
            external_profile_cap
            if external_profile_cap is not None
            else PathSpeedCap(config.velocity_profile_config)
            if config.velocity_profile_config is not None
            else None
        )

        # Optional measured top-speed ceiling for the curvature cap. None ⟹ no
        # extra cap (the cap's max_linear_speed follows the requested speed
        # directly). Subclasses that derive a profile from a measured envelope
        # (e.g. RPPPathFollowerTask) set this to the artifact's max so a runtime
        # set_speed() never lets the cap exceed what the plant can hold.
        self._v_max_cap: float | None = None

        self._state: PathFollowerState = "idle"
        self._path: Path | None = None
        self._distancer: PathDistancer | None = None
        self._current_odom: PoseStamped | None = None
        # Pursuit-speed proxy for adaptive lookahead: the |vx| the controller
        # commanded last tick. 0 on the first tick ⟹ lookahead starts at
        # lookahead_min and grows as the robot accelerates.
        self._v_cur: float = 0.0
        # Closed-path gate: track the furthest-along path index reached so
        # that closed paths (where goal==start) don't trip arrival on tick 1.
        self._max_progress_idx: int = 0
        # Optional per-waypoint speed cap supplied directly by a caller
        # (e.g. Benchmarker handing in RG-derived speeds across RPC). When
        # set, takes precedence over self._profile_cap in compute(). See
        # start_path() for how it's installed.
        self._velocity_profile: np.ndarray | None = None
        self._velocity_profile_pts: np.ndarray | None = None

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
        return self._state in ("initial_rotation", "path_following", "final_rotation")

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        if not self.is_active():
            return None
        if self._path is None or self._distancer is None:
            return None

        # Pull pose from CoordinatorState. The twist-base ConnectedHardware
        # routes adapter.read_odometry() -> [x, y, yaw]
        pos = state.joints.joint_positions
        x = pos.get(self._joint_names_list[0])
        y = pos.get(self._joint_names_list[1])
        yaw = pos.get(self._joint_names_list[2])
        if x is not None and y is not None and yaw is not None:
            self._current_odom = PoseStamped(
                ts=state.t_now,
                position=Vector3(float(x), float(y), 0.0),
                orientation=Quaternion.from_euler(Vector3(0.0, 0.0, float(yaw))),
            )
        if self._current_odom is None:
            return None

        match self._state:
            case "initial_rotation":
                vx, vy, wz = self._step_initial_rotation()
            case "path_following":
                vx, vy, wz = self._step_path_following()
            case "final_rotation":
                vx, vy, wz = self._step_final_rotation()
            case _:
                return None

        # Pursuit-speed proxy for next tick's adaptive lookahead: the |vx|
        # this controller just commanded (controller frame, pre-FF). Captured
        # before FF/profile shaping so lookahead tracks the geometric pursuit
        # speed, not the gain-compensated command.
        self._v_cur = abs(vx)

        # Speed cap FIRST, on the controller's DESIRED velocities — BEFORE the
        # feedforward gain inversion. Capping after FF would clamp the
        # gain-inflated command back down to the cap, leaving achieved = cap·K
        # (an undershoot when K < 1); capping the desired first lets FF invert
        # the plant gain so the robot actually ACHIEVES the capped speed. The
        # cap scales (vx, vy, wz) uniformly so the desired turn radius is
        # preserved, and FF's per-axis inversion preserves the achieved radius.
        #
        # Prefer a precomputed per-waypoint profile (e.g. the --rg arm's
        # RG-derived speeds shipped as a list[float]) over the auto-built
        # curvature-based PathSpeedCap. The lookahead window mirrors
        # PathSpeedCap.speed_limit_at — min over the next ~8 waypoints so
        # braking starts BEFORE a corner rather than at it.
        if self._velocity_profile is not None and self._velocity_profile_pts is not None:
            x = self._current_odom.position.x
            y = self._current_odom.position.y
            i = int(np.argmin(np.sum((self._velocity_profile_pts - np.array([x, y])) ** 2, axis=1)))
            j = min(len(self._velocity_profile), i + 8)
            vlim = float(np.min(self._velocity_profile[i:j]))
            s = abs(vx)
            if s > vlim and s > 1e-9:
                k = vlim / s
                vx, vy, wz = vx * k, vy * k, wz * k
        elif self._profile_cap is not None:
            vx, vy, wz = self._profile_cap.cap(
                self._current_odom.position.x, self._current_odom.position.y, vx, vy, wz
            )

        # Inner-loop gain compensation (mutually exclusive - PI wins if both
        # set). Applied AFTER the cap so it inverts the plant gain on the capped
        # desired velocity.
        if self._pid is not None:
            actual_vx = state.joints.joint_velocities.get(self._joint_names_list[0], 0.0)
            actual_vy = state.joints.joint_velocities.get(self._joint_names_list[1], 0.0)
            actual_wz = state.joints.joint_velocities.get(self._joint_names_list[2], 0.0)
            vx, vy, wz = self._pid.compute(vx, vy, wz, actual_vx, actual_vy, actual_wz)
        elif self._ff is not None:
            # Static gain compensation: cmd_to_robot = controller_cmd / K_plant
            vx, vy, wz = self._ff.compute(vx, vy, wz)

        # Regulated-pure-pursuit output conditioning, applied last on the
        # commanded values actually sent to the base:
        #   1. Yaw-rate saturation to the measured turn-rate ceiling. This is
        #      a hard actuator limit, so wz is clamped without rescaling vx
        #      (matching real saturation — the curvature-speed profile above
        #      already slows corners enough that the clamp rarely binds).
        #   2. Forward-only contract: pursuit never strafes (vy == 0) or
        #      reverses (vx >= 0).
        if self._config.max_yaw_rate is not None:
            cap = abs(self._config.max_yaw_rate)
            wz = max(-cap, min(cap, wz))
        if self._config.forward_only:
            assert abs(vy) < 1e-6, f"PathFollowerTask forward_only: vy={vy} must be 0"
            vx = max(0.0, vx)

        return JointCommandOutput(
            joint_names=self._joint_names_list,
            velocities=[vx, vy, wz],
            mode=ControlMode.VELOCITY,
        )

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        if joints & self._joint_names and self.is_active():
            logger.warning(f"PathFollowerTask '{self._name}' preempted by {by_task}")
            self._state = "aborted"

    # State-machine bodies (mirrors LocalPlanner._compute_*)

    def _step_initial_rotation(self) -> tuple[float, float, float]:
        assert self._path is not None and self._current_odom is not None
        first_yaw = self._path.poses[0].orientation.euler[2]
        robot_yaw = self._current_odom.orientation.euler[2]
        yaw_err = angle_diff(first_yaw, robot_yaw)

        if abs(yaw_err) < self._config.orientation_tolerance:
            self._state = "path_following"
            return self._step_path_following()

        twist = self._controller.rotate(yaw_err)
        return float(twist.linear.x), float(twist.linear.y), float(twist.angular.z)

    def _windowed_closest(self, pos: np.ndarray, window: int = 20) -> int:
        """Closest path index searched only in a forward window from
        ``_max_progress_idx``. Prevents wrap-around matches on closed paths
        (e.g. circle where path[0] == path[-1] would otherwise let argmin
        return the last index on tick 1 → spurious 'arrived').
        """
        assert self._path is not None
        n = len(self._path.poses)
        lo = self._max_progress_idx
        hi = min(n, lo + window + 1)
        best_idx = lo
        best_d_sq = float("inf")
        for i in range(lo, hi):
            p = self._path.poses[i].position
            d_sq = (p.x - pos[0]) ** 2 + (p.y - pos[1]) ** 2
            if d_sq < best_d_sq:
                best_d_sq = d_sq
                best_idx = i
        return best_idx

    def _step_path_following(self) -> tuple[float, float, float]:
        assert self._path is not None
        assert self._distancer is not None
        assert self._current_odom is not None

        pos = np.array([self._current_odom.position.x, self._current_odom.position.y])

        closest = self._windowed_closest(pos)
        if closest > self._max_progress_idx:
            self._max_progress_idx = closest

        # Arrival is only valid AFTER we've traversed enough of the path.
        # Otherwise closed paths (goal==start) would arrive on tick 1.
        progress_threshold = max(1, int(0.7 * (len(self._path.poses) - 1)))
        if (
            self._max_progress_idx >= progress_threshold
            and self._distancer.distance_to_goal(pos) < self._config.goal_tolerance
        ):
            self._state = "final_rotation"
            return self._step_final_rotation()

        # Adaptive lookahead (RPP): size the lookahead to the current pursuit
        # speed so straights get a long, stable horizon and slow corners get a
        # short, tight one. lookahead_speed_scale == 0 keeps the fixed dist.
        if self._config.lookahead_speed_scale > 0.0:
            self._distancer._lookahead_dist = float(
                np.clip(
                    self._config.lookahead_speed_scale * self._v_cur,
                    self._config.lookahead_min,
                    self._config.lookahead_max,
                )
            )

        lookahead = self._distancer.find_lookahead_point(closest)
        twist = self._controller.advance(lookahead, self._current_odom)
        return float(twist.linear.x), float(twist.linear.y), float(twist.angular.z)

    def _step_final_rotation(self) -> tuple[float, float, float]:
        assert self._path is not None and self._current_odom is not None
        goal_yaw = self._path.poses[-1].orientation.euler[2]
        robot_yaw = self._current_odom.orientation.euler[2]
        yaw_err = angle_diff(goal_yaw, robot_yaw)

        if abs(yaw_err) < self._config.orientation_tolerance:
            self._state = "arrived"
            logger.info(f"PathFollowerTask '{self._name}' arrived")
            return 0.0, 0.0, 0.0

        twist = self._controller.rotate(yaw_err)
        return float(twist.linear.x), float(twist.linear.y), float(twist.angular.z)

    # Public API (called by runner — typically over RPC from a tool)

    def configure(
        self,
        speed: float | None = None,
        k_angular: float | None = None,
        lookahead_dist: float | None = None,
        lookahead_min: float | None = None,
        lookahead_max: float | None = None,
        lookahead_speed_scale: float | None = None,
        max_yaw_rate: float | None | object = _UNSET,
        forward_only: bool | None = None,
        ff_config: FeedforwardGainConfig | None | object = _UNSET,
        velocity_profile_config: VelocityProfileConfig | None | object = _UNSET,
        external_profile_cap: PathSpeedCapProtocol | None | object = _UNSET,
        **ignored: Any,
    ) -> bool:
        """Override per-run knobs before start_path. ``ff_config``,
        ``velocity_profile_config``, ``external_profile_cap`` and
        ``max_yaw_rate`` use a sentinel so callers can explicitly clear them
        by passing ``None`` (distinct from "don't touch"). Only valid while
        idle/arrived/aborted — refuses while the task is actively
        driving the robot.

        ``external_profile_cap`` wins over ``velocity_profile_config``
        when both are set; this is how a ReferenceGovernor is installed
        as the per-tick cap source.

        Unknown kwargs are accepted and logged so callers built for a
        sibling task's configure signature (e.g. the trajectory tracker's
        eso/deadtime knobs) work unchanged.
        """
        if self.is_active():
            logger.warning(f"PathFollowerTask '{self._name}': cannot configure while active")
            return False
        if speed is not None:
            self._config.speed = speed
            self._controller._speed = speed  # PController exposes _speed
        if k_angular is not None:
            self._config.k_angular = k_angular
            self._controller._k_angular = k_angular
        if lookahead_dist is not None:
            # Takes effect when the next start_path() rebuilds the PathDistancer.
            self._config.lookahead_dist = lookahead_dist
        if lookahead_min is not None:
            self._config.lookahead_min = lookahead_min
        if lookahead_max is not None:
            self._config.lookahead_max = lookahead_max
        if lookahead_speed_scale is not None:
            self._config.lookahead_speed_scale = lookahead_speed_scale
        if max_yaw_rate is not _UNSET:
            self._config.max_yaw_rate = max_yaw_rate  # type: ignore[assignment]
        if forward_only is not None:
            self._config.forward_only = forward_only
        if ff_config is not _UNSET:
            self._config.ff_config = ff_config  # type: ignore[assignment]
            self._ff = (
                FeedforwardGainCompensator(ff_config)  # type: ignore[arg-type]
                if ff_config is not None
                else None
            )
        # external_profile_cap takes precedence over velocity_profile_config.
        if external_profile_cap is not _UNSET:
            self._profile_cap = external_profile_cap  # type: ignore[assignment]
            # Track in config only when we're falling back to the auto-built path;
            # external_profile_cap is not serialisable into VelocityProfileConfig.
            self._config.velocity_profile_config = None
        elif velocity_profile_config is not _UNSET:
            self._config.velocity_profile_config = velocity_profile_config  # type: ignore[assignment]
            self._profile_cap = (
                PathSpeedCap(velocity_profile_config)  # type: ignore[arg-type]
                if velocity_profile_config is not None
                else None
            )
        if ignored:
            logger.info(
                f"PathFollowerTask '{self._name}': ignoring unknown configure "
                f"kwargs {sorted(ignored)}"
            )
        return True

    def start_path(
        self,
        path: Path,
        current_odom: PoseStamped,
    ) -> bool:
        if path is None or len(path.poses) < 2:
            logger.warning(f"PathFollowerTask '{self._name}': invalid path")
            return False
        self._path = path
        self._distancer = PathDistancer(path, self._config.lookahead_dist)
        self._current_odom = current_odom
        self._max_progress_idx = 0
        self._v_cur = 0.0
        self._controller.reset_errors()
        if self._pid is not None:
            self._pid.reset()
        if self._ff is not None:
            self._ff.reset()
        if self._profile_cap is not None:
            self._profile_cap.for_path(path)
        # Reset the per-waypoint speed cap so the parent's compute() path
        # is well-defined. Subclasses (e.g. PrecisionPathFollowerTask) may
        # repopulate these slots in their own start_path() override.
        self._velocity_profile = None
        self._velocity_profile_pts = None

        first_yaw = path.poses[0].orientation.euler[2]
        robot_yaw = current_odom.orientation.euler[2]
        yaw_err = angle_diff(first_yaw, robot_yaw)
        self._controller.reset_yaw_error(yaw_err)

        if abs(yaw_err) < self._config.orientation_tolerance:
            # Note: production LocalPlanner transitions to "final_rotation" when
            # the robot is exactly at path[0] (pos_d < 0.01). That's broken for
            # open paths - we'd snap to "arrived" immediately. Always start in
            # path_following when aligned; arrival is detected by distance_to_goal.
            self._state = "path_following"
        else:
            self._state = "initial_rotation"

        logger.info(
            f"PathFollowerTask '{self._name}' started "
            f"({len(path.poses)} poses, initial state={self._state})"
        )
        return True

    def update_odom(self, odom: PoseStamped) -> None:
        # Pose now flows in through compute()'s CoordinatorState (sourced
        # from the twist-base adapter's read_odometry → joint positions).
        # This setter is kept as a no-op-or-override seam so out-of-tree
        # callers that still pump odom externally don't break.
        self._current_odom = odom

    def set_path(self, path: Path, odom: PoseStamped | None = None) -> None:
        """Coordinator broadcast hook for nav-stack-emitted paths.

        ``ControlCoordinator._on_path`` calls this on every task exposing
        ``set_path`` with a fresh odom snapshot, so a clicked nav goal (or a
        replan) arms the follower the same way the benchmark's RPC
        ``start_path`` does. Prefers the caller-supplied ``odom``; falls back to
        ``self._current_odom`` for single-arg callers. Each call resets the
        follower onto the new path — exactly what makes pursuit robust to
        replanning (no clock to re-sync, no re-ramp from rest).

        A path with fewer than 2 poses is a nav-planner stop signal (the MLS
        planner emits an empty ``Path`` when nothing ahead is traversable), so
        it cancels any active follow instead of leaving the stale path armed.
        """
        if len(path.poses) < 2:
            if self.cancel():
                logger.info(
                    f"PathFollowerTask '{self._name}': empty path — cancelled active follow"
                )
            return
        use_odom = odom if odom is not None else self._current_odom
        if use_odom is None:
            logger.warning(
                f"PathFollowerTask '{self._name}': received path but no odom available; dropping."
            )
            return
        logger.info(
            f"PathFollowerTask '{self._name}': received path from stream (n={len(path.poses)})"
        )
        self.start_path(path, use_odom)

    def set_speed(self, speed: float) -> None:
        """Set the follower's target/cruise speed at runtime.

        ``ControlCoordinator._on_speed`` calls this on every task exposing
        ``set_speed`` when a ``speed`` message arrives, so an external source
        (e.g. the benchmark sweeping speeds) can retune the follower over the
        transport without RPC. Updates the pursuit speed and rebuilds the
        curvature cap so corner regulation tracks the new top speed (clamped to
        the measured ceiling ``_v_max_cap`` when a subclass set one). A no-op
        while actively driving — a mid-run jump would discontinuously move the
        cap; the next path picks up the new speed cleanly.
        """
        if self.is_active():
            logger.warning(f"PathFollowerTask '{self._name}': ignoring set_speed while active")
            return
        speed = float(speed)
        self._config.speed = speed
        self._controller._speed = speed  # PController exposes _speed
        self._apply_profile_speed(speed)
        logger.info(f"PathFollowerTask '{self._name}': set_speed({speed:.3f})")

    def _apply_profile_speed(self, speed: float) -> None:
        """Rebuild the auto-built curvature cap so its top speed follows
        ``speed`` (clamped to ``_v_max_cap``). No-op when no curvature profile
        is configured, or when an external non-``PathSpeedCap`` governor is
        installed (don't stomp a caller-supplied cap)."""
        if self._config.velocity_profile_config is None:
            return
        if self._profile_cap is not None and not isinstance(self._profile_cap, PathSpeedCap):
            return
        eff = speed if self._v_max_cap is None else min(speed, self._v_max_cap)
        cfg = replace(self._config.velocity_profile_config, max_linear_speed=eff)
        self._config.velocity_profile_config = cfg
        self._profile_cap = PathSpeedCap(cfg)

    def cancel(self) -> bool:
        if not self.is_active():
            return False
        self._state = "aborted"
        return True

    def reset(self) -> bool:
        if self.is_active():
            return False
        self._state = "idle"
        self._path = None
        self._distancer = None
        self._current_odom = None
        return True

    def get_state(self) -> PathFollowerState:
        return self._state


class PathFollowerTaskParams(BaseConfig):
    speed: float = 0.55
    control_frequency: float = 10.0
    goal_tolerance: float = 0.2
    orientation_tolerance: float = 0.35
    k_angular: float = 0.5
    lookahead_dist: float = 0.5
    lookahead_min: float = 0.3
    lookahead_max: float = 0.9
    lookahead_speed_scale: float = 0.0
    max_yaw_rate: float | None = None
    forward_only: bool = False


def create_task(cfg: Any, hardware: Any) -> PathFollowerTask:
    params = PathFollowerTaskParams.model_validate(cfg.params)
    return PathFollowerTask(
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
    )
