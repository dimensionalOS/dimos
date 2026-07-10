# Copyright 2026 Dimensional Inc.
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

"""Reactive lidar safety shield gating ``cmd_vel`` against the live point cloud.

``LidarShield`` is a drop-in middleware between the velocity source
(``MovementManager.cmd_vel``, remapped) and the robot connection. Every lidar
frame maintains a filtered obstacle set; every velocity command is gated
*directionally* — only the motion component toward an obstacle is blocked,
every other direction passes at its natural speed:

- **Braking capsule** — obstacles inside the stopping envelope of the
  *commanded* velocity (``shield_radius_m`` plus reaction and braking
  distance, within a corridor of ``corridor_halfwidth_m``) zero the linear
  command, so the robot stops short of a wall even at full speed, in any
  direction of travel. Yaw passes through.
- **Contact bubble** — points inside ``shield_radius_m`` block commands
  aimed at them; commands moving away or tangent pass, clamped to
  ``escape_speed_mps``. Pure rotation always passes (clamped to
  ``escape_yaw_rps`` while in contact).

Release is hysteretic: the zones must stay clear (with
``release_hysteresis_m`` margin) for ``clear_frames_to_release`` consecutive
frames.

Planner awareness: while engaged the breach points are published as dense
vertical columns back onto the lidar topic (``map_out`` remapped to the
mapper's input), tagged with an ``intensities`` channel so the shield skips
its own echo. The voxel map — and hence ``MLSPlannerNative``, which replans
on every map frame — immediately sees a wall at the breach and routes around
it. ``VoxelGridMapper``'s column carving erases the columns once the real
lidar sees through that space again.

Staleness is motion-aware: the Go2's utlidar voxel-map stream *pauses while
the robot is stationary in a static scene* (any scene change or robot motion
resumes it), so a stale lidar with fresh odometry and no body motion is the
normal idle state — commands pass clamped to ``stale_crawl_speed_mps`` until
the stream resumes. A stale lidar while *moving* (or stale odometry) is a
real dropout and fails closed when ``fail_closed`` is set.

Frame notes (Go2 WebRTC lidar): the decoded cloud has world-aligned X/Y that
matches ``odom`` X/Y, but its Z band follows the robot rather than the world.
The ground plane is therefore estimated per frame from a low percentile of the
cloud itself (``ground_percentile``), and the obstacle band is taken relative
to that estimate; ``ground_z_fixed_m`` overrides the estimate when set.
"""

from __future__ import annotations

from collections import deque
import math
import threading
import time
from typing import Any

from dimos_lcm.std_msgs import Bool  # type: ignore[import-untyped]
import numpy as np
from numpy.typing import NDArray
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_EMPTY_PTS: NDArray[np.floating] = np.zeros((0, 3), dtype=np.float32)

# Tangent tolerance for the contact bubble: points are "approached" when the
# command direction has more than this fraction of its magnitude toward them.
_APPROACH_DOT = 0.05


class LidarShieldConfig(ModuleConfig):
    enabled: bool = True

    # Contact bubble around the robot center. The Go2 nose is ~0.35 m ahead
    # of center, so anything below ~0.4 only fires after contact.
    shield_radius_m: float = 0.45
    release_hysteresis_m: float = 0.15
    min_points: int = 3
    clear_frames_to_release: int = 3

    # Point prefilter. Points inside the body footprint box are self-returns
    # or post-contact smear, never real obstacles.
    max_range_m: float = 4.0
    ignore_radius_m: float = 0.0
    body_halflength_m: float = 0.38
    body_halfwidth_m: float = 0.22

    # Braking capsule along the commanded velocity.
    corridor_halfwidth_m: float = 0.30
    brake_decel_mps2: float = 1.5
    reaction_time_s: float = 0.45
    min_speed_mps: float = 0.05

    # Obstacle band relative to the per-frame ground estimate. The moving
    # Go2's voxel map stacks the floor 2-4 levels high, so a point in the
    # band only counts when its XY column rises to min_obstacle_top_m —
    # walls, legs, and furniture qualify; floor dilation never does.
    ground_percentile: float = 10.0
    ground_z_fixed_m: float | None = None
    min_obstacle_height_m: float = 0.12
    max_obstacle_height_m: float = 1.5
    min_obstacle_top_m: float = 0.30
    column_bin_m: float = 0.10

    # Behavior while in contact (points inside the bubble).
    allow_escape: bool = True
    escape_speed_mps: float = 0.3
    escape_yaw_rps: float = 0.5

    # Staleness handling. Stationary robot + stale lidar is the Go2's normal
    # idle state (the voxel-map stream pauses); commands then pass at crawl.
    fail_closed: bool = True
    sensor_timeout_s: float = 1.5
    stationary_speed_mps: float = 0.05
    stale_crawl_speed_mps: float = 0.25

    # Planner awareness via map reinforcement.
    reinforce_map: bool = True
    inject_height_m: float = 0.6
    inject_voxel_m: float = 0.05
    max_inject_columns: int = 1500


class _ShieldCore:
    """Transport-free shield state machine over numpy point sets.

    All coordinates are world-frame X/Y (shared by the Go2 lidar cloud and
    odom). Callers pass ``now`` explicitly so the core stays clock-free and
    unit-testable.
    """

    def __init__(self, config: LidarShieldConfig) -> None:
        self._cfg = config
        self._robot_xy: NDArray[np.floating] | None = None
        self._yaw = 0.0
        self._odom_time = -math.inf
        self._odom_hist: deque[tuple[float, float, float]] = deque()
        self._lidar_time = -math.inf
        self._pts: NDArray[np.floating] = _EMPTY_PTS
        self._intent_v_world: NDArray[np.floating] = np.zeros(2)
        self.ground_z = 0.0
        self.engaged = False
        self.breach: NDArray[np.floating] = _EMPTY_PTS
        self.nearest_m = math.inf
        self._clear_streak = 0

    @property
    def points_in_band(self) -> int:
        return int(self._pts.shape[0])

    def robot_speed(self) -> float:
        """Body speed estimated from the odometry history window."""
        if len(self._odom_hist) < 2:
            return 0.0
        t0, x0, y0 = self._odom_hist[0]
        t1, x1, y1 = self._odom_hist[-1]
        dt = t1 - t0
        if dt < 0.1:
            return 0.0
        return math.hypot(x1 - x0, y1 - y0) / dt

    def stale_state(self, now: float) -> str:
        """``fresh``, ``paused`` (stationary idle, benign) or ``dropout``."""
        timeout = self._cfg.sensor_timeout_s
        if now - self._odom_time > timeout:
            return "dropout"
        if now - self._lidar_time <= timeout:
            return "fresh"
        if self.robot_speed() <= self._cfg.stationary_speed_mps:
            return "paused"
        return "dropout"

    def sensor_ages(self, now: float) -> tuple[float, float]:
        return now - self._lidar_time, now - self._odom_time

    def on_odom(self, x: float, y: float, yaw: float, now: float) -> None:
        self._robot_xy = np.array([x, y])
        self._yaw = yaw
        self._odom_time = now
        self._odom_hist.append((now, x, y))
        while self._odom_hist and now - self._odom_hist[0][0] > 1.0:
            self._odom_hist.popleft()

    def on_lidar(self, points: NDArray[np.floating], now: float) -> bool:
        """Ingest a lidar frame; returns True when the engaged state flipped."""
        self._lidar_time = now
        if self._robot_xy is None or points.shape[0] == 0:
            self._pts = _EMPTY_PTS
            return self._evaluate()

        cfg = self._cfg
        rel = points[:, :2] - self._robot_xy
        d2 = rel[:, 0] ** 2 + rel[:, 1] ** 2
        keep = d2 <= cfg.max_range_m**2
        if cfg.ignore_radius_m > 0.0:
            keep &= d2 >= cfg.ignore_radius_m**2
        c, s = math.cos(self._yaw), math.sin(self._yaw)
        body_x = c * rel[:, 0] + s * rel[:, 1]
        body_y = -s * rel[:, 0] + c * rel[:, 1]
        keep &= (np.abs(body_x) > cfg.body_halflength_m) | (np.abs(body_y) > cfg.body_halfwidth_m)
        pts = points[keep]

        if pts.shape[0]:
            ground = cfg.ground_z_fixed_m
            if ground is None:
                ground = float(np.percentile(pts[:, 2], cfg.ground_percentile))
            self.ground_z = ground
            z_rel = pts[:, 2] - ground
            band = (z_rel >= cfg.min_obstacle_height_m) & (z_rel <= cfg.max_obstacle_height_m)
            pts, z_band = pts[band], z_rel[band]
            if pts.shape[0]:
                prominent = _prominent_column_mask(
                    pts[:, :2], z_band, cfg.column_bin_m, cfg.min_obstacle_top_m
                )
                pts = pts[prominent]

        self._pts = pts
        return self._evaluate()

    def on_cmd(self, twist: Twist, now: float) -> tuple[Twist, bool]:
        """Gate one velocity command; returns (output, engaged state flipped).

        Directional: only commands whose braking capsule hits obstacles, or
        that push toward points already inside the contact bubble, get their
        linear part zeroed. Everything else passes; contact proximity only
        clamps the speed.
        """
        cfg = self._cfg
        vx, vy, wz = float(twist.linear.x), float(twist.linear.y), float(twist.angular.z)
        c, s = math.cos(self._yaw), math.sin(self._yaw)
        v_world = np.array([c * vx - s * vy, s * vx + c * vy])
        self._intent_v_world = v_world
        speed = float(np.hypot(v_world[0], v_world[1]))

        if self.engaged and not cfg.allow_escape:
            return Twist(), False

        if self._robot_xy is None or self._pts.shape[0] == 0 or speed < cfg.min_speed_mps:
            return twist, False

        rel = self._pts[:, :2] - self._robot_xy
        d2 = rel[:, 0] ** 2 + rel[:, 1] ** 2
        margin = cfg.release_hysteresis_m if self.engaged else 0.0
        bubble = d2 <= (cfg.shield_radius_m + margin) ** 2
        u = v_world / speed

        threat = self._capsule_mask(rel, u, speed, margin)
        in_contact = bool(bubble.any())
        if in_contact:
            fwd = rel @ u
            threat |= bubble & (fwd > _APPROACH_DOT * np.sqrt(d2))

        if int(np.count_nonzero(threat)) >= cfg.min_points:
            was_engaged = self.engaged
            self._engage(threat)
            wz_out = max(-cfg.escape_yaw_rps, min(cfg.escape_yaw_rps, wz)) if in_contact else wz
            blocked = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, wz_out))
            return blocked, not was_engaged

        if in_contact:
            scale = min(1.0, cfg.escape_speed_mps / speed)
            wz_out = max(-cfg.escape_yaw_rps, min(cfg.escape_yaw_rps, wz))
            return Twist(Vector3(vx * scale, vy * scale, 0.0), Vector3(0.0, 0.0, wz_out)), False

        return twist, False

    def _capsule_mask(
        self,
        rel: NDArray[np.floating],
        u: NDArray[np.floating],
        speed: float,
        margin: float,
    ) -> NDArray[np.bool_]:
        cfg = self._cfg
        reach = (
            cfg.shield_radius_m
            + margin
            + speed * cfg.reaction_time_s
            + speed * speed / (2.0 * cfg.brake_decel_mps2)
        )
        fwd = rel @ u
        lat = np.abs(rel[:, 0] * u[1] - rel[:, 1] * u[0])
        result: NDArray[np.bool_] = (
            (fwd > 0.0) & (fwd <= reach) & (lat <= cfg.corridor_halfwidth_m + margin)
        )
        return result

    def _evaluate(self) -> bool:
        """Latch/release the engaged state from the latest lidar frame.

        Presence in the bubble (any direction) or in the intent capsule keeps
        the shield engaged — engagement drives map injection and viz, not the
        gate, which stays directional per command.
        """
        cfg = self._cfg
        if self._pts.shape[0] == 0 or self._robot_xy is None:
            return self._settle(threat=False)

        margin = cfg.release_hysteresis_m if self.engaged else 0.0
        rel = self._pts[:, :2] - self._robot_xy
        d2 = rel[:, 0] ** 2 + rel[:, 1] ** 2
        mask = d2 <= (cfg.shield_radius_m + margin) ** 2

        speed = float(np.hypot(*self._intent_v_world))
        if speed >= cfg.min_speed_mps:
            mask |= self._capsule_mask(rel, self._intent_v_world / speed, speed, margin)

        if int(np.count_nonzero(mask)) >= cfg.min_points:
            was_engaged = self.engaged
            self._engage(mask)
            return not was_engaged
        return self._settle(threat=False)

    def _engage(self, mask: NDArray[np.bool_]) -> None:
        assert self._robot_xy is not None
        self.breach = self._pts[mask]
        rel = self.breach[:, :2] - self._robot_xy
        self.nearest_m = float(np.sqrt(np.min(rel[:, 0] ** 2 + rel[:, 1] ** 2)))
        self._clear_streak = 0
        self.engaged = True

    def _settle(self, threat: bool) -> bool:
        if threat or not self.engaged:
            return False
        self._clear_streak += 1
        if self._clear_streak >= self._cfg.clear_frames_to_release:
            self.engaged = False
            self.breach = _EMPTY_PTS
            self.nearest_m = math.inf
            return True
        return False


class LidarShield(Module):
    """Gate ``cmd_vel`` against the live lidar and reinforce the planner map.

    Splice into a blueprint with three remappings: ``cmd_vel_in`` from the
    velocity source's (renamed) output, ``cmd_vel_out`` to the robot's
    ``cmd_vel``, and ``map_out`` to the lidar topic (injected obstacle
    columns ride the normal mapping path; the shield skips its own echo by
    the ``intensities`` tag). Removing the module and the remappings restores
    the original wiring.
    """

    config: LidarShieldConfig

    lidar: In[PointCloud2]
    odom: In[PoseStamped]
    cmd_vel_in: In[Twist]

    cmd_vel_out: Out[Twist]
    map_out: Out[PointCloud2]
    shield_points: Out[PointCloud2]
    shield_engaged: Out[Bool]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._core = _ShieldCore(self.config)
        self._lock = threading.Lock()
        self._last_stale_warn = -math.inf
        self._last_pause_log = -math.inf

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.odom.subscribe(self._on_odom)))
        self.register_disposable(Disposable(self.lidar.subscribe(self._on_lidar)))
        self.register_disposable(Disposable(self.cmd_vel_in.subscribe(self._on_cmd)))

    @rpc
    def stop(self) -> None:
        self.cmd_vel_out.publish(Twist())
        super().stop()

    def _on_odom(self, msg: PoseStamped) -> None:
        with self._lock:
            self._core.on_odom(
                msg.position.x, msg.position.y, msg.orientation.euler[2], time.monotonic()
            )

    def _on_lidar(self, msg: PointCloud2) -> None:
        if not self.config.enabled:
            return

        pcd = msg.pointcloud_tensor
        if "intensities" in pcd.point:
            return  # our own injected columns echoing back on the topic
        pts = pcd.point["positions"].numpy() if "positions" in pcd.point else _EMPTY_PTS

        with self._lock:
            changed = self._core.on_lidar(pts, time.monotonic())
            engaged = self._core.engaged
            breach = self._core.breach
            ground_z = self._core.ground_z
            nearest = self._core.nearest_m

        if changed and engaged:
            self.cmd_vel_out.publish(Twist())

        if engaged and breach.shape[0]:
            if self.config.reinforce_map:
                columns = _obstacle_columns(
                    breach[:, :2],
                    ground_z,
                    self.config.inject_voxel_m,
                    self.config.inject_height_m,
                    self.config.max_inject_columns,
                )
                self.map_out.publish(
                    PointCloud2.from_numpy(
                        columns,
                        frame_id=msg.frame_id,
                        timestamp=msg.ts,
                        intensities=np.ones(columns.shape[0], dtype=np.float32),
                    )
                )
            self.shield_points.publish(
                PointCloud2.from_numpy(
                    breach.astype(np.float32, copy=False), frame_id=msg.frame_id, timestamp=msg.ts
                )
            )
        if changed:
            self._announce(engaged, nearest)

    def _on_cmd(self, msg: Twist) -> None:
        now = time.monotonic()
        if not self.config.enabled:
            self.cmd_vel_out.publish(msg)
            return

        out: Twist | None
        lidar_age = odom_age = 0.0
        changed, nearest, state = False, math.inf, "fresh"
        with self._lock:
            state = self._core.stale_state(now)
            if state == "dropout":
                out = None
                lidar_age, odom_age = self._core.sensor_ages(now)
            else:
                out, changed = self._core.on_cmd(msg, now)
                nearest = self._core.nearest_m

        if out is None:
            if not self.config.fail_closed:
                self.cmd_vel_out.publish(msg)
                return
            if now - self._last_stale_warn > 2.0:
                self._last_stale_warn = now
                logger.warning(
                    "Lidar shield: sensor dropout, holding zero velocity",
                    lidar_age_s=round(lidar_age, 2),
                    odom_age_s=round(odom_age, 2),
                )
            self.cmd_vel_out.publish(Twist())
            return

        if state == "paused":
            out = _clamp_linear(out, self.config.stale_crawl_speed_mps)
            if now - self._last_pause_log > 5.0:
                self._last_pause_log = now
                logger.info(
                    "Lidar shield: lidar paused while stationary — "
                    "passing commands at crawl speed until the stream resumes"
                )

        self.cmd_vel_out.publish(out)
        if changed:
            self._announce(engaged=True, nearest=nearest)

    def _announce(self, engaged: bool, nearest: float) -> None:
        self.shield_engaged.publish(Bool(data=engaged))
        if engaged:
            logger.warning(
                "Lidar shield ENGAGED — obstacle in stop zone", nearest_m=round(nearest, 2)
            )
        else:
            logger.info("Lidar shield released")
            self.shield_points.publish(PointCloud2(frame_id="world", ts=time.time()))

    @rpc
    def set_enabled(self, enabled: bool) -> None:
        self.config.enabled = enabled
        logger.info("Lidar shield %s", "enabled" if enabled else "disabled (pass-through)")

    @rpc
    def set_params(self, **params: Any) -> dict[str, Any]:
        """Live-update config fields (validated); returns the effective config."""
        updates = _validated_updates(self.config, params)
        with self._lock:
            for key, value in updates.items():
                setattr(self.config, key, value)
        if updates:
            logger.info("Lidar shield params updated: %s", updates)
        return self.config.model_dump()

    @rpc
    def get_status(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            lidar_age, odom_age = self._core.sensor_ages(now)
            return {
                "enabled": self.config.enabled,
                "engaged": self._core.engaged,
                "stale_state": self._core.stale_state(now),
                "robot_speed_mps": round(self._core.robot_speed(), 3),
                "points_in_band": self._core.points_in_band,
                "breach_points": int(self._core.breach.shape[0]),
                "nearest_m": self._core.nearest_m,
                "ground_z": self._core.ground_z,
                "lidar_age_s": lidar_age,
                "odom_age_s": odom_age,
            }


def _clamp_linear(twist: Twist, max_speed: float) -> Twist:
    """Return ``twist`` with its linear XY speed capped at ``max_speed``."""
    speed = math.hypot(float(twist.linear.x), float(twist.linear.y))
    if speed <= max_speed:
        return twist
    scale = max_speed / speed
    return Twist(
        Vector3(twist.linear.x * scale, twist.linear.y * scale, 0.0),
        Vector3(0.0, 0.0, twist.angular.z),
    )


def _validated_updates(config: LidarShieldConfig, params: dict[str, Any]) -> dict[str, Any]:
    """Coerce ``params`` through the config model; raises on unknown keys."""
    unknown = set(params) - set(LidarShieldConfig.model_fields)
    if unknown:
        raise ValueError(f"unknown parameters: {sorted(unknown)}")
    merged = LidarShieldConfig.model_validate({**config.model_dump(), **params})
    return {key: getattr(merged, key) for key in params}


def _prominent_column_mask(
    xy: NDArray[np.floating],
    z_rel: NDArray[np.floating],
    bin_m: float,
    min_top_m: float,
) -> NDArray[np.bool_]:
    """Keep points whose XY column reaches ``min_top_m`` above ground.

    The Go2 voxel map dilates the floor vertically while the robot walks, so
    isolated points at 0.1-0.2 m are floor, not obstacles; real obstacles
    (walls, legs, furniture) always own a column of points rising higher.
    """
    kx = np.floor(xy[:, 0] / bin_m).astype(np.int64)
    ky = np.floor(xy[:, 1] / bin_m).astype(np.int64)
    key = (kx << 32) | (ky & np.int64(0xFFFFFFFF))
    uniq, inv = np.unique(key, return_inverse=True)
    top = np.zeros(uniq.shape[0])
    np.maximum.at(top, inv, z_rel)
    result: NDArray[np.bool_] = top[inv] >= min_top_m
    return result


def _obstacle_columns(
    xy: NDArray[np.floating],
    ground_z: float,
    voxel: float,
    height: float,
    max_columns: int,
) -> NDArray[np.float32]:
    """Vertical point columns over the deduplicated voxel columns of ``xy``."""
    keys = np.unique(np.floor(xy / voxel).astype(np.int64), axis=0)
    if keys.shape[0] > max_columns:
        keys = keys[:: int(np.ceil(keys.shape[0] / max_columns))]
    centers = (keys.astype(np.float32) + 0.5) * voxel
    levels = (ground_z + np.arange(voxel, height + 0.5 * voxel, voxel)).astype(np.float32)
    n, m = centers.shape[0], levels.shape[0]
    out = np.empty((n * m, 3), dtype=np.float32)
    out[:, 0] = np.repeat(centers[:, 0], m)
    out[:, 1] = np.repeat(centers[:, 1], m)
    out[:, 2] = np.tile(levels, n)
    return out
