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

"""Why did the local plan change its mind? Offline post-mortem of a recording.

Reads a recording of the go2-zenoh-motion graph (local_map, odometry, tf,
planner_path, path) and runs four passes:

  churn    what the local map does to the planner's world, frame to frame,
           split into crop-boundary (the map window moved) and interior
           (obstacles flickering in place) — the second kind is the bad kind
  plans    when the published plan flipped, holds, and whether the flips land
           on the frames where a new local map arrived
  replay   the planner re-run on the recorded inputs (same tf-resolved pose and
           carrot the module used), then a one-input-at-a-time ablation that
           attributes each flip to the cloud, the pose, or the carrot
  latency  how old the inputs each tick planned on actually were

    python -m dimos.navigation.motion.adapter.diagnose ml-trajectory-research/20260805-033007.zenoh.mcap
    python -m dimos.navigation.motion.adapter.diagnose rec.mcap --only churn --spawn
    python -m dimos.navigation.motion.adapter.diagnose rec.mcap --only replay --no-anchor

The replay anchors the cloud to the floor exactly as the module does, off the
same tf prior -- unless the recording says otherwise: the replay sniffs which
band the DEPLOYED module actually ran (whose holds agree with the recorded
plans) and follows it. `--no-anchor` forces the raw band.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from itertools import pairwise
from pathlib import Path as FsPath
import time
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.msgs.helpers import resolve_msg_type
from dimos.navigation.motion.adapter.floor import anchor_to_floor, estimate_floor
from dimos.navigation.motion.adapter.planner import carrot_along, route_changed
from dimos.navigation.motion.control.profile import ceilings_to_clearance, decode_ceilings
from dimos.navigation.motion.planner.referee.geometry import AvoidanceConfig
from dimos.navigation.motion.planner.referee.planners.base import load as load_planner
from dimos.navigation.motion.planner.referee.planners.target import band_mask
from dimos.navigation.motion.planner.referee.scenarios import EMBODIMENTS, Scenario
from dimos.navigation.motion.planner.referee.types import PointCloud2 as RefereeCloud
from dimos.navigation.tf_pose import OdomBasePose, base_height_above_ground

if TYPE_CHECKING:
    from dimos.memory2.store.base import Store
    from dimos.navigation.motion.planner.referee.planners.base import PlannerEpisode

# Stream names as the zenoh recorder slugs them (topic -> name, "/" -> "_").
LOCAL_MAP = "dimos_local_map_sensor_msgs.PointCloud2"
ODOMETRY = "dimos_odometry_nav_msgs.Odometry"
PLANNER_PATH = "dimos_planner_path_nav_msgs.Path"
PATH = "dimos_path_nav_msgs.Path"
TF = "dimos_tf_tf2_msgs.TFMessage"

Z_BAND = (0.05, 0.45)  # planners/target.py: the cloud slice that can touch the body
FLIP_M = 0.5  # plan divergence (m, mean over the common arc) that counts as a mind change
KEY_OFF, KEY_SPAN = 8192, 16384  # voxel key packing: +-655 m at 0.08 m
SUPPORT_MIN = 4  # RayTracingVoxelMapConfig.support_min on the deployed blueprint


class _LcmCodec:
    """Decode one mcap channel whose schema names a dimos msg type."""

    def __init__(self, payload_type: type) -> None:
        self.payload_type = payload_type

    def decode(self, data: bytes) -> Any:
        return self.payload_type.lcm_decode(data)  # type: ignore[attr-defined]


def open_lcm_mcap(path: str) -> Store:
    """Open an mcap of lcm-encoded dimos msgs, decoding by schema name."""
    from mcap.reader import make_reader

    from dimos.memory2.store.mcap import McapStore

    with open(path, "rb") as f:
        summary = make_reader(f).get_summary()
    codecs: dict[str, Any] = {}
    for channel in (summary.channels if summary else {}).values():
        schema = summary.schemas.get(channel.schema_id) if summary else None
        msg_type = resolve_msg_type(schema.name) if schema else None
        if msg_type is not None:
            codecs[channel.topic] = _LcmCodec(msg_type)
    return McapStore(path=path, codecs=codecs)


# ------------------------------------------------------------------ loading --


@dataclass
class Tick:
    """One published plan and the inputs the module held when it made it."""

    ts: float
    imap: int  # index into Recording.maps
    pose: tuple[float, ...]  # x, y, yaw, base z
    goal: tuple[float, float]
    recorded: np.ndarray  # the published plan, xy
    # Where tf put the ground under the base at this tick: the bound the floor
    # estimate is held to, exactly as MotionPlanner._floor_prior computes it.
    floor_prior: float | None = None
    # Bumped when the global route MOVED, so (imap, route_seq) is the pair
    # MotionPlanner's replan gate keys on.
    route_seq: int = 0
    # Per-waypoint clearance decoded from the plan's own stamps -- the precision
    # the planner asked for, not anything recomputed offline.
    stamped_clear: np.ndarray | None = None


@dataclass
class Recording:
    """A motion-stack recording, decoded and pose-resolved the way the stack does."""

    path: str
    maps: list[tuple[float, np.ndarray]]  # local_map: ts, points (n, 3)
    odom_ts: np.ndarray
    odom_xy: np.ndarray  # sensor position, which is what the raycaster crops around
    poses: list[tuple[float, ...] | None]  # base_link (x, y, yaw, z)
    globals: list[tuple[float, np.ndarray]]  # planner_path: ts, xy
    plans: list[tuple[float, np.ndarray]]  # path: ts, (x, y, yaw)
    ticks: list[Tick] = field(default_factory=list)

    @property
    def t0(self) -> float:
        return self.maps[0][0]


def _xy(msg: Any) -> np.ndarray:
    return np.array([[p.position.x, p.position.y] for p in msg.poses]).reshape(-1, 2)


def _xyy(msg: Any) -> np.ndarray:
    """Plan poses as (x, y, yaw) -- the yaw is real, in-place turns carry it."""
    return np.array(
        [[p.position.x, p.position.y, p.orientation.euler[2]] for p in msg.poses]
    ).reshape(-1, 3)


def _stream(store: Store, name: str) -> list[Any]:
    """Decoded observations of one stream."""
    return list(store.stream(name))


def _before(ts: float, stamps: np.ndarray) -> int:
    """Index of the newest sample at or before ts (-1 when there is none)."""
    return int(np.searchsorted(stamps, ts, "right")) - 1


def load_recording(
    path: str, base_frame: str, lookahead: float, lidar_height: float = 0.0
) -> Recording:
    """Decode the streams and rebuild each tick's inputs (tf-resolved pose, carrot)."""
    from dimos.memory2.tf import StreamTF
    from dimos.utils.data import get_data

    store = open_lcm_mcap(str(get_data(path)))
    tf = StreamTF.from_store(store, TF)
    if tf is None:
        raise SystemExit(f"{path}: no {TF} stream — cannot resolve the base pose")
    base = OdomBasePose(tf, base_frame)
    maps = [(o.ts, o.data.points_f32()) for o in _stream(store, LOCAL_MAP)]
    odom = [(o.ts, o.data) for o in _stream(store, ODOMETRY)]
    poses: list[tuple[float, ...] | None] = []
    priors: list[float | None] = []
    base_height: float | None = None
    for _, msg in odom:
        p = base.resolve(msg)
        poses.append(
            (p.position.x, p.position.y, p.orientation.euler[2], p.position.z) if p else None
        )
        if lidar_height > 0.0 and base_height is None and msg.child_frame_id != base_frame:
            leg = base.sensor_to_base(msg.child_frame_id)
            if leg is not None:
                base_height = base_height_above_ground(lidar_height, -leg)
        priors.append(None if p is None or base_height is None else p.position.z - base_height)
    globals_ = [(o.ts, _xy(o.data)) for o in _stream(store, PLANNER_PATH)]
    raw_plans = _stream(store, PATH)
    plans = [(o.ts, _xyy(o.data)) for o in raw_plans]
    stamped: list[np.ndarray | None] = []
    for o in raw_plans:
        ceilings = decode_ceilings(o.data)
        stamped.append(ceilings_to_clearance(ceilings) if ceilings is not None else None)

    rec = Recording(
        path=path,
        maps=maps,
        odom_ts=np.array([t for t, _ in odom]),
        odom_xy=np.array([[m.pose.position.x, m.pose.position.y] for _, m in odom]),
        poses=poses,
        globals=globals_,
        plans=plans,
    )
    map_ts = np.array([t for t, _ in maps])
    global_ts = np.array([t for t, _ in globals_])
    # MLS republishes its route at ~1 Hz whether or not it moved; the module's
    # gate keys on the route CHANGING, so number them the same way.
    route_seq = [0]
    for a, b in pairwise(globals_):
        route_seq.append(route_seq[-1] + int(route_changed(a[1], b[1])))
    for n, (ts, xy) in enumerate(plans):
        i, j, k = _before(ts, map_ts), _before(ts, rec.odom_ts), _before(ts, global_ts)
        if min(i, j, k) < 0 or rec.poses[j] is None or not len(globals_[k][1]):
            continue
        pose = rec.poses[j]
        assert pose is not None
        goal = carrot_along(globals_[k][1], (pose[0], pose[1]), lookahead)
        rec.ticks.append(
            Tick(
                ts=ts,
                imap=i,
                pose=pose,
                goal=goal,
                recorded=xy,
                floor_prior=priors[j],
                route_seq=route_seq[k],
                stamped_clear=stamped[n],
            )
        )
    return rec


def gated_ticks(ticks: list[Tick]) -> list[Tick]:
    """The ticks a replan gate would keep: the first of each (map, route) pair."""
    kept: list[Tick] = []
    for tick in ticks:
        key = (tick.imap, tick.route_seq)
        if not kept or (kept[-1].imap, kept[-1].route_seq) != key:
            kept.append(tick)
    return kept


# ------------------------------------------------------------------ metrics --


def resample(xy: np.ndarray, step: float = 0.1) -> np.ndarray:
    """Path resampled at even arc length, so two plans compare point for point."""
    xy = xy[:, :2] if xy.ndim == 2 else xy
    if len(xy) < 2:
        return xy.reshape(-1, 2)
    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))])
    if arc[-1] <= 0:
        return xy[:1]
    s = np.arange(0.0, arc[-1], step)
    return np.column_stack([np.interp(s, arc, xy[:, 0]), np.interp(s, arc, xy[:, 1])])


def arclen(xy: np.ndarray) -> float:
    xy = xy[:, :2] if xy.ndim == 2 else xy
    return float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum()) if len(xy) > 1 else 0.0


def divergence(a: np.ndarray, b: np.ndarray) -> float:
    """Mean distance between two plans over the arc they share (m)."""
    ra, rb = resample(a), resample(b)
    n = min(len(ra), len(rb))
    if n == 0:
        return float("nan")
    return float(np.linalg.norm(ra[:n] - rb[:n], axis=1).mean())


def is_hold(xy: np.ndarray) -> bool:
    """A single-pose path is the planner saying "hold, no safe route"."""
    return len(xy) < 2


def voxel_keys(points: np.ndarray, voxel: float) -> np.ndarray:
    """Unique packed voxel keys for a cloud."""
    k = np.floor(points / voxel).astype(np.int64) + KEY_OFF
    return np.asarray(np.unique((k[:, 0] * KEY_SPAN + k[:, 1]) * KEY_SPAN + k[:, 2]))


def voxel_centers(keys: np.ndarray, voxel: float) -> np.ndarray:
    """Voxel centres for packed keys."""
    kz = keys % KEY_SPAN
    ky = (keys // KEY_SPAN) % KEY_SPAN
    kx = keys // (KEY_SPAN * KEY_SPAN)
    ijk = np.column_stack([kx, ky, kz]) - KEY_OFF
    return (ijk + 0.5) * voxel


def neighbours(keys: np.ndarray, among: np.ndarray) -> np.ndarray:
    """How many of each key's 26 neighbours are present in `among` (sorted)."""
    if not len(keys):
        return np.zeros(0, dtype=np.int64)
    offsets = [
        (dx * KEY_SPAN + dy) * KEY_SPAN + dz
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
        for dz in (-1, 0, 1)
        if (dx, dy, dz) != (0, 0, 0)
    ]
    return np.asarray(np.sum([np.isin(keys + o, among) for o in offsets], axis=0))


# -------------------------------------------------------------------- churn --


@dataclass
class Crop:
    """The local map's emitted window: a cylinder on the batch's mean origin."""

    centre: np.ndarray  # xy
    radius: float
    z_lo: float
    z_hi: float

    def inside(self, pts: np.ndarray, margin: float) -> np.ndarray:
        d = np.linalg.norm(pts[:, :2] - self.centre, axis=1)
        return np.asarray(
            (d < self.radius - margin)
            & (pts[:, 2] > self.z_lo + margin)
            & (pts[:, 2] < self.z_hi - margin)
        )


def crop_of(rec: Recording, ts: float, points: np.ndarray, window: float) -> Crop:
    """Recover the emitted window from the cloud itself (the raycaster hard-crops it)."""
    lo, hi = _before(ts - window, rec.odom_ts), _before(ts, rec.odom_ts)
    centre = rec.odom_xy[max(lo, 0) : max(hi + 1, 1)].mean(axis=0)
    radius = float(np.linalg.norm(points[:, :2] - centre, axis=1).max())
    return Crop(np.asarray(centre), radius, float(points[:, 2].min()), float(points[:, 2].max()))


@dataclass
class Frame:
    """One emitted local map, voxelized."""

    ts: float
    points: np.ndarray
    keys: np.ndarray
    band: np.ndarray  # keys of the slice the planner reads
    crop: Crop


def churn(rec: Recording, voxel: float, rr: Any = None) -> list[dict[str, float]]:
    """Per-frame voxel appear/disappear, split boundary vs interior, whole map vs band."""
    frames = []
    for ts, pts in rec.maps:
        slab = pts[(pts[:, 2] > Z_BAND[0]) & (pts[:, 2] < Z_BAND[1])]
        frames.append(
            Frame(
                ts=ts,
                points=pts,
                keys=voxel_keys(pts, voxel),
                band=voxel_keys(slab, voxel) if len(slab) else np.empty(0, dtype=np.int64),
                crop=crop_of(rec, ts, pts, 1.0),
            )
        )
    rows: list[dict[str, float]] = []
    print(f"\n=== churn ({voxel:.2f} m voxels, {len(frames)} local_map frames) ===")
    print(
        f"{'t':>7} {'voxels':>7} {'appear':>7} {'gone':>7} "
        f"{'iAppear':>8} {'iGone':>7} {'crop_r':>7} | {'band':>6} {'bApp':>5} {'bGone':>6}"
    )
    for a, b in pairwise(frames):
        appear = np.setdiff1d(b.keys, a.keys, assume_unique=True)
        gone = np.setdiff1d(a.keys, b.keys, assume_unique=True)

        def interior(keys: np.ndarray, ca: Crop = a.crop, cb: Crop = b.crop) -> np.ndarray:
            """The ones that changed well inside both windows — not window edge."""
            if not len(keys):
                return keys
            c = voxel_centers(keys, voxel)
            return np.asarray(keys[ca.inside(c, 2 * voxel) & cb.inside(c, 2 * voxel)])

        i_appear, i_gone = interior(appear), interior(gone)
        # Thin-obstacle signature: a voxel the raycaster only ever emitted because
        # it was hit THIS frame (support_min unsupported) vanishes as soon as the
        # emitting sweep misses it.
        thin = neighbours(i_gone, a.keys) < SUPPORT_MIN
        row = {
            "ts": b.ts,
            "voxels": float(len(b.keys)),
            "appear": float(len(appear)),
            "gone": float(len(gone)),
            "interior_appear": float(len(i_appear)),
            "interior_gone": float(len(i_gone)),
            "interior_gone_thin": float(thin.sum()),
            "interior_radius": float(
                np.median(
                    np.linalg.norm(voxel_centers(i_gone, voxel)[:, :2] - b.crop.centre, axis=1)
                )
                if len(i_gone)
                else np.nan
            ),
            "crop_r": b.crop.radius,
            "band": float(len(b.band)),
            "band_appear": float(len(np.setdiff1d(b.band, a.band, assume_unique=True))),
            "band_gone": float(len(np.setdiff1d(a.band, b.band, assume_unique=True))),
        }
        rows.append(row)
        print(
            f"{b.ts - rec.t0:7.1f} {len(b.keys):7d} {len(appear):7d} {len(gone):7d} "
            f"{len(i_appear):8d} {len(i_gone):7d} {b.crop.radius:7.2f} | "
            f"{len(b.band):6d} {row['band_appear']:5.0f} {row['band_gone']:6.0f}"
        )
        if rr is not None:
            rr.set_time("time", timestamp=b.ts)
            rr.log("world/map", rr.Points3D(b.points[::4], radii=0.01, colors=[[90, 90, 105]]))
            rr.log(
                "world/appeared",
                rr.Points3D(voxel_centers(appear, voxel), radii=0.015, colors=[[60, 220, 60]]),
            )
            rr.log(
                "world/disappeared",
                rr.Points3D(voxel_centers(gone, voxel), radii=0.015, colors=[[230, 50, 50]]),
            )

    def col(key: str) -> np.ndarray:
        return np.array([r[key] for r in rows])

    n, radius = col("voxels").mean(), col("crop_r")
    total = col("appear") + col("gone")
    inside = col("interior_appear") + col("interior_gone")
    band = col("band").mean()
    print(
        f"\nmap {n:7.0f} voxels | appear {col('appear').mean():5.0f} "
        f"({100 * col('appear').mean() / n:.1f}%) gone {col('gone').mean():5.0f} "
        f"({100 * col('gone').mean() / n:.1f}%) | interior "
        f"{col('interior_appear').mean():.0f}/{col('interior_gone').mean():.0f} = "
        f"{100 * inside.sum() / total.sum():.0f}% of it\n"
        f"crop radius {radius.min():.2f}..{radius.max():.2f} m, |step| mean "
        f"{np.abs(np.diff(radius)).mean():.2f} m max {np.abs(np.diff(radius)).max():.2f} m\n"
        f"planner band z {Z_BAND[0]}..{Z_BAND[1]}: {band:.0f} voxels | appear "
        f"{col('band_appear').mean():.0f} ({100 * col('band_appear').mean() / band:.1f}%) gone "
        f"{col('band_gone').mean():.0f} ({100 * col('band_gone').mean() / band:.1f}%) per frame\n"
        f"interior vanishings with < {SUPPORT_MIN} neighbours (thin/isolated): "
        f"{100 * col('interior_gone_thin').sum() / col('interior_gone').sum():.0f}%, "
        f"median {np.nanmedian(col('interior_radius')):.2f} m from the sensor"
    )
    return rows


# -------------------------------------------------------------------- plans --


def plans(rec: Recording) -> list[dict[str, float]]:
    """Consecutive-plan divergence, holds, and whether flips land on new map frames."""
    rows: list[dict[str, float]] = []
    for a, b in pairwise(rec.ticks):
        rows.append(
            {
                "ts": b.ts,
                "div": divergence(a.recorded, b.recorded),
                "dlen": abs(arclen(b.recorded) - arclen(a.recorded)),
                "new_map": float(a.imap != b.imap),
                "hold": float(is_hold(a.recorded) or is_hold(b.recorded)),
                "goal_jump": float(np.linalg.norm(np.array(b.goal) - np.array(a.goal))),
                "pose_step": float(np.hypot(b.pose[0] - a.pose[0], b.pose[1] - a.pose[1])),
            }
        )
    d = np.array([r["div"] for r in rows])
    live = np.array([not r["hold"] for r in rows])
    new = np.array([bool(r["new_map"]) for r in rows]) & live
    same = ~np.array([bool(r["new_map"]) for r in rows]) & live
    holds = sum(1 for t in rec.ticks if is_hold(t.recorded))
    detour = np.array(
        [
            arclen(t.recorded) / max(np.hypot(*(np.array(t.goal) - np.array(t.pose[:2]))), 1e-6)
            for t in rec.ticks
            if not is_hold(t.recorded)
        ]
    )
    print(f"\n=== plans ({len(rec.ticks)} published, {holds} holds) ===")
    print(
        f"consecutive divergence: mean {np.nanmean(d[live]):.2f} m  median "
        f"{np.nanmedian(d[live]):.2f}  p90 {np.nanpercentile(d[live], 90):.2f}  max "
        f"{np.nanmax(d[live]):.2f}\n"
        f"  same local_map: {same.sum():4d} pairs  mean {np.nanmean(d[same]):.2f} m  p90 "
        f"{np.nanpercentile(d[same], 90):.2f}  flips>{FLIP_M} m: {int((d[same] > FLIP_M).sum())}\n"
        f"  new  local_map: {new.sum():4d} pairs  mean {np.nanmean(d[new]):.2f} m  p90 "
        f"{np.nanpercentile(d[new], 90):.2f}  flips>{FLIP_M} m: {int((d[new] > FLIP_M).sum())}\n"
        f"plan arc / goal distance: mean {detour.mean():.2f}x  p90 "
        f"{np.percentile(detour, 90):.2f}x  max {detour.max():.2f}x\n"
        f"plan length step between ticks: mean {np.nanmean([r['dlen'] for r in rows]):.2f} m  "
        f"max {np.nanmax([r['dlen'] for r in rows]):.2f} m"
    )
    flips = [r for r in rows if r["div"] > FLIP_M and not r["hold"]]
    print(
        f"across a flip the robot moved {np.mean([r['pose_step'] for r in flips]):.3f} m and the "
        f"carrot moved {np.mean([r['goal_jump'] for r in flips]):.2f} m\n"
        f"flips > {FLIP_M} m ({len(flips)}): " + " ".join(f"{r['ts'] - rec.t0:.1f}" for r in flips)
    )
    return rows


# ------------------------------------------------------------------- replay --


def _plan_bodies(rr: Any, plan: np.ndarray, base_z: float, emb: Any, color: list[int]) -> Any:
    """Wireframe body boxes along a plan, one every ~0.3 m of arc or 23 deg of yaw.

    A third column is the pose's own yaw (in-place turns show); with only xy the
    yaw is derived from the segment direction.
    """
    xy = plan[:, :2]
    yaw = plan[:, 2] if plan.shape[1] >= 3 else None
    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))])
    keep = [0]
    for i in range(1, len(xy)):
        turned = yaw is not None and abs(yaw[i] - yaw[keep[-1]]) >= np.radians(23)
        if arc[i] - arc[keep[-1]] >= 0.3 or turned:
            keep.append(i)
    yaws = []
    for i in keep:
        if yaw is not None:
            yaws.append(float(yaw[i]))
            continue
        d = xy[min(i + 1, len(xy) - 1)] - xy[max(i - 1, 0)]
        yaws.append(
            float(np.arctan2(d[1], d[0])) if np.linalg.norm(d) > 1e-9 else yaws[-1] if yaws else 0.0
        )
    return rr.Boxes3D(
        centers=[[float(xy[i][0]), float(xy[i][1]), base_z] for i in keep],
        half_sizes=[[emb.length / 2, emb.width / 2, 0.2]] * len(keep),
        rotation_axis_angles=[rr.RotationAxisAngle(axis=(0, 0, 1), radians=y) for y in yaws],
        colors=[color] * len(keep),
    )


FULL_SPEED_CLEAR = 0.35  # AvoidanceConfig.speed_clearance: above this, no governing


def _precision_circles(rr: Any, plan: np.ndarray, clear: np.ndarray, z: float, emb: Any) -> Any:
    """Requested-precision circles from the plan's own stamps, replay.py's palette.

    Radius = decoded clearance (capped at full speed); red at the precision
    floor, yellow governed, green full speed.
    """
    xy = plan[:, :2]
    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))])
    idx = np.unique(np.searchsorted(arc, np.arange(0.0, arc[-1] + 1e-9, 0.35)))
    a = np.linspace(0.0, 2 * np.pi, 33)
    circles, cols = [], []
    for i in idx:
        i = min(int(i), len(xy) - 1, len(clear) - 1)
        r = float(min(max(clear[i], 0.02), FULL_SPEED_CLEAR))
        circles.append(
            np.column_stack([xy[i][0] + r * np.cos(a), xy[i][1] + r * np.sin(a), np.full(33, z)])
        )
        if clear[i] <= emb.precision:
            cols.append([255, 60, 60])
        elif clear[i] < FULL_SPEED_CLEAR:
            cols.append([255, 220, 60])
        else:
            cols.append([80, 220, 80])
    return rr.LineStrips3D(circles, colors=cols, radii=0.004)


def episode(planner: str, embodiment: str) -> PlannerEpisode:
    scenario = Scenario("diagnose", [], goal=(0.0, 0.0), emb=EMBODIMENTS[embodiment])
    ep = load_planner(planner)(scenario, AvoidanceConfig())
    ep.reset()
    return ep


def replay(
    rec: Recording,
    planner: str,
    embodiment: str,
    z_offset: float,
    ablate: bool,
    anchor: bool | None = None,
    ground_margin: float = 0.16,
    gate: bool = False,
    rr: Any = None,
) -> list[dict[str, float]]:
    """Re-plan every recorded tick from its own inputs; optionally ablate one input at a time."""
    ep = episode(planner, embodiment)
    ticks = gated_ticks(rec.ticks) if gate else rec.ticks
    offset = np.array([0.0, 0.0, z_offset], dtype=np.float32)
    # what plan() last handed the episode, so the renderer can mark the very
    # cloud the search saw instead of re-deriving one
    seen: dict[str, Any] = {"pts": None, "floor": 0.0}

    def plan(
        imap: int,
        pose: tuple[float, ...],
        goal: tuple[float, float],
        prior: float | None = None,
    ) -> np.ndarray:
        # The module's own order (adapter/planner.py::_plan_once): the trim
        # corrects the map's z origin, then the floor is measured off that map.
        pts = rec.maps[imap][1] + offset
        seen["floor"] = 0.0
        if prior is not None:
            floor = estimate_floor(pts, (pose[0], pose[1]), prior=prior)
            if floor is not None:
                pts = anchor_to_floor(pts, floor, ground_margin)
                seen["floor"] = floor
        seen["pts"] = pts
        planned = ep.plan(
            RefereeCloud.from_numpy(pts, frame_id="odom"), (pose[0], pose[1], pose[2]), goal
        )
        return np.array(
            [[q.position.x, q.position.y, q.orientation.euler.yaw] for q in planned.poses]
        ).reshape(-1, 3)

    t = ticks[len(ticks) // 2]
    twice = plan(t.imap, t.pose, t.goal, t.floor_prior), plan(t.imap, t.pose, t.goal, t.floor_prior)
    deterministic = twice[0].shape == twice[1].shape and bool(np.allclose(*twice))

    # Which band did the DEPLOYED module run? Anchoring degrades to the raw
    # band in the field when tf/lidar_height are not in place, so sniff: the
    # band whose holds agree with the recorded plans is the band the robot
    # used. `anchor` None = auto; --no-anchor forces raw.
    use_anchor = anchor
    if use_anchor is None:
        sample = ticks[:: max(1, len(ticks) // 40)]
        agree_a = agree_r = 0
        for t in sample:
            held = is_hold(t.recorded)
            agree_a += is_hold(plan(t.imap, t.pose, t.goal, t.floor_prior)) == held
            agree_r += is_hold(plan(t.imap, t.pose, t.goal, None)) == held
        use_anchor = agree_a >= agree_r
        print(
            f"config sniff: anchored holds agree {agree_a}/{len(sample)}, raw "
            f"{agree_r}/{len(sample)} -> replaying {'anchored' if use_anchor else 'RAW band'}"
            + ("" if use_anchor else " (the robot ran without floor anchoring)")
        )

    def prior_of(t: Tick) -> float | None:
        return t.floor_prior if use_anchor else None

    anchored = bool(use_anchor) and any(t.floor_prior is not None for t in rec.ticks)

    rows: list[dict[str, float]] = []
    started = time.perf_counter()
    previous: np.ndarray | None = None
    emb = EMBODIMENTS[embodiment]
    for tick in ticks:
        out = plan(tick.imap, tick.pose, tick.goal, prior_of(tick))
        row = {
            "ts": tick.ts,
            "n_rec": float(len(tick.recorded)),
            "n_replay": float(len(out)),
            "div": divergence(tick.recorded, out),
            "arc": arclen(out),
            "arc_rec": arclen(tick.recorded),
            "consec": float("nan") if previous is None else divergence(previous, out),
        }
        rows.append(row)
        previous = out
        if rr is not None:
            rr.set_time("time", timestamp=tick.ts)
            rr.log("world/carrot", rr.Points3D([[*tick.goal, 0.0]], radii=0.07))
            # the planner's own obstacle slice (band_mask) of the very cloud
            # plan() handed it, holds included -- shifted back onto the map
            obs = seen["pts"][band_mask(seen["pts"])]
            rr.log(
                "world/obstacles",
                rr.Points3D(
                    np.column_stack([obs[:, :2], obs[:, 2] + seen["floor"]]),
                    radii=0.022,
                    colors=[[255, 120, 0]],
                ),
            )
        if rr is not None:
            # each source cleared on its own hold: a hold means "no active
            # path", and a plan left on screen would be one the follower was
            # not actually tracking
            base_z = tick.pose[3] if len(tick.pose) > 3 else 0.0
            if len(out) > 1:
                z = np.full(len(out), 0.02)
                rr.log(
                    "world/replay",
                    rr.LineStrips3D(
                        [np.column_stack([out[:, :2], z])], colors=[[150, 150, 150]], radii=0.012
                    ),
                )
                rr.log(
                    "world/replay/bodies", _plan_bodies(rr, out, base_z, emb, [150, 150, 150, 140])
                )
            else:
                rr.log("world/replay", rr.Clear(recursive=True))
            if len(tick.recorded) > 1:
                rr.log(
                    "world/requested",
                    rr.LineStrips3D(
                        [
                            np.column_stack(
                                [tick.recorded[:, :2], np.full(len(tick.recorded), 0.03)]
                            )
                        ],
                        colors=[[100, 160, 255]],
                        radii=0.012,
                    ),
                )
                rr.log(
                    "world/requested/bodies",
                    _plan_bodies(rr, tick.recorded, base_z, emb, [100, 160, 255, 140]),
                )
                if tick.stamped_clear is not None:
                    rr.log(
                        "world/requested/precision",
                        _precision_circles(rr, tick.recorded, tick.stamped_clear, 0.04, emb),
                    )
            else:
                rr.log("world/requested", rr.Clear(recursive=True))
    wall = time.perf_counter() - started

    d = np.array([r["div"] for r in rows])
    rec_hold = np.array([r["n_rec"] < 2 for r in rows])
    rep_hold = np.array([r["n_replay"] < 2 for r in rows])
    both = ~rec_hold & ~rep_hold
    band = "floor-anchored" if anchored else "raw z-band"
    cadence = "gated on input arrival" if gate else "every recorded tick"
    print(
        f"\n=== replay ({planner}, {band}, {cadence}, "
        f"z_offset {z_offset:+.2f} m, {len(rows)} ticks) ==="
    )
    print(
        f"deterministic (same inputs twice): {deterministic}\n"
        f"holds: recorded {int(rec_hold.sum())}, replayed {int(rep_hold.sum())}, agreeing "
        f"{int((rec_hold & rep_hold).sum())}\n"
        f"replay vs recorded plan: mean {np.nanmean(d[both]):.2f} m  median "
        f"{np.nanmedian(d[both]):.2f}  p90 {np.nanpercentile(d[both], 90):.2f}\n"
        f"replayed arc mean {np.nanmean([r['arc'] for r in rows]):.2f} m "
        f"(recorded {np.nanmean([r['arc_rec'] for r in rows]):.2f} m); "
        f"consecutive replay divergence mean "
        f"{np.nanmean([r['consec'] for r in rows if r['consec'] == r['consec']]):.2f} m\n"
        f"plan wall time {1e3 * wall / len(rows):.1f} ms/tick"
    )
    if ablate:
        _ablate(ticks, plan, prior_of)
    return rows


def _ablate(ticks: list[Tick], plan: Any, prior_of: Any) -> None:
    """Re-plan each tick pair changing ONE input, to attribute the change."""
    rows = []
    for a, b in pairwise(ticks):
        # the floor prior travels with the pose: it is read off the same
        # odometry message the pose is
        base = plan(a.imap, a.pose, a.goal, prior_of(a))
        rows.append(
            {
                "ts": b.ts,
                "all": divergence(base, plan(b.imap, b.pose, b.goal, prior_of(b))),
                "cloud": divergence(base, plan(b.imap, a.pose, a.goal, prior_of(a))),
                "pose": divergence(base, plan(a.imap, b.pose, a.goal, prior_of(b))),
                "goal": divergence(base, plan(a.imap, a.pose, b.goal, prior_of(a))),
                "hold": float(is_hold(base)),
                "new_map": float(a.imap != b.imap),
            }
        )
    live = np.array([not r["hold"] for r in rows])
    big = live & np.array([r["all"] > FLIP_M for r in rows])
    cols = ("all", "cloud", "pose", "goal")

    def mean(where: np.ndarray, key: str) -> float:
        return float(np.nanmean([r[key] for r, m in zip(rows, where, strict=True) if m]))

    print(
        f"\nablation ({int(live.sum())} tick pairs): mean divergence when only ONE input moves\n"
        + "  "
        + "  ".join(f"{c}: {mean(live, c):.3f} m" for c in cols)
        + "\n"
        f"  flips > {FLIP_M} m: {int(big.sum())} ("
        f"{int(sum(r['new_map'] for r, m in zip(rows, big, strict=True) if m))} on a new local_map)\n"
        + "  "
        + "  ".join(f"{c}: {mean(big, c):.3f} m" for c in cols)
    )


# ------------------------------------------------------------------ latency --


def latency(rec: Recording) -> list[dict[str, float]]:
    """Age of the inputs each tick planned on, and the publish cadence of each stream."""
    map_ts = np.array([t for t, _ in rec.maps])
    global_ts = np.array([t for t, _ in rec.globals])
    rows = [
        {
            "ts": t.ts,
            "map_age": t.ts - map_ts[t.imap],
            "odom_age": t.ts - rec.odom_ts[_before(t.ts, rec.odom_ts)],
            "global_age": t.ts - global_ts[_before(t.ts, global_ts)],
        }
        for t in rec.ticks
    ]
    print("\n=== latency ===")
    print(f"{'stream':>14} {'n':>6} {'median dt':>10} {'p95':>9} {'max':>9}")
    for name, stamps in (
        ("local_map", map_ts),
        ("odometry", rec.odom_ts),
        ("planner_path", global_ts),
        ("path", np.array([t for t, _ in rec.plans])),
    ):
        dt = np.diff(stamps) * 1e3
        print(
            f"{name:>14} {len(stamps):6d} {np.median(dt):9.1f}ms {np.percentile(dt, 95):8.1f}ms "
            f"{dt.max():8.1f}ms"
        )
    print(f"\n{'input at a plan tick':>22} {'mean':>8} {'p95':>8} {'max':>8}")
    ages = (("map_age", "local_map"), ("odom_age", "odometry"), ("global_age", "planner_path"))
    for key, label in ages:
        v = np.array([r[key] for r in rows]) * 1e3
        print(f"{label:>22} {v.mean():7.0f}ms {np.percentile(v, 95):7.0f}ms {v.max():7.0f}ms")
    return rows


# -------------------------------------------------------------------- plots --


def write_plots(
    churn_rows: list[dict[str, float]],
    plan_rows: list[dict[str, float]],
    latency_rows: list[dict[str, float]],
    out: FsPath,
) -> None:
    """Churn / flip / age time series as SVG (dimos.memory2.vis.plot)."""
    from dimos.memory2.vis import color
    from dimos.memory2.vis.plot.elements import Series, VLine
    from dimos.memory2.vis.plot.plot import Plot

    out.mkdir(parents=True, exist_ok=True)
    if churn_rows:
        p = Plot()
        ts = [r["ts"] for r in churn_rows]
        p.add(Series(ts=ts, values=[r["appear"] + r["gone"] for r in churn_rows], label="churn"))
        p.add(
            Series(
                ts=ts,
                values=[r["interior_appear"] + r["interior_gone"] for r in churn_rows],
                label="interior churn",
                color=color.orange.hex(),
            )
        )
        p.add(
            Series(
                ts=ts,
                values=[r["crop_r"] for r in churn_rows],
                label="crop radius (m)",
                axis="m",
                color=color.blue.hex(),
            )
        )
        for r in plan_rows:
            if r["div"] > FLIP_M and not r["hold"]:
                p.add(VLine(x=r["ts"], color=color.red.hex(), opacity=0.5))
        p.to_svg(str(out / "churn.svg"))
    if plan_rows:
        p = Plot()
        p.add(
            Series(
                ts=[r["ts"] for r in plan_rows],
                values=[r["div"] for r in plan_rows],
                label="plan divergence (m)",
            )
        )
        p.add(
            Series(
                ts=[r["ts"] for r in plan_rows],
                values=[r["dlen"] for r in plan_rows],
                label="plan length change (m)",
                color=color.orange.hex(),
            )
        )
        for r in plan_rows:
            if r["hold"]:
                p.add(VLine(x=r["ts"], color=color.red.hex(), opacity=0.3))
        p.to_svg(str(out / "plans.svg"))
    if latency_rows:
        p = Plot()
        for key, label, c in (
            ("map_age", "local_map age (s)", color.blue),
            ("odom_age", "odometry age (s)", color.green),
            ("global_age", "planner_path age (s)", color.orange),
        ):
            p.add(
                Series(
                    ts=[r["ts"] for r in latency_rows],
                    values=[r[key] for r in latency_rows],
                    label=label,
                    color=c.hex(),
                )
            )
        p.to_svg(str(out / "latency.svg"))
    print("\nplots: " + " ".join(str(f) for f in sorted(out.glob("*.svg"))))


# ---------------------------------------------------------------------- cli --


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("recording", help="path to the .mcap recording")
    ap.add_argument("--only", default="churn,plans,replay,latency", help="passes to run")
    ap.add_argument("--voxel", type=float, default=0.08, help="raycaster voxel size")
    ap.add_argument("--planner", default="target")
    ap.add_argument("--embodiment", default="go2")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--lookahead", type=float, default=5.0, help="carrot arc along planner_path")
    ap.add_argument("--z-offset", type=float, default=0.0, help="MotionPlanner cloud_z_offset")
    ap.add_argument(
        "--lidar-height",
        type=float,
        default=0.45,
        help="lidar height above ground; tf turns it into the floor prior (0 = none)",
    )
    ap.add_argument(
        "--ground-margin", type=float, default=0.16, help="MotionPlanner ground_margin_m"
    )
    ap.add_argument(
        "--no-anchor", action="store_true", help="force the raw z-band (default: sniff)"
    )
    ap.add_argument(
        "--gate", action="store_true", help="replay only the ticks a replan gate would keep"
    )
    ap.add_argument("--no-ablate", action="store_true", help="skip the one-input-at-a-time replay")
    ap.add_argument("--spawn", action="store_true", help="live rerun viewer instead of an rrd")
    ap.add_argument("--out", default="recordings", help="where the rrd and svgs land")
    ap.add_argument(
        "--rrd", default=None, help="rrd path (default: <out>/<recording>-diagnose.rrd)"
    )
    ap.add_argument("--plots", default=None, help="svg dir (default: <out>/<recording>-diagnose)")
    args = ap.parse_args()

    passes = {p.strip() for p in args.only.split(",")}
    rec = load_recording(args.recording, args.base_frame, args.lookahead, args.lidar_height)
    stem = FsPath(args.out) / f"{FsPath(args.recording).stem}-diagnose"
    stem.parent.mkdir(parents=True, exist_ok=True)
    print(
        f"{args.recording}: {len(rec.maps)} local_map, {len(rec.plans)} plans, "
        f"{len(rec.ticks)} ticks with complete inputs, "
        f"{rec.plans[-1][0] - rec.t0:.1f} s"
    )

    import rerun as rr

    rrd = args.rrd or f"{stem}.rrd"
    rr.init("motion-diagnose", spawn=args.spawn)
    if not args.spawn:
        rr.save(rrd)

    # the base_link box at full odometry rate (30 Hz), whatever passes run --
    # plan ticks are ~1 Hz under the replan gate, far too coarse to watch
    emb = EMBODIMENTS[args.embodiment]
    rr.log(
        "world/robot",
        rr.Boxes3D(half_sizes=[[emb.length / 2, emb.width / 2, 0.2]], colors=[[0, 255, 127]]),
        static=True,
    )
    last_body: tuple[float, ...] | None = None
    n_bodies = 0
    for ots, pose in zip(rec.odom_ts, rec.poses, strict=True):
        if pose is None:
            continue
        rr.set_time("time", timestamp=float(ots))
        rr.log(
            "world/robot",
            rr.Transform3D(
                translation=[pose[0], pose[1], pose[3] if len(pose) > 3 else 0.0],
                rotation=rr.RotationAxisAngle(axis=(0, 0, 1), radians=pose[2]),
            ),
        )
        # breadcrumb bodies: a wireframe stays behind at each pose the body
        # actually swept, appearing at the moment it was there
        moved = last_body is None or np.hypot(pose[0] - last_body[0], pose[1] - last_body[1]) >= 0.3
        turned = last_body is not None and abs(pose[2] - last_body[2]) >= np.radians(23)
        if moved or turned:
            rr.log(
                f"world/track/bodies/{n_bodies:04d}",
                rr.Boxes3D(
                    centers=[[pose[0], pose[1], pose[3] if len(pose) > 3 else 0.0]],
                    half_sizes=[[emb.length / 2, emb.width / 2, 0.2]],
                    rotation_axis_angles=[rr.RotationAxisAngle(axis=(0, 0, 1), radians=pose[2])],
                    colors=[[0, 200, 100, 140]],
                ),
            )
            last_body = pose
            n_bodies += 1
    # the track the body actually drove -- "recorded", vs the plans it was asked
    # to drive (requested) and what the planner says offline (replay)
    walked = np.array([[p[0], p[1], 0.01] for p in rec.poses if p is not None])
    if len(walked) > 1:
        rr.log(
            "world/track",
            rr.LineStrips3D([walked], colors=[[0, 200, 100]], radii=0.008),
            static=True,
        )

    churn_rows = churn(rec, args.voxel, rr) if "churn" in passes else []
    plan_rows = plans(rec) if "plans" in passes else []
    if "replay" in passes:
        replay(
            rec,
            args.planner,
            args.embodiment,
            args.z_offset,
            not args.no_ablate,
            anchor=False if args.no_anchor else None,
            ground_margin=args.ground_margin,
            gate=args.gate,
            rr=rr,
        )
    latency_rows = latency(rec) if "latency" in passes else []
    write_plots(churn_rows, plan_rows, latency_rows, FsPath(args.plots or str(stem)))
    if not args.spawn:
        print(f"rerun: {rrd}")


if __name__ == "__main__":
    main()
