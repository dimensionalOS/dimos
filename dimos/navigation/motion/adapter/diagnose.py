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
planner_path, path, nav_cmd_vel, stop_movement) and runs six passes:

  churn     what the local map does to the planner's world, frame to frame,
            split into crop-boundary (the map window moved) and interior
            (obstacles flickering in place) — the second kind is the bad kind
  plans     when the published plan flipped, holds, and whether the flips land
            on the frames where a new local map arrived
  tracking  how precisely the body held the plan it was following: cross-track
            error at every odometry sample, split into a SUSTAINED offset
            (rolling median) and the gait-bounce bumps riding on it
  replay    the planner re-run on the recorded inputs (same tf-resolved pose and
            carrot the module used), then a one-input-at-a-time ablation that
            attributes each flip to the cloud, the pose, or the carrot
  latency   how old the inputs each tick planned on actually were -- inter-arrival
            cadence always, plus the TRUE pipeline age (receipt minus payload
            stamp) on recordings whose stamps speak sensor time
  follower  the FOLLOWER re-run: at every recorded nav_cmd_vel tick, the
            deployed law on the deployed config against the twist that actually
            went out, one tick classified match / boundary / hold / MISMATCH

    python -m dimos.navigation.motion.adapter.diagnose ml-trajectory-research/20260805-033007.zenoh.mcap
    python -m dimos.navigation.motion.adapter.diagnose rec.mcap --only churn --spawn
    python -m dimos.navigation.motion.adapter.diagnose rec.mcap --only replay --model raw_band
    python -m dimos.navigation.motion.adapter.diagnose rec.mcap --only follower \\
        --host-config motion-host.json --from 6.9 --to 8.6

The replay reads obstacles through `motion/obstacles.py`, the same models the
module runs -- and by default it SNIFFS which one the deployed module actually
ran, replaying a tick subsample under each and keeping the one whose holds
agree with the recorded plans. `--model` names one instead.

The follower pass reconstructs each tick's inputs the way the module does (the
tf-resolved base pose, the latest path, and — on the `hinted` track — the room
hint RECOMPUTED from the latest local map through that same obstacle model,
not decoded from the path's stamps; the `blind` track decodes the stamps). The
deployed config arrives as one JSON: `--host-config` reads
`modules.trajectory_follower.config` off a motion-host blob, and without it the
`go2-zenoh-motion` blueprint's own values stand in. The law is stateful (it
rate-limits its own command), so ticks run in order through ONE law instance
and the window only decides which ticks are REPORTED — a window's first tick
inherits the state the robot's did.

`--from` / `--to` bound every pass. Both take either seconds from the start of
the recording (`--from 6.9`) or a wall-clock time of day in UTC, matching the
message stamps (`--from 06:34:35.400`).
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from itertools import pairwise
import json
import math
from pathlib import Path as FsPath
import time
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.helpers import resolve_msg_type
from dimos.navigation.motion.adapter.follower import GoalLatch, path_clearance
from dimos.navigation.motion.adapter.planner import REPLAN_CARROT_M, carrot_along, replan_due
from dimos.navigation.motion.control.controller import ControllerConfig, load as load_law
from dimos.navigation.motion.control.profile import ceilings_to_clearance, decode_ceilings
from dimos.navigation.motion.control.referee.judge import REROLL_M
from dimos.navigation.motion.control.tracks import TRACKS
from dimos.navigation.motion.embodiment import EMBODIMENTS
from dimos.navigation.motion.geometry import AvoidanceConfig, divergence
from dimos.navigation.motion.obstacles import (
    OBSTACLE_MODELS,
    RAW_BAND,
    ObstacleModel,
    hard_points,
    load as load_model,
)
from dimos.navigation.motion.planner.planners.base import load as load_planner
from dimos.navigation.motion.scenarios import Scenario
from dimos.navigation.tf_pose import OdomBasePose

if TYPE_CHECKING:
    from dimos.memory2.store.base import Store
    from dimos.msgs.nav_msgs.Path import Path as NavPath
    from dimos.navigation.motion.control.controller import TrajectoryController
    from dimos.navigation.motion.planner.planners.base import PlannerEpisode

# Stream names as the zenoh recorder slugs them (topic -> name, "/" -> "_").
LOCAL_MAP = "dimos_local_map_sensor_msgs.PointCloud2"
ODOMETRY = "dimos_odometry_nav_msgs.Odometry"
PLANNER_PATH = "dimos_planner_path_nav_msgs.Path"
PATH = "dimos_path_nav_msgs.Path"
TF = "dimos_tf_tf2_msgs.TFMessage"
NAV_CMD_VEL = "dimos_nav_cmd_vel_geometry_msgs.Twist"
STOP_MOVEMENT = "dimos_stop_movement_std_msgs.Bool"

FLIP_M = 0.5  # plan divergence (m, mean over the common arc) that counts as a mind change
KEY_OFF, KEY_SPAN = 8192, 16384  # voxel key packing: +-655 m at 0.08 m
SUPPORT_MIN = 4  # RayTracingVoxelMapConfig.support_min on the deployed blueprint
DAY = 86400.0


# ------------------------------------------------------------------- window --


@dataclass(frozen=True)
class Instant:
    """A `--from`/`--to` bound, before it knows what recording it bounds."""

    seconds: float
    absolute: bool  # a UTC time of day, rather than an offset from the start

    def resolve(self, t0: float) -> float:
        """The unix second this names, given the recording starts at `t0`."""
        if not self.absolute:
            return t0 + self.seconds
        midnight = math.floor(t0 / DAY) * DAY
        # a recording may cross midnight, so take the occurrence of this time of
        # day nearest the start rather than assuming the start's own date
        return min(
            (midnight + d + self.seconds for d in (-DAY, 0.0, DAY)), key=lambda t: abs(t - t0)
        )


def parse_instant(text: str) -> Instant:
    """`6.9` = seconds into the recording; `06:34:35.4` = that UTC time of day."""
    if ":" not in text:
        return Instant(float(text), absolute=False)
    parts = text.split(":")
    if len(parts) != 3:
        raise ValueError(f"{text!r} is neither seconds nor HH:MM:SS[.fff]")
    h, m, s = (float(p) for p in parts)
    return Instant(h * 3600.0 + m * 60.0 + s, absolute=True)


@dataclass(frozen=True)
class Window:
    """The slice of a recording every pass is held to (unix seconds)."""

    lo: float = -math.inf
    hi: float = math.inf

    @classmethod
    def between(cls, start: Instant | None, end: Instant | None, t0: float) -> Window:
        return cls(
            -math.inf if start is None else start.resolve(t0),
            math.inf if end is None else end.resolve(t0),
        )

    @property
    def bounded(self) -> bool:
        return math.isfinite(self.lo) or math.isfinite(self.hi)

    def __contains__(self, ts: float) -> bool:
        return self.lo <= ts <= self.hi

    def mask(self, stamps: np.ndarray) -> np.ndarray:
        return np.asarray((stamps >= self.lo) & (stamps <= self.hi))

    def label(self, t0: float) -> str:
        lo = "start" if self.lo == -math.inf else f"{self.lo - t0:.1f}"
        hi = "end" if self.hi == math.inf else f"{self.hi - t0:.1f}"
        return f"{lo}..{hi} s"


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


# ------------------------------------------------------------------- stamps --

# What a payload stamp turns out to mean, by the size of receipt minus stamp.
# go2web only started publishing sensor-referenced unix stamps in 7316c06; every
# recording before that carries the lidar's boot-relative clock, an epoch-sized
# offset from the recorder's. So the dialect is measured, never assumed.
FOREIGN_CLOCK_S = 3600.0  # past an hour it is another epoch, not a pipeline age
RECEIPT_ECHO_S = 0.005  # a stamp no older than its own receipt carries no age


@dataclass(frozen=True)
class Dialect:
    """What one stream's payload stamp means, and the age it implies per message."""

    verdict: str  # sensor-time | receipt-echo | foreign-clock | no-stamp
    ts: np.ndarray = field(repr=False, compare=False)  # receipts it was measured against
    age: np.ndarray = field(repr=False, compare=False)  # receipt - stamp, s (nan where no stamp)

    @property
    def sensor_time(self) -> bool:
        """True when receipt minus stamp is a real pipeline age worth reporting."""
        return self.verdict == "sensor-time"

    @property
    def delta(self) -> float:
        """Median receipt minus stamp, seconds (nan when nothing carries a stamp)."""
        good = self.age[np.isfinite(self.age)]
        return float(np.median(good)) if len(good) else float("nan")


def payload_ts(msg: Any) -> float:
    """The stamp a decoded message carries, or nan when its type has none."""
    ts = getattr(msg, "ts", None)
    return float("nan") if ts is None else float(ts)


def stamp_dialect(receipts: np.ndarray, stamps: np.ndarray) -> Dialect:
    """Sniff whether a stream's payload stamp is sensor time, a receipt echo, or foreign.

    Receipt is when the recorder logged the message; stamp is what the payload
    says. Their median difference is the whole test: an epoch away is another
    clock, not measurably positive is a publish time carrying no information,
    and a small positive age is the pipeline latency this tool could never see.
    """
    ts = np.asarray(receipts, dtype=float)
    age = ts - np.asarray(stamps, dtype=float)
    good = age[np.isfinite(age)]
    if not len(good):
        return Dialect("no-stamp", ts, age)
    median = float(np.median(good))
    if abs(median) > FOREIGN_CLOCK_S:
        return Dialect("foreign-clock", ts, age)
    if median < RECEIPT_ECHO_S:
        # includes the negative medians a host clock skew makes: a stamp from
        # after its own receipt is not an age at all
        return Dialect("receipt-echo", ts, age)
    return Dialect("sensor-time", ts, age)


def print_dialects(dialects: dict[str, Dialect]) -> None:
    """One line per stream: what its payload stamp turned out to mean."""
    print("stamp sniff: payload stamp against receipt, median receipt - stamp")
    for name, d in dialects.items():
        delta = "" if math.isnan(d.delta) else f"{d.delta:+.3f} s"
        print(f"{name:>14} {d.verdict:>14} {delta:>16}")


# ------------------------------------------------------------------ loading --


@dataclass
class Tick:
    """One published plan and the inputs the module held when it made it."""

    ts: float
    imap: int  # index into Recording.maps
    pose: tuple[float, ...]  # x, y, yaw, base z
    goal: tuple[float, float]
    recorded: np.ndarray  # the published plan, xy
    # Per-waypoint clearance decoded from the plan's own stamps -- the precision
    # the planner asked for, not anything recomputed offline.
    stamped_clear: np.ndarray | None = None


@dataclass
class Recording:
    """A motion-stack recording, decoded and pose-resolved the way the stack does.

    The streams stay WHOLE however `window` is set: a tick inside the window
    still planned on the map and the plan that arrived before it, so trimming
    the inputs would replay a world the robot never had. The window decides
    which ticks each pass reports on, nothing else.

    Every stream stamp here is the RECEIPT (mcap log time), because that is what
    the live module reacted to and replay must pair the way the module paired.
    The payload stamps ride alongside in `odom_stamp_ts` / `dialects` for the
    two questions arrival cannot answer: how old an input really was, and when
    a pose was actually true.
    """

    path: str
    maps: list[tuple[float, np.ndarray]]  # local_map: ts, points (n, 3)
    odom_ts: np.ndarray  # receipt, the clock the module paired on
    odom_stamp_ts: np.ndarray  # payload stamp, nan where the stream carries none
    odom_xy: np.ndarray  # sensor position, which is what the raycaster crops around
    poses: list[tuple[float, ...] | None]  # base_link (x, y, yaw, z)
    globals: list[tuple[float, np.ndarray]]  # planner_path: ts, xy
    plans: list[tuple[float, np.ndarray]]  # path: ts, (x, y, yaw)
    plan_msgs: list[NavPath] = field(default_factory=list)  # the same plans, undecoded
    twists: list[tuple[float, tuple[float, float, float]]] = field(default_factory=list)
    stops: np.ndarray = field(default_factory=lambda: np.zeros(0))  # stop_movement True stamps
    all_ticks: list[Tick] = field(default_factory=list)  # every tick with complete inputs
    ticks: list[Tick] = field(default_factory=list)  # the in-window ones passes report on
    window: Window = field(default_factory=Window)
    dialects: dict[str, Dialect] = field(default_factory=dict)  # per stream, sniffed

    @property
    def t0(self) -> float:
        return self.maps[0][0]

    @property
    def odom_physics_ts(self) -> np.ndarray:
        """When each pose was TRUE: sensor stamps where honest, receipts otherwise.

        For physics only. Anything reproducing what the module DID keeps using
        `odom_ts`, because the module paired on arrival.
        """
        d = self.dialects.get("odometry")
        return self.odom_stamp_ts if d is not None and d.sensor_time else self.odom_ts


def _xy(msg: Any) -> np.ndarray:
    return np.array([[p.position.x, p.position.y] for p in msg.poses]).reshape(-1, 2)


def _xyy(msg: Any) -> np.ndarray:
    """Plan poses as (x, y, yaw) -- the yaw is real, in-place turns carry it."""
    return np.array(
        [[p.position.x, p.position.y, p.orientation.euler[2]] for p in msg.poses]
    ).reshape(-1, 3)


def _stream(store: Store, name: str) -> list[Any]:
    """Decoded observations of one stream, empty when the recording has none."""
    if name not in store.list_streams():
        return []
    return list(store.stream(name))


def _before(ts: float, stamps: np.ndarray) -> int:
    """Index of the newest sample at or before ts (-1 when there is none)."""
    return int(np.searchsorted(stamps, ts, "right")) - 1


def load_recording(
    path: str,
    base_frame: str,
    lookahead: float,
    start: Instant | None = None,
    end: Instant | None = None,
) -> Recording:
    """Decode the streams and rebuild each tick's inputs (tf-resolved pose, carrot)."""
    from dimos.memory2.tf import StreamTF
    from dimos.utils.data import get_data

    store = open_lcm_mcap(str(get_data(path)))
    tf = StreamTF.from_store(store, TF)
    if tf is None:
        raise SystemExit(f"{path}: no {TF} stream — cannot resolve the base pose")
    base = OdomBasePose(tf, base_frame)
    # Each stream is read as (receipt, payload stamp, payload) in ONE pass, so
    # the dialect sniff costs no extra decode and holds no extra cloud alive.
    map_rows = [(o.ts, payload_ts(o.data), o.data.points_f32()) for o in _stream(store, LOCAL_MAP)]
    maps = [(t, p) for t, _, p in map_rows]
    odom = [(o.ts, payload_ts(o.data), o.data) for o in _stream(store, ODOMETRY)]
    poses: list[tuple[float, ...] | None] = []
    for _, _, msg in odom:
        p = base.resolve(msg)
        poses.append(
            (p.position.x, p.position.y, p.orientation.euler[2], p.position.z) if p else None
        )
    global_rows = [(o.ts, payload_ts(o.data), _xy(o.data)) for o in _stream(store, PLANNER_PATH)]
    globals_ = [(t, xy) for t, _, xy in global_rows]
    raw_plans = _stream(store, PATH)
    plans = [(o.ts, _xyy(o.data)) for o in raw_plans]
    stamped: list[np.ndarray | None] = []
    for o in raw_plans:
        ceilings = decode_ceilings(o.data)
        stamped.append(ceilings_to_clearance(ceilings) if ceilings is not None else None)

    twist_rows = [
        (o.ts, payload_ts(o.data), (o.data.linear.x, o.data.linear.y, o.data.angular.z))
        for o in _stream(store, NAV_CMD_VEL)
    ]
    twists = [(t, v) for t, _, v in twist_rows]

    sniffed: dict[str, list[Any]] = {
        "local_map": map_rows,
        "odometry": odom,
        "planner_path": global_rows,
        "path": [(o.ts, payload_ts(o.data)) for o in raw_plans],
        "nav_cmd_vel": twist_rows,
    }
    dialects = {
        name: stamp_dialect(np.array([r[0] for r in rows]), np.array([r[1] for r in rows]))
        for name, rows in sniffed.items()
        if rows
    }

    rec = Recording(
        path=path,
        maps=maps,
        odom_ts=np.array([t for t, _, _ in odom]),
        odom_stamp_ts=np.array([s for _, s, _ in odom]),
        odom_xy=np.array([[m.pose.position.x, m.pose.position.y] for _, _, m in odom]),
        poses=poses,
        globals=globals_,
        plans=plans,
        plan_msgs=[o.data for o in raw_plans],
        twists=twists,
        stops=np.array([o.ts for o in _stream(store, STOP_MOVEMENT) if o.data.data]),
        dialects=dialects,
    )
    rec.window = Window.between(start, end, rec.t0)
    map_ts = np.array([t for t, _ in maps])
    global_ts = np.array([t for t, _ in globals_])
    for n, (ts, xy) in enumerate(plans):
        i, j, k = _before(ts, map_ts), _before(ts, rec.odom_ts), _before(ts, global_ts)
        if min(i, j, k) < 0 or rec.poses[j] is None or not len(globals_[k][1]):
            continue
        pose = rec.poses[j]
        assert pose is not None
        goal = carrot_along(globals_[k][1], (pose[0], pose[1]), lookahead)
        rec.all_ticks.append(
            Tick(
                ts=ts,
                imap=i,
                pose=pose,
                goal=goal,
                recorded=xy,
                stamped_clear=stamped[n],
            )
        )
    rec.ticks = [t for t in rec.all_ticks if t.ts in rec.window]
    return rec


def gated_ticks(ticks: list[Tick], carrot_m: float = REPLAN_CARROT_M) -> list[Tick]:
    """The ticks a replan gate would keep: a new local map, or a moved carrot.

    The map index stands in for the module's cloud counter, and the tick's own
    carrot for the one the kept plan was made for -- :func:`planner.replan_due`
    decides, so the replay cannot drift from the module.
    """
    kept: list[Tick] = []
    for tick in ticks:
        planned = (kept[-1].imap, kept[-1].goal) if kept else None
        if replan_due(planned, tick.imap, tick.goal, carrot_m):
            kept.append(tick)
    return kept


# ------------------------------------------------------------------ metrics --


def arclen(xy: np.ndarray) -> float:
    xy = xy[:, :2] if xy.ndim == 2 else xy
    return float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum()) if len(xy) > 1 else 0.0


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


def churn(
    rec: Recording, voxel: float, band_model: ObstacleModel, rr: Any = None
) -> list[dict[str, float]]:
    """Per-frame voxel appear/disappear, split boundary vs interior, whole map vs band.

    `band_model` reads the map in the frame it was recorded in -- this is a
    statement about what the mapper emitted, not about any body, so it is
    `raw_band`.
    """
    frames = []
    for ts, pts in rec.maps:
        if ts not in rec.window:
            continue
        slab = pts[band_model.field(pts).hard]
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
        f"map band z {RAW_BAND[0]}..{RAW_BAND[1]}: {band:.0f} voxels | appear "
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
    # Same-map re-rolls: the planner changed its mind, not its information --
    # the field twin of the battery's `rerolls` column (referee judge.REROLL_M).
    n_roll = int((d[same] > REROLL_M).sum())
    print(
        f"rerolls > {REROLL_M} m with an unchanged map: {n_roll} of {same.sum()} pairs"
        + (f"  ({100.0 * n_roll / max(1, same.sum()):.0f}%)" if same.sum() else "")
    )
    return rows


# ----------------------------------------------------------------- tracking --

BUMP_WIN_S = 0.7  # gait-bounce timescale; a rolling median this wide irons it out


def tracking(rec: Recording) -> list[dict[str, float]]:
    """Cross-track error: the odometry track against the plan it was following.

    Raw is the distance to the active plan's polyline at every odometry sample;
    SUSTAINED is a rolling median over ``BUMP_WIN_S``, so a gait bounce or a
    one-step stumble reads as a bump while a real offset survives the filter.
    Holds have no active plan and are skipped.
    """
    rows: list[dict[str, float]] = []
    plan_ts = np.array([t for t, _ in rec.plans])
    for ts, pose in zip(rec.odom_ts, rec.poses, strict=True):
        if pose is None or ts not in rec.window:
            continue
        k = _before(ts, plan_ts)
        if k < 0:
            continue
        xy = rec.plans[k][1]
        if is_hold(xy):
            continue
        p = np.array(pose[:2])
        a, b = xy[:-1, :2], xy[1:, :2]
        ab = b - a
        tt = np.clip(
            np.einsum("ij,ij->i", p - a, ab) / (np.einsum("ij,ij->i", ab, ab) + 1e-12), 0, 1
        )
        err = float(np.min(np.linalg.norm(p - (a + tt[:, None] * ab), axis=1)))
        rows.append({"ts": ts, "err": err})
    if not rows:
        print("\n=== tracking === no odometry with an active plan in the window")
        return rows
    ts_a = np.array([r["ts"] for r in rows])
    err_a = np.array([r["err"] for r in rows])
    dt = float(np.median(np.diff(ts_a))) if len(ts_a) > 1 else 0.033
    half = max(1, round(BUMP_WIN_S / max(dt, 1e-3) / 2))
    sustained = np.array(
        [np.median(err_a[max(0, i - half) : i + half + 1]) for i in range(len(err_a))]
    )
    for r, s in zip(rows, sustained, strict=True):
        r["sustained"] = float(s)
    bump = err_a - sustained
    worst = np.argsort(sustained)[-3:][::-1]
    print(f"\n=== tracking ({len(rows)} odom samples against the active plan) ===")
    print(
        f"cross-track raw: median {np.median(err_a):.3f} m  p90 "
        f"{np.percentile(err_a, 90):.3f}  p95 {np.percentile(err_a, 95):.3f}  "
        f"max {err_a.max():.3f}\n"
        f"sustained ({BUMP_WIN_S:.1f} s median): median {np.median(sustained):.3f} m  "
        f"p95 {np.percentile(sustained, 95):.3f}  max {sustained.max():.3f}\n"
        f"bumps over the sustained floor: p95 {np.percentile(bump, 95):.3f} m  "
        f"max {bump.max():.3f}"
    )
    print(
        "worst sustained: "
        + "  ".join(f"{sustained[i]:.3f} m at t={ts_a[i] - rec.t0:.1f}" for i in worst)
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


class Replanner:
    """One planner episode, re-solving a recorded tick from its own inputs."""

    def __init__(self, rec: Recording, planner: str, embodiment: str, z_offset: float) -> None:
        self.rec = rec
        self.planner = planner
        self.embodiment = embodiment
        self.z_offset = z_offset
        self.emb = EMBODIMENTS[embodiment]
        self.ep = episode(planner, embodiment)
        self.offset = np.array([0.0, 0.0, z_offset], dtype=np.float32)
        # what the last call handed the episode, so the renderer can mark the
        # very cloud the search saw instead of re-deriving one
        self.pts: np.ndarray | None = None
        self.shift = 0.0

    def ground_of(self, pose: tuple[float, ...]) -> float:
        """Where the surface under the robot is, off the body: base z - base_height."""
        return (pose[3] if len(pose) > 3 else 0.0) - self.emb.base_height

    def __call__(
        self, imap: int, pose: tuple[float, ...], goal: tuple[float, float], band: ObstacleModel
    ) -> np.ndarray:
        # The module's own order (adapter/planner.py::_plan_once): the trim
        # corrects the map's z origin, then the model reads the result.
        ground_z = self.ground_of(pose)
        pts = hard_points(band, self.rec.maps[imap][1] + self.offset, ground_z)
        self.pts = pts
        self.shift = ground_z if band.body_referenced else 0.0
        planned = self.ep.plan(pts[:, :2], (pose[0], pose[1], pose[2]), goal)
        return np.array(
            [[q.position.x, q.position.y, q.orientation.euler.yaw] for q in planned.poses]
        ).reshape(-1, 3)

    def sniff(self, ticks: list[Tick]) -> str:
        """Which obstacle model did the DEPLOYED stack run?

        A recording predates whatever is current, so sniff it rather than
        assume: replay a tick subsample under each model and keep the one whose
        holds agree with the recorded plans.
        """
        sample = ticks[:: max(1, len(ticks) // 40)]
        agree = {}
        for name in OBSTACLE_MODELS:
            candidate = load_model(name, self.emb)
            agree[name] = sum(
                is_hold(self(t.imap, t.pose, t.goal, candidate)) == is_hold(t.recorded)
                for t in sample
            )
        model = max(agree, key=lambda k: (agree[k], k == "body_band"))
        print(
            "config sniff: holds agree "
            + ", ".join(f"{n} {a}/{len(sample)}" for n, a in sorted(agree.items()))
            + f" -> {model}"
        )
        return model


def replay(
    rec: Recording,
    plan: Replanner,
    model: str,
    ablate: bool,
    gate: bool = False,
    rr: Any = None,
) -> list[dict[str, float]]:
    """Re-plan every recorded tick from its own inputs; optionally ablate one input at a time."""
    emb = plan.emb
    ticks = gated_ticks(rec.ticks) if gate else rec.ticks
    band = load_model(model, emb)

    t = ticks[len(ticks) // 2]
    twice = plan(t.imap, t.pose, t.goal, band), plan(t.imap, t.pose, t.goal, band)
    deterministic = twice[0].shape == twice[1].shape and bool(np.allclose(*twice))

    rows: list[dict[str, float]] = []
    started = time.perf_counter()
    previous: np.ndarray | None = None
    for tick in ticks:
        out = plan(tick.imap, tick.pose, tick.goal, band)
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
            # the winning model's hard set -- the very cloud plan() handed the
            # search, holds included -- shifted back onto the map. STRAYS (no
            # adjacent hard voxel in the column plane) drawn red and fat: a
            # lone voxel in a doorway throat closes it as surely as a wall.
            obs = plan.pts
            assert obs is not None
            from scipy.spatial import cKDTree

            xy = obs[:, :2]
            # isolation by COLUMN, not by point: a chair leg (or a ghost of
            # one) is one xy cell stacked with returns, and it must read stray
            cols, inv = np.unique(np.round(xy / 0.12).astype(np.int64), axis=0, return_inverse=True)
            near = cKDTree(cols * 0.12).query_ball_point(cols * 0.12, r=0.18, return_length=True)
            stray = (np.asarray(near) <= 2)[inv]  # itself + at most one column
            z = obs[:, 2] + plan.shift
            rr.log(
                "world/obstacles",
                rr.Points3D(
                    np.column_stack([xy[~stray], z[~stray]]),
                    radii=0.022,
                    colors=[[255, 120, 0]],
                ),
            )
            rr.log(
                "world/obstacles/strays",
                rr.Points3D(
                    np.column_stack([xy[stray], z[stray]]),
                    radii=0.035,
                    colors=[[255, 40, 40]],
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
    cadence = "gated on input arrival" if gate else "every recorded tick"
    print(
        f"\n=== replay ({plan.planner}, {model}, {cadence}, "
        f"z_offset {plan.z_offset:+.2f} m, {len(rows)} ticks) ==="
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
        _ablate(ticks, plan, band)
    return rows


def _ablate(ticks: list[Tick], plan: Replanner, band: ObstacleModel) -> None:
    """Re-plan each tick pair changing ONE input, to attribute the change."""
    rows = []
    for a, b in pairwise(ticks):
        # the ground reference travels with the pose: it is read off the same
        # odometry message the pose is
        base = plan(a.imap, a.pose, a.goal, band)
        rows.append(
            {
                "ts": b.ts,
                "all": divergence(base, plan(b.imap, b.pose, b.goal, band)),
                "cloud": divergence(base, plan(b.imap, a.pose, a.goal, band)),
                "pose": divergence(base, plan(a.imap, b.pose, a.goal, band)),
                "goal": divergence(base, plan(a.imap, a.pose, b.goal, band)),
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


def pipeline_age(rec: Recording) -> None:
    """True age of each stream at receipt: receipt minus the payload's own stamp.

    Only a recording whose stamps speak sensor time can answer this; the rest
    say so on their own line rather than reporting a number nobody measured.
    """
    if not rec.dialects:
        return
    print("\ntrue pipeline age (receipt - payload stamp)")
    print(f"{'stream':>14} {'dialect':>14} {'median':>10} {'p95':>10} {'max':>10}")
    for name, d in rec.dialects.items():
        age = d.age[rec.window.mask(d.ts)]
        age = age[np.isfinite(age)]
        if not d.sensor_time or not len(age):
            print(f"{name:>14} {d.verdict:>14} {'-':>10} {'-':>10} {'-':>10}")
            continue
        print(
            f"{name:>14} {d.verdict:>14} {1e3 * np.median(age):8.1f}ms "
            f"{1e3 * np.percentile(age, 95):8.1f}ms {1e3 * age.max():8.1f}ms"
        )
    if not any(d.sensor_time for d in rec.dialects.values()):
        print(
            "  no stream here carries a stamp on the recorder's clock, so the true age is "
            "unmeasurable — only the cadence above is real"
        )


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
    for name, all_stamps in (
        ("local_map", map_ts),
        ("odometry", rec.odom_ts),
        ("planner_path", global_ts),
        ("path", np.array([t for t, _ in rec.plans])),
        ("nav_cmd_vel", np.array([t for t, _ in rec.twists])),
    ):
        stamps = all_stamps[rec.window.mask(all_stamps)]
        if len(stamps) < 2:
            continue
        dt = np.diff(stamps) * 1e3
        print(
            f"{name:>14} {len(stamps):6d} {np.median(dt):9.1f}ms {np.percentile(dt, 95):8.1f}ms "
            f"{dt.max():8.1f}ms"
        )
    pipeline_age(rec)
    print(f"\n{'input at a plan tick':>22} {'mean':>8} {'p95':>8} {'max':>8}")
    ages = (("map_age", "local_map"), ("odom_age", "odometry"), ("global_age", "planner_path"))
    for key, label in ages:
        v = np.array([r[key] for r in rows]) * 1e3
        print(f"{label:>22} {v.mean():7.0f}ms {np.percentile(v, 95):7.0f}ms {v.max():7.0f}ms")
    return rows


# ----------------------------------------------------------------- follower --


@dataclass(frozen=True)
class FollowerSetup:
    """The deployed `trajectory_follower` config, however it was obtained."""

    track: str
    controller: ControllerConfig
    control_frequency: float
    goal_tolerance: float
    embodiment: str
    max_path_age_s: float
    obstacle_model: str | None  # None = fall back to the planner sniff's winner
    source: str

    @property
    def period(self) -> float:
        return 1.0 / self.control_frequency


def blueprint_setup() -> FollowerSetup:
    """The `go2-zenoh-motion` follower, read off the blueprint rather than copied.

    `max_path_age_s` has no python twin — it is the baked host's deadman — so
    it comes off :class:`TrajectoryFollowerNativeConfig`, which is the module
    that actually runs on the robot.
    """
    from dimos.navigation.motion.adapter.follower import (
        TrajectoryFollower,
        TrajectoryFollowerConfig,
    )
    from dimos.navigation.motion.adapter.follower_native import TrajectoryFollowerNativeConfig
    from dimos.robot.unitree.go2.zenoh.blueprints import go2_zenoh_motion

    kwargs: dict[str, Any] = {}
    for atom in go2_zenoh_motion.blueprints:
        if atom.module is TrajectoryFollower:
            kwargs = dict(atom.kwargs)
    cfg = TrajectoryFollowerConfig(**kwargs)
    return FollowerSetup(
        track=cfg.track,
        controller=cfg.controller_config,
        control_frequency=cfg.control_frequency,
        goal_tolerance=cfg.goal_tolerance,
        embodiment=cfg.embodiment,
        max_path_age_s=TrajectoryFollowerNativeConfig.model_fields["max_path_age_s"].default,
        obstacle_model=cfg.obstacle_model,
        source="go2-zenoh-motion blueprint",
    )


def host_setup(path: str) -> FollowerSetup:
    """`modules.trajectory_follower.config` off a motion-host stdin blob."""
    blob = json.loads(FsPath(path).read_text())
    try:
        cfg = blob["modules"]["trajectory_follower"]["config"]
    except (KeyError, TypeError) as e:
        raise SystemExit(f"{path}: no modules.trajectory_follower.config") from e
    fallback = FollowerSetup(
        track="hinted",
        controller=ControllerConfig(),
        control_frequency=10.0,
        goal_tolerance=0.20,
        embodiment="go2",
        max_path_age_s=2.5,
        obstacle_model="body_band",
        source=path,
    )
    return FollowerSetup(
        track=cfg.get("track", fallback.track),
        controller=ControllerConfig(**cfg.get("controller_config", {})),
        control_frequency=cfg.get("control_frequency", fallback.control_frequency),
        goal_tolerance=cfg.get("goal_tolerance", fallback.goal_tolerance),
        embodiment=cfg.get("embodiment", fallback.embodiment),
        max_path_age_s=cfg.get("max_path_age_s", fallback.max_path_age_s),
        obstacle_model=cfg.get("obstacle_model", fallback.obstacle_model),
        source=path,
    )


def bind_law(track: str, cfg: ControllerConfig) -> tuple[str, TrajectoryController]:
    """The deployed rust law, or its python twin when the extension is missing."""
    name = TRACKS[track].controller
    try:
        law = load_law(f"{name}-rs")(cfg)
        return f"{name}-rs", law
    except ImportError as e:
        print(
            f"  {e}\n"
            "  falling back to the python law. The wheel goes stale SILENTLY: rebuild it "
            "after any control/rust change, or a parity bug replays as a match."
        )
        return name, load_law(name)(cfg)


ZERO: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class Command:
    """One recorded nav_cmd_vel tick, next to what the law would have said."""

    ts: float
    recorded: tuple[float, float, float]
    law: tuple[float, float, float]  # zero where the module never ran its law
    verdict: str
    reason: str  # why it was a hold: stale, latched, stub
    gap: float
    age: float  # how old the plan this tick tracked was

    @property
    def hold(self) -> bool:
        return self.verdict == "hold"


def classify(
    recorded: tuple[float, float, float],
    law: tuple[float, float, float] | None,
    boundary: bool,
    threshold: float,
) -> tuple[str, float]:
    """One tick's verdict and the worst per-component gap.

    `law` is None where the module never reached its law — the deadman, the
    goal latch, or a single-pose stub — and the recorded twist is then held
    against zero instead. `boundary` marks a tick a plan landed on within one
    control period, where which plan the module held is genuinely ambiguous.
    """
    against = ZERO if law is None else law
    gap = max(abs(a - b) for a, b in zip(recorded, against, strict=True))
    if law is None:
        return ("hold" if gap < threshold else "MISMATCH"), gap
    if boundary:
        return "boundary", gap
    return ("match" if gap < threshold else "MISMATCH"), gap


def _pose_at(pose: tuple[float, ...], ts: float, frame_id: str) -> PoseStamped:
    """The tf-resolved base pose as the law's own input type."""
    return PoseStamped(
        ts=ts,
        frame_id=frame_id,
        position=Vector3(pose[0], pose[1], pose[3] if len(pose) > 3 else 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, pose[2])),
    )


def follower(
    rec: Recording,
    setup: FollowerSetup,
    model: str,
    threshold: float,
    world_frame: str = "odom",
) -> list[Command]:
    """Re-run the deployed law at every recorded nav_cmd_vel tick and classify it.

    The law rate-limits its own command, so it is the SAME instance across the
    whole recording, in order — a window's first tick inherits the state the
    robot's did. Only in-window ticks are reported.
    """
    if not rec.twists:
        print("\n=== follower ===\nno nav_cmd_vel stream in this recording")
        return []
    emb = EMBODIMENTS[setup.embodiment]
    band = load_model(model, emb)
    half_width = emb.width / 2.0
    track = TRACKS[setup.track]
    binding, law = bind_law(setup.track, setup.controller)
    law.reset()
    latch = GoalLatch(setup.goal_tolerance)

    plan_ts = np.array([t for t, _ in rec.plans])
    map_ts = np.array([t for t, _ in rec.maps])
    room: np.ndarray | None = None
    room_key: tuple[int, int] | None = None
    fed = -1  # the newest plan whose goal the latch has been shown

    def clearance_for(ip: int, imap: int, path: NavPath, pose: tuple[float, ...]) -> Any:
        """`follower.py::_clearance_for`, off the recorded map instead of a live one."""
        nonlocal room, room_key
        if not track.annotate_clearance:
            return None  # the blind track: the law reads the path's own stamps
        if imap < 0:
            ceilings = decode_ceilings(path)
            return ceilings_to_clearance(ceilings) if ceilings is not None else None
        if (ip, imap) != room_key:
            wp = np.array([[p.position.x, p.position.y] for p in path.poses]).reshape(-1, 2)
            ground_z = (pose[3] if len(pose) > 3 else 0.0) - emb.base_height
            pts = hard_points(band, rec.maps[imap][1], ground_z)
            room = path_clearance(wp, pts, half_width)
            room_key = (ip, imap)
        return room

    rows: list[Command] = []
    warmed = 0
    preempted = False
    for ts, recorded in rec.twists:
        ip, j = _before(ts, plan_ts), _before(ts, rec.odom_ts)
        if ip < 0 or j < 0 or rec.poses[j] is None:
            continue  # the module was Idle here: no pose, or no plan yet
        pose = rec.poses[j]
        assert pose is not None
        path = rec.plan_msgs[ip]
        # every plan that arrived since the last tick, in order, so the latch
        # sees the same set_goal SEQUENCE the module's subscription saw
        for k in range(fed + 1, ip + 1):
            poses = rec.plan_msgs[k].poses
            if len(poses) >= 2:
                latch.set_goal((poses[-1].position.x, poses[-1].position.y))
        fed = ip
        age = ts - plan_ts[ip]

        # the deployed branch (adapter/rust/src/follower.rs::decide), in its own
        # order: the deadman outranks arrival, because a goal reached against a
        # plan nobody is refreshing is a coincidence
        out: tuple[float, float, float] | None
        reason = ""
        istop = _before(ts, rec.stops)
        if istop >= 0 and rec.stops[istop] > plan_ts[ip]:
            # stop_movement nulled the held path and reset the law; the module
            # is Idle until the planner publishes again. Resetting once per
            # stretch is the same law state as resetting per message, since
            # nothing steps it in between.
            if not preempted:
                law.reset()
                preempted = True
            out, reason = None, "stopped"
        elif age > setup.max_path_age_s:
            out, reason = None, "stale"
        elif latch.arrive((pose[0], pose[1])) or latch.reached:
            out, reason = None, "latched"
        else:
            preempted = False
            tw = law.update(
                _pose_at(pose, ts, world_frame),
                path,
                ts,
                clearance_for(ip, _before(ts, map_ts), path, pose),
            )
            out = (tw.linear.x, tw.linear.y, tw.angular.z)
            if len(path.poses) < 2:
                # the law obeys the planner's refusal with a zero of its own;
                # naming it a hold keeps the stats about TRACKING
                out, reason = None, "stub"
        if ts not in rec.window:
            warmed += 1
            continue
        verdict, gap = classify(recorded, out, boundary=age < setup.period, threshold=threshold)
        rows.append(
            Command(
                ts=ts,
                recorded=recorded,
                law=out or ZERO,
                verdict=verdict,
                reason=reason,
                gap=gap,
                age=age,
            )
        )

    counts = {c: sum(1 for r in rows if r.verdict == c) for c in ("match", "boundary", "hold")}
    reasons = {
        w: sum(1 for r in rows if r.hold and r.reason == w)
        for w in ("stale", "latched", "stopped", "stub")
    }
    bad = [r for r in rows if r.verdict == "MISMATCH"]
    comparable = np.array([r.gap for r in rows if r.verdict in ("match", "MISMATCH")])
    print(
        f"\n=== follower ({setup.track}/{binding}, {model} clearance, "
        f"{len(rows)} ticks over {rec.window.label(rec.t0)}) ==="
    )
    print(
        f"config: {setup.source} (max_path_age {setup.max_path_age_s} s, "
        f"{setup.control_frequency:g} Hz, max_speed {setup.controller.max_speed})\n"
        f"law state warmed over {warmed} pre-window ticks\n"
        f"match {counts['match']}  boundary {counts['boundary']}  "
        f"hold {counts['hold']} (" + ", ".join(f"{w} {n}" for w, n in reasons.items()) + ")  "
        f"MISMATCH {len(bad)}"
    )
    if len(comparable):
        print(
            f"comparable ticks ({len(comparable)}): median {np.median(comparable):.3f}  "
            f"p95 {np.percentile(comparable, 95):.3f}  max {comparable.max():.3f} "
            f"(worst component, m/s or rad/s)"
        )
    # The cheapest proof that the config is not the one the robot ran, and the
    # one failure this pass would otherwise report as a wall of MISMATCH: a
    # twist the law's own envelope cannot produce did not come from this config.
    fastest = max((math.hypot(*r.recorded[:2]) for r in rows), default=0.0)
    if fastest > setup.controller.max_speed + 1e-6:
        print(
            f"the recorded twist reaches {fastest:.2f} m/s, over this config's max_speed "
            f"{setup.controller.max_speed} -- {setup.source} is NOT what the robot ran"
        )
    for r in bad:
        print(
            f"  MISMATCH t={r.ts - rec.t0:6.2f}  recorded "
            f"({r.recorded[0]:+.3f} {r.recorded[1]:+.3f} {r.recorded[2]:+.3f})  law "
            f"({r.law[0]:+.3f} {r.law[1]:+.3f} {r.law[2]:+.3f})  gap {r.gap:.3f}"
        )
    return rows


# -------------------------------------------------------------------- plots --


def write_plots(
    churn_rows: list[dict[str, float]],
    plan_rows: list[dict[str, float]],
    tracking_rows: list[dict[str, float]],
    latency_rows: list[dict[str, float]],
    follower_rows: list[Command],
    out: FsPath,
) -> None:
    """Churn / flip / age / twist time series as SVG (dimos.memory2.vis.plot)."""
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
    if tracking_rows:
        p = Plot()
        p.add(
            Series(
                ts=[r["ts"] for r in tracking_rows],
                values=[r["err"] for r in tracking_rows],
                label="cross-track (m)",
                color=color.blue.hex(),
                opacity=0.5,
            )
        )
        p.add(
            Series(
                ts=[r["ts"] for r in tracking_rows],
                values=[r["sustained"] for r in tracking_rows],
                label="sustained (m)",
                color=color.green.hex(),
            )
        )
        p.to_svg(str(out / "tracking.svg"))
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
    if follower_rows:
        p = Plot()
        ts = [cmd.ts for cmd in follower_rows]
        for k, label, c in ((0, "vx", color.blue), (1, "vy", color.green), (2, "wz", color.orange)):
            p.add(
                Series(
                    ts=ts,
                    values=[cmd.recorded[k] for cmd in follower_rows],
                    label=label,
                    color=c.hex(),
                )
            )
            p.add(
                Series(
                    ts=ts,
                    values=[cmd.law[k] for cmd in follower_rows],
                    label=f"{label} (law)",
                    color=c.hex(),
                    opacity=0.45,
                )
            )
        for cmd in follower_rows:
            if cmd.hold:
                p.add(VLine(x=cmd.ts, color=color.purple.hex(), opacity=0.2))
            elif cmd.verdict == "MISMATCH":
                p.add(VLine(x=cmd.ts, color=color.red.hex(), opacity=0.6))
        p.to_svg(str(out / "follower.svg"))
    print("\nplots: " + " ".join(str(f) for f in sorted(out.glob("*.svg"))))


# ---------------------------------------------------------------------- cli --


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("recording", help="path to the .mcap recording")
    ap.add_argument(
        "--only", default="churn,plans,tracking,replay,latency,follower", help="passes to run"
    )
    ap.add_argument(
        "--from",
        dest="start",
        default=None,
        help="window start: seconds into the recording, or HH:MM:SS[.fff] UTC",
    )
    ap.add_argument("--to", dest="end", default=None, help="window end, same two forms")
    ap.add_argument("--voxel", type=float, default=0.08, help="raycaster voxel size")
    ap.add_argument("--planner", default="target")
    ap.add_argument("--embodiment", default="go2")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--lookahead", type=float, default=5.0, help="carrot arc along planner_path")
    ap.add_argument(
        "--z-offset",
        type=float,
        default=0.0,
        help="trim the recorded map's z origin (counterfactual)",
    )
    ap.add_argument(
        "--model",
        default="auto",
        choices=["auto", *OBSTACLE_MODELS],
        help="obstacle model to replay under; auto sniffs which one the robot ran",
    )
    ap.add_argument(
        "--no-anchor",
        action="store_true",
        help=argparse.SUPPRESS,  # deprecated alias for --model raw_band
    )
    ap.add_argument(
        "--gate", action="store_true", help="replay only the ticks a replan gate would keep"
    )
    ap.add_argument(
        "--host-config",
        default=None,
        help="motion-host.json the follower ran; without it the blueprint's values stand in",
    )
    ap.add_argument(
        "--threshold",
        type=float,
        default=0.15,
        help="per-component twist gap (m/s, rad/s) under which a follower tick matches",
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
    named = "raw_band" if args.no_anchor else (None if args.model == "auto" else args.model)
    start = parse_instant(args.start) if args.start else None
    end = parse_instant(args.end) if args.end else None
    rec = load_recording(args.recording, args.base_frame, args.lookahead, start, end)
    setup = host_setup(args.host_config) if args.host_config else blueprint_setup()
    stem = FsPath(args.out) / f"{FsPath(args.recording).stem}-diagnose"
    stem.parent.mkdir(parents=True, exist_ok=True)
    print(
        f"{args.recording}: {len(rec.maps)} local_map, {len(rec.plans)} plans, "
        f"{len(rec.twists)} nav_cmd_vel, {len(rec.ticks)} ticks with complete inputs, "
        f"{rec.plans[-1][0] - rec.t0:.1f} s"
        + (f" | window {rec.window.label(rec.t0)}" if rec.window.bounded else "")
    )
    print_dialects(rec.dialects)

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
    # the body's own timeline, so a sensor-time recording draws each pose where
    # the lidar says it was rather than where the link delivered it
    for ots, pose in zip(rec.odom_physics_ts, rec.poses, strict=True):
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
    # the MLS global route the carrot rides on, at each republish; an empty
    # route is MLS saying "no way through", so nothing should stay on screen
    for gts, gxy in rec.globals:
        rr.set_time("time", timestamp=float(gts))
        if len(gxy) > 1:
            rr.log(
                "world/global",
                rr.LineStrips3D(
                    [np.column_stack([gxy, np.full(len(gxy), 0.04)])],
                    colors=[[200, 100, 255]],
                    radii=0.010,
                ),
            )
        else:
            rr.log("world/global", rr.Clear(recursive=True))

    # One planner episode serves both the model sniff and the replay, and is
    # built only if something asks for it -- `--only follower` should not pay
    # for a planner it never runs.
    replanner: Replanner | None = None
    sniffed: str | None = named

    def obstacle_model() -> str | None:
        """The model the recording was made under; None when nothing can tell."""
        nonlocal replanner, sniffed
        if sniffed is None and rec.all_ticks:
            if replanner is None:
                replanner = Replanner(rec, args.planner, args.embodiment, args.z_offset)
            # over the WHOLE recording: which model ran is a fact about the
            # robot, not about whatever window is being looked at
            sniffed = replanner.sniff(rec.all_ticks)
        return sniffed

    churn_rows = (
        churn(rec, args.voxel, load_model("raw_band", emb), rr) if "churn" in passes else []
    )
    plan_rows = plans(rec) if "plans" in passes else []
    tracking_rows = tracking(rec) if "tracking" in passes else []
    if "replay" in passes:
        model = obstacle_model() or "body_band"
        if replanner is None:
            replanner = Replanner(rec, args.planner, args.embodiment, args.z_offset)
        replay(rec, replanner, model, not args.no_ablate, gate=args.gate, rr=rr)
    latency_rows = latency(rec) if "latency" in passes else []
    follower_rows: list[Command] = []
    if "follower" in passes:
        # --model wins outright; otherwise the sniff, because it measures what
        # the robot RAN while a config only says what it was asked to run
        room_model = named or obstacle_model() or setup.obstacle_model or "body_band"
        if setup.obstacle_model and room_model != setup.obstacle_model:
            print(
                f"\nnote: {setup.source} says obstacle_model={setup.obstacle_model}, the sniff "
                f"says {room_model} -- measuring the room hint off {room_model} (--model overrides)"
            )
        follower_rows = follower(rec, setup, room_model, args.threshold)
    write_plots(
        churn_rows,
        plan_rows,
        tracking_rows,
        latency_rows,
        follower_rows,
        FsPath(args.plots or str(stem)),
    )
    if not args.spawn:
        print(f"rerun: {rrd}")


if __name__ == "__main__":
    main()
