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

"""A real recording's map, frozen into a static world the sim and judge can run.

The raycaster's ``local_map`` is an accumulated world-frame cloud that
flickers: roughly a tenth of its voxels appear and disappear frame to frame.
So the static world is not the union of every frame but a STABILITY-FILTERED
union — a voxel survives only if it was seen in at least ``stability`` of the
frames spanning its own first-to-last sighting.

The artifact is a small npz. It carries the voxels (MuJoCo collision + the
perception crop), the 2D band rectangles the planner sees as
:class:`~...scenarios.Box` obstacles, the estimated floor, the recorded start
pose, the recorded global path and its goal.

  python -m dimos.navigation.motion.simulation.recorded_world rec.mcap --out w.npz
  python -m dimos.navigation.motion.simulation.recorded_world w.npz --view
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path as FilePath
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.navigation.motion import obstacles
from dimos.navigation.motion.embodiment import GO2

if TYPE_CHECKING:
    import mujoco

    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

VOXEL = 0.08
# Fraction of a voxel's own visibility span it must be seen in to be real.
STABILITY = 0.5
MIN_FRAMES = 2  # ...and never fewer than this many sightings, span or no span

# Upper edge of the planner band over the estimated floor. Both the planner
# rectangles and the scenery slice `obstacles.LOW` up, like the deployed
# body_band model: the floor's own returns quantise into the first two voxel
# layers, and keeping them turns the floor the recorded robot demonstrably
# walked on into a carpet of phantom 0.08 m obstacles.
BAND_TOP = 0.45
BAND_HEIGHT = 0.6

SCENE_MAX_Z = 1.5  # collision boxes above the floor; the ceiling is not a wall
# Moving-body envelope (the GO2 union) plus its standing height: what the robot
# sweeps, and therefore what its own lidar returns must be carved out of. Read
# off the embodiment rather than restated: one re-baseline, one number.
BODY = (GO2.length, GO2.width, GO2.height)
# Clearance the swept footprint is granted on top of that envelope. The robot's
# own returns bleed a voxel or so past its skin, so a carve of exactly the body
# leaves the spawn pose touching a wall and every oracle refuses at step one.
CARVE = 0.10
LOCAL_MAP_RANGE = 5.0  # raycaster local_map radius (measured on the recording)


@dataclass
class RecordedWorld:
    """A recording's static map: voxels, planner band, floor, start, goal."""

    name: str
    voxel: float
    voxels: np.ndarray  # (n, 3) int32 grid indices
    floor_z: float
    start: np.ndarray  # (3,) x, y, yaw of the base at the first mapped frame
    goal: np.ndarray  # (2,) end of the recorded global path
    path: np.ndarray  # (m, 2) recorded global path
    track: np.ndarray  # (k, 3) x, y, yaw the base actually walked
    rects: np.ndarray  # (r, 4) cx, cy, sx, sy of the merged band footprint
    band_height: float
    seen_total: int  # distinct voxels before the stability filter
    carved: int  # ...of the stable ones, how many the body swept away
    frames: int

    @property
    def dropped(self) -> int:
        """Voxels the stability filter rejected as flicker."""
        return self.seen_total - len(self.voxels) - self.carved

    def centers(self) -> np.ndarray:
        """Voxel centres in world metres."""
        return (self.voxels.astype(np.float64) + 0.5) * self.voxel

    def select(self, max_z: float | None = None, radius: float | None = None) -> np.ndarray:
        """Voxel centres inside a height band over the floor and, optionally, a
        corridor of ``radius`` around the recorded track."""
        pts = self.centers()
        keep = np.ones(len(pts), dtype=bool)
        if max_z is not None:
            keep &= (pts[:, 2] > self.floor_z + obstacles.LOW) & (pts[:, 2] < self.floor_z + max_z)
        if radius is not None and len(self.track):
            from scipy.spatial import cKDTree

            d, _ = cKDTree(self.track[:, :2]).query(pts[:, :2])
            keep &= d < radius
        return np.asarray(pts[keep])

    def save(self, path: str | FilePath) -> FilePath:
        out = FilePath(path)
        np.savez_compressed(
            out,
            name=self.name,
            voxel=self.voxel,
            voxels=self.voxels,
            floor_z=self.floor_z,
            start=self.start,
            goal=self.goal,
            path=self.path,
            track=self.track,
            rects=self.rects,
            band_height=self.band_height,
            seen_total=self.seen_total,
            carved=self.carved,
            frames=self.frames,
        )
        return out if out.suffix else out.with_suffix(".npz")

    @classmethod
    def load(cls, path: str | FilePath) -> RecordedWorld:
        d = np.load(FilePath(path))
        return cls(
            name=str(d["name"]),
            voxel=float(d["voxel"]),
            voxels=d["voxels"],
            floor_z=float(d["floor_z"]),
            start=d["start"],
            goal=d["goal"],
            path=d["path"],
            track=d["track"],
            rects=d["rects"],
            band_height=float(d["band_height"]),
            seen_total=int(d["seen_total"]),
            carved=int(d["carved"]),
            frames=int(d["frames"]),
        )


def stable_mask(
    seen: np.ndarray,
    span: np.ndarray,
    frac: float = STABILITY,
    min_frames: int = MIN_FRAMES,
) -> np.ndarray:
    """Voxels seen in at least ``frac`` of their own visibility span."""
    need = np.maximum(min_frames, np.ceil(frac * span))
    return np.asarray(seen >= need)


def voxel_history(
    clouds: list[np.ndarray], voxel: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Per-voxel (index, times seen, frames spanned) over a sequence of clouds."""
    seen: dict[tuple[int, int, int], int] = {}
    first: dict[tuple[int, int, int], int] = {}
    last: dict[tuple[int, int, int], int] = {}
    for i, pts in enumerate(clouds):
        for key in map(tuple, np.unique(np.floor(pts / voxel).astype(np.int32), axis=0)):
            k: Any = key
            seen[k] = seen.get(k, 0) + 1
            first.setdefault(k, i)
            last[k] = i
    keys = np.array(list(seen), dtype=np.int32).reshape(-1, 3)
    counts = np.array([seen[tuple(k)] for k in keys], dtype=np.int32)  # type: ignore[index]
    spans = np.array([last[tuple(k)] - first[tuple(k)] + 1 for k in keys], dtype=np.int32)  # type: ignore[index]
    return keys, counts, spans


def merge(idx: np.ndarray) -> np.ndarray:
    """Greedy box cover of an integer voxel set: (m, 6) of lo_ijk + size_ijk.

    Grows +x, then +y, then +z; a flat set (one k) yields 2D rectangles.
    Allocates the bounding grid densely, so crop before merging a big world.
    """
    if not len(idx):
        return np.zeros((0, 6), dtype=np.int64)
    idx = np.asarray(idx, dtype=np.int64).reshape(-1, 3)
    lo = idx.min(0)
    nx, ny, nz = (idx.max(0) - lo + 1).tolist()
    free = np.zeros((nx, ny, nz), dtype=bool)
    free[tuple((idx - lo).T)] = True
    out = []
    for i, j, k in zip(*np.nonzero(free), strict=True):
        if not free[i, j, k]:
            continue
        di = 1
        while i + di < nx and free[i + di, j, k]:
            di += 1
        dj = 1
        while j + dj < ny and free[i : i + di, j + dj, k].all():
            dj += 1
        dk = 1
        while k + dk < nz and free[i : i + di, j : j + dj, k + dk].all():
            dk += 1
        free[i : i + di, j : j + dj, k : k + dk] = False
        out.append((i + lo[0], j + lo[1], k + lo[2], di, dj, dk))
    return np.array(out, dtype=np.int64)


def merged_boxes(pts: np.ndarray, voxel: float) -> tuple[np.ndarray, np.ndarray]:
    """Greedy-merged (centres, half-sizes) in metres for a set of voxel centres."""
    if not len(pts):
        return np.zeros((0, 3)), np.zeros((0, 3))
    idx = np.floor(pts / voxel).astype(np.int64)
    boxes = merge(idx)
    lo = boxes[:, :3] * voxel
    size = boxes[:, 3:] * voxel
    return lo + size / 2.0, size / 2.0


def band_rects(pts: np.ndarray, voxel: float, floor_z: float) -> np.ndarray:
    """The body-band footprint as merged rectangles: (r, 4) cx, cy, sx, sy."""
    band = pts[(pts[:, 2] > floor_z + obstacles.LOW) & (pts[:, 2] < floor_z + BAND_TOP)]
    if not len(band):
        return np.zeros((0, 4))
    idx = np.floor(band[:, :2] / voxel).astype(np.int64)
    idx = np.unique(idx, axis=0)
    flat = np.column_stack([idx, np.zeros(len(idx), dtype=np.int64)])
    boxes = merge(flat)
    lo = boxes[:, :2] * voxel
    size = boxes[:, 3:5] * voxel
    # The live planner sees voxel CENTRES; a full-cell extrusion puts every
    # face half a voxel closer and fattens each wall by 8 cm total, which is
    # enough to seal a doorway the robot walked through. Shrink to the span
    # of the centres (a one-voxel run becomes a thin slab, like the point it is).
    return np.column_stack([lo + size / 2.0, np.maximum(size - voxel, 1e-3)])


def clearance(rects: np.ndarray, xy: np.ndarray) -> np.ndarray:
    """Signed distance from points to the nearest band rectangle, negative inside."""
    xy = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
    if not len(rects):
        return np.full(len(xy), np.inf)
    dx = np.abs(xy[:, None, 0] - rects[None, :, 0]) - rects[None, :, 2] / 2.0
    dy = np.abs(xy[:, None, 1] - rects[None, :, 1]) - rects[None, :, 3] / 2.0
    sdf = np.hypot(np.maximum(dx, 0.0), np.maximum(dy, 0.0)) + np.minimum(np.maximum(dx, dy), 0.0)
    return np.asarray(sdf.min(axis=1))


def swept(xy: np.ndarray, track: np.ndarray, length: float, width: float) -> np.ndarray:
    """Mask of xy points the body covered at some pose of the recorded track."""
    import math

    from scipy.spatial import cKDTree

    hit = np.zeros(len(xy), dtype=bool)
    if not len(xy) or not len(track):
        return hit
    half_diag = math.hypot(length, width) / 2.0
    d, _ = cKDTree(track[:, :2]).query(xy)
    cand = np.flatnonzero(d <= half_diag)
    px, py = xy[cand, 0], xy[cand, 1]
    for x, y, yaw in track:
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = px - x, py - y
        inside = (np.abs(c * dx + s * dy) <= length / 2.0) & (
            np.abs(-s * dx + c * dy) <= width / 2.0
        )
        hit[cand[inside]] = True
    return hit


def _base_leg(child_frame: str) -> Any:
    """Static mount transform from an odometry child frame up to ``base_link``."""
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.protocol.tf.static_tf_publisher import frames_to_edge_transforms
    from dimos.robot.unitree.go2.go2_mid360_static_transforms import FRAMES

    if child_frame == "base_link":
        return Transform.identity()
    edges = {t.child_frame_id: t for t in frames_to_edge_transforms(FRAMES)}
    if child_frame not in edges:
        raise ValueError(f"no static mount leg for odometry child frame {child_frame!r}")
    leg = -edges[child_frame]
    while leg.child_frame_id != "base_link":
        leg = leg + (-edges[leg.child_frame_id])
    return leg


def _raw(store: Any, name: str) -> list[tuple[float, bytes]]:
    """(timestamp, LCM payload) pairs for one recorded stream."""
    return [(o.ts, o.data) for o in store.stream(name)]


def extract(
    source: str | FilePath,
    voxel: float = VOXEL,
    frac: float = STABILITY,
    min_frames: int = MIN_FRAMES,
    carve: float = CARVE,
    map_stream: str = "dimos_local_map_sensor_msgs.PointCloud2",
    odom_stream: str = "dimos_odometry_nav_msgs.Odometry",
    path_stream: str = "dimos_path_nav_msgs.Path",
) -> RecordedWorld:
    """Build the static world from a mem2/mcap navigation recording."""
    from dimos.memory2.cli.dataset import open_dataset
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.nav_msgs.Odometry import Odometry
    from dimos.msgs.nav_msgs.Path import Path
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
    from dimos.utils.data import get_data

    store = open_dataset(str(get_data(source)))
    with store:
        raw = {n: _raw(store, n) for n in (map_stream, odom_stream, path_stream)}
    maps = [(ts, PointCloud2.lcm_decode(b).points_f32()) for ts, b in raw[map_stream]]
    odom = [(ts, Odometry.lcm_decode(b)) for ts, b in raw[odom_stream]]
    paths = [Path.lcm_decode(b) for _, b in raw[path_stream]]
    if not maps:
        raise ValueError(f"{source}: no {map_stream} frames")
    if not odom:
        raise ValueError(f"{source}: no {odom_stream} messages")

    keys, counts, spans = voxel_history([c for _, c in maps], voxel)
    kept = keys[stable_mask(counts, spans, frac, min_frames)]
    centres = (kept.astype(np.float64) + 0.5) * voxel

    # Odometry rides the sensor frame; the base pose is the one the sim spawns
    # and the planner is judged at, so resolve it through the static mount leg.
    leg = _base_leg(odom[0][1].child_frame_id)
    bases = [
        (Transform.from_pose(m.child_frame_id, m.to_pose_stamped()) + leg).to_pose()
        for _, m in odom
    ]
    track = np.array([(p.position.x, p.position.y, p.orientation.euler[2]) for p in bases]).reshape(
        -1, 3
    )
    # The ground the way the live stack knows it: under the BODY, not off the
    # scene. A percentile of voxel heights lands on below-floor returns and
    # sinks the band a full layer under what the deployed model slices, turning
    # steppable clutter into walls.
    floor_z = float(np.median([p.position.z for p in bases])) - GO2.base_height
    ots = np.array([t for t, _ in odom])
    start = track[int(np.clip(np.searchsorted(ots, maps[0][0]), 0, len(track) - 1))]

    # The global path the operator's goal produced; the longest one carries the
    # whole route (the first messages are single-pose stubs).
    best = max(paths, key=lambda p: len(p.poses), default=None)
    route = (
        np.array([(p.position.x, p.position.y) for p in best.poses]).reshape(-1, 2)
        if best is not None
        else np.zeros((0, 2))
    )
    goal = route[-1] if len(route) else start[:2]

    # The robot maps its own body and legs. Everything the body swept is free by
    # demonstration, so carve the swept footprint out of the walkable band —
    # otherwise the sim spawns the robot inside a wall built from its own return.
    # A voxel counts as swept when its CELL overlaps the footprint, not its
    # centre (the cell is what becomes a collision box), hence the half voxel.
    grow = voxel + 2.0 * carve
    low = centres[:, 2] < floor_z + BODY[2]
    hit = np.zeros(len(centres), dtype=bool)
    hit[low] = swept(centres[low][:, :2], track, BODY[0] + grow, BODY[1] + grow)
    kept, centres = kept[~hit], centres[~hit]

    return RecordedWorld(
        name=FilePath(source).stem,
        voxel=voxel,
        voxels=kept.astype(np.int32),
        floor_z=floor_z,
        start=np.asarray(start, dtype=np.float64),
        goal=np.asarray(goal, dtype=np.float64),
        path=route,
        track=track,
        rects=band_rects(centres, voxel, floor_z),
        band_height=BAND_HEIGHT,
        seen_total=len(keys),
        carved=int(hit.sum()),
        frames=len(maps),
    )


def local_map(
    rw: RecordedWorld,
    pose: tuple[float, float, float],
    range_m: float = LOCAL_MAP_RANGE,
    ts: float = 0.0,
) -> PointCloud2:
    """The static world around a pose, shaped like the raycaster's local_map.

    World frame, range-limited in the plane — the same accumulated-map dialect
    the planner subscribes to on the robot.
    """
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

    pts = rw.centers()
    near = pts[np.linalg.norm(pts[:, :2] - np.asarray(pose[:2]), axis=1) <= range_m]
    return PointCloud2.from_numpy(near.astype(np.float32), frame_id="world", timestamp=ts)


def scene(
    rw: RecordedWorld,
    max_z: float = SCENE_MAX_Z,
    radius: float | None = None,
    menagerie: FilePath | None = None,
    physics: dict[str, float] | None = None,
) -> tuple[mujoco.MjModel, mujoco.MjData]:
    """The fitted Go2 scene with the recorded world as static collision boxes.

    Floor plane sits at the estimated floor, so everything stays in the
    recording's own coordinates. Geoms use the referee's ``wall_`` prefix, so
    :func:`control.world.wall_contact` reads collisions unchanged. They all
    hang off the worldbody rather than one body each: thousands of bodies
    overflow the collision stack, and geoms sharing a body never pair up.
    """
    import mujoco

    from dimos.navigation.motion.control.referee import world
    from dimos.navigation.motion.simulation import model as go2_model
    from dimos.navigation.motion.simulation.evaluate import apply_physics

    pts = rw.select(max_z=max_z, radius=radius)
    spec = mujoco.MjSpec.from_file(str(go2_model.scene_path(menagerie)))
    spec.geom("floor").pos = [0.0, 0.0, rw.floor_z]
    # Planner-band voxels in the wall colour, overhead scenery translucent:
    # what LOOKS impassable in the viewer must be what the planner slices.
    in_band = pts[:, 2] < rw.floor_z + BAND_TOP
    i = 0
    for sel, rgba in ((in_band, world.WALL_RGBA), (~in_band, (0.55, 0.58, 0.62, 0.35))):
        centres, half = merged_boxes(pts[sel], rw.voxel)
        for c, h in zip(centres, half, strict=True):
            geom = spec.worldbody.add_geom()
            geom.type = mujoco.mjtGeom.mjGEOM_BOX
            geom.name = f"{world.WALL_PREFIX}{i}"
            geom.pos = tuple(c)
            geom.size = tuple(h)
            geom.rgba = rgba
            i += 1
    model = spec.compile()
    if physics:
        apply_physics(model, physics)
    return model, mujoco.MjData(model)


def spawn(
    model: mujoco.MjModel, data: mujoco.MjData, rw: RecordedWorld, default_pose: np.ndarray
) -> None:
    """Keyframe stand at the recorded start pose, on the estimated floor."""
    import math

    import mujoco

    kid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")  # type: ignore[attr-defined]
    if kid >= 0:
        mujoco.mj_resetDataKeyframe(model, data, kid)
    data.qpos[7:19] = default_pose
    data.qpos[0] = rw.start[0]
    data.qpos[1] = rw.start[1]
    data.qpos[2] += rw.floor_z
    yaw = float(rw.start[2])
    data.qpos[3:7] = (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))
    mujoco.mj_forward(model, data)


DEFAULT_POLICY = "ml-trajectory-research/freewalk_mcf.bin"


def view(rw: RecordedWorld, max_z: float, radius: float | None, policy_blob: str) -> None:
    """Hold the stance at the recorded start pose inside the recorded world."""
    import time

    import mujoco
    from mujoco import viewer as mj_viewer

    from dimos.navigation.motion.simulation.policy import FreePolicy
    from dimos.navigation.motion.simulation.walk import TORQUE_LIMITS
    from dimos.utils.data import get_data

    policy = FreePolicy.load(get_data(policy_blob))
    model, data = scene(rw, max_z=max_z, radius=radius)
    spawn(model, data, rw, policy.default_pose)
    with mj_viewer.launch_passive(model, data) as v:
        while v.is_running():
            tau = policy.kp * (policy.default_pose - data.qpos[7:19]) - policy.kd * data.qvel[6:18]
            data.ctrl[:] = np.clip(tau, -TORQUE_LIMITS, TORQUE_LIMITS)
            mujoco.mj_step(model, data)
            v.sync()
            time.sleep(model.opt.timestep)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("source", help="a recording (mcap/db) to extract, or an existing .npz")
    ap.add_argument("--out", help="write the extracted world here (.npz)")
    ap.add_argument("--voxel", type=float, default=VOXEL, help=f"grid size (default {VOXEL})")
    ap.add_argument(
        "--stability",
        type=float,
        default=STABILITY,
        help=f"fraction of its visibility span a voxel must be seen in (default {STABILITY})",
    )
    ap.add_argument("--min-frames", type=int, default=MIN_FRAMES, help="absolute sighting floor")
    ap.add_argument(
        "--carve",
        type=float,
        default=CARVE,
        help=f"clearance granted to the recorded body sweep (default {CARVE} m)",
    )
    ap.add_argument("--max-z", type=float, default=SCENE_MAX_Z, help="collision height over floor")
    ap.add_argument(
        "--radius", type=float, default=None, help="crop to a corridor around the track"
    )
    ap.add_argument("--view", action="store_true", help="MuJoCo viewer at the recorded start pose")
    ap.add_argument("--policy", default=DEFAULT_POLICY, help="FREE policy blob (stance gains)")
    args = ap.parse_args()

    src = FilePath(args.source)
    if src.suffix == ".npz":
        rw = RecordedWorld.load(src)
    else:
        rw = extract(src, args.voxel, args.stability, args.min_frames, args.carve)
    if args.out:
        print(f"wrote {rw.save(args.out)}")

    pts = rw.centers()
    scene_pts = rw.select(max_z=args.max_z, radius=args.radius)
    centres, _ = merged_boxes(scene_pts, rw.voxel)
    lo, hi = pts.min(0), pts.max(0)
    print(
        f"{rw.name}: {rw.frames} map frames, {rw.seen_total} voxels seen, "
        f"{len(rw.voxels)} kept / {rw.dropped} flicker / {rw.carved} body-swept "
        f"({rw.voxel} m grid)"
    )
    print(
        f"  floor z {rw.floor_z:+.2f}  extent x [{lo[0]:.2f}, {hi[0]:.2f}] "
        f"y [{lo[1]:.2f}, {hi[1]:.2f}] z [{lo[2]:.2f}, {hi[2]:.2f}]"
    )
    print(
        f"  scene band (floor+{obstacles.LOW} .. floor+{args.max_z}"
        + (f", corridor {args.radius} m" if args.radius else "")
        + f"): {len(scene_pts)} voxels -> {len(centres)} geoms"
    )
    print(f"  planner band: {len(rw.rects)} rectangles, {rw.band_height} m tall")
    # Point clearance, not body clearance: a negative goal means the operator
    # aimed at something the accumulated map calls occupied, and every oracle
    # will refuse the world. Worth seeing before wondering why.
    room = clearance(rw.rects, np.array([rw.start[:2], rw.goal]))
    print(
        f"  start ({rw.start[0]:.2f}, {rw.start[1]:.2f}, {rw.start[2]:+.2f} rad) "
        f"room {room[0]:+.2f}  goal ({rw.goal[0]:.2f}, {rw.goal[1]:.2f}) room "
        f"{room[1]:+.2f}  path {len(rw.path)} poses"
    )

    if args.view:
        view(rw, args.max_z, args.radius, args.policy)


if __name__ == "__main__":
    main()
