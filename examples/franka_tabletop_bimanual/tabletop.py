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

"""Find whatever stands on the table, without knowing what it is (no motion).

Reads depth from the three RealSenses through a running `dimos run fr3-tabletop-bimanual`,
puts every point in the world frame (the right arm's base), keeps the points standing
clear of the tabletop and away from the arms' bases, groups them into objects and fits
each a box: centre, yaw, length / width / height.

    ~/dimos-franka/.venv/bin/python tabletop.py        # prints the objects, saves tabletop.png
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path

import cv2
import numpy as np
from scipy import ndimage
from scipy.spatial.transform import Rotation

HERE = Path(__file__).parent
TABLE_Z = 0.0  # the tabletop in the world frame, m (both arms measured it within 1 cm)
# Points this far above the table belong to objects, m: the tripod's depth is noisier
# from 1.3 m away than a wrist camera's from 0.4 m.
MIN_HEIGHT = {"scene": 0.02, "right_wrist": 0.006, "left_wrist": 0.006}
MAX_HEIGHT = 0.30
CELL = 0.005  # grid for grouping points, m
# Groups with fewer points are noise (points at every second pixel).
MIN_POINTS = {"scene": 80, "right_wrist": 150, "left_wrist": 150}
BASE_CLEARANCE = (
    0.30  # ignore points this close to either arm's base axis (cables; too folded to grasp), m
)
REACH = 0.85  # ignore points farther than this from both bases, m
# The calibration boards stay taped down; their print upsets the tripod's depth.
FIXTURE_MARGIN = 0.015  # m, around each board
# Each camera's colour-optical frame from its camera_link: RealSense extrinsics.
LINK_OPT = {
    "scene": "scene_link_opt.npy",
    "right_wrist": "handeye_data.npz",
    "left_wrist": "handeye2_data.npz",
}


@dataclass
class TableObject:
    centre: np.ndarray  # x, y of the box centre; z of its middle height, m (world)
    yaw: float  # direction of its long side in the table plane, rad
    length: float  # along yaw, m
    width: float  # across, m
    height: float  # above the table, m
    points: int
    seen_by: str
    table_z: float = TABLE_Z  # the tabletop's height under it, as its camera measured it, m

    @property
    def axis(self) -> np.ndarray:
        return np.array([math.cos(self.yaw), math.sin(self.yaw), 0.0])

    def __str__(self) -> str:
        return (
            f"{self.length * 100:.1f} x {self.width * 100:.1f} x {self.height * 100:.1f} cm "
            f"at ({self.centre[0]:.3f}, {self.centre[1]:.3f}), long side at "
            f"{math.degrees(self.yaw):.0f} deg, {self.points} points, from {self.seen_by}"
        )


def tf(R: np.ndarray, t) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, np.ravel(t)
    return T


def _transform(t) -> np.ndarray:
    q = t.rotation
    return tf(
        Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix(),
        [t.translation.x, t.translation.y, t.translation.z],
    )


# The tripod camera's colour optical frame in the world, re-measured from the table
# marker by refresh_scene(); None until then (the calibration's value is used).
SCENE_T_WORLD_OPT: np.ndarray | None = None
SCENE_MOVED_WARN = 0.02  # m: say so when the tripod has moved this much
SCENE_POSE = HERE / "scene_pose.json"  # the last good re-measure, for when the marker is hidden


def _detect_marker(img: np.ndarray, K, dist):
    """The 4x3 marker board in the tripod image, found on a 2x enlargement: from where it
    is taped now its squares are ~12 px and at night only 4 of 6 corners came out at 1x."""
    import bimanual_cal as bc

    board = bc.BOARDS["marker"]
    big = cv2.resize(img, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
    corners, ids, _, _ = cv2.aruco.CharucoDetector(board).detectBoard(big)
    n = 0 if ids is None else len(ids)
    if n < bc.MIN_CORNERS["marker"]:
        return None, n, float("nan")
    obj, imgp = board.matchImagePoints(corners / 2.0, ids)
    _, r, t = cv2.solvePnP(obj, imgp, K, dist)
    proj, _ = cv2.projectPoints(obj, r, t, K, dist)
    err = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - imgp.reshape(-1, 2)) ** 2, 1))))
    return tf(cv2.Rodrigues(r)[0], t), n, err


def _last_scene_pose() -> np.ndarray | None:
    if SCENE_POSE.exists():
        return np.array(json.loads(SCENE_POSE.read_text())["T_world_opt"])
    return None


def refresh_scene(app, frames: int = 5, tries: int = 4) -> np.ndarray | None:
    """Re-measure where the tripod camera is from the 4x3 marker taped to the table.

    The marker's pose in the world comes from the bimanual calibration; the camera's
    view of it gives the camera's pose. Returns the new pose, or None (keeping the
    last one) when the marker is not seen whole.
    """

    global SCENE_T_WORLD_OPT
    cal = json.loads((HERE / "bimanual_cal.json").read_text())["arm1_to_marker"]
    T_world_marker = tf(Rotation.from_quat(cal["rotation_xyzw"]).as_matrix(), cal["translation_m"])
    info = app.scene.peek_stream("camera_info", 5.0)
    K, dist = np.asarray(info.K).reshape(3, 3), np.asarray(info.D, dtype=float)
    for attempt in range(tries):
        imgs = [
            np.asarray(app.scene.peek_stream("color_image", 5.0).data).astype(np.float32)
            for _ in range(frames)
        ]
        img = cv2.cvtColor(np.mean(imgs, 0).astype(np.uint8), cv2.COLOR_RGB2BGR)
        T_opt_marker, n, err = _detect_marker(img, K, dist)
        if T_opt_marker is not None and n >= 6:
            break
        import time

        time.sleep(0.5)
    last = _last_scene_pose()
    previous = (
        SCENE_T_WORLD_OPT
        if SCENE_T_WORLD_OPT is not None
        else (last if last is not None else _scene_from_calibration())
    )
    if T_opt_marker is None or n < 6:
        if SCENE_T_WORLD_OPT is None and last is not None:
            SCENE_T_WORLD_OPT = last
        print(
            f"  tripod: the marker is not seen whole ({n} of 6 corners); keeping the last good pose "
            f"({'saved' if last is not None else 'calibration'})"
        )
        return None
    T = T_world_marker @ np.linalg.inv(T_opt_marker)
    moved = float(np.linalg.norm(T[:3, 3] - previous[:3, 3]))
    turned = math.degrees(
        (Rotation.from_matrix(previous[:3, :3]).inv() * Rotation.from_matrix(T[:3, :3])).magnitude()
    )
    note = "  <- it has moved: check the tripod" if moved > SCENE_MOVED_WARN else ""
    print(
        f"  tripod re-measured from the marker ({err:.2f} px): {moved * 1000:.0f} mm and "
        f"{turned:.1f} deg from the last good pose{note}"
    )
    SCENE_T_WORLD_OPT = T
    SCENE_POSE.write_text(
        json.dumps({"T_world_opt": T.tolist(), "marker_px_error": float(err)}, indent=1)
    )
    return T


def _scene_from_calibration() -> np.ndarray:
    from dimos.robot.manipulators.franka.tabletop_bimanual import scene_camera_transform

    return _transform(scene_camera_transform()) @ np.load(HERE / LINK_OPT["scene"])


def world_from_optical(camera: str, joints: dict[str, list[float]], model) -> np.ndarray:
    """T_world_opt for a camera, from its calibration and, on a wrist, the arm's joints."""
    from dimos.robot.manipulators.franka.tabletop_bimanual import (
        scene_camera_transform,
        wrist_camera_transform,
    )

    src = HERE / LINK_OPT[camera]
    T_link_opt = np.load(src) if src.suffix == ".npy" else np.load(src)["T_link_opt"]
    if camera == "scene":
        return (
            SCENE_T_WORLD_OPT
            if SCENE_T_WORLD_OPT is not None
            else _transform(scene_camera_transform()) @ T_link_opt
        )
    side = camera.split("_")[0]
    return model.hand(side, joints[side]) @ _transform(wrist_camera_transform(side)) @ T_link_opt


class Model:
    """Forward kinematics of both arms in the shared world (the right arm's base)."""

    def __init__(self) -> None:
        from dimos.manipulation.planning.spec.validation import prepare_robot_model
        from dimos.manipulation.planning.world.drake_world import DrakeWorld
        from dimos.robot.manipulators.franka.tabletop_bimanual import (
            make_tabletop_bimanual_model_config,
        )

        self.config = make_tabletop_bimanual_model_config()
        self.world = DrakeWorld(enable_viz=False)
        self.world.load_model(prepare_robot_model(self.config))
        self.world.finalize()
        self.ctx = self.world.get_live_context()

    def set(self, right, left) -> None:
        from dimos.msgs.sensor_msgs.JointState import JointState

        self.world.set_joint_state(
            self.ctx, JointState(name=self.config.joint_names, position=[*right, *left])
        )

    def pose(self, link: str) -> np.ndarray:
        return np.asarray(self.world.get_link_pose(self.ctx, link))

    def hand(self, side: str, q) -> np.ndarray:
        ready = [0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398]
        self.set(q, ready) if side == "right" else self.set(ready, q)
        return self.pose(f"{side}/fr3_hand")


def world_points(
    depth_m: np.ndarray, K: np.ndarray, T_world_opt: np.ndarray, step: int = 2
) -> np.ndarray:
    v, u = np.mgrid[0 : depth_m.shape[0] : step, 0 : depth_m.shape[1] : step]
    z = depth_m[v, u]
    ok = (z > 0.15) & (z < 2.5)
    u, v, z = u[ok], v[ok], z[ok]
    pts = np.c_[(u - K[0, 2]) / K[0, 0] * z, (v - K[1, 2]) / K[1, 1] * z, z, np.ones_like(z)]
    return (T_world_opt @ pts.T).T[:, :3]


def fixtures() -> list[np.ndarray]:
    """The taped-down calibration boards, each as its four corners (x, y) on the table."""
    return [np.array(c) for c in json.loads((HERE / "fixtures.json").read_text()).values()]


def inside(poly: np.ndarray, xy: np.ndarray, margin: float) -> np.ndarray:
    """Which points lie inside a convex polygon grown by ``margin``."""
    c = poly.mean(0)
    keep = np.ones(len(xy), dtype=bool)
    for a, b in zip(poly, np.roll(poly, -1, axis=0)):
        n = np.array([b[1] - a[1], a[0] - b[0]])
        n /= np.linalg.norm(n)
        n = n if n @ (a - c) > 0 else -n  # outward
        keep &= (xy - a) @ n < margin
    return keep


def fit_table(pts: np.ndarray, rng=np.random.default_rng(0)) -> tuple[np.ndarray, float]:
    """The tabletop as seen by one camera: unit normal n (up) and offset d, n.p = d.

    Each camera's calibration puts the table a little off z = 0 and a little tilted;
    fitting the plane in its own points makes "above the table" exact for it.
    """
    near = pts[np.abs(pts[:, 2] - TABLE_Z) < 0.05]
    best, best_n = None, -1
    for _ in range(200):
        a, b, c = near[rng.choice(len(near), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        n = n if n[2] > 0 else -n
        if n[2] < 0.95:  # within ~18 deg of level
            continue
        count = np.count_nonzero(np.abs(near @ n - n @ a) < 0.004)
        if count > best_n:
            best, best_n = (n, float(n @ a)), count
    n, d = best
    inliers = near[np.abs(near @ n - d) < 0.004]
    c = inliers.mean(0)
    n = np.linalg.svd(inliers - c, full_matrices=False)[2][2]
    n = n if n[2] > 0 else -n
    return n, float(n @ c)


def above_table(
    pts: np.ndarray, bases: list[np.ndarray], camera: str
) -> tuple[np.ndarray, np.ndarray, tuple[np.ndarray, float]]:
    """Points standing on the table (inside the outline of the tabletop this camera
    sees), and each one's height above it."""
    from scipy.spatial import Delaunay

    n, d = fit_table(pts)
    h = pts @ n - d
    surface = pts[np.abs(h) < 0.004][:, :2]
    surface = surface[
        np.random.default_rng(0).choice(len(surface), min(3000, len(surface)), replace=False)
    ]
    on_table = Delaunay(surface).find_simplex(pts[:, :2]) >= 0
    keep = (h > MIN_HEIGHT[camera]) & (h < MAX_HEIGHT) & on_table
    reach = np.zeros(len(pts), dtype=bool)
    for b in bases:
        r = np.hypot(pts[:, 0] - b[0], pts[:, 1] - b[1])
        keep &= r > BASE_CLEARANCE
        reach |= r < REACH
    keep &= reach
    for poly in fixtures():
        keep &= ~inside(poly, pts[:, :2], FIXTURE_MARGIN)
    return pts[keep], h[keep], (n, d)


def group(
    pts: np.ndarray, heights: np.ndarray, plane: tuple[np.ndarray, float], seen_by: str
) -> list[TableObject]:
    if len(pts) == 0:
        return []
    ij = np.floor(pts[:, :2] / CELL).astype(int)
    lo = ij.min(0)
    grid = np.zeros(tuple(ij.max(0) - lo + 1), dtype=bool)
    grid[tuple((ij - lo).T)] = True
    grid = ndimage.binary_closing(grid, iterations=1)
    labels, n = ndimage.label(grid)
    which = labels[tuple((ij - lo).T)]
    found = []
    for k in range(1, n + 1):
        mine, mine_h = pts[which == k], heights[which == k]
        if len(mine) < MIN_POINTS[seen_by]:
            continue
        xy = mine[:, :2]
        c = xy.mean(0)
        evals, evecs = np.linalg.eigh(np.cov((xy - c).T))
        a = evecs[:, 1]  # long side
        along, across = (xy - c) @ a, (xy - c) @ np.array([-a[1], a[0]])
        lo_a, hi_a = np.percentile(along, [2, 98])
        lo_c, hi_c = np.percentile(across, [2, 98])
        centre_xy = c + a * (lo_a + hi_a) / 2 + np.array([-a[1], a[0]]) * (lo_c + hi_c) / 2
        top = float(np.percentile(mine_h, 95))
        yaw = math.atan2(a[1], a[0])
        yaw = (yaw + math.pi / 2) % math.pi - math.pi / 2  # a side has no direction
        n, d = plane
        table_z = float((d - n[0] * centre_xy[0] - n[1] * centre_xy[1]) / n[2])
        found.append(
            TableObject(
                centre=np.r_[centre_xy, table_z + top / 2],
                yaw=yaw,
                length=float(hi_a - lo_a),
                width=float(hi_c - lo_c),
                height=top,
                points=len(mine),
                seen_by=seen_by,
                table_z=table_z,
            )
        )
    return found


def table_outline(app, model: Model | None = None) -> np.ndarray:
    """The tabletop's outline as the tripod camera sees it: a convex polygon (x, y)."""
    from scipy.spatial import ConvexHull

    model = model or Model()
    mod = app.scene
    info = mod.peek_stream("camera_info", 5.0)
    depth = np.asarray(mod.peek_stream("depth_image", 5.0).data).astype(float) / 1000.0
    pts = world_points(
        depth, np.asarray(info.K).reshape(3, 3), world_from_optical("scene", {}, model)
    )
    n, d = fit_table(pts)
    from dimos.robot.manipulators.franka.tabletop_bimanual import LEFT_BASE_XYZ

    surface = pts[np.abs(pts @ n - d) < 0.006][:, :2]
    near = (
        np.minimum(np.hypot(*surface.T), np.hypot(*(surface - np.array(LEFT_BASE_XYZ[:2])).T)) < 1.0
    )
    surface = surface[near]  # the table the arms stand on, not other surfaces farther off
    hull = ConvexHull(surface)
    return surface[hull.vertices]


def look(app, model: Model | None = None, cameras=("scene", "right_wrist", "left_wrist")):
    """Objects on the table from each camera, and the images, via a running stack."""
    from dimos.robot.manipulators.franka.tabletop_bimanual import LEFT_BASE_XYZ

    model = model or Model()
    groups = app.ManipulationModule.get_state().groups
    joints = {s: list(groups[s].joints.position) for s in ("right", "left")}
    bases = [np.zeros(2), np.array(LEFT_BASE_XYZ[:2])]
    out, images = {}, {}
    for cam in cameras:
        mod = getattr(app, cam)
        info = mod.peek_stream("camera_info", 5.0)
        depth = np.asarray(mod.peek_stream("depth_image", 5.0).data).astype(float) / 1000.0
        images[cam] = np.asarray(mod.peek_stream("color_image", 5.0).data)
        K = np.asarray(info.K).reshape(3, 3)
        T = world_from_optical(cam, joints, model)
        pts, h, plane = above_table(world_points(depth, K, T), bases, cam)
        out[cam] = (group(pts, h, plane, cam), K, T)
    return out, images


COLOUR_DIFF = 30.0  # Lab distance from the table's own colour that counts as "not table"
MIN_PIXELS = 25  # smaller colour blobs are noise
# Colour also sees the boards' paper borders and tape, and the cables around the bases.
COLOUR_FIXTURE_MARGIN = 0.04  # m
COLOUR_BASE_CLEARANCE = 0.35  # m


def colour_candidates(app, model: Model | None = None) -> list[TableObject]:
    """What on the tabletop does not look like the tabletop, from the tripod's colour
    image: thin or flat things the depth misses. Each blob is put on the table plane;
    its height is unknown (a wrist camera measures it)."""
    from dimos.robot.manipulators.franka.tabletop_bimanual import LEFT_BASE_XYZ

    model = model or Model()
    mod = app.scene
    info = mod.peek_stream("camera_info", 5.0)
    K = np.asarray(info.K).reshape(3, 3)
    depth = np.asarray(mod.peek_stream("depth_image", 5.0).data).astype(float) / 1000.0
    rgb = np.asarray(mod.peek_stream("color_image", 5.0).data)
    T = world_from_optical("scene", {}, model)
    pts = world_points(depth, K, T)
    n, d = fit_table(pts)
    table = table_outline(app, model)
    # Every pixel's ray, onto the table plane.
    H, W = rgb.shape[:2]
    v, u = np.mgrid[0:H, 0:W]
    rays = np.stack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones_like(u, float)], -1)
    R, o = T[:3, :3], T[:3, 3]
    dirs = rays.reshape(-1, 3) @ R.T
    t = (d - o @ n) / (dirs @ n)
    hit = o + dirs * t[:, None]
    xy = hit[:, :2]
    ok = (t > 0) & inside(table, xy, -0.03)
    bases = [np.zeros(2), np.array(LEFT_BASE_XYZ[:2])]
    for b in bases:
        ok &= np.hypot(*(xy - b).T) > COLOUR_BASE_CLEARANCE
    for poly in fixtures():
        ok &= ~inside(poly, xy, COLOUR_FIXTURE_MARGIN)
    lab = cv2.cvtColor(rgb, cv2.COLOR_RGB2LAB).reshape(-1, 3).astype(float)
    ref = np.median(lab[ok], axis=0)  # the table's colour
    diff = np.linalg.norm(lab - ref, axis=1)
    mask = (ok & (diff > COLOUR_DIFF)).reshape(H, W).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    # Parts of one object in different colours (a marker's barrel and cap) join up.
    joined = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(joined)
    labels = np.where(mask > 0, labels, 0)
    found = []
    for k in range(1, count):
        blob = xy[(labels == k).ravel()]
        if len(blob) < MIN_PIXELS:
            continue
        c = blob.mean(0)
        evals, evecs = np.linalg.eigh(np.cov((blob - c).T))
        a = evecs[:, 1]
        along, across = (blob - c) @ a, (blob - c) @ np.array([-a[1], a[0]])
        yaw = (math.atan2(a[1], a[0]) + math.pi / 2) % math.pi - math.pi / 2
        found.append(
            TableObject(
                centre=np.r_[c, TABLE_Z + 0.01],
                yaw=yaw,
                length=float(np.ptp(along)),
                width=float(np.ptp(across)),
                height=0.0,
                points=len(blob),
                seen_by="scene colour",
            )
        )
    return sorted(found, key=lambda o: -o.points)


def draw(
    img: np.ndarray, objs: list[TableObject], K: np.ndarray, T_world_opt: np.ndarray
) -> np.ndarray:
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR).copy()
    T = np.linalg.inv(T_world_opt)
    for o in objs:
        a, c = o.axis, np.array([-o.axis[1], o.axis[0], 0.0])
        for z in (TABLE_Z, TABLE_Z + o.height):
            base = np.r_[o.centre[:2], z]
            corners = [
                base + sa * a * o.length / 2 + sc * c * o.width / 2
                for sa, sc in ((1, 1), (1, -1), (-1, -1), (-1, 1))
            ]
            p = (T @ np.c_[corners, np.ones(4)].T).T[:, :3]
            if np.any(p[:, 2] <= 0):
                continue
            uv = (K @ (p / p[:, 2:3]).T).T[:, :2].astype(int)
            cv2.polylines(img, [uv], True, (0, 255, 0), 2)
    return img


def main() -> None:
    from dimos.porcelain.dimos import Dimos

    app = Dimos.connect()
    try:
        found, images = look(app)
    finally:
        app.stop()
    tiles = []
    for cam, (objs, K, T) in found.items():
        print(f"{cam}: {len(objs)} object(s)")
        for o in objs:
            print(f"   {o}")
        tiles.append(draw(images[cam], objs, K, T))
    cv2.imwrite(str(HERE / "tabletop.png"), np.hstack(tiles))
    print("saved tabletop.png")


if __name__ == "__main__":
    main()
