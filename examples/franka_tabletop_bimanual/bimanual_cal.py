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

"""Arm 1 <-> arm 2 and the tripod camera, from two boards taped to the table.

Each arm in turn looks at the big ChArUco board (between the bases) and the
tripod marker (4x3, toward the tripod) with its hand-eye-calibrated wrist camera,
from a few poses, while the other arm stays still. Both boards then have a pose in
each arm's base frame, so each board gives arm 1 -> arm 2 on its own (the two
agree only if both hand-eye calibrations and both boards are good). The tripod
camera's pose comes from the marker.

    plan   no motion: work out the look poses (joint limits, self-collision,
           fingertips above the table, clearance from the other arm along every
           move). Saves bimanual_plan.json.
    run    moves each arm slowly through its poses (dimOS FrankaConnection, all
           guards on) and records the boards. Arm 1 steps aside (ready, turned
           45 deg away from arm 2), arm 2 takes its views and parks the same way,
           arm 1 takes its views and ends at ready, then arm 2 returns to ready.
           Saves bimanual_data.npz.
    solve  no motion: arm 1 -> arm 2 from each board, how well they agree, and
           arm 1 -> tripod camera. Saves bimanual_cal.json.

    ~/dimos-franka/.venv/bin/python bimanual_cal.py plan|run|solve
"""

import argparse
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
import pylibfranka
import pyrealsense2 as rs
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

import handeye
from handeye import ARMS, Model, tf

HERE = Path(__file__).parent
PLAN = HERE / "bimanual_plan.json"
DATA = HERE / "bimanual_data.npz"
TRIPOD = "261922070328"

BOARDS = {
    "board": cv2.aruco.CharucoBoard(
        (8, 5), 0.030, 0.022, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    ),
    "marker": cv2.aruco.CharucoBoard(
        (4, 3), 0.060, 0.045, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    ),
}
SIZE = {"board": (0.24, 0.15), "marker": (0.24, 0.18)}
MIN_CORNERS = {"board": 15, "marker": 4}

# Rough layout (tape measure): arm 2's base in arm 1's frame, 63.5 cm apart,
# each base turned 20 deg inward, so arm 2 is yawed -40 deg relative to arm 1.
ROUGH_A1_A2 = tf(Rotation.from_euler("z", -40, degrees=True).as_matrix(), [0.22, 0.60, 0.0])
# Board centres in arm 1's frame: the marker from 63.5 cm to each base; the big
# board from the tripod image (between the bases, a little in front of them).
GUESS_A1 = {"board": np.array([0.22, 0.26, 0.0]), "marker": np.array([0.63, 0.11, 0.0])}

# Once a solve has run, plan with what it measured instead.
if (HERE / "bimanual_cal.json").exists():
    _c = json.loads((HERE / "bimanual_cal.json").read_text())
    ROUGH_A1_A2 = tf(
        Rotation.from_quat(_c["arm1_to_arm2"]["rotation_xyzw"]).as_matrix(),
        _c["arm1_to_arm2"]["translation_m"],
    )
    if "arm1_board_centre" in _c:
        GUESS_A1 = {k: np.array(_c[f"arm1_{k}_centre"]) for k in ("board", "marker")}

N_VIEWS = 4  # per arm per board
# Arm 2 parks at ready turned this far away from arm 1 while arm 1 takes its views.
PARK_TURN = math.radians(45)
LOOK_DIST = 0.50  # camera to board centre, m
MIN_TIP_Z = 0.15  # fingertips above the table, m
CLEARANCE = 0.30  # least distance between the two arms' link frames, m
LO = np.array([-2.9007, -1.8361, -2.9007, -3.077, -2.8763, 0.4398, -3.0508]) + 0.1
HI = np.array([2.9007, 1.8361, 2.9007, -0.1169, 2.8763, 4.6216, 3.0508]) - 0.1
LINKS = [f"fr3_link{i}" for i in range(2, 8)] + ["fr3_hand", "fr3_hand_tcp"]
W, H = 640, 480


def read_q(ip: str) -> list[float]:
    return list(pylibfranka.Robot(ip, pylibfranka.RealtimeConfig.kIgnore).read_once().q)


def hand_link(arm: int) -> np.ndarray:
    cal = json.loads((HERE / f"{ARMS[arm][2]}_refined.json").read_text())
    return tf(Rotation.from_quat(cal["rotation_xyzw"]).as_matrix(), cal["translation_m"])


def link_opt(arm: int) -> np.ndarray:
    return np.load(HERE / f"{ARMS[arm][2]}_data.npz")["T_link_opt"]


def intrinsics(arm: int) -> tuple[np.ndarray, np.ndarray]:
    d = np.load(HERE / f"{ARMS[arm][2]}_data.npz")
    return d["K"], d["dist"]


def points(model: Model, q) -> np.ndarray:
    """Link-frame origins of an arm, in its own base frame."""
    ctx, _ = model._set(q)
    return np.array([np.asarray(model.world.get_link_pose(ctx, n))[:3, 3] for n in LINKS])


def gap(model: Model, q_a1, q_a2) -> float:
    p1 = points(model, q_a1)
    p2 = (ROUGH_A1_A2 @ np.c_[points(model, q_a2), np.ones(len(LINKS))].T).T[:, :3]
    return float(np.min(np.linalg.norm(p1[:, None] - p2[None], axis=2)))


def board_outline(
    centre: np.ndarray, size: tuple[float, float], margin: float = 0.01
) -> np.ndarray:
    """A ring of points around a board lying flat at centre (orientation unknown)."""
    r = math.hypot(*size) / 2 + margin
    a = np.linspace(0, 2 * math.pi, 12, endpoint=False)
    return np.c_[centre[0] + r * np.cos(a), centre[1] + r * np.sin(a), np.full(12, centre[2])]


class Arm:
    def __init__(self, n: int, model: Model) -> None:
        self.n, self.model = n, model
        self.T_hand_opt = hand_link(n) @ link_opt(n)
        self.K, self.dist = intrinsics(n)

    def view(self, q, pts) -> tuple[np.ndarray, np.ndarray]:
        """Image points and depths of base-frame pts from q."""
        T = np.linalg.inv(self.model.hand(q) @ self.T_hand_opt)
        c = (T @ np.c_[pts, np.ones(len(pts))].T).T[:, :3]
        uv, _ = cv2.projectPoints(c, np.zeros(3), np.zeros(3), self.K, self.dist)
        return uv.reshape(-1, 2), c[:, 2]

    def look_ik(self, centre: np.ndarray, seed) -> np.ndarray:
        """Joints that put centre in the middle of the image, LOOK_DIST away, looking down."""

        def res(q):
            T = self.model.hand(q) @ self.T_hand_opt
            c = np.linalg.inv(T) @ np.r_[centre, 1]
            down = T[:3, 2] @ [0, 0, -1]  # optical axis vs straight down
            return np.r_[
                c[0] / c[2] * 3,
                c[1] / c[2] * 3,
                (c[2] - LOOK_DIST) * 10,
                max(0.0, math.cos(math.radians(30)) - down) * 10,
                0.05 * (q - seed),
            ]

        return least_squares(res, seed, bounds=(LO, HI)).x

    def ok(self, q, outline) -> bool:
        if np.any(q < LO) or np.any(q > HI):
            return False
        if self.model.tcp(q)[2, 3] < MIN_TIP_Z or not self.model.collision_free(q):
            return False
        uv, z = self.view(q, outline)
        return bool(
            np.all(z > 0.1)
            and np.all(uv > 15)
            and np.all(uv[:, 0] < W - 15)
            and np.all(uv[:, 1] < H - 15)
        )


def path_ok(model: Model, a: Arm, q_from, q_to, q_other) -> float:
    """Least gap to the other arm, and table/self clearance, along the joint-space move."""
    least = 9.0
    # A move may start or end lower than MIN_TIP_Z (an arm parked low), but not dip below that.
    floor = min(MIN_TIP_Z, model.tcp(q_from)[2, 3], model.tcp(q_to)[2, 3]) - 0.01
    for s in np.linspace(0, 1, 25):
        q = np.asarray(q_from) + s * (np.asarray(q_to) - np.asarray(q_from))
        if model.tcp(q)[2, 3] < floor or not model.collision_free(q):
            return -1.0
        least = min(least, gap(model, q, q_other) if a.n == 1 else gap(model, q_other, q))
    return least


def plan() -> None:
    from dimos.robot.manipulators.franka.config import FR3_READY

    model = Model()
    q_now = {n: read_q(ARMS[n][0]) for n in (1, 2)}
    ready = np.array(FR3_READY)
    guess = {
        1: GUESS_A1,
        2: {k: (np.linalg.inv(ROUGH_A1_A2) @ np.r_[v, 1])[:3] for k, v in GUESS_A1.items()},
    }
    rng = np.random.default_rng(0)
    park1, park2 = ready.copy(), ready.copy()
    park1[0] -= PARK_TURN  # arm 1's -y is away from arm 2
    park2[0] += PARK_TURN  # arm 2's +y is away from arm 1
    # Arm 1 steps aside (parked) while arm 2 takes its views and parks; then arm 1
    # takes its views and ends at ready; then arm 2 comes back to ready.
    start = {1: park1, 2: np.array(q_now[2])}
    end = {2: park2, 1: ready}
    out = {
        "start": {str(n): q_now[n] for n in (1, 2)},
        "end": {str(n): list(map(float, end[n])) for n in (1, 2)},
        "park1": list(map(float, park1)),
        "arms": {},
    }
    g_pre = min(
        path_ok(model, Arm(1, model), q_now[1], ready, q_now[2]),
        path_ok(model, Arm(1, model), ready, park1, q_now[2]),
    )
    g_post = path_ok(model, Arm(2, model), park2, ready, ready)
    if g_pre < 0 or g_post < 0:
        raise SystemExit("parking arm 1 or bringing arm 2 back to ready is blocked")
    print(
        f"arm 1 to parked: closest to arm 2 {g_pre:.2f} m; arm 2 back to ready at the end: {g_post:.2f} m"
    )
    if min(g_pre, g_post) < CLEARANCE:
        raise SystemExit("the parking moves come too close; not saving")
    for n, other in ((2, 1), (1, 2)):
        a = Arm(n, model)
        q_other = park1 if n == 2 else park2
        seq, last, worst = [], ready, 9.0
        g = path_ok(model, a, start[n], ready, q_other)
        if g < 0:
            raise SystemExit(f"arm {n}: the move to ready is blocked")
        worst = min(worst, g)
        for name in ("board", "marker"):
            centre = guess[n][name]
            outline = board_outline(centre, SIZE[name])
            bearing = math.atan2(centre[1], centre[0])
            seed = ready.copy()
            seed[0] = np.clip(bearing, LO[0], HI[0])
            base = a.look_ik(centre, seed)
            if not a.ok(base, outline):
                uv, z = a.view(base, outline)
                raise SystemExit(
                    f"arm {n}: no clean look pose for the {name} at {centre.round(2)} "
                    f"(tip z {model.tcp(base)[2, 3]:.2f}, image x {uv[:, 0].min():.0f}-"
                    f"{uv[:, 0].max():.0f}, y {uv[:, 1].min():.0f}-{uv[:, 1].max():.0f})"
                )
            views = [base]
            for _ in range(3000):
                if len(views) == N_VIEWS:
                    break
                q = base + rng.uniform(-1, 1, 7) * np.radians([6, 6, 6, 6, 12, 12, 30])
                if a.ok(q, outline) and min(np.max(np.abs(q - v)) for v in views) > math.radians(5):
                    views.append(q)
            for q in views:
                g = path_ok(model, a, last, q, q_other)
                if g < 0:
                    raise SystemExit(f"arm {n}: a move to the {name} views is blocked")
                worst = min(worst, g)
                seq.append({"board": name, "q": list(map(float, q))})
                last = q
        g = path_ok(model, a, last, ready, q_other)
        g2 = path_ok(model, a, ready, end[n], q_other)
        if g < 0 or g2 < 0:
            raise SystemExit(f"arm {n}: the move back is blocked")
        worst = min(worst, g, g2)
        tip_min = min(model.tcp(s["q"])[2, 3] for s in seq)
        biggest = max(np.max(np.abs(np.array(s["q"]) - ready)) for s in seq)
        print(
            f"arm {n}: {len(seq)} views ({sum(s['board'] == 'board' for s in seq)} board, "
            f"{sum(s['board'] == 'marker' for s in seq)} marker); fingertips at least {tip_min:.2f} m up; "
            f"largest joint change from ready {math.degrees(biggest):.0f} deg; "
            f"closest to arm {other} (link frames, rough layout) {worst:.2f} m"
        )
        if worst < CLEARANCE:
            raise SystemExit(f"arm {n} would come within {worst:.2f} m of arm {other}; not saving")
        out["arms"][str(n)] = seq
    PLAN.write_text(json.dumps(out))
    print(f"saved {PLAN.name}")


# ---------------------------------------------------------------- run


def detect(name: str, img: np.ndarray, K, dist):
    board = BOARDS[name]
    corners, ids, mc, mi = cv2.aruco.CharucoDetector(board).detectBoard(img)
    n = 0 if ids is None else len(ids)
    if n < MIN_CORNERS[name]:
        return None, n, float("nan")
    obj, imgp = board.matchImagePoints(corners, ids)
    _, r, t = cv2.solvePnP(obj, imgp, K, dist)
    proj, _ = cv2.projectPoints(obj, r, t, K, dist)
    err = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - imgp.reshape(-1, 2)) ** 2, 1))))
    return tf(cv2.Rodrigues(r)[0], t), n, err


def tripod_view(frames: int = 20) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    cfg = rs.config()
    cfg.enable_device(TRIPOD)
    cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)
    pipe = rs.pipeline()
    prof = pipe.start(cfg)
    try:
        imgs = []
        for k in range(30 + frames):
            f = pipe.wait_for_frames(5000)
            if k >= 30:
                imgs.append(np.asarray(f.get_color_frame().get_data()).astype(np.float32))
        i = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    finally:
        pipe.stop()
    K = np.array([[i.fx, 0, i.ppx], [0, i.fy, i.ppy], [0, 0, 1]])
    return np.mean(imgs, 0).astype(np.uint8), K, np.array(i.coeffs)


def run() -> None:
    p = json.loads(PLAN.read_text())
    for n in (1, 2):
        now = read_q(ARMS[n][0])
        if max(abs(a - b) for a, b in zip(now, p["start"][str(n)])) > math.radians(1):
            raise SystemExit(f"arm {n} has moved since the plan was made; run `plan` again")
    from dimos.robot.manipulators.franka.config import FR3_READY

    img, K, dist = tripod_view()
    T_tri, nt, et = detect("marker", img, K, dist)
    print(
        f"tripod: marker {'seen, %d corners, %.2f px' % (nt, et) if T_tri is not None else 'NOT seen'}"
    )
    cv2.imwrite(str(HERE / "bimanual_tripod.png"), img)
    rec = {"tripod_K": K, "tripod_dist": dist}
    if T_tri is not None:
        rec["tripod_T_opt_marker"] = T_tri

    print("  arm 1 stepping aside (ready, turned away from arm 2)")
    handeye.use_arm(1)
    arm = handeye.Arm()
    try:
        arm.move(FR3_READY)
        arm.move(p["park1"])
    finally:
        arm.close()
    for n in (2, 1):
        handeye.use_arm(n)
        cam = handeye.Camera()
        arm = handeye.Arm()
        qs, names, Ts = [], [], []
        try:
            arm.move(FR3_READY)
            for i, v in enumerate(p["arms"][str(n)]):
                arm.move(v["q"])
                time.sleep(0.7)
                frame = cam.grab()
                T, c, e = detect(v["board"], frame, cam.K, cam.dist)
                print(
                    f"  arm {n} view {i + 1}/{len(p['arms'][str(n)])} ({v['board']}): "
                    + (f"{c} corners, {e:.2f} px" if T is not None else f"skipped ({c} corners)")
                )
                if T is not None:
                    qs.append(arm.q())
                    names.append(v["board"])
                    Ts.append(T)
                    cv2.imwrite(str(HERE / f"bimanual_a{n}_{i:02d}.png"), frame)
            arm.move(FR3_READY)
            print(f"  arm {n} to {'ready' if n == 1 else 'parked (ready, turned away from arm 1)'}")
            arm.move(p["end"][str(n)])
        finally:
            arm.close()
            cam.close()
        rec[f"a{n}_q"], rec[f"a{n}_board"], rec[f"a{n}_T_opt"] = (
            np.array(qs),
            np.array(names),
            np.array(Ts),
        )
    print("  arm 2 back to ready")
    handeye.use_arm(2)
    arm = handeye.Arm()
    try:
        arm.move(FR3_READY)
    finally:
        arm.close()
    np.savez(DATA, **rec)
    print(f"saved {DATA.name}")


# ---------------------------------------------------------------- solve


def mean_pose(Ts) -> tuple[np.ndarray, float, float]:
    """Average of rigid transforms, and the largest position / rotation deviation from it."""
    R = Rotation.from_matrix([T[:3, :3] for T in Ts]).mean()
    t = np.mean([T[:3, 3] for T in Ts], 0)
    dp = max(np.linalg.norm(T[:3, 3] - t) for T in Ts)
    dr = max((R.inv() * Rotation.from_matrix(T[:3, :3])).magnitude() for T in Ts)
    return tf(R.as_matrix(), t), float(dp), math.degrees(dr)


def solve() -> None:
    d = np.load(DATA)
    model = Model()
    in_base = {}
    for n in (1, 2):
        T_hand_opt = hand_link(n) @ link_opt(n)
        for name in ("board", "marker"):
            Ts = [
                model.hand(q) @ T_hand_opt @ T
                for q, b, T in zip(d[f"a{n}_q"], d[f"a{n}_board"], d[f"a{n}_T_opt"])
                if b == name
            ]
            if len(Ts) < 2:
                raise SystemExit(f"arm {n} saw the {name} in only {len(Ts)} views")
            in_base[n, name], dp, dr = mean_pose(Ts)
            print(
                f"arm {n} sees the {name:6s} at {in_base[n, name][:3, 3].round(4).tolist()} "
                f"({len(Ts)} views agree within {dp * 1000:.1f} mm, {dr:.2f} deg)"
            )
    est = {name: in_base[1, name] @ np.linalg.inv(in_base[2, name]) for name in ("board", "marker")}
    for name, T in est.items():
        yaw = Rotation.from_matrix(T[:3, :3]).as_euler("xyz", degrees=True)
        print(
            f"arm 1 -> arm 2 via the {name:6s}: {T[:3, 3].round(4).tolist()} m, "
            f"rpy {yaw.round(2).tolist()} deg"
        )
    dt = np.linalg.norm(est["board"][:3, 3] - est["marker"][:3, 3])
    dr = math.degrees(
        (
            Rotation.from_matrix(est["board"][:3, :3]).inv()
            * Rotation.from_matrix(est["marker"][:3, :3])
        ).magnitude()
    )
    print(f"the two boards agree within {dt * 1000:.1f} mm and {dr:.2f} deg")
    T_a1_a2, _, _ = mean_pose(list(est.values()))
    centres = {}
    for name in ("board", "marker"):
        c = (in_base[1, name] @ np.r_[np.array(SIZE[name]) / 2, 0, 1])[:3]
        c2 = (T_a1_a2 @ in_base[2, name] @ np.r_[np.array(SIZE[name]) / 2, 0, 1])[:3]
        centres[f"arm1_{name}_centre"] = ((c + c2) / 2).tolist()
    out = {
        "arm1_to_arm2": {
            "translation_m": T_a1_a2[:3, 3].tolist(),
            "rotation_xyzw": Rotation.from_matrix(T_a1_a2[:3, :3]).as_quat().tolist(),
            "board_disagreement_mm": dt * 1000,
            "board_disagreement_deg": dr,
        },
        **centres,
    }
    if "tripod_T_opt_marker" in d:
        # Marker in arm 1's frame: arm 1's own view, and arm 2's view carried over.
        T_a1_m, _, _ = mean_pose([in_base[1, "marker"], T_a1_a2 @ in_base[2, "marker"]])
        T_a1_tri = T_a1_m @ np.linalg.inv(d["tripod_T_opt_marker"])
        print(
            f"tripod camera (colour optical frame) in arm 1's frame: {T_a1_tri[:3, 3].round(3).tolist()} m"
        )
        out["arm1_to_tripod_optical"] = {
            "translation_m": T_a1_tri[:3, 3].tolist(),
            "rotation_xyzw": Rotation.from_matrix(T_a1_tri[:3, :3]).as_quat().tolist(),
        }
        out["arm1_to_marker"] = {
            "translation_m": T_a1_m[:3, 3].tolist(),
            "rotation_xyzw": Rotation.from_matrix(T_a1_m[:3, :3]).as_quat().tolist(),
        }
    (HERE / "bimanual_cal.json").write_text(json.dumps(out, indent=2))
    print("saved bimanual_cal.json")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("step", choices=["plan", "run", "solve"])
    {"plan": plan, "run": run, "solve": solve}[ap.parse_args().step]()
