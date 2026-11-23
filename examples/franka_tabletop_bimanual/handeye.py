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

"""Eye-in-hand calibration of the FR3 wrist RealSense with a ChArUco board.

    plan   no motion: detect the board from the current pose, then choose ~15
           joint poses around it where the whole board should stay in view, the
           arm is collision-free and the camera stays well above the table.
           Saves the plan to handeye_plan.json.
    run    moves through the plan slowly through dimOS's FrankaConnection (all
           guards on), records an image + joint angles at each pose, then
           returns to where it started. Saves handeye_data.npz.
    solve  no motion: solves fr3_hand -> camera_link from the recorded data and
           reports how consistent it is.

    ~/dimos-franka/.venv/bin/python handeye.py plan|run|solve|refine [--arm 2]

--arm picks the arm, its wrist camera and its files: arm 1 (default) uses
handeye_*, arm 2 uses handeye2_*.
"""

import argparse
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation

HERE = Path(__file__).parent
# Per arm: IP, wrist camera serial, file prefix.
ARMS = {
    1: ("10.42.0.230", "261922077925", "handeye"),
    2: ("10.43.0.53", "261922072855", "handeye2"),
}
IP, SERIAL, PREFIX = ARMS[1]
PLAN = HERE / f"{PREFIX}_plan.json"
DATA = HERE / f"{PREFIX}_data.npz"


def use_arm(n: int) -> None:
    global IP, SERIAL, PREFIX, PLAN, DATA
    IP, SERIAL, PREFIX = ARMS[n]
    PLAN = HERE / f"{PREFIX}_plan.json"
    DATA = HERE / f"{PREFIX}_data.npz"


BOARD = cv2.aruco.CharucoBoard(
    (8, 5), 0.030, 0.022, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
)
BOARD_CORNERS = np.array([[0, 0, 0], [0.24, 0, 0], [0.24, 0.15, 0], [0, 0.15, 0]])
W, H = 640, 480
MIN_CORNERS = 15

MAX_SPEED = 0.15  # rad/s, fastest joint
PERIOD = 0.01
N_POSES = 15

# RealSense body (camera_link) from optical axes, as dimOS's RealSense module uses.
B = Rotation.from_quat([-0.5, 0.5, -0.5, 0.5]).as_matrix()


def tf(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, np.ravel(t)
    return T


# ---------------------------------------------------------------- camera


class Camera:
    def __init__(self) -> None:
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_device(SERIAL)
        cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)
        cfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)
        prof = self.pipe.start(cfg)
        color = prof.get_stream(rs.stream.color).as_video_stream_profile()
        depth = prof.get_stream(rs.stream.depth).as_video_stream_profile()
        i = color.get_intrinsics()
        self.K = np.array([[i.fx, 0, i.ppx], [0, i.fy, i.ppy], [0, 0, 1]])
        self.dist = np.array(i.coeffs[:5])
        # camera_link (on the depth imager, body axes) -> colour optical frame,
        # built exactly as dimOS's RealSense module builds it.
        e = color.get_extrinsics_to(depth)
        R_o = np.array(e.rotation).reshape(3, 3).T  # librealsense: column-major
        t_o = np.array(e.translation)
        self.T_link_opt = tf(B @ R_o @ B.T, B @ t_o) @ tf(B, np.zeros(3))
        for _ in range(30):
            self.pipe.wait_for_frames(5000)

    def grab(self) -> np.ndarray:
        for _ in range(5):  # flush stale frames
            f = self.pipe.wait_for_frames(5000)
        return np.asanyarray(f.get_color_frame().get_data()).copy()

    def board_pose(self, img: np.ndarray) -> tuple[np.ndarray | None, int, float]:
        """Board pose in the colour optical frame (T_opt_board), corners found, reproj px."""
        corners, ids, _, _ = cv2.aruco.CharucoDetector(BOARD).detectBoard(img)
        n = 0 if ids is None else len(ids)
        if n < MIN_CORNERS:
            return None, n, float("nan")
        obj, imgp = BOARD.matchImagePoints(corners, ids)
        _, rvec, tvec = cv2.solvePnP(obj, imgp, self.K, self.dist, flags=cv2.SOLVEPNP_ITERATIVE)
        proj, _ = cv2.projectPoints(obj, rvec, tvec, self.K, self.dist)
        err = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - imgp.reshape(-1, 2)) ** 2, 1))))
        return tf(cv2.Rodrigues(rvec)[0], tvec), n, err

    def close(self) -> None:
        self.pipe.stop()


# ---------------------------------------------------------------- kinematics


class Model:
    """FK of fr3_hand from dimOS's own FR3 model, plus its collision check."""

    def __init__(self) -> None:
        from dimos.manipulation.planning.spec.validation import prepare_robot_model
        from dimos.manipulation.planning.world.drake_world import DrakeWorld
        from dimos.msgs.sensor_msgs.JointState import JointState
        from dimos.robot.manipulators.franka.config import make_fr3_model_config

        self.JointState = JointState
        self.config = make_fr3_model_config()
        self.world = DrakeWorld(enable_viz=False)
        self.world.load_model(prepare_robot_model(self.config))
        self.world.finalize()

    def _set(self, q) -> object:
        ctx = self.world.get_live_context()
        js = self.JointState({"name": self.config.joint_names, "position": list(q)})
        self.world.set_joint_state(ctx, js)
        return ctx, js

    def hand(self, q) -> np.ndarray:
        ctx, _ = self._set(q)
        return np.asarray(self.world.get_link_pose(ctx, "fr3_hand"))

    def tcp(self, q) -> np.ndarray:
        ctx, _ = self._set(q)
        return np.asarray(self.world.get_link_pose(ctx, "fr3_hand_tcp"))

    def collision_free(self, q) -> bool:
        _, js = self._set(q)
        return bool(self.world.check_config_collision_free(js))


def rough_hand_link() -> np.ndarray:
    from dimos.robot.manipulators.franka.config import fr3_wrist_camera_transform

    t = fr3_wrist_camera_transform()
    q = t.rotation
    return tf(
        Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix(),
        [t.translation.x, t.translation.y, t.translation.z],
    )


# ---------------------------------------------------------------- arm


class Arm:
    def __init__(self) -> None:
        from dimos.robot.manipulators.franka.connection import FrankaConnection
        from live_test import JOINTS, LocalTransport

        self.JOINTS = JOINTS
        self.module = FrankaConnection(ip=IP, deadman_timeout_s=0.5)
        ports = {n: LocalTransport() for n in {**self.module.inputs, **self.module.outputs}}
        for n, t in ports.items():
            self.module.set_transport(n, t)  # type: ignore[arg-type]
        self.state, self.cmd = ports["joint_state"], ports["position_command"]
        self.module.start()
        time.sleep(0.5)
        self.check()

    def q(self) -> list[float]:
        return list(self.state.latest[tuple(self.JOINTS)].position)

    def check(self) -> None:
        err = self.module.status().last_error
        if err:
            raise SystemExit(f"STOPPED: {err}")

    def move(self, goal) -> None:
        from dimos.msgs.sensor_msgs.JointState import JointState

        start = self.q()
        delta = [g - s for g, s in zip(goal, start)]
        dur = max(1.5, max(abs(d) for d in delta) / MAX_SPEED * math.pi / 2)
        steps = int(dur / PERIOD)
        for i in range(steps + 1):
            s = 0.5 - 0.5 * math.cos(math.pi * i / steps)
            self.cmd.publish(
                JointState(
                    name=list(self.JOINTS), position=[a + s * d for a, d in zip(start, delta)]
                )
            )
            time.sleep(PERIOD)
            if i % 50 == 0:
                self.check()
        for _ in range(80):  # hold while it settles
            self.cmd.publish(JointState(name=list(self.JOINTS), position=list(goal)))
            time.sleep(PERIOD)
        self.check()

    def follow(self, path) -> None:
        """Stream a dense joint path at MAX_SPEED (fastest joint), easing in and out."""
        from dimos.msgs.sensor_msgs.JointState import JointState

        pts = np.r_[np.array(self.q())[None], np.asarray(path)]
        seg = np.max(np.abs(np.diff(pts, axis=0)), axis=1)
        cum = np.r_[0, np.cumsum(seg)]
        total = cum[-1]
        if total < 1e-4:
            return
        dur = max(1.5, total / MAX_SPEED * math.pi / 2)
        steps = int(dur / PERIOD)
        for i in range(steps + 1):
            s = total * (0.5 - 0.5 * math.cos(math.pi * i / steps))
            q = [float(np.interp(s, cum, pts[:, j])) for j in range(7)]
            self.cmd.publish(JointState(name=list(self.JOINTS), position=q))
            time.sleep(PERIOD)
            if i % 50 == 0:
                self.check()
        for _ in range(80):
            self.cmd.publish(JointState(name=list(self.JOINTS), position=list(pts[-1])))
            time.sleep(PERIOD)
        self.check()

    def hold(self, seconds: float) -> None:
        """Stay put; with no commands the connection holds the arm where it is."""
        import time as _t

        end = _t.monotonic() + seconds
        while _t.monotonic() < end:
            _t.sleep(0.1)
            self.check()

    def close(self) -> None:
        self.module.stop()


def read_q_once() -> list[float]:
    import pylibfranka

    return list(pylibfranka.Robot(IP, pylibfranka.RealtimeConfig.kIgnore).read_once().q)


# ---------------------------------------------------------------- plan


def predict(model: Model, cam: Camera, T_base_board, T_hand_link, q):
    """Where the board's corners land in the image at q, its distance and tilt."""
    T_base_opt = model.hand(q) @ T_hand_link @ cam.T_link_opt
    T_opt_board = np.linalg.inv(T_base_opt) @ T_base_board
    pts = (T_opt_board @ np.c_[BOARD_CORNERS, np.ones(4)].T).T[:, :3]
    if np.any(pts[:, 2] <= 0.05):
        return None
    uv, _ = cv2.projectPoints(pts, np.zeros(3), np.zeros(3), cam.K, cam.dist)
    tilt = math.degrees(math.acos(abs(T_opt_board[2, 2])))
    return uv.reshape(-1, 2), float(np.mean(pts[:, 2])), tilt


def plan() -> None:
    q0 = read_q_once()
    model = Model()
    T_hand_link = rough_hand_link()
    lo = np.array([-2.9007, -1.8361, -2.9007, -3.077, -2.8763, 0.4398, -3.0508]) + 0.1
    hi = np.array([2.9007, 1.8361, 2.9007, -0.1169, 2.8763, 4.6216, 3.0508]) - 0.1
    span = np.radians([12, 10, 10, 10, 15, 15, 25])
    margin = 30
    cam = Camera()
    try:
        T_opt_board, n, err = cam.board_pose(cam.grab())
        if T_opt_board is None:
            raise SystemExit(
                f"board not detected well enough from here ({n} corners); centre it under the camera"
            )
        T_base_board = model.hand(q0) @ T_hand_link @ cam.T_link_opt @ T_opt_board
        table_z = T_base_board[2, 3]
        print(f"start (deg): {np.degrees(q0).round(1).tolist()}")
        print(
            f"board seen with {n} corners, {err:.2f} px; table about {table_z:+.3f} m from the base in z"
        )
        rng = np.random.default_rng(0)
        keep = []
        for _ in range(4000):
            q = np.array(q0) + rng.uniform(-1, 1, 7) * span
            if np.any(q < lo) or np.any(q > hi):
                continue
            pr = predict(model, cam, T_base_board, T_hand_link, q)
            if pr is None:
                continue
            uv, dist, tilt = pr
            if not (
                np.all(uv[:, 0] > margin)
                and np.all(uv[:, 0] < W - margin)
                and np.all(uv[:, 1] > margin)
                and np.all(uv[:, 1] < H - margin)
            ):
                continue
            if not (0.25 < dist < 0.65 and tilt < 35):
                continue
            if model.tcp(q)[2, 3] - table_z < 0.20:  # fingertips 20 cm above the table
                continue
            if not model.collision_free(q):
                continue
            keep.append(q)
    finally:
        cam.close()
    if len(keep) < N_POSES:
        raise SystemExit(
            f"only {len(keep)} usable poses found; move the board nearer the middle of the view"
        )

    # Greedy: spread the poses out in hand orientation, so the solve is well posed.
    rots = [Rotation.from_matrix(model.hand(q)[:3, :3]) for q in keep]
    chosen = [int(np.argmin([np.linalg.norm(q - q0) for q in keep]))]
    while len(chosen) < N_POSES:
        best = max(
            range(len(keep)),
            key=lambda i: min((rots[i].inv() * rots[j]).magnitude() for j in chosen),
        )
        chosen.append(best)
    poses = [keep[i] for i in chosen]
    # Visit them in an order that keeps each move short.
    order, rest = [0], list(range(1, len(poses)))
    while rest:
        nxt = min(rest, key=lambda i: np.max(np.abs(poses[i] - poses[order[-1]])))
        order.append(nxt)
        rest.remove(nxt)
    poses = [poses[i] for i in order]
    biggest = max(np.max(np.abs(p - q0)) for p in poses)
    spread = max(
        (rots[chosen[i]].inv() * rots[chosen[j]]).magnitude()
        for i in range(len(chosen))
        for j in range(len(chosen))
    )
    PLAN.write_text(json.dumps({"start": list(q0), "poses": [list(map(float, p)) for p in poses]}))
    print(
        f"{len(keep)} usable poses found; chose {len(poses)}, spread over "
        f"{math.degrees(spread):.0f} deg of hand rotation"
    )
    print(
        f"largest joint change from start: {math.degrees(biggest):.0f} deg; "
        f"about {len(poses) * 6 / 60:.0f}-{len(poses) * 9 / 60:.0f} min to run"
    )
    print(f"saved {PLAN.name}")


# ---------------------------------------------------------------- run


def run() -> None:
    p = json.loads(PLAN.read_text())
    start = p["start"]
    q_now = read_q_once()
    if max(abs(a - b) for a, b in zip(q_now, start)) > math.radians(1):
        raise SystemExit("the arm has moved since the plan was made; run `plan` again")
    cam = Camera()
    arm = Arm()
    qs, boards, imgs = [], [], []
    try:
        for i, goal in enumerate(p["poses"]):
            arm.move(goal)
            time.sleep(0.7)
            img = cam.grab()
            q = arm.q()
            T, n, err = cam.board_pose(img)
            status = f"{n} corners, {err:.2f} px" if T is not None else f"skipped ({n} corners)"
            print(f"  pose {i + 1:2d}/{len(p['poses'])}: {status}")
            if T is not None:
                qs.append(q)
                boards.append(T)
                imgs.append(img)
        print("returning to the start pose")
        arm.move(start)
    finally:
        arm.close()
        cam.close()
    np.savez(
        DATA,
        q=np.array(qs),
        T_opt_board=np.array(boards),
        T_link_opt=cam.T_link_opt,
        K=cam.K,
        dist=cam.dist,
    )
    for k, img in enumerate(imgs):
        cv2.imwrite(str(HERE / f"{PREFIX}_{k:02d}.png"), img)
    print(f"recorded {len(qs)} usable views; saved {DATA.name}")


# ---------------------------------------------------------------- solve


def solve() -> None:
    d = np.load(DATA)
    if len(d["q"]) < 8:
        raise SystemExit(f"only {len(d['q'])} usable views; need at least 8")
    model = Model()
    T_hand = [model.hand(q) for q in d["q"]]
    R_g = [T[:3, :3] for T in T_hand]
    t_g = [T[:3, 3] for T in T_hand]
    R_t = [T[:3, :3] for T in d["T_opt_board"]]
    t_t = [T[:3, 3] for T in d["T_opt_board"]]

    results = {}
    for name, method in [
        ("Tsai", cv2.CALIB_HAND_EYE_TSAI),
        ("Park", cv2.CALIB_HAND_EYE_PARK),
        ("Horaud", cv2.CALIB_HAND_EYE_HORAUD),
        ("Daniilidis", cv2.CALIB_HAND_EYE_DANIILIDIS),
    ]:
        R, t = cv2.calibrateHandEye(R_g, t_g, R_t, t_t, method=method)
        T_hand_opt = tf(R, t)
        # Consistency: the board is fixed, so every view should put it in the same place.
        boards = [Th @ T_hand_opt @ Tb for Th, Tb in zip(T_hand, d["T_opt_board"])]
        pos = np.array([b[:3, 3] for b in boards])
        rot_spread = max(
            (Rotation.from_matrix(a[:3, :3]).inv() * Rotation.from_matrix(b[:3, :3])).magnitude()
            for a in boards
            for b in boards
        )
        results[name] = (
            T_hand_opt,
            float(np.linalg.norm(pos - pos.mean(0), axis=1).max()),
            math.degrees(rot_spread),
        )
        print(
            f"{name:11s} board position spread {results[name][1] * 1000:5.1f} mm, "
            f"rotation spread {results[name][2]:.2f} deg"
        )

    best = min(results, key=lambda k: results[k][1])
    T_hand_opt = results[best][0]
    T_hand_link = T_hand_opt @ np.linalg.inv(d["T_link_opt"])
    q = Rotation.from_matrix(T_hand_link[:3, :3]).as_quat()
    rough = rough_hand_link()
    dt = np.linalg.norm(T_hand_link[:3, 3] - rough[:3, 3])
    dr = math.degrees(
        (
            Rotation.from_matrix(rough[:3, :3]).inv() * Rotation.from_matrix(T_hand_link[:3, :3])
        ).magnitude()
    )
    print(f"\nbest: {best}")
    print(f"fr3_hand -> camera_link translation (m): {T_hand_link[:3, 3].round(4).tolist()}")
    print(f"fr3_hand -> camera_link rotation (xyzw): {q.round(6).tolist()}")
    print(f"differs from the ruler estimate by {dt * 1000:.0f} mm and {dr:.1f} deg")
    (HERE / f"{PREFIX}_result.json").write_text(
        json.dumps(
            {
                "parent_frame": "fr3_hand",
                "child_frame": "camera_link",
                "method": best,
                "translation_m": T_hand_link[:3, 3].tolist(),
                "rotation_xyzw": q.tolist(),
                "board_position_spread_m": results[best][1],
                "board_rotation_spread_deg": results[best][2],
                "views": len(d["q"]),
            },
            indent=2,
        )
    )
    print(f"saved {PREFIX}_result.json")


# ---------------------------------------------------------------- refine


def refine() -> None:
    """Joint bundle adjustment of fr3_hand -> colour optical and base -> board,
    minimising the reprojection error of every detected corner in every view."""
    from scipy.optimize import least_squares

    d = np.load(DATA)
    res = json.loads((HERE / f"{PREFIX}_result.json").read_text())
    model = Model()
    K, dist = d["K"], d["dist"]
    detector = cv2.aruco.CharucoDetector(BOARD)
    views = []
    for k, q in enumerate(d["q"]):
        img = cv2.imread(str(HERE / f"{PREFIX}_{k:02d}.png"))
        corners, ids, _, _ = detector.detectBoard(img)
        obj, imgp = BOARD.matchImagePoints(corners, ids)
        views.append((model.hand(q), obj.reshape(-1, 3), imgp.reshape(-1, 2)))

    def pack(T):
        return np.r_[Rotation.from_matrix(T[:3, :3]).as_rotvec(), T[:3, 3]]

    def unpack(x):
        return tf(Rotation.from_rotvec(x[:3]).as_matrix(), x[3:6])

    T_hand_opt0 = (
        tf(Rotation.from_quat(res["rotation_xyzw"]).as_matrix(), res["translation_m"])
        @ d["T_link_opt"]
    )
    T_base_board0 = views[0][0] @ T_hand_opt0 @ d["T_opt_board"][0]

    def residuals(x):
        T_ho, T_bb = unpack(x[:6]), unpack(x[6:])
        out = []
        for T_bh, obj, imgp in views:
            T_ob = np.linalg.inv(T_bh @ T_ho) @ T_bb
            rv = cv2.Rodrigues(T_ob[:3, :3])[0]
            proj, _ = cv2.projectPoints(obj, rv, T_ob[:3, 3], K, dist)
            out.append((proj.reshape(-1, 2) - imgp).ravel())
        return np.concatenate(out)

    x0 = np.r_[pack(T_hand_opt0), pack(T_base_board0)]
    r0 = residuals(x0)
    sol = least_squares(residuals, x0, loss="huber", f_scale=1.0)
    r1 = sol.fun
    per_view = []
    i = 0
    for _, obj, _ in views:
        n = 2 * len(obj)
        per_view.append(float(np.sqrt(np.mean(r1[i : i + n] ** 2))))
        i += n
    T_ho = unpack(sol.x[:6])
    T_hand_link = T_ho @ np.linalg.inv(d["T_link_opt"])
    T_bb = unpack(sol.x[6:])
    # Uncertainty from the Jacobian (1 sigma), for the hand -> camera part.
    J = sol.jac
    sigma2 = np.sum(r1**2) / max(1, len(r1) - len(sol.x))
    cov = np.linalg.pinv(J.T @ J) * sigma2
    sd = np.sqrt(np.diag(cov))
    print(f"corners used: {len(r1) // 2} over {len(views)} views")
    print(
        f"reprojection RMS: {np.sqrt(np.mean(r0**2)):.2f} px before, {np.sqrt(np.mean(r1**2)):.2f} px after"
    )
    print(f"worst view RMS: {max(per_view):.2f} px")
    print(
        f"1-sigma: rotation {math.degrees(np.linalg.norm(sd[:3])):.2f} deg, translation {np.linalg.norm(sd[3:6]) * 1000:.1f} mm"
    )
    print(f"board origin in base (m): {T_bb[:3, 3].round(4).tolist()}")
    q = Rotation.from_matrix(T_hand_link[:3, :3]).as_quat()
    prev = tf(Rotation.from_quat(res["rotation_xyzw"]).as_matrix(), res["translation_m"])
    print(f"fr3_hand -> camera_link translation (m): {T_hand_link[:3, 3].round(4).tolist()}")
    print(f"fr3_hand -> camera_link rotation (xyzw): {q.round(6).tolist()}")
    print(
        f"moved {np.linalg.norm(T_hand_link[:3, 3] - prev[:3, 3]) * 1000:.1f} mm and "
        f"{math.degrees((Rotation.from_matrix(prev[:3, :3]).inv() * Rotation.from_matrix(T_hand_link[:3, :3])).magnitude()):.2f} deg from the closed-form solve"
    )
    (HERE / f"{PREFIX}_refined.json").write_text(
        json.dumps(
            {
                "parent_frame": "fr3_hand",
                "child_frame": "camera_link",
                "method": "bundle adjustment",
                "translation_m": T_hand_link[:3, 3].tolist(),
                "rotation_xyzw": q.tolist(),
                "reprojection_rms_px": float(np.sqrt(np.mean(r1**2))),
                "views": len(views),
            },
            indent=2,
        )
    )
    print(f"saved {PREFIX}_refined.json")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("step", choices=["plan", "run", "solve", "refine"])
    ap.add_argument("--arm", type=int, choices=sorted(ARMS), default=1)
    a = ap.parse_args()
    use_arm(a.arm)
    print(f"arm {a.arm}: {IP}, camera {SERIAL}, files {PREFIX}_*")
    {"plan": plan, "run": run, "solve": solve, "refine": refine}[a.step]()
