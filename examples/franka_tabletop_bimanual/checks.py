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

"""Two checks of the bimanual calibration.

still        arm 1 moves to one of its calibration poses and, from the moment the
             move ends, films the big board for 3 s while logging joint speeds:
             does the board's pose in the camera drift while the arm settles?
             Then it moves back.
hover plan   no motion: where each hover goes, and its clearances.
hover 1      arm 1 puts its fingertips 5 cm above the chosen board corner, from
             its own measurement of the board, and stays there.
hover 2      arm 1 lifts and turns away; arm 2 puts its fingertips 5 cm above the
             same corner, from arm 1's measurement carried over by the arm 1 ->
             arm 2 calibration, and stays there.
hover done   arm 2 lifts and goes to ready, then arm 1 goes to ready.

~/dimos-franka/.venv/bin/python checks.py still|hover <step>
"""

import argparse
import json
import math
from pathlib import Path
import threading
import time

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

import bimanual_cal as bc
import handeye
from handeye import Model, tf

HERE = Path(__file__).parent
HOVER = HERE / "hover_plan.json"
ABOVE = 0.05  # fingertips above the corner, m
APPROACH = 0.15  # first stop above the corner, m


def a1_to_a2() -> np.ndarray:
    c = json.loads((HERE / "bimanual_cal.json").read_text())["arm1_to_arm2"]
    return tf(Rotation.from_quat(c["rotation_xyzw"]).as_matrix(), c["translation_m"])


def board_in_arm1(model: Model) -> np.ndarray:
    """The big board in arm 1's base, from arm 1's latest hand-eye run (15 views)."""
    d = np.load(HERE / "handeye_data.npz")
    T_hand_opt = bc.hand_link(1) @ bc.link_opt(1)
    T, dp, dr = bc.mean_pose(
        [model.hand(q) @ T_hand_opt @ Tb for q, Tb in zip(d["q"], d["T_opt_board"])]
    )
    return T


def topdown_ik(model: Model, target: np.ndarray, seed) -> np.ndarray:
    """Joints putting fr3_hand_tcp at target, gripper pointing straight down."""

    def res(q):
        T = model.tcp(q)
        return np.r_[(T[:3, 3] - target) * 100, (T[:3, 2] - [0, 0, -1]) * 3, 0.02 * (q - seed)]

    q = least_squares(res, seed, bounds=(bc.LO, bc.HI)).x
    err = np.linalg.norm(model.tcp(q)[:3, 3] - target)
    if err > 0.002:
        raise SystemExit(f"no top-down pose reaches {target.round(3)} (off by {err * 1000:.0f} mm)")
    return q


# ---------------------------------------------------------------- still


def still() -> None:
    from dimos.msgs.sensor_msgs.JointState import JointState

    p = json.loads((HERE / "handeye_plan.json").read_text())
    start, goal = p["start"], p["poses"][0]
    if max(abs(a - b) for a, b in zip(bc.read_q("10.42.0.230"), start)) > math.radians(1):
        raise SystemExit("arm 1 is not where its calibration plan started; not moving")
    handeye.use_arm(1)
    cam = handeye.Camera()
    arm = handeye.Arm()
    rows = []
    try:
        # The same move the calibration makes, but filming from the moment the
        # streamed path ends (the calibration waits ~1.5 s before its photo).
        q0 = arm.q()
        delta = [g - s for g, s in zip(goal, q0)]
        dur = max(1.5, max(abs(d) for d in delta) / handeye.MAX_SPEED * math.pi / 2)
        steps = int(dur / handeye.PERIOD)
        for i in range(steps + 1):
            s = 0.5 - 0.5 * math.cos(math.pi * i / steps)
            arm.cmd.publish(
                JointState(name=list(arm.JOINTS), position=[a + s * d for a, d in zip(q0, delta)])
            )
            time.sleep(handeye.PERIOD)
        stop = threading.Event()

        def hold() -> None:
            while not stop.is_set():
                arm.cmd.publish(JointState(name=list(arm.JOINTS), position=list(goal)))
                time.sleep(handeye.PERIOD)

        th = threading.Thread(target=hold, daemon=True)
        th.start()
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0:
            f = cam.pipe.wait_for_frames(5000)
            img = np.asanyarray(f.get_color_frame().get_data()).copy()
            t = time.monotonic() - t0
            st = arm.module._state
            dq = max(abs(v) for v in st.dq) if st is not None else float("nan")
            T, n, _ = cam.board_pose(img)
            if T is not None:
                rows.append((t, T, dq))
        stop.set()
        th.join()
        arm.check()
        arm.move(start)
    finally:
        arm.close()
        cam.close()
    final = rows[-1][1]
    print(" time after stop | joint speed (deg/s) | board moved vs last frame (mm, deg)")
    for t, T, dq in rows[::3]:
        dp = np.linalg.norm(T[:3, 3] - final[:3, 3]) * 1000
        dr = math.degrees(
            (
                Rotation.from_matrix(final[:3, :3]).inv() * Rotation.from_matrix(T[:3, :3])
            ).magnitude()
        )
        print(
            f"   {t:5.2f} s       |   {math.degrees(dq):6.2f}            |   {dp:5.2f} mm  {dr:5.3f} deg"
        )


# ---------------------------------------------------------------- hover


def hover_plan() -> None:
    from dimos.robot.manipulators.franka.config import FR3_READY

    model = Model()
    T_a1_b = board_in_arm1(model)
    T_a1_a2 = a1_to_a2()
    tripod = np.array(
        json.loads((HERE / "bimanual_cal.json").read_text())["arm1_to_tripod_optical"][
            "translation_m"
        ]
    )
    corners = (T_a1_b @ np.c_[handeye.BOARD_CORNERS, np.ones(4)].T).T[:, :3]
    ready = np.array(FR3_READY)
    park1 = ready.copy()
    park1[0] -= bc.PARK_TURN
    q1, q2 = np.array(bc.read_q("10.42.0.230")), np.array(bc.read_q("10.43.0.53"))
    a1, a2 = bc.Arm(1, model), bc.Arm(2, model)
    bc.ROUGH_A1_A2 = T_a1_a2
    seeds = json.loads(bc.PLAN.read_text())["arms"]
    best = None
    for k, c1 in enumerate(corners):
        c2 = (np.linalg.inv(T_a1_a2) @ np.r_[c1, 1])[:3]
        try:
            up1 = topdown_ik(model, c1 + [0, 0, APPROACH], np.array(seeds["1"][0]["q"]))
            low1 = topdown_ik(model, c1 + [0, 0, ABOVE], up1)
            up2 = topdown_ik(model, c2 + [0, 0, APPROACH], np.array(seeds["2"][0]["q"]))
            low2 = topdown_ik(model, c2 + [0, 0, ABOVE], up2)
        except SystemExit:
            continue
        moves = [  # (arm, from, to, other arm's pose)
            (1, q1, up1, q2),
            (1, up1, low1, q2),
            (1, low1, up1, q2),
            (1, up1, park1, q2),
            (2, q2, up2, park1),
            (2, up2, low2, park1),
            (2, low2, up2, park1),
            (2, up2, ready, park1),
            (1, park1, ready, ready),
        ]
        gaps = [bc.path_ok(model, a1 if n == 1 else a2, a, b, o) for n, a, b, o in moves]
        worst = -1.0 if min(gaps) < 0 else min(gaps)
        print(f"  corner {k}: closest the arms get {worst:.2f} m")
        if best is None or worst > best[0]:
            best = (worst, k, c1, c2, up1, low1, up2, low2)
    worst, k, c1, c2, up1, low1, up2, low2 = best
    # The layout is now measured (good to ~1 cm), not taped: 25 cm between link
    # frames still leaves well over 10 cm between the arms' surfaces.
    if worst < 0.25:
        raise SystemExit(f"best corner still comes within {worst:.2f} m; not saving")
    a2_base = T_a1_a2[:3, 3]
    side = "arm 2" if np.linalg.norm(c1[:2] - a2_base[:2]) < np.linalg.norm(c1[:2]) else "arm 1"
    near = (
        "nearest"
        if np.linalg.norm(c1[:2] - tripod[:2])
        <= np.median(np.linalg.norm(corners[:, :2] - tripod[:2], axis=1))
        else "farthest from"
    )
    d = np.load(HERE / "bimanual_data.npz")
    T_hand_opt2 = bc.hand_link(2) @ bc.link_opt(2)
    T_a2_b, _, _ = bc.mean_pose(
        [
            model.hand(q) @ T_hand_opt2 @ T
            for q, b, T in zip(d["a2_q"], d["a2_board"], d["a2_T_opt"])
            if b == "board"
        ]
    )
    own2 = (T_a2_b @ np.r_[handeye.BOARD_CORNERS[k], 1])[:3]
    print(
        f"chosen: the outer corner of the big board's pattern on the {side} side, {near} the tripod"
    )
    print(
        f"  arm 1 frame: {c1.round(4).tolist()}; arm 2 frame (via calibration): {c2.round(4).tolist()}"
    )
    print(f"  arm 2's own view puts it {np.linalg.norm(own2 - c2) * 1000:.1f} mm from that")
    print(f"closest the arms get over every move (link frames, measured layout): {worst:.2f} m")
    plan = {
        "1": {"up": up1.tolist(), "low": low1.tolist()},
        "2": {"up": up2.tolist(), "low": low2.tolist()},
        "start1": q1.tolist(),
        "start2": q2.tolist(),
        "park1": park1.tolist(),
        "corner_side": side,
    }
    HOVER.write_text(json.dumps(plan))
    print(f"saved {HOVER.name}")


def go(n: int, *goals) -> None:
    handeye.use_arm(n)
    arm = handeye.Arm()
    try:
        for g in goals:
            arm.move(g)
    finally:
        arm.close()


def hover(step: str) -> None:
    from dimos.robot.manipulators.franka.config import FR3_READY

    if step == "plan":
        return hover_plan()
    p = json.loads(HOVER.read_text())
    if step == "1":
        go(1, p["1"]["up"], p["1"]["low"])
        print("arm 1 is hovering")
    elif step == "2":
        go(1, p["1"]["up"], p["park1"])
        go(2, p["2"]["up"], p["2"]["low"])
        print("arm 2 is hovering")
    elif step == "done":
        go(2, p["2"]["up"], FR3_READY)
        go(1, FR3_READY)
        print("both at ready")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("check", choices=["still", "hover"])
    ap.add_argument("step", nargs="?", default="plan", choices=["plan", "1", "2", "done"])
    a = ap.parse_args()
    still() if a.check == "still" else hover(a.step)
