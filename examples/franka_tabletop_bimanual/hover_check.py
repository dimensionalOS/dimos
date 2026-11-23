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

"""Camera-to-arm check: find a ChArUco corner with the wrist camera, then hover the
fingertips over it.

    plan  no motion: detect the board, pick the inner corner nearest the board's
          centre, solve joint targets 10 cm and 2 cm above it (gripper pointing
          straight down as it is now), check them, save hover_plan.json.
    go    move to 10 cm above, pause 3 s, lower to 2 cm above, hold 20 s,
          then go back up to 10 cm and back to the start pose.

The fingertips (fr3_hand_tcp) should end up over the corner, with the gap
looking like 2 cm.

    ~/dimos-franka/.venv/bin/python hover_check.py plan|go
"""

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares

from handeye import BOARD, Arm, Camera, Model, read_q_once, rough_hand_link

HERE = Path(__file__).parent
PLAN = HERE / "hover_plan.json"
HEIGHTS = (0.10, 0.05)  # m above the board: approach, then the check
LIMITS_LO = np.array([-2.9007, -1.8361, -2.9007, -3.077, -2.8763, 0.4398, -3.0508]) + 0.1
LIMITS_HI = np.array([2.9007, 1.8361, 2.9007, -0.1169, 2.8763, 4.6216, 3.0508]) - 0.1


def solve_ik(model: Model, q0: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Joints putting fr3_hand_tcp at target (4x4), starting from q0, staying near q0."""

    def err(q):
        T = model.tcp(q)
        dp = T[:3, 3] - target[:3, 3]
        # Only the approach axis (tcp z) must point the same way: the turn about
        # the vertical is free, so the wrist (and its camera cable) need not wind.
        dr = np.cross(T[:3, 2], target[:3, 2])
        # 1 mm and ~0.6 deg cost as much as 1 rad of joint motion: stay on the
        # target, then move the joints as little as possible.
        return np.r_[dp * 1000, dr * 100, q - q0]

    sol = least_squares(err, q0, bounds=(LIMITS_LO, LIMITS_HI), xtol=1e-10, ftol=1e-10)
    return sol.x


def plan() -> None:
    q0 = np.array(read_q_once())
    model = Model()
    cam = Camera()
    try:
        img = cam.grab()
        corners, ids, _, _ = cv2.aruco.CharucoDetector(BOARD).detectBoard(img)
        T_opt_board, n, err = cam.board_pose(img)
    finally:
        cam.close()
    if T_opt_board is None:
        raise SystemExit(
            f"board not detected well enough ({n} corners); put it back under the camera"
        )

    T_base_opt = model.hand(q0) @ rough_hand_link() @ cam.T_link_opt
    T_base_board = T_base_opt @ T_opt_board
    # The detected inner corner nearest the board's centre (0.12, 0.075) m.
    chess = BOARD.getChessboardCorners()
    seen = ids.ravel()
    k = int(seen[np.argmin(np.linalg.norm(chess[seen, :2] - [0.12, 0.075], axis=1))])
    p_corner = (T_base_board @ np.r_[chess[k], 1])[:3]

    T_tcp0 = model.tcp(q0)
    # Straight lines in space, solved every 1 cm, each seeded from the last:
    # sideways at the current height, then straight down.
    above = np.r_[p_corner[:2], T_tcp0[2, 3]]
    legs = [
        (T_tcp0[:3, 3], above),
        (above, p_corner + [0, 0, HEIGHTS[0]]),
        (p_corner + [0, 0, HEIGHTS[0]], p_corner + [0, 0, HEIGHTS[1]]),
    ]
    paths, q_prev = [], q0
    for a, b in legs:
        steps = max(2, int(np.ceil(np.linalg.norm(b - a) / 0.01)) + 1)
        path = []
        for t in np.linspace(0, 1, steps)[1:]:
            T = T_tcp0.copy()
            T[:3, 3] = a + t * (b - a)
            q = solve_ik(model, q_prev, T)
            got = model.tcp(q)
            if math.degrees(math.acos(np.clip(got[:3, 2] @ T[:3, 2], -1, 1))) > 0.5:
                raise SystemExit(f"gripper not pointing down at {T[:3, 3].round(3)}")
            if np.linalg.norm(got[:3, 3] - T[:3, 3]) > 0.001:
                raise SystemExit(f"no joint solution at {T[:3, 3].round(3)}")
            if np.max(np.abs(q - q_prev)) > math.radians(8):
                raise SystemExit(
                    f"joints jump {math.degrees(np.max(np.abs(q - q_prev))):.0f} deg between "
                    f"1 cm steps near {T[:3, 3].round(3)}; refusing"
                )
            if not model.collision_free(q):
                raise SystemExit(f"self-collision at {T[:3, 3].round(3)}")
            path.append(q)
            q_prev = q
        paths.append(path)
    qs = [p_[-1].tolist() for p_ in paths]
    deltas = np.degrees(np.array(qs) - q0)

    # Mark the chosen corner on the image for the record.
    uv = corners.reshape(-1, 2)[list(seen).index(k)].astype(int)
    cv2.drawMarker(img, tuple(uv), (0, 255, 0), cv2.MARKER_CROSS, 40, 2)
    cv2.imwrite(str(HERE / "hover_target.png"), img)

    move = np.max(np.abs(deltas))
    PLAN.write_text(
        json.dumps(
            {
                "start": q0.tolist(),
                "corner": p_corner.tolist(),
                "paths": [[q.tolist() for q in p_] for p_ in paths],
            }
        )
    )
    print(
        f"board: {n} corners, {err:.2f} px; target corner id {k} at base "
        f"({p_corner[0]:.3f}, {p_corner[1]:.3f}, {p_corner[2]:+.3f}) m"
    )
    print(
        f"fingertips now at ({T_tcp0[0, 3]:.3f}, {T_tcp0[1, 3]:.3f}, {T_tcp0[2, 3]:.3f}); "
        f"they will move {np.linalg.norm(above[:2] - T_tcp0[:2, 3]) * 100:.1f} cm sideways at this height, "
        f"then straight down to {HEIGHTS[0] * 100:.0f} cm and {HEIGHTS[1] * 100:.0f} cm above the board"
    )
    for name, d_ in zip(
        ["after sideways", f"at {HEIGHTS[0] * 100:.0f} cm", f"at {HEIGHTS[1] * 100:.0f} cm"], deltas
    ):
        print(f"  joint change {name:14s}: " + " ".join(f"{v:+4.0f}" for v in d_))
    print(
        f"largest joint change: {move:.0f} deg; {sum(len(p_) for p_ in paths)} path steps all checked; "
        "marked corner saved in hover_target.png"
    )


def go() -> None:
    p = json.loads(PLAN.read_text())
    if max(abs(a - b) for a, b in zip(read_q_once(), p["start"])) > math.radians(1):
        raise SystemExit("the arm has moved since the plan was made; run `plan` again")
    side, down, last = (np.array(x) for x in p["paths"])
    start = np.array(p["start"])
    arm = Arm()
    try:
        arm.follow(side)
        arm.follow(down)
        print(f"{HEIGHTS[0] * 100:.0f} cm above the corner; lowering in 3 s")
        arm.hold(3.0)
        arm.follow(last)
        print(
            f"{HEIGHTS[1] * 100:.0f} cm above the corner; holding 60 s. Measure the offset from "
            "the midpoint between the fingertips to the corner, and the gap."
        )
        arm.hold(60.0)
        arm.follow(np.r_[last[::-1][1:], down[-1:]])
        arm.follow(np.r_[down[::-1][1:], side[-1:]])
        arm.follow(np.r_[side[::-1][1:], start[None]])
        print("back at the start pose")
    finally:
        arm.close()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("step", choices=["plan", "go"])
    {"plan": plan, "go": go}[ap.parse_args().step]()
