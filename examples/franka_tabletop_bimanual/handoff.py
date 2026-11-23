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

"""Hand an object from one FR3 to the other, through a running `fr3-tabletop-bimanual`.

Nothing about the object is known beforehand: the tripod camera finds what stands on
the table (tabletop.py), a wrist camera measures it close up, and the grasps,
handover and placement come from its box. Run the stages in order; each saves what
the next needs in handoff_state.json and stops with the arms holding still.

    cycle     all of the below, once (--relay: set it down in the middle instead)
    loop      cycles until Ctrl+C (or -n N), --wait seconds apart; logs handoff_log.jsonl
    look      the nearer arm puts its wrist camera over the object and measures it
    plan      no motion: the handover pose, and the grasp leading into it
    pick      that arm (the giver) grasps the end nearer itself from above, lifts it
    present   the giver brings it to the middle of the table, its free end toward
              the other arm, in the handover pose the search picks
    take      the receiver grasps the free end; the giver lets go and backs away
    place     the receiver sets it down somewhere free; both arms go home

    ~/dimos-franka/.venv/bin/python handoff.py look|pick|present|take|place [--dry]

--dry plans and checks a stage and prints it without moving anything.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

import tabletop
from tabletop import Model, TableObject

HERE = Path(__file__).parent
STATE = HERE / "handoff_state.json"
SIDES = ("right", "left")
READY = [0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398]
LO = np.array([-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159]) + 0.05
HI = np.array([2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159]) - 0.05
LOOK_HEIGHT = 0.35  # wrist camera above the object's centre, m
TABLE_TOP = 0.0  # world z of the tabletop, m
SPEED = 0.5  # share of the planner's speed for these moves
LINEAR_SPEED = 0.3  # share of the straight-line speed (10 cm/s) for straight moves
GRIP_IN = 0.022  # grasp this far in from the object's end, m (the pads are ~2 cm across)
# Fingertips below the TCP, m: 9.5 mm from franka_description, plus 1.75 mm measured on
# both arms (fingertips sent 3.0 cm above the camera-measured table stood at 31.75 mm).
FINGERTIP_REACH = 0.0095 + 0.00175
TIP_CLEARANCE = 0.002  # fingertips above the table (as measured under the object) at the grasp, m
PALM_BEHIND_TCP = 0.0374  # the Hand's body ends this far behind the TCP (franka_description), m
PALM_CLEARANCE = 0.005  # the palm stays this far above the object's top, m
APPROACH = 0.12  # pre-grasp and lift, straight up from the grasp, m
MAX_OPENING = 0.078  # the Franka Hand opens 80 mm


def other(side: str) -> str:
    return "left" if side == "right" else "right"


def load() -> dict:
    return json.loads(STATE.read_text()) if STATE.exists() else {}


def save(state: dict) -> None:
    STATE.write_text(json.dumps(state, indent=1))


class Failed(Exception):
    """A step did not do what it should; the arms are left where that step left them."""


class Arms:
    """Both arms through the running stack, planning in its shared world."""

    def __init__(self, dry: bool) -> None:
        from dimos.porcelain.dimos import Dimos

        self.dry = dry
        self.app = Dimos.connect()
        self.mm = self.app.ManipulationModule
        self.model = Model()

    def close(self) -> None:
        self.app.stop()

    def q(self, side: str) -> list[float]:
        joints = self.mm.get_state().groups[side].joints
        if joints is None:
            raise Failed(f"no joint reading from the {side} arm (is its connection down?)")
        return list(joints.position)

    def add_table(self) -> None:
        """The tabletop as a box, so no plan dips into it."""
        from dimos.msgs.geometry_msgs.Pose import Pose

        if "table" in self.mm.get_obstacles():
            self.mm.remove_obstacle("table")
        # Its top 1 cm under the tabletop: the arms' bases stand on the real one.
        pose = Pose(position=[0.45, 0.35, TABLE_TOP - 0.06], orientation=[0, 0, 0, 1])
        self.mm.add_obstacle("table", pose, "box", [1.6, 1.8, 0.1])
        print("added the tabletop to the planner as an obstacle")

    def finger(self, side: str) -> float:
        """The Hand's opening, m (both fingers)."""
        return 2.0 * float(
            self.app.ControlCoordinator.get_joint_positions()[f"{side}/fr3_finger_joint1"]
        )

    def gripper(self, side: str, opening: float, what: str) -> float:
        """Open (1.0) or close (0.0) a Hand and return its opening once it has stopped, m.
        Opening must reach nearly fully open; closing must visibly close."""
        print(f"  {side} gripper {what}")
        if self.dry:
            return 0.08 if opening > 0.5 else 0.0
        before = self.finger(side)
        refused = self.refused()
        self.mm.set_gripper_position(opening, side)
        now, settled_since = before, time.monotonic()
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:  # until the fingers stop
            time.sleep(0.2)
            w = self.finger(side)
            if abs(w - now) > 0.0005:
                now, settled_since = w, time.monotonic()
            elif time.monotonic() - settled_since > 0.6 and abs(now - before) > 0.002:
                break
        self.check(side, refused)  # a grasp can trip the arm's contact reflex
        if opening > 0.5 and now < 0.07:
            raise Failed(f"{side} Hand did not open (reads {now * 1000:.0f} mm)")
        if opening < 0.5 and before - now < 0.01:
            raise Failed(
                f"{side} Hand did not close (reads {now * 1000:.0f} mm, was {before * 1000:.0f})"
            )
        return now

    def refused(self) -> dict:
        """The guard's refusal counts, and each arm's latest error, to compare after a move."""
        out = dict(self.app.SeparationGuard.status()["refused"])
        for s in SIDES:
            out[f"{s}_error"] = str(self.app.get_module(f"{s}_arm").status().last_error or "")
        return out

    def check(self, side: str, before_refused: dict[str, int]) -> None:
        """Fail if the guard held either arm, or an arm connection faulted, since before."""
        if self.dry:
            return
        now = self.refused()
        held = {
            k: now[k] - before_refused.get(k, 0) for k in SIDES if now[k] > before_refused.get(k, 0)
        }
        if held:
            raise Failed(f"the separation guard held {held} targets during the {side} arm's move")
        for s in SIDES:
            err = now[f"{s}_error"]
            # A connection keeps its last error after it is cleared: only a new one counts.
            if "clear_errors" in err and err != before_refused.get(f"{s}_error"):
                raise Failed(f"the {s} arm faulted: {err}")

    def at(self, side: str, goal, tol_deg: float = 1.5) -> None:
        """Fail unless the arm's measured joints are at goal."""
        if self.dry:
            return
        err = np.degrees(np.abs(np.asarray(self.q(side)) - np.asarray(goal)).max())
        if err > tol_deg:
            raise Failed(f"the {side} arm is {err:.1f} deg from where it was sent")

    def tcp(self, side: str) -> np.ndarray:
        q = self.q(side)
        self.model.set(q, self.q("left")) if side == "right" else self.model.set(self.q("right"), q)
        return self.model.pose(f"{side}/fr3_hand_tcp")[:3, 3]

    def reset(self) -> None:
        """Both arms healthy, open and at ready, measured."""
        for s in SIDES:
            err = str(self.app.get_module(f"{s}_arm").status().last_error or "")
            if "clear_errors" in err:
                print(f"  clearing the {s} arm's fault: {err}")
                self.app.get_module(f"{s}_arm").clear_errors()
                time.sleep(1.0)
        for s in SIDES:
            if self.finger(s) < 0.07:
                self.gripper(s, 1.0, "open")
        for s in SIDES:
            if np.degrees(np.abs(np.asarray(self.q(s)) - READY).max()) > 1.0:
                self.move(s, READY, "to ready")

    def straight(self, side: str, dz: float, what: str, speed: float = LINEAR_SPEED) -> bool:
        print(f"  {side} {what}: straight {'down' if dz < 0 else 'up'} {abs(dz) * 100:.0f} cm")
        if self.dry:
            return True
        return self.along(side, [0.0, 0.0, dz], what, speed)

    def along(self, side: str, d, what: str, speed: float = LINEAR_SPEED) -> bool:
        """Straight-line move of the TCP by world vector d, m."""
        d = np.asarray(d, float)
        print(f"  {side} {what}: straight {np.linalg.norm(d) * 100:.0f} cm")
        if self.dry:
            return True
        start, refused = self.tcp(side), self.refused()
        r = self.mm.move_linear(
            dx=float(d[0]),
            dy=float(d[1]),
            dz=float(d[2]),
            planning_group=side,
            speed_scale=speed,
            blocking=True,
            timeout=30.0,
        )
        time.sleep(0.5)
        ok = "COMPLETED" in str(getattr(r, "execution", r))
        moved = self.tcp(side) - start
        miss = float(np.linalg.norm(moved - d))
        print(
            f"  {side} {what}: {'COMPLETED' if ok else r}; moved {np.linalg.norm(moved) * 100:.1f} cm, "
            f"{miss * 1000:.0f} mm off"
        )
        self.check(side, refused)
        if (
            ok
            and 0.003 < miss
            and float(np.linalg.norm(d)) > 0.01
            and not getattr(self, "_finishing", False)
        ):
            # Ended short without a fault: finish the rest of the line once.
            self._finishing = True
            try:
                return self.along(side, d - moved, f"{what} (the rest)", speed)
            finally:
                self._finishing = False
        if not ok or miss > 0.01:
            raise Failed(
                f"{side} {what}: the TCP ended {miss * 1000:.0f} mm from where it was sent"
            )
        return True

    def obstacle(self, name: str, T: np.ndarray, size) -> None:
        from dimos.msgs.geometry_msgs.Pose import Pose

        if name in self.mm.get_obstacles():
            self.mm.remove_obstacle(name)
        q = Rotation.from_matrix(T[:3, :3]).as_quat()
        self.mm.add_obstacle(
            name, Pose(position=list(T[:3, 3]), orientation=list(q)), "box", list(size)
        )

    def move(self, side: str, goal, what: str, speed: float = SPEED) -> bool:
        """Plan and execute to goal, then check the measured joints are there. If the arm
        ended short without a fault (one FR3's robot ends motions early, 2026-11-23),
        plan again from where it is, twice at most."""
        from dimos.msgs.sensor_msgs.JointState import JointState
        from dimos.robot.manipulators.franka.tabletop_bimanual import arm_joint_names

        for attempt in range(3):
            plan = self.mm.plan_to_joints(
                {side: JointState(name=arm_joint_names(side), position=list(goal))},
                speed_scale=speed,
            )
            print(
                f"  {side} {what}: plan {plan.status.name}"
                + (f" (try {attempt + 1})" if attempt else "")
            )
            if plan.status.name != "SUCCEEDED":
                print(f"    {plan.message}")
                raise Failed(f"{side} {what}: no plan")
            if self.dry:
                return True
            refused = self.refused()
            result = self.mm.execute(blocking=True, timeout=60.0)
            time.sleep(0.5)
            print(f"  {side} {what}: {result.status.name}")
            self.check(side, refused)
            if result.status.name != "COMPLETED":
                raise Failed(f"{side} {what}: {result.status.name}")
            short = float(np.degrees(np.abs(np.asarray(self.q(side)) - np.asarray(goal)).max()))
            if short <= 1.5:
                return True
            print(f"  {side} {what}: ended {short:.1f} deg short; planning the rest")
        self.at(side, goal)
        return True


# ---------------------------------------------------------------- kinematics


def solve(
    model: Model,
    side: str,
    others_q,
    frame_link: str,
    T_link_frame: np.ndarray,
    target: np.ndarray,
    seed,
    *,
    free_yaw: bool = False,
) -> np.ndarray | None:
    """Joints putting a frame fixed to ``frame_link`` at ``target`` (4x4), the other arm still.

    With ``free_yaw`` only the frame's z axis has to match (turning about it is free).
    """

    def pose(q):
        model.set(q, others_q) if side == "right" else model.set(others_q, q)
        return model.pose(frame_link) @ T_link_frame

    def res(q):
        T = pose(q)
        rot = (
            (T[:3, 2] - target[:3, 2]) * 3 if free_yaw else (T[:3, :3] - target[:3, :3]).ravel() * 2
        )
        return np.r_[(T[:3, 3] - target[:3, 3]) * 100, rot, 0.01 * (np.asarray(q) - READY)]

    best = None
    for s in seed:
        q = least_squares(res, s, bounds=(LO, HI)).x
        T = pose(q)
        if np.linalg.norm(T[:3, 3] - target[:3, 3]) > 0.003:
            continue
        if (
            np.linalg.norm(T[:3, 2] - target[:3, 2])
            if free_yaw
            else np.linalg.norm(T[:3, :3] - target[:3, :3])
        ) > 0.05:
            continue
        model.set(q, others_q) if side == "right" else model.set(others_q, q)
        if not model.world.is_collision_free(model.ctx) or lowest_part(model, side) < 0.003:
            continue  # a branch folded into itself or the other arm, or into the table
        if best is None or np.linalg.norm(q - READY) < np.linalg.norm(best - READY):
            best = q
    return best


def seeds(base, toward: float | None = None) -> list[np.ndarray]:
    """Starting guesses for the IK: where the arm is, ready, and (given the bearing of the
    target from the arm's base) natural reaching postures with the wrist rolled each way."""
    base = np.asarray(base, dtype=float)
    out = [base, np.asarray(READY)]
    if toward is not None:
        for j2, j4 in ((0.0, -2.0), (0.5, -1.6)):
            for j5, j7 in ((0.0, 0.785), (1.4, 0.785), (-1.4, 0.785), (0.0, -0.785), (0.0, 2.3)):
                out.append(np.array([toward, j2, 0.0, j4, j5, 1.9, j7]))
    for d in (-0.6, 0.6):
        s = np.asarray(READY).copy()
        s[0] += d
        out.append(s)
    for d in (-1.5, 1.5):
        s = np.asarray(READY).copy()
        s[6] += d
        out.append(s)
    return out


def camera_optical(side: str) -> np.ndarray:
    """fr3_hand -> the wrist camera's colour optical frame."""
    from dimos.robot.manipulators.franka.tabletop_bimanual import wrist_camera_transform

    T_link_opt = np.load(HERE / ("handeye_data.npz" if side == "right" else "handeye2_data.npz"))[
        "T_link_opt"
    ]
    return tabletop._transform(wrist_camera_transform(side)) @ T_link_opt


# ---------------------------------------------------------------- stages


def nearer_arm(o: TableObject) -> str:
    from dimos.robot.manipulators.franka.tabletop_bimanual import LEFT_BASE_XYZ

    d = {
        "right": np.hypot(*o.centre[:2]),
        "left": np.hypot(*(o.centre[:2] - np.array(LEFT_BASE_XYZ[:2]))),
    }
    return min(d, key=d.get)


def stage_look(arms: Arms, at: np.ndarray | None = None, side: str | None = None) -> None:
    arms.add_table()
    if at is None:
        # The tripod sees the whole table except under the arms; the wrist cameras, at
        # ready, see under their own hands. What more cameras agree on comes first.
        found, _ = tabletop.look(arms.app, arms.model)
        table = tabletop.table_outline(arms.app, arms.model)
        seen = [
            c
            for cam, (objs, _, _) in found.items()
            for c in objs
            if c.height > 0.015 and tabletop.inside(table, c.centre[None, :2], -0.03)[0]
        ]
        groups: list[list[TableObject]] = []
        for c in seen:
            g = next(
                (g for g in groups if np.hypot(*(g[0].centre[:2] - c.centre[:2])) < 0.06), None
            )
            groups.append([c]) if g is None else g.append(c)
        if not groups:
            # Depth sees nothing standing up: look for anything not the table's colour
            # (thin or flat things); the wrist camera then measures it close up.
            groups = [[c] for c in tabletop.colour_candidates(arms.app, arms.model)]
        if not groups:
            raise SystemExit("no camera sees anything on the table")
        groups.sort(
            key=lambda g: (-len({c.seen_by for c in g}), -max(c.length * c.width for c in g))
        )
        for g in groups:
            print(f"candidate seen by {sorted({c.seen_by for c in g})}: {g[0]}")
        tries = [(g[0], nearer_arm(g[0])) for g in groups[:4]]
    else:
        tries = [
            (
                TableObject(
                    centre=np.asarray(at, float),
                    yaw=0.0,
                    length=0.0,
                    width=0.0,
                    height=0.0,
                    points=0,
                    seen_by="given",
                ),
                side,
            )
        ]
    for o, side in tries:
        if look_at(arms, o, side):
            return
    raise SystemExit("no candidate turned out to be an object up close")


def look_at(arms: Arms, o: TableObject, side: str) -> bool:
    """Put that arm's wrist camera over a candidate and measure it; False if nothing is there."""
    import cv2

    print(
        f"looking at the {o.length * 100:.0f} cm object at ({o.centre[0]:.2f}, {o.centre[1]:.2f}) "
        f"with the {side} arm's wrist camera"
    )
    target = np.eye(4)
    target[:3, :3] = np.diag([1.0, -1.0, -1.0])  # optical z straight down
    target[:3, 3] = [o.centre[0], o.centre[1], TABLE_TOP + LOOK_HEIGHT]
    q = solve(
        arms.model,
        side,
        arms.q(other(side)),
        f"{side}/fr3_hand",
        camera_optical(side),
        target,
        seeds(arms.q(side), bearing(side, target[:3, 3])),
        free_yaw=True,
    )
    if q is None:
        print("  no pose puts the camera over it")
        return False
    arms.move(side, q, "to look")
    if arms.dry:
        return True
    time.sleep(1.0)
    found, images = tabletop.look(arms.app, arms.model, cameras=(f"{side}_wrist",))
    near = [m for m in found[f"{side}_wrist"][0] if np.hypot(*(m.centre[:2] - o.centre[:2])) < 0.08]
    if not near:
        print("  the wrist camera sees nothing there")
        return False
    m = max(near, key=lambda m: m.points)
    print(f"measured: {m}")
    objs, K, T = found[f"{side}_wrist"]
    cv2.imwrite(str(HERE / "handoff_look.png"), tabletop.draw(images[f"{side}_wrist"], [m], K, T))
    save(
        {
            "giver": side,
            "object": {
                "centre": m.centre.tolist(),
                "yaw": m.yaw,
                "length": m.length,
                "width": m.width,
                "height": m.height,
                "table_z": m.table_z,
            },
        }
    )
    print("saved handoff_state.json and handoff_look.png")
    return True


def grasp_pose(o: dict, giver: str, end: int | None = None) -> tuple[np.ndarray, np.ndarray]:
    """Where the giver's TCP grasps the object: near the end nearer its base, from above,
    fingers closing across the object, its camera (on the hand's +x) over the held end,
    away from the free end. Returns the TCP pose and the free end's direction."""
    from dimos.robot.manipulators.franka.tabletop_bimanual import LEFT_BASE_XYZ

    c = np.asarray(o["centre"][:2])
    a = np.array([math.cos(o["yaw"]), math.sin(o["yaw"])])
    base = np.zeros(2) if giver == "right" else np.array(LEFT_BASE_XYZ[:2])
    if end is None:  # the end nearer the giver's base
        out = a if np.linalg.norm(c + a - base) < np.linalg.norm(c - a - base) else -a
    else:
        out = a * end  # toward the held end
    g = c + out * (o["length"] / 2 - GRIP_IN)
    x = np.r_[out, 0.0]
    z = np.array([0.0, 0.0, -1.0])
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, :3] = np.c_[x, y, z]
    # Pads at mid-height, but never with the fingertips on the table, nor the palm on
    # the object's top (a tall object is gripped higher).
    up = max(
        o["height"] / 2,
        o["height"] + PALM_CLEARANCE - PALM_BEHIND_TCP,
        FINGERTIP_REACH + TIP_CLEARANCE,
    )
    T[:3, 3] = [g[0], g[1], o.get("table_z", TABLE_TOP) + up]
    return T, -np.r_[out, 0.0]


def stage_pick(arms: Arms) -> None:
    st = load()
    o, giver = st["object"], st["giver"]
    if o["width"] > MAX_OPENING - 0.01:
        raise SystemExit(f"{o['width'] * 100:.1f} cm wide: too wide for the Hand")
    T = np.array(st["plan"]["T_grasp"]) if "plan" in st else grasp_pose(o, giver)[0]
    free = -T[:3, 0]
    pre = T.copy()
    pre[2, 3] += APPROACH
    tcp = np.eye(4)
    q_pre = (
        np.array(st["plan"]["q_pre"])
        if "plan" in st and "q_pre" in st["plan"]
        else solve(
            arms.model,
            giver,
            arms.q(other(giver)),
            f"{giver}/fr3_hand_tcp",
            tcp,
            pre,
            seeds(arms.q(giver), bearing(giver, pre[:3, 3])),
        )
    )
    if q_pre is None:
        raise SystemExit("no pose reaches above the grasp")
    q_grasp = solve(
        arms.model,
        giver,
        arms.q(other(giver)),
        f"{giver}/fr3_hand_tcp",
        tcp,
        T,
        [q_pre, *seeds(q_pre, bearing(giver, T[:3, 3]))],
    )
    if q_grasp is None:
        raise SystemExit("no pose reaches the grasp")
    print(
        f"grasp at ({T[0, 3]:.3f}, {T[1, 3]:.3f}), TCP {T[2, 3] * 100:.1f} cm up, fingertips "
        f"{(T[2, 3] - FINGERTIP_REACH) * 100:.1f} cm above the table; the free end points "
        f"{math.degrees(math.atan2(free[1], free[0])):.0f} deg"
    )
    print(f"  above the grasp (deg): {np.degrees(q_pre).round(1).tolist()}")
    print(f"  at the grasp (deg):    {np.degrees(q_grasp).round(1).tolist()}")
    arms.gripper(giver, 1.0, "open")
    if not arms.move(giver, q_pre, "above the grasp"):
        return
    if not arms.straight(giver, -APPROACH, "down to the grasp"):
        return
    opening = arms.gripper(giver, 0.0, "close")
    if not arms.dry:
        print(
            f"  {giver} Hand stopped at {opening * 1000:.0f} mm (object {o['width'] * 1000:.0f} mm wide)"
        )
        if not 0.6 * o["width"] < opening < o["width"] + 0.015:
            arms.gripper(giver, 1.0, "open (missed)")
            arms.straight(giver, APPROACH, "back up")
            raise Failed("the Hand is not holding it")
    if not arms.straight(giver, APPROACH, "lift"):
        return
    st["held"] = {"T_grasp": T.tolist(), "free": free.tolist()}
    st["object"]["grip_dz"] = float(T[2, 3] - o["centre"][2])
    if not arms.dry:
        save(st)
    print("holding it 12 cm up")


# ---------------------------------------------------------------- handover search

HANDOVER_FORWARD = (0.42,)  # object centre, in front of the middle of the two bases, m
HANDOVER_HEIGHT = (0.36,)  # object centre above the table, m
HEADINGS = (
    0,
    -30,
    30,
    -60,
    60,
)  # the object's direction, turned from straight at the receiver, deg
TILTS = (0, 30, 60, 90)  # the free end raised toward the receiver, deg
ROLLS = (0, 90, 270)  # the object turned about its long axis, deg
GRIP_IN_RECEIVER = 0.025  # the receiver grasps this far in from the free end, m
APPROACH_TAKE = 0.10  # the receiver comes in along its approach from this far back, m
WRIST_TURN_MAX = math.radians(
    float(__import__("os").environ.get("WRIST_MAX_DEG", "170"))
)  # joints 5-7 within this of ready; the cables have slack for full wrist turns (2026-11-22)
MIN_PART_HEIGHT = 0.05  # every hand, camera and fingertip at least this far above the table, m


def wrist_ok(q) -> bool:
    return bool(np.all(np.abs(np.asarray(q)[4:7] - np.asarray(READY)[4:7]) <= WRIST_TURN_MAX))


def lowest_part(model: Model, side: str) -> float:
    """The lowest hand part of one arm, as posed in the model, above the table, m."""
    from dimos.robot.manipulators.franka.tabletop_bimanual import wrist_camera_link

    links = [f"{side}/fr3_{n}" for n in ("hand", "leftfinger", "rightfinger", "link7", "hand_tcp")]
    z = [model.pose(n)[2, 3] for n in links] + [model.pose(wrist_camera_link(side))[2, 3]]
    tip = model.pose(f"{side}/fr3_hand_tcp")
    z.append((tip @ np.r_[0, 0, FINGERTIP_REACH, 1])[2])
    return min(z) - TABLE_TOP


def bearing(side: str, p) -> float:
    """Joint 1's angle pointing that arm at world point p."""
    from dimos.robot.manipulators.franka.tabletop_bimanual import LEFT_BASE_RPY, LEFT_BASE_XYZ

    if side == "right":
        return math.atan2(p[1], p[0])
    d = np.asarray(p[:2]) - np.array(LEFT_BASE_XYZ[:2])
    return math.atan2(d[1], d[0]) - LEFT_BASE_RPY[2]


def frame(x, z) -> np.ndarray:
    x, z = np.asarray(x, float), np.asarray(z, float)
    return np.c_[x, np.cross(z, x), z]


def object_in_giver(o: dict) -> np.ndarray:
    """The held object's frame in the giver's TCP frame: x toward the held end, z up from
    the table at the pick, origin at its centre."""
    T = np.eye(4)
    # At the pick the TCP had x = toward the held end, z = down: object x = TCP x,
    # object z = -TCP z, object y = -TCP y; the centre is behind the TCP along -x, and
    # below it (along TCP +z) by however far above the centre the grasp was made.
    T[:3, :3] = np.diag([1.0, -1.0, -1.0])
    T[:3, 3] = [-(o["length"] / 2 - GRIP_IN), 0.0, o.get("grip_dz", 0.0)]
    return T


def receiver_grips(o: dict) -> list[tuple[str, np.ndarray, float]]:
    """Where the receiver may grasp, in the object's frame: its TCP pose at the free end
    approaching a face, and the size it closes across. Not the giver's face (top)."""
    p = np.array([-(o["length"] / 2 - GRIP_IN_RECEIVER), 0.0, 0.0])
    x = np.array([-1.0, 0.0, 0.0])  # the camera over the free end, away from the giver
    grips = []
    for name, approach, across, depth in (
        ("bottom", [0, 0, 1], o["width"], o["height"]),
        ("side +y", [0, -1, 0], o["height"], o["width"]),
        ("side -y", [0, 1, 0], o["height"], o["width"]),
    ):
        if across > MAX_OPENING - 0.01:
            continue
        a = np.asarray(approach, float)
        # Pads at the middle, unless the palm would go in through the face it enters by:
        # then only as deep as keeps the palm outside it.
        s = min(0.0, -depth / 2 + PALM_BEHIND_TCP - PALM_CLEARANCE)
        T = np.eye(4)
        T[:3, :3] = frame(x, approach)
        T[:3, 3] = p + s * a
        grips.append((name, T, across))
    return grips


def handover_search(arms: Arms, o: dict, giver: str, q_g0=None, q_r0=None) -> dict | None:
    from dimos.robot.manipulators.franka.separation_guard import SeparationGuard
    from dimos.robot.manipulators.franka.tabletop_bimanual import LEFT_BASE_XYZ

    receiver = other(giver)
    guard = SeparationGuard()
    guard._load_world()
    b = np.array(LEFT_BASE_XYZ[:2])
    u = b / np.linalg.norm(b)  # right base -> left base
    toward = u if giver == "right" else -u  # giver -> receiver
    fwd = np.array([u[1], -u[0]])
    T_obj_tcp = np.linalg.inv(object_in_giver(o))  # the giver's TCP in the object's frame
    q_g0 = np.array(arms.q(giver) if q_g0 is None else q_g0)
    q_r0 = np.array(arms.q(receiver) if q_r0 is None else q_r0)
    tcp = np.eye(4)
    found: list[dict] = []
    t0 = time.monotonic()
    poses = [
        (f, h, hd, tl, rl)
        for f in HANDOVER_FORWARD
        for h in HANDOVER_HEIGHT
        for hd in HEADINGS
        for tl in TILTS
        for rl in ROLLS
    ]
    grips = receiver_grips(o)
    # What has worked before for this giver comes first; then stop early.
    wins = load_wins(giver)
    poses.sort(key=lambda p: -wins.get(_key(p[2], p[3], p[4]), 0))
    for f, h, heading, tilt, roll in poses:
        if len(found) >= SEARCH_ENOUGH or (found and time.monotonic() - t0 > SEARCH_BUDGET_S):
            break
        if time.monotonic() - t0 > SEARCH_GIVE_UP_S:
            print("  giving up the search")
            break
        centre = np.r_[b / 2 + f * fwd, TABLE_TOP + h]
        c_, s_ = math.cos(math.radians(heading)), math.sin(math.radians(heading))
        aim = np.array([c_ * toward[0] - s_ * toward[1], s_ * toward[0] + c_ * toward[1]])
        t = math.radians(tilt)
        A = np.r_[math.cos(t) * aim, math.sin(t)]  # giver -> free end
        x_o = -A
        z0 = np.array([0, 0, 1.0]) - x_o[2] * x_o
        z0 /= np.linalg.norm(z0)
        z_o = Rotation.from_rotvec(x_o * math.radians(roll)).apply(z0)
        T_w_o = np.eye(4)
        T_w_o[:3, :3] = np.c_[x_o, np.cross(z_o, x_o), z_o]
        T_w_o[:3, 3] = centre
        T_g = T_w_o @ T_obj_tcp
        q_g = solve(
            arms.model,
            giver,
            q_r0,
            f"{giver}/fr3_hand_tcp",
            tcp,
            T_g,
            seeds(q_g0, bearing(giver, centre)),
        )
        if q_g is None or not wrist_ok(q_g):
            continue
        for name, T_o_r, across in sorted(
            grips, key=lambda g: -wins.get(_key(heading, tilt, roll, g[0]), 0)
        ):
            T_r = T_w_o @ T_o_r
            q_r = solve(
                arms.model,
                receiver,
                q_g,
                f"{receiver}/fr3_hand_tcp",
                tcp,
                T_r,
                seeds(q_r0, bearing(receiver, centre)),
            )
            if q_r is None:
                continue
            pre = T_r.copy()
            pre[:3, 3] -= APPROACH_TAKE * T_r[:3, 2]
            q_pre = solve(arms.model, receiver, q_g, f"{receiver}/fr3_hand_tcp", tcp, pre, [q_r])
            if q_pre is None or not (wrist_ok(q_r) and wrist_ok(q_pre)):
                continue
            low = []
            for qq in (q_r, q_pre):
                arms.model.set(*((q_g, qq) if giver == "right" else (qq, q_g)))
                low.append(min(lowest_part(arms.model, "right"), lowest_part(arms.model, "left")))
            if min(low) < MIN_PART_HEIGHT:
                continue

            # The guard judges every step of the straight approach and of the giver
            # backing out, not just the end: a plan it would stop midway leaves the
            # receiver stalled beside the giver.
            def pair(qg, qr):
                return (qg, qr) if giver == "right" else (qr, qg)

            steps, q_prev, clear = [], q_pre, True
            for k in (0.25, 0.5, 0.75):
                T_k = pre.copy()
                T_k[:3, 3] = pre[:3, 3] + k * (T_r[:3, 3] - pre[:3, 3])
                q_k = solve(
                    arms.model, receiver, q_g, f"{receiver}/fr3_hand_tcp", tcp, T_k, [q_prev]
                )
                if q_k is None:
                    clear = False
                    break
                steps.append(pair(q_g, q_k))
                q_prev = q_k
            T_back = T_g.copy()
            T_back[:3, 3] -= APPROACH_TAKE * T_g[:3, 2]
            q_back = solve(arms.model, giver, q_r, f"{giver}/fr3_hand_tcp", tcp, T_back, [q_g])
            if not clear or q_back is None:
                continue
            steps.append(pair(q_back, q_r))
            if (
                min(guard._gaps("right", list(x), list(y)).slack(guard.config) for x, y in steps)
                < 0.005
            ):
                continue
            qr_full = pair(q_g, q_r)
            gaps = guard._gaps("right", list(qr_full[0]), list(qr_full[1]))
            arms.model.set(*qr_full)
            if gaps.slack(guard.config) < 0.01 or not arms.model.world.is_collision_free(
                arms.model.ctx
            ):
                continue
            # Least motion: the largest single joint turn counts most, then the total.
            turns = np.r_[np.abs(q_g - q_g0), np.abs(q_r - q_r0)]
            found.append(
                dict(
                    cost=float(turns.max() + 0.2 * turns.sum()),
                    forward=f,
                    height=h,
                    tilt=tilt,
                    roll=roll,
                    grip=name,
                    heading=heading,
                    across=across,
                    q_giver=q_g.tolist(),
                    q_receiver=q_r.tolist(),
                    q_receiver_pre=q_pre.tolist(),
                    approach=T_r[:3, 2].tolist(),
                    T_w_o=T_w_o.tolist(),
                    T_o_r=T_o_r.tolist(),
                    grippers=gaps.grippers,
                    arms=gaps.arms,
                )
            )
            break  # one receiver grip per object pose is enough
    print(f"  {len(found)} feasible found in {time.monotonic() - t0:.0f} s")
    return min(found, key=lambda c: c["cost"]) if found else None


SEARCH_ENOUGH = 3  # stop once this many handovers are feasible
SEARCH_BUDGET_S = 20.0  # or after this long, once one is
SEARCH_GIVE_UP_S = 30.0  # and after this long with none, relay instead
WINS = HERE / "handover_wins.json"


def _key(*parts) -> str:
    return ",".join(str(p) for p in parts)


def load_wins(giver: str) -> dict[str, int]:
    """How often each handover pose (and pose with receiver grip) has worked, for this giver."""
    return json.loads(WINS.read_text()).get(giver, {}) if WINS.exists() else {}


def record_win(giver: str, h: dict) -> None:
    allw = json.loads(WINS.read_text()) if WINS.exists() else {}
    mine = allw.setdefault(giver, {})
    for k in (
        _key(h["heading"], h["tilt"], h["roll"]),
        _key(h["heading"], h["tilt"], h["roll"], h["grip"]),
    ):
        mine[k] = mine.get(k, 0) + 1
    WINS.write_text(json.dumps(allw, indent=1))


def stage_plan(arms: Arms) -> None:
    """No motion: the handover first, then the grasp that leads into it with least wrist turn."""
    st = load()
    o, giver = st["object"], st["giver"]
    o["grip_dz"] = float(grasp_pose(o, giver)[0][2, 3] - o["centre"][2])  # TCP above the centre
    st["object"] = o
    print("searching handover poses ...")
    best = handover_search(arms, o, giver, q_g0=READY, q_r0=READY)
    if best is None:
        raise SystemExit(
            "no direct handover fits this object; hand it over by setting it down instead"
        )
    tcp = np.eye(4)
    q_r = arms.q(other(giver))
    options = []
    for end in (1, -1):
        T, _ = grasp_pose(o, giver, end)
        pre = T.copy()
        pre[2, 3] += APPROACH
        q_pre = solve(
            arms.model,
            giver,
            q_r,
            f"{giver}/fr3_hand_tcp",
            tcp,
            pre,
            seeds(arms.q(giver), bearing(giver, T[:3, 3])),
        )
        if q_pre is None or not wrist_ok(q_pre):
            print(f"  grasping end {end:+d}: not reachable within the wrist limit")
            continue
        # The grasp itself, straight below, must be reachable too (from the pose above
        # first, so the straight move down stays on one branch).
        q_grasp = solve(
            arms.model,
            giver,
            q_r,
            f"{giver}/fr3_hand_tcp",
            tcp,
            T,
            [q_pre, *seeds(q_pre, bearing(giver, T[:3, 3]))],
        )
        if q_grasp is None:
            print(f"  grasping end {end:+d}: the grasp itself is not reachable")
            continue
        turn = float(np.abs(q_pre - np.array(best["q_giver"]))[4:7].max())
        print(
            f"  grasping end {end:+d}: wrist turns {math.degrees(turn):.0f} deg on to the handover"
        )
        options.append((turn, end, T, q_pre))
    if not options:
        raise SystemExit("neither end can be grasped within the wrist limit")
    turn, end, T, q_pre = min(options, key=lambda t: t[0])
    print(
        f"plan: grasp end {end:+d} at ({T[0, 3]:.3f}, {T[1, 3]:.3f}); hand over with the object "
        f"heading {best['heading']} deg, tilted {best['tilt']}, rolled {best['roll']}, "
        f"{best['height'] * 100:.0f} cm up; receiver grips the {best['grip']}"
    )
    print(f"  giver at the handover (deg): {np.degrees(best['q_giver']).round(1).tolist()}")
    print(f"  receiver at the handover (deg): {np.degrees(best['q_receiver']).round(1).tolist()}")
    st["plan"] = {"T_grasp": T.tolist(), "q_pre": q_pre.tolist(), "handover": best}
    save(st)
    print("saved the plan")


def stage_present(arms: Arms) -> None:
    st = load()
    o, giver = st["object"], st["giver"]
    if "held" not in st:
        raise SystemExit("nothing held; run pick first")
    best = st.get("plan", {}).get("handover")
    if best is None:
        print("searching handover poses ...")
        best = handover_search(arms, o, giver)
    if best is None:
        raise SystemExit(
            "no direct handover fits this object; hand it over by setting it down instead"
        )
    print(
        f"handover: object {best['forward'] * 100:.0f} cm out, {best['height'] * 100:.0f} cm up, tilted "
        f"{best['tilt']} deg, rolled {best['roll']} deg; receiver grips the {best['grip']} across "
        f"{best['across'] * 100:.1f} cm; grippers {best['grippers'] * 100:.1f} cm apart, rest "
        f"{best['arms'] * 100:.1f} cm"
    )
    print(f"  giver ({giver}) to (deg): {np.degrees(best['q_giver']).round(1).tolist()}")
    print(
        f"  receiver approach start (deg): {np.degrees(best['q_receiver_pre']).round(1).tolist()}"
    )
    if not arms.move(giver, best["q_giver"], "present"):
        return
    st["handover"] = best
    if not arms.dry:
        save(st)
    print("presenting it")


def stage_take(arms: Arms) -> None:
    st = load()
    o, giver = st["object"], st["giver"]
    receiver = other(giver)
    h = st.get("handover")
    if h is None:
        raise SystemExit("nothing presented; run present first")
    T_w_o = np.array(h["T_w_o"])
    # The free half of the object, as an obstacle while the receiver lines up (the
    # giver's fingers are on the other half). Removed before the receiver closes in.
    half = np.eye(4)
    half[:3, 3] = [-o["length"] / 4, 0.0, 0.0]
    arms.obstacle(
        "held_object", T_w_o @ half, [o["length"] / 2 + 0.02, o["width"] + 0.02, o["height"] + 0.02]
    )
    print("  added the object's free half to the planner")
    try:
        arms.gripper(receiver, 1.0, "open")
        if not arms.move(receiver, h["q_receiver_pre"], "lines up"):
            return
    finally:
        arms.mm.remove_obstacle("held_object")
    if not arms.along(receiver, APPROACH_TAKE * np.array(h["approach"]), "closes in"):
        return
    opening = arms.gripper(receiver, 0.0, "close")
    if not arms.dry:
        print(
            f"  {receiver} Hand stopped at {opening * 1000:.0f} mm (expected about {h['across'] * 1000:.0f})"
        )
        if not 0.6 * h["across"] < opening < h["across"] + 0.015:
            arms.gripper(receiver, 1.0, "open (missed)")
            arms.along(receiver, -APPROACH_TAKE * np.array(h["approach"]), "backs out")
            raise Failed("the receiver is not holding it; the giver still does")
    arms.gripper(giver, 1.0, "open")
    T_g = np.array(h["T_w_o"]) @ np.linalg.inv(object_in_giver(o))
    if not arms.along(giver, -APPROACH_TAKE * T_g[:3, 2], "backs out"):
        return
    arms.move(giver, READY, "to ready")
    st["receiver_holds"] = True
    if not arms.dry:
        save(st)
        record_win(giver, h)
    print(f"the {receiver} arm holds it")


PLACE_REACH = (0.30, 0.50)  # placements this far from the placing arm's base, m
PLACE_DROP = 0.002  # let go with the object this far above the table, m
MIDDLE_DEPTH = (0.30, 0.50)  # the relay spot, this far in front of the middle of the bases, m


def place_object(arms: Arms, side: str, o: dict, T_o_hand: np.ndarray, region: str) -> np.ndarray:
    """Set the held object down flat somewhere free, with the hand straight down, and
    back away; returns where (x, y). region: "own" (the placing arm's side of the
    table) or "middle" (where both arms reach, for a relay)."""
    from dimos.robot.manipulators.franka.tabletop_bimanual import LEFT_BASE_XYZ

    a_o = T_o_hand[:3, 2]  # the hand's approach, in the object's frame: that face goes down
    base = np.zeros(2) if side == "right" else np.array(LEFT_BASE_XYZ[:2])
    other_base = np.array(LEFT_BASE_XYZ[:2]) if side == "right" else np.zeros(2)
    middle = np.array(LEFT_BASE_XYZ[:2]) / 2
    fwd = np.array([LEFT_BASE_XYZ[1], -LEFT_BASE_XYZ[0]]) / np.hypot(*LEFT_BASE_XYZ[:2])
    half_h = 0.5 * float(np.abs(np.array([o["length"], o["width"], o["height"]]) @ np.abs(a_o)))
    tcp = np.eye(4)
    rng = np.random.default_rng()
    polys = tabletop.fixtures()
    table = tabletop.table_outline(arms.app, arms.model)
    footprint = math.hypot(o["length"], o["width"]) / 2 + 0.03
    R_align = Rotation.align_vectors([[0.0, 0.0, -1.0]], [a_o])[0]
    for _ in range(300):
        if region == "own":
            r, ang = rng.uniform(*PLACE_REACH), rng.uniform(-math.pi, math.pi)
            p = base + r * np.array([math.cos(ang), math.sin(ang)])
            if np.linalg.norm(p - other_base) < np.linalg.norm(p - base) + 0.15:  # its own side
                continue
        else:
            p = (
                middle
                + rng.uniform(*MIDDLE_DEPTH) * fwd
                + rng.uniform(-0.08, 0.08) * np.array([-fwd[1], fwd[0]])
            )
        if p[0] < 0.15:  # in front of the bases
            continue
        if any(tabletop.inside(poly, p[None], footprint)[0] for poly in polys):
            continue
        if not tabletop.inside(table, p[None], -(footprint + 0.03))[
            0
        ]:  # on the table, clear of its edge
            continue
        # Of a dozen turns about the vertical, the one keeping the wrist nearest mid-travel.
        best = None
        for yaw in np.linspace(-math.pi, math.pi, 12, endpoint=False) + rng.uniform(0, math.pi / 6):
            T_w_o = np.eye(4)
            T_w_o[:3, :3] = (Rotation.from_rotvec([0, 0, yaw]) * R_align).as_matrix()
            T_w_o[:3, 3] = [p[0], p[1], TABLE_TOP + half_h + PLACE_DROP]
            pre = T_w_o @ T_o_hand
            pre[2, 3] += APPROACH
            q = solve(
                arms.model,
                side,
                arms.q(other(side)),
                f"{side}/fr3_hand_tcp",
                tcp,
                pre,
                seeds(arms.q(side), bearing(side, pre[:3, 3])),
            )
            if q is None or not wrist_ok(q):
                continue
            margin = float(np.min(np.r_[q - LO, HI - q]))
            if best is None or margin > best[0]:
                best = (margin, yaw, q)
        if best is None:
            continue
        _, yaw, q_pre = best
        arms.model.set(*((q_pre, arms.q("left")) if side == "right" else (arms.q("right"), q_pre)))
        if lowest_part(arms.model, side) < APPROACH - 0.02:
            continue
        print(f"  place at ({p[0]:.3f}, {p[1]:.3f}), turned {math.degrees(yaw):.0f} deg")
        break
    else:
        raise Failed("no free, reachable spot to set it down")
    arms.move(side, q_pre, "above the spot")
    arms.straight(side, -APPROACH, "down")
    arms.gripper(side, 1.0, "open")
    arms.straight(side, APPROACH, "lift away")
    arms.move(side, READY, "to ready")
    return p


def stage_place(arms: Arms) -> None:
    st = load()
    if not st.get("receiver_holds"):
        raise SystemExit("the receiver holds nothing; run take first")
    receiver = other(st["giver"])
    p = place_object(arms, receiver, st["object"], np.array(st["handover"]["T_o_r"]), "own")
    if not arms.dry:
        st.pop("receiver_holds", None)
        st["placed"] = {"at": p.tolist()}
        save(st)
    print("placed")


def relay(arms: Arms) -> None:
    """No direct handover fits: the giver sets the object down in the middle and the
    receiver picks it up there and puts it on its own side."""
    st = load()
    giver, o = st["giver"], st["object"]
    receiver = other(giver)
    print(f"relay: the {giver} arm sets it down in the middle")
    T_o_hand = np.linalg.inv(object_in_giver(o))
    p = place_object(arms, giver, o, T_o_hand, "middle")
    stage_look(arms, at=np.r_[p, o["height"] / 2], side=receiver)
    st = load()
    st["giver"] = receiver
    st.pop("plan", None)
    save(st)
    stage_pick(arms)
    st = load()
    place_object(arms, receiver, st["object"], np.linalg.inv(object_in_giver(st["object"])), "own")


THIN = 0.02  # objects thinner than this across go by the relay: a mid-air grasp has no margin, m
SHORT = 0.09  # objects shorter than this go by the relay: no room for both grippers, m
RECOVER_CLEAR = 0.15  # lift an arm straight up until its TCP is this far above the table, m


def recover(arms: Arms) -> None:
    """After a fault and its reset in Desk: lift any arm left low straight up, then
    both to ready. Grippers are left as they are (one may still hold something)."""
    for s in SIDES:
        err = str(arms.app.get_module(f"{s}_arm").status().last_error or "")
        if "clear_errors" in err:
            print(f"  clearing the {s} arm's fault: {err}")
            arms.app.get_module(f"{s}_arm").clear_errors()
            time.sleep(1.0)
    arms.add_table()
    for s in SIDES:
        z = float(arms.tcp(s)[2]) - TABLE_TOP
        if z < RECOVER_CLEAR:
            arms.straight(s, RECOVER_CLEAR - z, f"lift clear (TCP {z * 100:.0f} cm up)")
    for s in SIDES:
        if np.degrees(np.abs(np.asarray(arms.q(s)) - READY).max()) > 1.0:
            arms.move(s, READY, "to ready")
    print("both arms at ready")


def cycle(arms: Arms, *, force_relay: bool = False) -> dict:
    """Find whatever is on the table, hand it to the other arm, and put it down there."""
    t0 = time.time()
    STATE.unlink(missing_ok=True)
    arms.reset()
    tabletop.refresh_scene(arms.app)
    stage_look(arms)
    st = load()
    out = {"giver": st["giver"], "object": st["object"]}
    o = st["object"]
    if o["width"] > MAX_OPENING - 0.01:
        raise Failed(
            f"{o['width'] * 100:.1f} cm wide: the Hand opens {MAX_OPENING * 100:.0f} cm; "
            "turn it so a narrower side faces up"
        )
    thin = min(o["width"], o["height"]) < THIN
    short = o["length"] < SHORT
    if thin:
        print(
            f"  {min(o['width'], o['height']) * 100:.1f} cm across: too thin to hand over in the air; relay"
        )
    elif short:
        print(f"  {o['length'] * 100:.1f} cm long: too short for two grippers at once; relay")
    thin = thin or short
    try:
        stage_plan(arms) if not (force_relay or thin) else None
        direct = not (force_relay or thin)
    except SystemExit as e:
        print(f"  {e}")
        direct = False
    if direct:
        stage_pick(arms)
        stage_present(arms)
        stage_take(arms)
        stage_place(arms)
    else:
        stage_pick(arms)
        relay(arms)
    out.update(kind="handover" if direct else "relay", seconds=round(time.time() - t0, 1))
    return out


def loop(arms: Arms, n: int, wait: float) -> None:
    log = HERE / "handoff_log.jsonl"
    done = 0
    try:
        while n <= 0 or done < n:
            print(f"=== cycle {done + 1} ===")
            try:
                result = cycle(arms)
                result["ok"] = True
            except (Failed, SystemExit) as e:
                result = {"ok": False, "error": str(e)}
            result["t"] = time.strftime("%H:%M:%S")
            with log.open("a") as f:
                f.write(json.dumps(result) + "\n")
            print(
                f"=== cycle {done + 1}: {'OK' if result['ok'] else 'FAILED: ' + result['error']} "
                f"({result.get('kind', '')}, {result.get('seconds', '')} s) ==="
            )
            done += 1
            if not result["ok"]:
                print("stopping: the arms are where the failed step left them")
                return
            time.sleep(wait)
    except KeyboardInterrupt:
        print("\nstopped by you; cancelling any motion")
        arms.mm.cancel()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "stage",
        choices=["look", "plan", "pick", "present", "take", "place", "cycle", "loop", "recover"],
    )
    ap.add_argument("--dry", action="store_true")
    ap.add_argument("--relay", action="store_true", help="cycle: hand over by setting it down")
    ap.add_argument("-n", type=int, default=0, help="loop: cycles to run (0: until Ctrl+C)")
    ap.add_argument("--wait", type=float, default=5.0, help="loop: seconds between cycles")
    a = ap.parse_args()
    arms = Arms(a.dry)
    try:
        if a.stage == "cycle":
            print(cycle(arms, force_relay=a.relay))
        elif a.stage == "recover":
            recover(arms)
        elif a.stage == "loop":
            loop(arms, a.n, a.wait)
        else:
            {
                "look": stage_look,
                "plan": stage_plan,
                "pick": stage_pick,
                "present": stage_present,
                "take": stage_take,
                "place": stage_place,
            }[a.stage](arms)
    except Failed as e:
        print(f"FAILED: {e}; the arms are where that step left them")
    finally:
        arms.close()


if __name__ == "__main__":
    main()
