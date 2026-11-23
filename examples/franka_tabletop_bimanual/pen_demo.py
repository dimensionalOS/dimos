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

"""Pen demo: one FR3 holds a click pen upright, the other clicks it, the first draws a
smiley on the paper, the other clicks it again, and the pen goes back where it was.

The pen (blue body, silver tip, a button at the other end) and the paper are found with
the cameras wherever they are. The arm nearer the paper draws ("the drawer"); the other
clicks ("the clicker"). The drawer picks the pen across its barrel, so that held upright
the tip is below its hand and the button above it. If only the clicker can reach the pen,
it picks it and sets it down in the middle first.

    ~/dimos-franka/.venv/bin/python pen_demo.py look plan    # find, then plan (no motion in plan)
    ~/dimos-franka/.venv/bin/python pen_demo.py pick click touch draw click putdown
    ~/dimos-franka/.venv/bin/python pen_demo.py all

Each stage saves to pen_state.json and the next picks up from it.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np

import handoff as ho
from handoff import (
    APPROACH,
    FINGERTIP_REACH,
    MIN_PART_HEIGHT,
    READY,
    Arms,
    Failed,
    bearing,
    camera_optical,
    lowest_part,
    seeds,
    solve,
    wrist_ok,
)
import marker_demo as md
from marker_demo import AWAY, CARRY, FREE, LAST, NEAR, TOUCH, smiley
import tabletop as tt

HERE = Path(__file__).parent
STATE = HERE / "pen_state.json"

# The drawer grips the barrel this far from the tip, m. Held upright, its wrist camera
# (on the hand's +x, turned toward the tip so it is away from the clicker) reaches
# CAMERA_BELOW_TCP below the TCP: the tip has to reach lower, and the button stays
# above the hand for the clicker.
#
# Held at 45 deg to draw with the camera below (8.5 cm from the tip), the pen swivelled in
# the pads and wrote only faintly: a ballpoint wants to be upright. So the camera goes on
# the button side (PEN_E = -1: the hand's x toward the button), the pen is gripped nearer
# its tip so the button stands clear of the camera for the clicker, and it draws upright.
GRIP_FROM_TIP = 0.06
PEN_LENGTH = 0.14  # tip to the top of the button, m (measured by hand)
TIP_TOWARD_TRIPOD = True  # placement rule: the pen lies with its tip toward the tripod (+x)
PEN_E = -1
CAMERA_BELOW_TCP = 0.072  # from the wrist camera box in tabletop_bimanual, m
HAND_CLEAR = 0.01  # the camera at least this far over the paper while drawing, m
# Fingertips over the table at the pick, as commanded, m. The left arm's fingertips sat
# a couple of mm higher than commanded over the middle of the table (seen 2026-11-22), so
# it aims 2.5 mm lower.
TIP_CLEARANCE = {
    "right": 0.001,
    "left": -0.003,
}  # left 1.5 mm up from -3.5: two FSoE faults 4 s after grasps there (2026-11-23)
PEN_BASE_CLEARANCE = 0.22  # the tripod looks for the pen this far from the bases, m
CLICK_FORWARD = (0.34, 0.40)  # the held pen, in front of the middle of the bases, m
CLICK_HEIGHT = (0.22, 0.28)  # the drawer's grip on the pen above the table, m
HEADINGS = np.radians(np.arange(0, 360, 30))
CLICK_HEADINGS = np.radians(np.arange(0, 360, 60))  # coarser, for the clicker
COMFORT = 0.62  # m from the base: farther, the drawer would pick at the edge of its reach
RELAY_DISTANCE = (0.42, 0.52)  # the relay spot from the drawer's base, m
CLICK_FROM = 0.03  # the clicker starts this far over the button, m
CLICK_REACH = 0.05  # and pushes down at most this far, m
CLICK_FORCE = 12.0  # N: the button is all the way in
DRAW_LIFT = (
    0.025  # the tip this far over the paper before feeling for it (from the guessed length), m
)
DRAW_SEARCH = 0.05  # m to feel down for the paper
TOUCH_FORCE = 3.0
PRESS = 0.0025  # m below where the tip first touched: 1 mm drew only faintly (2026-11-23)
PEN_LIFT = 0.012
DRAW_SPEED = 0.04
LOOK_PEN = 0.30
# The pen demo runs the stack with the separation guard's arm margin lowered from 5 to
# 1.5 cm (--separationguard.arm_separation_m 0.015): clicking straight down puts the
# clicker's wrist camera over the drawer's hand, 2-5 cm away. Plans here use the same.
ARM_SEPARATION = 0.015
DRAW_TILT = 0.0  # the pen upright while drawing (at 45 deg it swivelled in the pads)
BUTTON_CAMERA_HIGH = 0.009  # the clicker's camera reads the pen's top this much high, m
STEP = 0.001  # m per step when feeling for contact while standing still
STEP_DETOUR = 0.02  # each step backs off this far first: a move turning every joint under 0.01 rad does not run (8 mm did not, for an outstretched arm)
TOUCH_MODEL_OFFSET = -0.015  # m: by eye, 2026-11-23: -10 mm drew still ~5 mm high
STEP_FORCE = 3.0  # N, read standing still (0.2 N of noise, ~2 N of drift after moves)
# How each arm clicks: feeling for the button in steps (right), or one straight press
# to CLICK_PRESS_BELOW under the button's top as the pen's geometry puts it (left: its
# robot ends small moves short since 2026-11-23, so steps drift).
CLICK_BY_STEPS = {
    "right": False,
    "left": False,
}  # steps end short on both arms now: one straight press
CLICK_PRESS_BELOW = 0.010  # m under the button's top (7 mm was a few mm short)
CLICK_AIM_OFFSET = (
    -0.008,
    0.0,
)  # m, world x, y: undoes the observed miss (with the tip-toward-tripod placement: 1 cm back from -18 mm)
CLICK_STROKE = 0.012  # m: the most the clicker pushes on from its first touch
CLICK_STOP = 20.0  # N, held for 3 readings: the button is all the way in
# Picking the pen: feel for the table with the fingertips, then rise this far and close.
FEEL_FROM = 0.015  # m over the expected fingertip height to start feeling
FEEL_REACH = 0.03  # m to feel down at most
FEEL_FORCE = 3.0  # N
GRIP_ABOVE_TABLE = 0.002  # fingertips this far over the table when closing, m
LOOK_PAPER = 0.42


def load() -> dict:
    return json.loads(STATE.read_text()) if STATE.exists() else {}


def save(st: dict) -> None:
    STATE.write_text(json.dumps(st, indent=1))


def other(side: str) -> str:
    return ho.other(side)


def base_of(side: str) -> np.ndarray:
    from dimos.robot.manipulators.franka.tabletop_bimanual import LEFT_BASE_XYZ

    return np.zeros(2) if side == "right" else np.array(LEFT_BASE_XYZ[:2])


# ---------------------------------------------------------------- perception


def find_pen(
    rgb, depth, K, T, plane, near=None, radius=0.15, base_clearance=PEN_BASE_CLEARANCE, table=None
):
    """A blue pen lying on the table: its tip (the silver end) and button end."""
    n, d = plane
    P, hit = md._pixels_on_plane(K, T, n, d, 0.005, rgb.shape[:2])
    xy = P[..., :2]
    ok = hit & ~(md.heights(depth, K, T, plane) > 0.04)
    if table is not None:  # on the tabletop, clear of its edge
        ok &= tt.inside(table, xy.reshape(-1, 2), -0.03).reshape(ok.shape)
    if near is not None:
        ok &= np.hypot(*(xy - near[:2]).transpose(2, 0, 1)) < radius
    if base_clearance:
        for side in ho.SIDES:
            ok &= np.hypot(*(xy - base_of(side)).transpose(2, 0, 1)) > base_clearance
        for poly in tt.fixtures():
            ok &= ~tt.inside(poly, xy.reshape(-1, 2), tt.COLOUR_FIXTURE_MARGIN).reshape(ok.shape)
    lab = cv2.cvtColor(rgb, cv2.COLOR_RGB2LAB).astype(float)
    ref = np.median(lab[ok], axis=0)
    L, A, B = lab[..., 0], lab[..., 1] - 128, lab[..., 2] - 128
    blue = ok & (B < (ref[2] - 128) - 18)
    blue = cv2.morphologyEx(blue.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    blue = cv2.morphologyEx(blue, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8)) > 0
    count, labels = cv2.connectedComponents(blue.astype(np.uint8))
    best = None
    for k in range(1, count):
        pts = xy[labels == k]
        if len(pts) < 15:
            continue
        c = pts.mean(0)
        a = np.linalg.eigh(np.cov((pts - c).T))[1][:, 1]
        length = np.ptp((pts - c) @ a)
        width = np.ptp((pts - c) @ np.array([-a[1], a[0]]))
        if not (0.06 < length < 0.18 and width < 0.035):
            continue
        score = len(pts) - (0 if near is None else 3000 * np.hypot(*(c - near[:2])))
        if best is None or score > best[0]:
            best = (score, pts, c, a, length)
    if best is None:
        return None
    _, body, c0, a0, length0 = best
    s = (xy - c0) @ a0
    off = np.abs((xy - c0) @ np.array([-a0[1], a0[0]]))
    chroma = np.hypot(A, B)
    ref_chroma = math.hypot(ref[1] - 128, ref[2] - 128)
    silver = (
        ok & (chroma < 0.5 * ref_chroma) & (off < 0.010) & (np.abs(s) < length0 / 2 + 0.04) & ~blue
    )
    silver_off = off[silver]  # each silver pixel's distance from the pen's line
    tip_px = xy[silver]
    pts = np.r_[body, tip_px] if len(tip_px) else body
    c = pts.mean(0)
    a = np.linalg.eigh(np.cov((pts - c).T))[1][:, 1]
    sb = (body - c) @ a
    if len(tip_px) < 8:
        return dict(found_tip=False, centre=c.tolist(), axis=a.tolist(), length=float(np.ptp(sb)))
    # The tip is silver sticking out past the blue; silver lying along the blue (the
    # clip) is at the button end.
    st = (tip_px - c) @ a
    lo, hi = np.percentile(sb, [1, 99])
    # The clip lies beside the body, off the pen's line, at the button end; the tip's
    # silver cone is on the line, past the blue. (Scoring silver past the blue alone was
    # fooled by the clip's shine: 1100 px of it.)
    on_line = silver_off < 0.0035
    # The clip stands out past the body's edge (radius ~5 mm): anything not the table's
    # colour 5.5-10 mm off the line, in each half.
    off_c = np.abs((xy - c) @ np.array([-a[1], a[0]]))
    s_c = (xy - c) @ a
    beside = (
        ok
        & (np.linalg.norm(lab - ref, axis=-1) > 25)
        & (off_c > 0.0055)
        & (off_c < 0.010)
        & (np.abs(s_c) < 0.08)
    )
    clip = {+1: int(np.sum(beside & (s_c > 0))), -1: int(np.sum(beside & (s_c < 0)))}
    past = {
        +1: int(np.sum(on_line & (st > hi + 0.003))),
        -1: int(np.sum(on_line & (st < lo - 0.003))),
    }
    # (Counting pixels beside the body did not single out the clip: the body itself spills
    # past 5.5 mm in the image. Only the silver past the blue decides; check pen_wrist.png.)
    score = past
    print(f"    pen ends: clip px {clip}, on-line silver past the blue {past}")
    if score[-1] > score[1]:
        a, sb, st = -a, -sb, -st
        lo, hi = -hi, -lo
    # The ends: everything along the pen's line that is not the table's colour (silver
    # tip, blue body, dark button), its two extremes. The silver decided which is the tip.
    diff = np.linalg.norm(lab - ref, axis=-1)
    off_a = np.abs((xy - c) @ np.array([-a[1], a[0]]))
    s_a = (xy - c) @ a
    line = ok & (diff > 25) & (off_a < 0.006) & (np.abs(s_a) < length0 / 2 + 0.06)
    line = cv2.morphologyEx(line.astype(np.uint8), cv2.MORPH_OPEN, np.ones((2, 2), np.uint8)) > 0
    sl = np.sort(s_a[line])
    # The longest run without a gap over 4 mm through the blue body: not stray specks.
    gaps = np.nonzero(np.diff(sl) > 0.004)[0]
    runs = np.split(sl, gaps + 1)
    run = max(runs, key=lambda r_: np.count_nonzero((r_ > lo) & (r_ < hi))) if len(sl) else sb
    # Which end is the tip, by shape: it tapers to a point (the silver cone); the button
    # end stays full width (and the clip widens it). The narrower last 12 mm is the tip.
    # (Silver past the blue alone, used above to point a, was wrong about half the time.)
    across_a = (xy - c) @ np.array([-a[1], a[0]])

    def end_width(lo_s, hi_s):
        sel = line & (s_a >= lo_s) & (s_a <= hi_s)
        v = across_a[sel]
        return float(np.ptp(np.percentile(v, [10, 90]))) if v.size >= 6 else float("nan")

    w_far = end_width(run[-1] - 0.012, run[-1])
    w_near = end_width(run[0], run[0] + 0.012)
    print(
        f"    pen ends: last 12 mm {w_far * 1000:.1f} mm wide (silver's tip end) vs {w_near * 1000:.1f} mm"
    )
    if np.isfinite(w_far) and np.isfinite(w_near) and w_far > w_near + 0.0015:
        a, run = -a, (-run)[::-1]
        print("    pen ends: the silver's tip end is the wider one: swapped")
    # The placement rule decides, not the looks: the pen is put down with its tip toward
    # the tripod (+x, the front of the table). Neither silver nor the ends' widths told
    # the ends apart reliably from above (wrong about half the time, 2026-11-23).
    if TIP_TOWARD_TRIPOD and a[0] < 0:
        a, run = -a, (-run)[::-1]
    button = c + a * run[0]
    tip = c + a * run[-1]
    if not 0.12 < run[-1] - run[0] < 0.17:  # implausible: one pen length from the button
        tip = button + a * PEN_LENGTH
    across = (body - c) @ np.array([-a[1], a[0]])
    return dict(
        found_tip=True,
        tip=tip.tolist(),
        button=button.tolist(),
        axis=a.tolist(),
        length=float(np.linalg.norm(tip - button)),
        tip_pixels=int(max(past.values())),
        diameter=float(np.clip(np.ptp(np.percentile(across, [3, 97])), 0.008, 0.016)),
        table_z=float((d - n[0] * c[0] - n[1] * c[1]) / n[2]),
    )


def reaches(side: str, p) -> bool:
    r = float(np.hypot(*(np.asarray(p[:2]) - base_of(side))))
    return 0.28 < r < 0.72


def stage_look(arms: Arms) -> None:
    app = arms.app
    arms.reset()  # both arms at ready first: neither hides the tripod's marker
    tt.refresh_scene(app)
    arms.add_table()
    rgb, depth, K, T, plane = md._frames(app, "scene", arms.model)
    pen0 = find_pen(rgb, depth, K, T, plane, table=tt.table_outline(app, arms.model))
    paper0 = md.find_paper(rgb, depth, K, T, plane)
    if pen0 is None or paper0 is None:
        raise Failed(f"the tripod does not see {'the pen' if pen0 is None else 'the paper'}")
    mid = (
        (np.asarray(pen0["tip"]) + np.asarray(pen0["button"])) / 2
        if pen0["found_tip"]
        else np.asarray(pen0["centre"])
    )
    print(
        f"tripod: pen {pen0['length'] * 100:.1f} cm at ({mid[0]:.3f}, {mid[1]:.3f})"
        f"{'' if pen0['found_tip'] else ' (tip not told apart)'}; paper at "
        f"({paper0['centre'][0]:.3f}, {paper0['centre'][1]:.3f})"
    )
    drawer = min(ho.SIDES, key=lambda s: np.hypot(*(np.asarray(paper0["centre"]) - base_of(s))))
    looker = drawer if np.hypot(*(mid - base_of(drawer))) <= COMFORT else other(drawer)
    md.look_with(arms, looker, mid, LOOK_PEN)
    joints = {s: arms.q(s) for s in ho.SIDES}
    rgb_w, depth_w, K_w, T_w, plane_w = md._frames(app, f"{looker}_wrist", arms.model, joints)
    pen = find_pen(rgb_w, depth_w, K_w, T_w, plane_w, near=mid, radius=0.15, base_clearance=0.0)
    if pen is None or not pen["found_tip"]:
        pen = pen0
    if not pen["found_tip"]:
        arms.move(looker, READY, "to ready", FREE)
        raise Failed("cannot tell the pen's tip from its button")
    if looker:
        img = cv2.cvtColor(rgb_w, cv2.COLOR_RGB2BGR).copy()
        Ti = np.linalg.inv(T_w)
        for key, colour in (("tip", (0, 0, 255)), ("button", (255, 0, 0))):
            q = Ti @ np.r_[pen[key], tt.TABLE_Z + 0.005, 1.0]
            cv2.circle(img, tuple(int(x) for x in (K_w @ (q[:3] / q[2]))[:2]), 7, colour, 2)
        g = np.asarray(pen["tip"]) - np.asarray(pen["axis"]) * GRIP_FROM_TIP
        q = Ti @ np.r_[g, tt.TABLE_Z + 0.005, 1.0]
        cv2.drawMarker(
            img,
            tuple(int(x) for x in (K_w @ (q[:3] / q[2]))[:2]),
            (0, 255, 0),
            cv2.MARKER_CROSS,
            20,
            2,
        )
        cv2.imwrite(str(HERE / "pen_wrist.png"), img)
        print("saved pen_wrist.png (red: tip, blue: button, green cross: where it will grip)")
    print(
        f"{looker} wrist: pen {pen['length'] * 100:.1f} cm, {pen['diameter'] * 1000:.0f} mm thick; tip "
        f"({pen['tip'][0]:.3f}, {pen['tip'][1]:.3f}) ({pen['tip_pixels']} silver px), button "
        f"({pen['button'][0]:.3f}, {pen['button'][1]:.3f})"
    )
    arms.move(looker, READY, "to ready", FREE)
    md.look_with(arms, drawer, paper0["centre"], LOOK_PAPER)
    joints = {s: arms.q(s) for s in ho.SIDES}
    rgb_p, depth_p, K_p, T_p, plane_p = md._frames(app, f"{drawer}_wrist", arms.model, joints)
    paper = (
        md.find_paper(
            rgb_p, depth_p, K_p, T_p, plane_p, near=np.asarray(paper0["centre"]), radius=0.3
        )
        or paper0
    )
    print(
        f"{drawer} wrist: paper {paper['size'][0] * 100:.1f} x {paper['size'][1] * 100:.1f} cm at "
        f"({paper['centre'][0]:.3f}, {paper['centre'][1]:.3f}), top {paper['top_z'] * 1000:.0f} mm up"
    )
    arms.move(drawer, READY, "to ready", FREE)
    st = {"pen": pen, "paper": paper, "drawer": drawer}
    save(st)
    draw_scene(arms, st)


def draw_scene(arms: Arms, st: dict) -> None:
    rgb, _, K, T, _ = md._frames(arms.app, "scene", arms.model)
    img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR).copy()
    Tinv = np.linalg.inv(T)

    def px(p):
        q = Tinv @ np.r_[p, 1.0]
        return tuple(int(v) for v in (K @ (q[:3] / q[2]))[:2])

    pen = st["pen"]
    z = 0.005
    cv2.line(img, px(np.r_[pen["button"], z]), px(np.r_[pen["tip"], z]), (0, 255, 0), 2)
    cv2.circle(img, px(np.r_[pen["tip"], z]), 5, (0, 0, 255), -1)
    for stroke in smiley(st["paper"]):
        cv2.polylines(
            img,
            [np.array([px(np.r_[p, st["paper"]["top_z"]]) for p in stroke])],
            False,
            (255, 0, 255),
            2,
        )
    cv2.imwrite(str(HERE / "pen_scene.png"), img)
    print("saved pen_scene.png (green: pen, red dot: its tip, magenta: the smiley)")


# ---------------------------------------------------------------- geometry


def pen_in_hand(pen: dict, e: int, side: str = "left") -> tuple[np.ndarray, float]:
    """The pen's tip and the drawer's TCP. At the pick the hand points down with its x
    along the pen, toward the tip if e = +1 (else toward the button), gripping
    GRIP_FROM_TIP from the tip. Returns the tip in the TCP frame and the TCP's height."""
    up = max(pen["diameter"] / 2, FINGERTIP_REACH + TIP_CLEARANCE[side])
    below = up - pen["diameter"] / 2  # the pen's axis under the TCP (along TCP z)
    return np.array([e * GRIP_FROM_TIP, 0.0, below]), up


def pick_pose(pen: dict, e: int, side: str = "left") -> np.ndarray:
    a = np.r_[pen["axis"], 0.0]  # toward the tip
    g = np.asarray(pen["tip"]) - np.asarray(pen["axis"]) * GRIP_FROM_TIP
    _, up = pen_in_hand(pen, e, side)
    T = np.eye(4)
    T[:3, :3] = ho.frame(e * a, [0.0, 0.0, -1.0])
    T[:3, 3] = [g[0], g[1], tt.TABLE_Z + up]
    return T


def upright(pen: dict, e: int, grip_point, heading: float) -> np.ndarray:
    """The drawer's TCP with the pen upright, tip down, its axis through grip_point (the
    point gripped), the hand's approach horizontal at ``heading``."""
    tip_in_tcp, _ = pen_in_hand(pen, e)
    x = np.array([0.0, 0.0, -1.0]) * e  # TCP x toward the tip (e=+1) -> down
    z = np.array([math.cos(heading), math.sin(heading), 0.0])
    T = np.eye(4)
    T[:3, :3] = ho.frame(x, z)
    # The axis point beside the TCP is TCP + below * z; put it at grip_point.
    T[:3, 3] = np.asarray(grip_point) - tip_in_tcp[2] * z
    return T


def tilted(pen: dict, e: int, tip_point, heading: float, tilt: float) -> np.ndarray:
    """The drawer's TCP with the pen leaning ``tilt`` from vertical (its tip at tip_point,
    the button back toward ``heading`` + 180 deg), the hand's back (and camera) raised."""
    tip_in_tcp, _ = pen_in_hand(pen, e)
    down = np.array([0.0, 0.0, -1.0])
    h = np.array([math.cos(heading), math.sin(heading), 0.0])
    t = math.cos(tilt) * down + math.sin(tilt) * h  # toward the tip
    x = e * t
    z = math.sin(tilt) * down - math.cos(tilt) * h  # so -z (the hand's back) points up
    if e < 0:
        z = -z
    T = np.eye(4)
    T[:3, :3] = ho.frame(x, z)
    T[:3, 3] = np.asarray(tip_point) - T[:3, :3] @ tip_in_tcp
    return T


def step_touch(
    arms: Arms, side: str, d, steps: int, what: str, step: float = STEP
) -> tuple[bool, float]:
    """Move the TCP along unit d a step at a time, reading the force standing still after
    each, until it passes STEP_FORCE. Returns (touched, how far it went)."""
    d = np.asarray(d, float)
    J = md.jacobian(arms, side, arms.q(side))
    start = arms.tcp(side)

    def detour(step: float) -> None:
        r = arms.mm.move_linear_path(
            waypoints=[tuple(-STEP_DETOUR * d), tuple(step * d)],
            planning_group=side,
            speed_scale=0.5,
            blocking=True,
            timeout=20.0,
        )
        if not r.succeeded:
            raise Failed(f"{side} {what}: a step did not run ({r})")
        time.sleep(0.6)

    # The reference after one up-and-back in free air: a move leaves ~2 N of drift.
    detour(0.0)
    tau0 = md.torques(arms, side, 10)
    print(
        f"  {side} {what}: up to {steps} steps of {step * 1000:.0f} mm, stopping at {STEP_FORCE} N"
    )
    for k in range(1, steps + 1):
        detour(step)  # each step on from where the last ended
        F = np.linalg.lstsq(J.T, md.torques(arms, side, 10) - tau0, rcond=None)[0]
        along = abs(float(F @ d))
        went = float((arms.tcp(side) - start) @ d)
        print(f"    step {k}: {went * 1000:5.1f} mm in, {along:4.1f} N")
        if along > STEP_FORCE:
            print(f"  {side} {what}: touched at step {k}, {went * 1000:.1f} mm in ({along:.1f} N)")
            return True, went
    print(f"  {side} {what}: no contact after {went * 1000:.1f} mm")
    return False, went


def push_hold(arms: Arms, side: str, d, force: float, what: str) -> tuple[bool, float]:
    """One continuous straight push by d; stop early if the force stays over ``force`` for
    3 readings after the first 0.3 s (the start's own acceleration reads high)."""
    d = np.asarray(d, float)
    unit = d / np.linalg.norm(d)
    J = md.jacobian(arms, side, arms.q(side))
    tau0 = md.torques(arms, side, 10)
    start, refused = arms.tcp(side), arms.refused()
    r = arms.mm.move_linear(
        dx=float(d[0]),
        dy=float(d[1]),
        dz=float(d[2]),
        planning_group=side,
        speed_scale=0.1,
        blocking=False,
    )
    if not r.plan.succeeded:
        raise Failed(f"{side} {what}: no straight path")
    t0, over, stopped = time.monotonic(), 0, False
    while time.monotonic() - t0 < 10.0:
        F = np.linalg.lstsq(J.T, md.torques(arms, side) - tau0, rcond=None)[0]
        over = over + 1 if (time.monotonic() - t0 > 0.3 and abs(float(F @ unit)) > force) else 0
        if over >= 3:
            arms.mm.cancel()
            stopped = True
            break
        if float((arms.tcp(side) - start) @ unit) > np.linalg.norm(d) - 0.0005:
            break
    time.sleep(0.4)
    arms.check(side, refused)
    return stopped, float((arms.tcp(side) - start) @ unit)


def tip_of(T_tcp: np.ndarray, pen: dict, e: int) -> np.ndarray:
    return (T_tcp @ np.r_[pen_in_hand(pen, e)[0], 1.0])[:3]


# The clicker presses with the flat outer side of its closed fingers (gripper held
# sideways, the fingers stacked one over the other): about 2 x 2 cm, so a button a few mm
# off still lands on it (the fingertips missed by half a centimetre). The lower finger's
# outer face is about this far below the TCP.
CLICK_SIDE = False  # fingertips pointing down: what worked (2026-11-23)
FINGER_SIDE = 0.010  # m


def clicker_pose(button_top, heading: float, roll: int, lift: float) -> np.ndarray:
    """The clicker's TCP over the button with its pressing face ``lift`` over the
    button's top (negative: pushed in), turned to ``heading``. Sideways (CLICK_SIDE):
    approach horizontal at ``heading``, fingers stacked vertically, the lower finger's
    outer face down. Else pointing down, fingertips down. (``roll`` is unused.)"""
    T = np.eye(4)
    up = np.array([0.0, 0.0, 1.0])
    if CLICK_SIDE:
        z = np.array([math.cos(heading), math.sin(heading), 0.0])
        T[:3, :3] = ho.frame(np.cross(up, z), z)  # y = z x x = up
        T[:3, 3] = np.asarray(button_top) + up * (FINGER_SIDE + lift)
    else:
        T[:3, :3] = ho.frame([math.cos(heading), math.sin(heading), 0.0], [0.0, 0.0, -1.0])
        T[:3, 3] = np.asarray(button_top) + up * (FINGERTIP_REACH + lift)
    return T


def click_poses(arms: Arms, d: str, q_d, button_top) -> dict | None:
    """The clicker's joints 10 cm over, just over (CLICK_FROM), and pushed 8 mm into the
    button, with the drawer at q_d: every one inside the guard's limits."""
    from dimos.robot.manipulators.franka.separation_guard import SeparationGuard

    c = other(d)
    guard = SeparationGuard(arm_separation_m=ARM_SEPARATION)
    guard._load_world()
    tcp = np.eye(4)
    best = None
    for hc in HEADINGS:
        q_c = solve(
            arms.model,
            c,
            q_d,
            f"{c}/fr3_hand_tcp",
            tcp,
            clicker_pose(button_top, hc, 0, CLICK_FROM),
            seeds(READY, bearing(c, button_top)),
        )
        if q_c is None or not wrist_ok(q_c):
            continue
        q_h = solve(
            arms.model,
            c,
            q_d,
            f"{c}/fr3_hand_tcp",
            tcp,
            clicker_pose(button_top, hc, 0, CLICK_FROM + 0.10),
            [q_c],
        )
        q_p = solve(
            arms.model,
            c,
            q_d,
            f"{c}/fr3_hand_tcp",
            tcp,
            clicker_pose(button_top, hc, 0, -0.008),
            [q_c],
        )
        if q_h is None or q_p is None:
            continue
        pairs = [(q, q_d) if c == "right" else (q_d, q) for q in (q_h, q_c, q_p)]
        slack = min(guard._gaps("right", list(a), list(b)).slack(guard.config) for a, b in pairs)
        if slack < 0.003:  # beyond the guard's own margin (the sideways press keeps ~5 mm)
            continue
        cost = float(np.abs(q_h - READY).max())
        if best is None or cost < best["cost"]:
            best = dict(
                cost=cost,
                heading=float(hc),
                q_high=q_h.tolist(),
                q_clicker=q_c.tolist(),
                slack=slack,
            )
    return best


# ---------------------------------------------------------------- planning


def stage_plan(arms: Arms) -> None:
    from dimos.robot.manipulators.franka.separation_guard import SeparationGuard
    from dimos.robot.manipulators.franka.tabletop_bimanual import LEFT_BASE_XYZ

    st = load()
    pen, paper, drawer = st["pen"], st["paper"], st["drawer"]
    clicker = other(drawer)
    mid = (np.asarray(pen["tip"]) + np.asarray(pen["button"])) / 2
    if np.hypot(*(mid - base_of(drawer))) > COMFORT:
        raise Failed(f"the pen is too far for the {drawer} arm: run relay first")
    guard = SeparationGuard(arm_separation_m=ARM_SEPARATION)
    guard._load_world()
    tcp = np.eye(4)
    b = np.array(LEFT_BASE_XYZ[:2])
    u = b / np.linalg.norm(b)
    fwd = np.array([u[1], -u[0]])

    def pair(q_d, q_c):
        return (q_d, q_c) if drawer == "right" else (q_c, q_d)

    best, found = None, 0
    t0 = time.monotonic()
    if PEN_E > 0 and GRIP_FROM_TIP - CAMERA_BELOW_TCP < HAND_CLEAR:
        raise Failed("gripped there, the wrist camera would reach the paper before the tip")
    for e in (PEN_E,):
        T_pick = pick_pose(pen, e, drawer)
        pre = T_pick.copy()
        pre[2, 3] += APPROACH
        # Some pen headings need the last wrist joint turned ~180 deg from ready: seed
        # those too (the plain seeds never reach them).
        base_seeds = seeds(READY, bearing(drawer, pre[:3, 3]))
        turned = []
        for s0 in base_seeds:
            for dj7 in (-math.pi, math.pi):
                s1 = np.array(s0, float).copy()
                s1[6] += dj7
                if ho.LO[6] <= s1[6] <= ho.HI[6]:
                    turned.append(s1)
        q_pre = solve(
            arms.model, drawer, READY, f"{drawer}/fr3_hand_tcp", tcp, pre, base_seeds + turned
        )
        if q_pre is None or not wrist_ok(q_pre):
            print(
                f"  pick with the hand's x toward the {'tip' if e > 0 else 'button'}: not reachable"
            )
            continue
        for f in CLICK_FORWARD:
            for h in CLICK_HEIGHT:
                grip = np.r_[b / 2 + f * fwd, tt.TABLE_Z + h]
                button_top = grip + np.array([0, 0, pen["length"] - GRIP_FROM_TIP])
                for hd in HEADINGS:
                    if found >= 3 or time.monotonic() - t0 > 240:
                        break
                    T_d = upright(pen, e, grip, hd)
                    q_d = solve(
                        arms.model,
                        drawer,
                        READY,
                        f"{drawer}/fr3_hand_tcp",
                        tcp,
                        T_d,
                        seeds(q_pre, bearing(drawer, grip)),
                    )
                    if q_d is None or not wrist_ok(q_d):
                        continue
                    for hc in CLICK_HEADINGS:
                        for roll in (0, 1):
                            T_c = clicker_pose(button_top, hc, roll, CLICK_FROM)
                            q_c = solve(
                                arms.model,
                                clicker,
                                q_d,
                                f"{clicker}/fr3_hand_tcp",
                                tcp,
                                T_c,
                                seeds(READY, bearing(clicker, button_top)),
                            )
                            if q_c is None or not wrist_ok(q_c):
                                continue
                            # Pressed as low as it may go, the clicker keeps clear of the drawer;
                            # judged at the button's height (the finger's side stops there).
                            T_mid = clicker_pose(button_top, hc, roll, 0.0)
                            q_mid = solve(
                                arms.model,
                                clicker,
                                q_d,
                                f"{clicker}/fr3_hand_tcp",
                                tcp,
                                T_mid,
                                [q_c],
                            )
                            if q_mid is None:
                                continue
                            slack = min(
                                guard._gaps("right", *map(list, pair(q_d, q))).slack(guard.config)
                                for q in (q_c, q_mid)
                            )
                            if slack < 0.005:
                                continue
                            arms.model.set(*pair(q_d, q_c))
                            if not arms.model.world.is_collision_free(arms.model.ctx):
                                continue
                            low = min(
                                lowest_part(arms.model, "right"), lowest_part(arms.model, "left")
                            )
                            if low < MIN_PART_HEIGHT:
                                continue
                            found += 1
                            turns = np.r_[np.abs(q_d - q_pre), np.abs(q_c - READY)]
                            cost = float(turns.max() + 0.2 * turns.sum())
                            if best is None or cost < best["cost"]:
                                best = dict(
                                    cost=cost,
                                    e=e,
                                    T_pick=T_pick.tolist(),
                                    q_pick_pre=q_pre.tolist(),
                                    grip=grip.tolist(),
                                    button_top=button_top.tolist(),
                                    heading=float(hd),
                                    q_drawer=q_d.tolist(),
                                    q_clicker=q_c.tolist(),
                                    roll=roll,
                                    clicker_heading=float(hc),
                                    slack=slack,
                                )
                            break
                        else:
                            continue
                        break
    print(f"  searched in {time.monotonic() - t0:.0f} s")
    if best is None:
        raise Failed("no pose lets one arm hold the pen up and the other click it")
    print(
        f"click: pen held {np.linalg.norm(np.asarray(best['grip'][:2]) - b / 2) * 100:.0f} cm out, grip "
        f"{best['grip'][2] * 100:.0f} cm up; the {drawer} hand's x toward the {'tip' if best['e'] > 0 else 'button'}; "
        f"the {clicker} gripper sideways (roll {best['roll']}); {best['slack'] * 1000:.0f} mm inside the guard's limits"
    )
    # Upright over the paper: the tip at the smiley's first point, a little above the paper.
    pen_up = None
    first = smiley(paper)[0][0]
    for hd in HEADINGS:
        tip = np.r_[first, paper["top_z"] + DRAW_LIFT + 0.02]
        T = tilted(pen, best["e"], tip, hd, DRAW_TILT)
        q = solve(
            arms.model,
            drawer,
            READY,
            f"{drawer}/fr3_hand_tcp",
            tcp,
            T,
            seeds(READY, bearing(drawer, tip)),
        )
        if q is None or not wrist_ok(q):
            continue
        arms.model.set(*pair(q, READY))
        low = lowest_part(arms.model, drawer) - tip[2]  # hand parts over the tip
        if low < 0.015:
            continue
        pen_up = dict(q=q.tolist(), heading=float(hd), hand_clear=float(low))
        break
    if pen_up is None:
        raise Failed(f"the {drawer} arm cannot hold the pen upright over the paper")
    print(
        f"draw: pen leaning {math.degrees(DRAW_TILT):.0f} deg; the hand's lowest part {pen_up['hand_clear'] * 1000:.0f} mm over the tip"
    )
    st["plan"] = {**best, "pen_up": pen_up}
    save(st)
    print("saved the plan")


# ---------------------------------------------------------------- stages


def stage_pick(arms: Arms) -> None:
    st = load()
    pen, plan, d = st["pen"], st["plan"], st["drawer"]
    arms.add_table()
    arms.gripper(d, 1.0, "open")
    arms.move(d, plan["q_pick_pre"], "over the pen", FREE)
    grasp_pen(arms, d, pen, float(np.asarray(plan["T_pick"])[2][3]))
    arms.straight(d, APPROACH, "lift", AWAY)
    if arms.finger(d) < 0.6 * pen["diameter"]:
        raise Failed(f"the {d} Hand lost the pen on the lift")
    carry_up(arms, d, plan["q_drawer"], "holds the pen upright", pen, plan["e"], tt.TABLE_Z)
    if arms.finger(d) < 0.6 * pen["diameter"]:
        raise Failed(f"the {d} Hand lost the pen turning it upright")
    st["held"] = True
    save(st)


def measure_button(arms: Arms, c: str, guess) -> np.ndarray | None:
    """Where the pen's top (the button) is across the table, from the clicker's wrist
    camera looking down from 30 cm: the blue pixels near where the guess falls in the
    image, their ray put at the guessed height. The button is too small and near for the
    camera's depth (blank there), so the height stays the guess (touch finds it)."""
    joints = {s_: arms.q(s_) for s_ in ho.SIDES}
    rgb, _, K, T, _ = md._frames(arms.app, f"{c}_wrist", arms.model, joints)
    q = np.linalg.inv(T) @ np.r_[guess, 1.0]
    u0, v0 = (K @ (q[:3] / q[2]))[:2]
    lab = cv2.cvtColor(rgb, cv2.COLOR_RGB2LAB).astype(float)
    H, W = lab.shape[:2]
    v, u = np.mgrid[0:H, 0:W]
    blue = (lab[..., 2] - 128 < -8) & (np.hypot(u - u0, v - v0) < 45)
    n = int(blue.sum())
    print(f"  {c} wrist camera: {n} blue pixels near where the button should be")
    img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR).copy()
    cv2.circle(img, (int(u0), int(v0)), 45, (0, 255, 255), 1)
    if n < 6:
        cv2.imwrite(str(HERE / "button_look.png"), img)
        return None
    uc, vc = u[blue].mean(), v[blue].mean()
    ray = T[:3, :3] @ np.array([(uc - K[0, 2]) / K[0, 0], (vc - K[1, 2]) / K[1, 1], 1.0])
    t = (guess[2] - T[2, 3]) / ray[2]
    found = T[:3, 3] + t * ray
    cv2.circle(img, (int(uc), int(vc)), 6, (0, 0, 255), 2)
    cv2.imwrite(str(HERE / "button_look.png"), img)
    off = found - np.asarray(guess)
    print(f"  the button is {off[0] * 1000:+.0f}, {off[1] * 1000:+.0f} mm (x, y) from the guess")
    return found


def stage_click(arms: Arms) -> None:
    st = load()
    plan, d = st["plan"], st["drawer"]
    c = other(d)
    arms.at(d, plan["q_drawer"])
    cp = click_poses(arms, d, arms.q(d), np.asarray(plan["button_top"]))
    if cp is None:
        raise Failed("no straight-down click fits inside the guard's limits")
    print(
        f"  {c} clicks pointing down, turned {math.degrees(cp['heading']):.0f} deg, "
        f"{cp['slack'] * 1000:.0f} mm inside the guard's limits"
    )
    if arms.finger(c) > 0.01:
        arms.gripper(c, 0.0, "closes")
    # Clear of the pen first (a retry can start with the clicker just over it, inside the
    # box that stands in for the pen below).
    lift_to(arms, c, float(plan["button_top"][2]) + 0.15, "straight up, clear of the pen")
    # The pen above the drawer's hand, for the planner, while the clicker comes over it.
    top = np.asarray(plan["button_top"])
    bottom = np.asarray(plan["grip"]) + np.array([0.0, 0.0, 0.05])
    T_box = np.eye(4)
    T_box[:3, 3] = (top + bottom) / 2
    arms.obstacle("pen", T_box, [0.025, 0.025, float(top[2] - bottom[2]) + 0.01])
    try:
        # Planned paths keep the arms from touching but may pass inside the separation
        # guard's limits (it then holds the arm midway): come to 10 cm over, then
        # straight down, a line whose ends were both checked against the guard.
        arms.move(c, cp["q_high"], "10 cm over the button, pointing down", FREE)
    finally:
        arms.mm.remove_obstacle("pen")
    # Where the button really is: the clicker's wrist camera (5 cm beside its fingers)
    # straight over the guessed button, 30 cm up, then the fingers over what it sees.
    md.look_with(arms, c, top[:2], float(top[2] - tt.TABLE_Z) + 0.30)  # depth is blank under ~20 cm
    found = measure_button(arms, c, top)
    if found is None:
        raise Failed("the clicker's camera does not see the pen's top (see button_look.png)")
    # Seen: the fingertips came down ~5 mm toward the tripod (+x) of the button
    # (2026-11-23), the camera's x offset from the geometry: correct by that.
    found = np.asarray(found, float) + np.r_[CLICK_AIM_OFFSET, 0.0]
    # (A second, closer look made things worse: from 15 cm the pen's whole upper body is
    # blue in view, not just the button. One look from 30 cm, as when it worked.)
    over = solve(
        arms.model,
        c,
        arms.q(d),
        f"{c}/fr3_hand_tcp",
        np.eye(4),
        clicker_pose(found, cp["heading"], 0, CLICK_FROM + 0.10),
        [arms.q(c)],
    )
    if over is None:
        raise Failed("no pose over the measured button")
    arms.move(c, over, "over the measured button", TOUCH)
    # (Correcting the fingertips onto the measured point did not help: as when it worked,
    # go by the arm's own move.)
    # The guessed height can be several cm out (where along the pen it was gripped): from
    # 6 cm over the guess, 4 mm steps to the first touch (less than a click's stroke),
    # back off 6 mm, then 1 mm steps to the button's top. Never a fast drop onto a guess.
    arms.straight(
        c, -(0.10 + CLICK_FROM - 0.015), "straight down to 1.5 cm over the guessed button", NEAR
    )
    if CLICK_BY_STEPS[c]:
        # 16 steps: the arm ends some short, and 12 got only 22 mm down (5 mm short).
        touched, went = step_touch(
            arms, c, [0.0, 0.0, -1.0], 16, "feels for the button (coarse)", 0.004
        )
        if not touched:
            arms.straight(c, 0.12, "back up", AWAY)
            raise Failed("the clicker never met the button")
    else:
        # The left FR3 cannot step reliably now (its robot ends small moves short): one
        # straight press to CLICK_PRESS_BELOW under the button's top, as the pen's
        # measured ends put it (the right arm's touch found it within 2 mm of that).
        arms.straight(
            c, -(0.015 + CLICK_PRESS_BELOW), "presses the button in, one straight push", TOUCH
        )
    # From the first touch, straight through: the button's spring reads only 1-2 N for
    # a light press, so a fine pass loses it; push until it bottoms out.
    # The step that felt the button already pushed it in (it reads over 3 N only once
    # the button is down); a further push went a couple of mm too far (2026-11-23).
    arms.straight(c, 0.12, "lets the button all the way out and lifts", AWAY)
    arms.move(c, READY, "to ready", FREE)
    st["clicks"] = st.get("clicks", 0) + 1
    save(st)


def stage_touch(arms: Arms) -> None:
    """Slide the upright pen straight across, at the height it is held at, to over the
    smiley's first point, then straight down to 2 cm over the paper (by the guess).
    Planned joint moves to a pose over the paper swung the pen near the table, and their
    retries folded the arm into its self-collision stop (2026-11-23): straight lines at
    constant orientation cannot. The pen may point any way about itself to draw."""
    st = load()
    plan, d, paper = st["plan"], st["drawer"], st["paper"]
    T = ho_tcp(arms, d)
    tip = tip_of(T, st["pen"], plan["e"])
    first = np.asarray(smiley(paper)[0][0])
    st["q_before_paper"] = arms.q(d)
    st["tcp_before_paper"] = T[:3, 3].tolist()
    save(st)
    arms.along(
        d,
        [first[0] - tip[0], first[1] - tip[1], 0.0],
        "slides the pen straight over the paper",
        CARRY,
    )
    tip = tip_of(ho_tcp(arms, d), st["pen"], plan["e"])
    above = tip[2] - paper["top_z"]
    print(f"  the tip should be {above * 1000:.0f} mm over the paper")
    arms.straight(d, -(above - 0.02), "straight down to 2 cm over the paper (by the guess)", NEAR)
    tip = tip_of(ho_tcp(arms, d), st["pen"], plan["e"])
    above = tip[2] - paper["top_z"]
    if CLICK_BY_STEPS[d]:
        touched, went = step_touch(arms, d, [0.0, 0.0, -1.0], 35, "feels for the paper")
        if not touched:
            raise Failed("the pen never touched the paper")
        st["z_touch"] = float(arms.tcp(d)[2])
    else:
        # The pen touched where this model put the tip TOUCH_MODEL_OFFSET over the
        # paper (2026-11-23); the drawing's ink check presses deeper if needed.
        st["z_touch"] = float(arms.tcp(d)[2] - (above - TOUCH_MODEL_OFFSET))
        print(f"  touch height from the model: TCP {st['z_touch'] * 1000:.1f} mm")
    arms.straight(d, PEN_LIFT, "lifts the pen", AWAY)
    save(st)


def ho_tcp(arms: Arms, side: str) -> np.ndarray:
    q = arms.q(side)
    arms.model.set(q, arms.q("left")) if side == "right" else arms.model.set(arms.q("right"), q)
    return arms.model.pose(f"{side}/fr3_hand_tcp")


INK_CHECK = False  # the camera's ink check missed: the drawing height is set by eye (lower_pen.py)
VIEW_DISTANCE = (0.45, 0.40, 0.52)  # the watching camera from the paper, m
VIEW_ELEVATION = np.radians((55, 45, 65))  # and its line of sight above the table
INK_PRESS_MAX = 0.003  # m deeper than PRESS at most, while the test tick shows no ink
TICK = 0.015  # m, the test stroke


def view_pose(arms: Arms, c: str, d: str, paper: dict, q_d) -> np.ndarray | None:
    """The clicker's joints putting its wrist camera on the paper from its own side,
    clear of the drawer at q_d (guard margins, collisions)."""
    from dimos.robot.manipulators.franka.separation_guard import SeparationGuard

    guard = SeparationGuard(arm_separation_m=ARM_SEPARATION)
    guard._load_world()
    P = np.r_[paper["centre"], paper["top_z"]]
    toward = base_of(c) - P[:2]
    toward /= np.linalg.norm(toward)
    for dist in VIEW_DISTANCE:
        for el in VIEW_ELEVATION:
            for turn in np.radians((0, -20, 20, -40, 40)):
                h = np.array(
                    [
                        math.cos(turn) * toward[0] - math.sin(turn) * toward[1],
                        math.sin(turn) * toward[0] + math.cos(turn) * toward[1],
                    ]
                )
                C = P + dist * np.r_[math.cos(el) * h, math.sin(el)]
                z = (P - C) / np.linalg.norm(P - C)
                x = np.cross([0.0, 0.0, 1.0], z)
                T = np.eye(4)
                T[:3, :3] = ho.frame(x / np.linalg.norm(x), z)
                T[:3, 3] = C
                q = solve(
                    arms.model,
                    c,
                    q_d,
                    f"{c}/fr3_hand",
                    camera_optical(c),
                    T,
                    seeds(READY, bearing(c, C)),
                    free_yaw=True,
                )
                if q is None or not wrist_ok(q):
                    continue
                pair = (q, q_d) if c == "right" else (q_d, q)
                if guard._gaps("right", list(pair[0]), list(pair[1])).slack(guard.config) < 0.02:
                    continue
                return q
    return None


def snap(arms: Arms, c: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """The clicker's wrist camera: a steady colour image (mean of 5), K, and T_world_opt."""
    from handoff import camera_optical as _co  # noqa: F401  (same frames as the look poses)

    mod = getattr(arms.app, f"{c}_wrist")
    K = np.asarray(mod.peek_stream("camera_info", 5.0).K).reshape(3, 3)
    imgs = [
        np.asarray(mod.peek_stream("color_image", 5.0).data).astype(np.float32) for _ in range(5)
    ]
    joints = {s_: arms.q(s_) for s_ in ho.SIDES}
    return np.mean(imgs, 0), K, tt.world_from_optical(f"{c}_wrist", joints, arms.model)


def new_ink(before, after, K, T, pts_world, z: float, name: str) -> float:
    """The share of the expected stroke's pixels that turned darker, before -> after."""
    Ti = np.linalg.inv(T)
    uv = []
    for p in pts_world:
        q = Ti @ np.r_[p, z, 1.0]
        uv.append((K @ (q[:3] / q[2]))[:2])
    mask = np.zeros(before.shape[:2], np.uint8)
    cv2.polylines(mask, [np.array(uv, np.int32)], False, 1, 7)
    g0 = cv2.cvtColor(before.astype(np.uint8), cv2.COLOR_RGB2GRAY).astype(float)
    g1 = cv2.cvtColor(after.astype(np.uint8), cv2.COLOR_RGB2GRAY).astype(float)
    darker = (g0 - g1) > 18
    share = float(darker[mask > 0].mean()) if mask.any() else 0.0
    img = cv2.cvtColor(after.astype(np.uint8), cv2.COLOR_RGB2BGR).copy()
    img[darker & (mask > 0)] = (0, 0, 255)
    cv2.polylines(img, [np.array(uv, np.int32)], False, (0, 255, 0), 1)
    cv2.imwrite(str(HERE / name), img)
    return share


def stage_draw(arms: Arms) -> None:
    st = load()
    d, plan, paper = st["drawer"], st["plan"], st["paper"]
    c = other(d)
    T = ho_tcp(arms, d)
    tip = tip_of(T, st["pen"], plan["e"])
    offset = (tip - T[:3, 3])[:2]  # the tip beside the TCP, in the table plane
    # The other arm watches the paper with its wrist camera: a test tick in a corner,
    # pressing deeper until ink shows, then the smiley at that depth.
    q_view = view_pose(arms, c, d, paper, arms.q(d)) if INK_CHECK else None
    watch = q_view is not None
    if watch:
        arms.move(c, q_view, "watches the paper", FREE)
    else:
        print(f"  no pose lets the {c} arm watch the paper: drawing without the ink check")
    press = PRESS
    if watch:
        up_, right_ = np.array([-1.0, 0.0]), np.array([0.0, 1.0])
        corner = np.asarray(paper["centre"]) + 0.075 * up_ + 0.07 * right_
        tick = [corner, corner + TICK * right_]
        start = arms.tcp(d)
        for attempt in range(int(round(INK_PRESS_MAX / 0.001)) + 1):
            before, K, Tc = snap(arms, c)
            a0 = np.asarray(tick[0]) - offset
            a1 = np.asarray(tick[1]) - offset
            here = arms.tcp(d)
            z_tick = st["z_touch"] - press
            r = arms.mm.move_linear_path(
                waypoints=[
                    (a0[0] - here[0], a0[1] - here[1], (z_tick + PEN_LIFT) - here[2]),
                    (a0[0] - here[0], a0[1] - here[1], z_tick - here[2]),
                    (a1[0] - here[0], a1[1] - here[1], z_tick - here[2]),
                    (a1[0] - here[0], a1[1] - here[1], z_tick + 0.04 - here[2]),
                    (a1[0] - 0.05 - here[0], a1[1] - here[1], z_tick + 0.04 - here[2]),
                ],
                planning_group=d,
                speed_scale=1.0,
                max_linear_speed=DRAW_SPEED,
                blend_deviation=0.0005,
                blocking=True,
                timeout=60.0,
            )
            if not r.succeeded:
                raise Failed(f"test tick: {r}")
            time.sleep(0.5)
            after, _, _ = snap(arms, c)
            share = new_ink(before, after, K, Tc, tick, paper["top_z"], "ink_tick.png")
            print(f"  test tick pressed {press * 1000:.1f} mm: {share * 100:.0f}% of it darker")
            if share > 0.3:
                break
            press += 0.001
        else:
            raise Failed("no ink on the test tick even pressing deeper (see ink_tick.png)")
        st["press"] = press
        save(st)
    strokes = [[np.asarray(p) - offset for p in s] for s in smiley(paper)]
    z_draw = st["z_touch"] - press
    if watch:
        before, K, Tc = snap(arms, c)
    start = T[:3, 3]
    first = strokes[0][0]
    arms.along(
        d,
        [first[0] - start[0], first[1] - start[1], (z_draw + PEN_LIFT) - start[2]],
        "over the first stroke",
        NEAR,
    )
    p0 = arms.tcp(d)
    way = []
    for stroke in strokes:
        s0, e0 = stroke[0], stroke[-1]
        way.append((s0[0] - p0[0], s0[1] - p0[1], 0.0))
        way += [(p[0] - p0[0], p[1] - p0[1], -PEN_LIFT) for p in stroke]
        way.append((e0[0] - p0[0], e0[1] - p0[1], 0.0))
    way = [
        w
        for i, w in enumerate(way)
        if np.linalg.norm(np.subtract(w, way[i - 1] if i else (0, 0, 0))) > 1e-4
    ]
    print(f"  {d} draws: {len(way)} waypoints, pen pressed {PRESS * 1000:.1f} mm")
    refused = arms.refused()
    r = arms.mm.move_linear_path(
        waypoints=way,
        planning_group=d,
        speed_scale=1.0,
        max_linear_speed=DRAW_SPEED,
        blend_deviation=0.002,
        blocking=True,
        timeout=180.0,
    )
    time.sleep(0.5)
    print(f"  {d} draws: {r.execution if r.plan.succeeded else r.plan}")
    arms.check(d, refused)
    if not r.succeeded:
        raise Failed(f"drawing: {r}")
    arms.straight(d, 0.05, "lifts the pen away", AWAY)
    if watch:
        arms.along(d, [-0.06, 0.0, 0.0], "moves the hand off the drawing", AWAY)
        time.sleep(0.5)
        after, _, _ = snap(arms, c)
        share = float(
            np.mean(
                [
                    new_ink(before, after, K, Tc, s_, paper["top_z"], "ink_smiley.png")
                    for s_ in smiley(paper)
                ]
            )
        )
        print(f"  the smiley: {share * 100:.0f}% of its lines darker (ink_smiley.png)")
        arms.move(c, READY, "to ready", FREE)


def settle_xy(arms: Arms, side: str, xy, what: str, tol: float = 0.001) -> None:
    """Bring the TCP over (x, y) exactly, keeping its height: up 2 cm, across, back down
    in one path (a short move alone may not run; the left FR3 ends moves short)."""
    for _ in range(3):
        here = arms.tcp(side)
        dx, dy = float(xy[0] - here[0]), float(xy[1] - here[1])
        if math.hypot(dx, dy) <= tol:
            return
        r = arms.mm.move_linear_path(
            waypoints=[(0.0, 0.0, STEP_DETOUR), (dx, dy, STEP_DETOUR), (dx, dy, 0.0)],
            planning_group=side,
            speed_scale=0.3,
            blocking=True,
            timeout=20.0,
        )
        time.sleep(0.4)
        now = arms.tcp(side)
        print(
            f"  {side} {what}: was {math.hypot(dx, dy) * 1000:.1f} mm off; now "
            f"{math.hypot(xy[0] - now[0], xy[1] - now[1]) * 1000:.1f} mm"
        )
        if not r.succeeded:
            raise Failed(f"{side} {what}: the correction did not run ({r})")
    here = arms.tcp(side)
    if math.hypot(xy[0] - here[0], xy[1] - here[1]) > 0.002:
        raise Failed(f"{side} {what}: could not get within 2 mm")


def settle_z(arms: Arms, side: str, z: float, what: str) -> None:
    """Bring the TCP to height z. A short move alone may not run, so each correction
    goes up STEP_DETOUR first and back down in one path. The left FR3 ends its descents
    5-7 mm short almost every time, so each correction aims below z by the shortfall it
    just saw (never more than 1 cm below), until within 1.5 mm."""
    aim, last = z, None
    for _ in range(4):
        here = float(arms.tcp(side)[2])
        err = here - z
        if abs(err) <= 0.0015:
            return
        if last is not None and abs(here - last) < 0.0007:
            # Two tries ended at the same height: the fingertips are on the table (it
            # stands a little higher here than the model has it). That is low enough;
            # pushing on trips the arm's collision stop (it did, 2026-11-23).
            print(f"  {side} {what}: resting on the table at {err * 1000:+.1f} mm; closing there")
            return
        last = here
        # Never more than 3 mm below the target: the table may be right there.
        aim = max(z - 0.003, min(z + 0.010, aim - err))
        r = arms.mm.move_linear_path(
            waypoints=[(0.0, 0.0, STEP_DETOUR), (0.0, 0.0, aim - here)],
            planning_group=side,
            speed_scale=0.3,
            blocking=True,
            timeout=20.0,
        )
        time.sleep(0.4)
        print(
            f"  {side} {what}: was {err * 1000:+.1f} mm off; aimed {(aim - z) * 1000:+.1f}; now "
            f"{(float(arms.tcp(side)[2]) - z) * 1000:+.1f} mm"
        )
        if not r.succeeded:
            raise Failed(f"{side} {what}: the correction did not run ({r})")


def grasp_pen(arms: Arms, side: str, pen: dict, z_grasp: float | None = None) -> None:
    """From over the pen (APPROACH above the planned grasp): down, feel for the table
    with the fingertips, rise a little, close, and check the Hand holds it."""
    # (Feeling for the table through the joint torques does not work while moving: the
    # arm's own accelerations read as 10-40 N, and moves under ~5 mm are not executed.)
    z_target = z_grasp if z_grasp is not None else float(arms.tcp(side)[2]) - APPROACH
    arms.straight(side, -(APPROACH - LAST), "down near the pen", NEAR)
    arms.straight(side, -LAST, "down to the pen", TOUCH)
    # The left FR3's robot ends some motions a few mm short: check the height itself.
    settle_z(arms, side, z_target, "grasp height")
    w = arms.gripper(side, 0.0, "close on the pen")
    print(f"  {side} Hand stopped at {w * 1000:.1f} mm (pen about {pen['diameter'] * 1000:.0f} mm)")
    if not 0.6 * pen["diameter"] < w < pen["diameter"] + 0.005:
        arms.gripper(side, 1.0, "open (not a good hold)")
        arms.straight(side, APPROACH, "back up", AWAY)
        raise Failed(f"the {side} Hand is not holding the pen well ({w * 1000:.1f} mm)")


CARRY_CLEAR = 0.03  # a held pen's tip and button at least this far over any surface, m
CARRY_HIGH = 0.15  # carrying moves go between poses raised this far, then straight down, m


def pen_points(T_tcp: np.ndarray, pen: dict, e: int, side: str) -> np.ndarray:
    """Points along the held pen, tip to button, in the world."""
    tip_in_tcp, _ = pen_in_hand(pen, e, side)
    button_in_tcp = tip_in_tcp - np.array([e * pen["length"], 0.0, 0.0])
    return np.array(
        [
            (T_tcp @ np.r_[tip_in_tcp + k * (button_in_tcp - tip_in_tcp), 1.0])[:3]
            for k in np.linspace(0.0, 1.0, 6)
        ]
    )


def carry(
    arms: Arms, side: str, goal, what: str, pen: dict, e: int, surface_z: float, dry: bool = False
) -> float:
    """A planned move with the pen in the Hand. The planner does not know the pen (it
    drove it into the table twice), so every point of the plan is checked first: the pen
    and the hand at least CARRY_CLEAR over ``surface_z``, else nothing moves."""
    from dimos.msgs.sensor_msgs.JointState import JointState
    from dimos.robot.manipulators.franka.tabletop_bimanual import arm_joint_names

    names = arm_joint_names(side)
    plan = arms.mm.plan_to_joints(
        {side: JointState(name=names, position=list(goal))}, speed_scale=CARRY
    )
    if not plan.succeeded or plan.plan is None:
        raise Failed(f"{side} {what}: no plan ({plan})")
    traj = plan.plan.trajectory
    idx = [traj.joint_names.index(n) for n in names]
    others = arms.q(other(side))
    worst, at = np.inf, 0.0
    pts = traj.points
    for k, pt in enumerate(pts):
        q = [pt.positions[i] for i in idx]
        arms.model.set(q, others) if side == "right" else arms.model.set(others, q)
        T = arms.model.pose(f"{side}/fr3_hand_tcp")
        low = min(
            float(pen_points(T, pen, e, side)[:, 2].min()),
            lowest_part(arms.model, side) + tt.TABLE_Z,
        )
        if low - surface_z < worst:
            worst, at = low - surface_z, k / max(1, len(pts) - 1)
    if worst < CARRY_CLEAR:
        arms.mm.clear_planned_path()
        raise Failed(
            f"{side} {what}: the planned path brings the pen within {worst * 1000:.0f} mm of the "
            f"table ({at * 100:.0f}% along); not moving"
        )
    print(f"  {side} {what}: checked {len(pts)} points, the pen keeps {worst * 1000:.0f} mm clear")
    if dry:
        arms.mm.clear_planned_path()
        return worst
    refused = arms.refused()
    result = arms.mm.execute(blocking=True, timeout=60.0)
    time.sleep(0.5)
    print(f"  {side} {what}: {result.status.name}")
    arms.check(side, refused)
    if result.status.name != "COMPLETED":
        raise Failed(f"{side} {what}: {result.status.name}")
    short = float(np.degrees(np.abs(np.asarray(arms.q(side)) - np.asarray(goal)).max()))
    if short > 1.5 and not dry and what.endswith("(the rest)") is False:
        # The left FR3's robot ends some motions short (no fault): check and plan the rest.
        print(f"  {side} {what}: ended {short:.1f} deg short; planning the rest")
        return carry(arms, side, goal, f"{what} (the rest)", pen, e, surface_z)
    arms.at(side, goal)
    return worst


def carry_up(arms: Arms, side: str, goal, what: str, pen: dict, e: int, surface_z: float) -> None:
    """carry(), and if its plan would bring the pen too near a surface, lift straight up
    8 cm more and plan again from there (twice at most)."""
    for attempt in range(4):
        try:
            carry(arms, side, goal, what, pen, e, surface_z)
            return
        except Failed as err:
            if "not moving" not in str(err) or attempt == 3:
                raise
            print(f"  {err}: lifting 10 cm and planning again")
            arms.straight(side, 0.10, "straight up to plan from higher", AWAY)


def raised(arms: Arms, side: str, q, up: float) -> np.ndarray:
    """The joints putting the TCP ``up`` straight above where q puts it, same orientation."""
    other_q = arms.q(other(side))
    arms.model.set(q, other_q) if side == "right" else arms.model.set(other_q, q)
    T = np.asarray(arms.model.pose(f"{side}/fr3_hand_tcp")).copy()
    T[2, 3] += up
    q_up = solve(arms.model, side, other_q, f"{side}/fr3_hand_tcp", np.eye(4), T, [q])
    if q_up is None or not wrist_ok(q_up):
        raise Failed(f"no pose {up * 100:.0f} cm over that one for the {side} arm")
    return q_up


def lift_to(arms: Arms, side: str, z: float, what: str) -> None:
    """Straight up until the TCP is at least z high."""
    dz = z - float(arms.tcp(side)[2])
    if dz > 0.005:
        arms.straight(side, dz, what, AWAY)


def stage_relay(arms: Arms) -> None:
    """If the pen is too far for the drawer, the clicker picks it up and sets it down
    where the drawer reaches comfortably; the drawer's wrist camera measures it again."""
    from dimos.robot.manipulators.franka.tabletop_bimanual import LEFT_BASE_XYZ

    st = load()
    pen, d = st["pen"], st["drawer"]
    c = other(d)
    mid = (np.asarray(pen["tip"]) + np.asarray(pen["button"])) / 2
    if np.hypot(*(mid - base_of(d))) <= COMFORT:
        print(f"  the {d} arm reaches the pen itself: no relay")
        return
    tcp = np.eye(4)
    pick = None
    for e in (1, -1):
        T = pick_pose(pen, e, c)
        pre = T.copy()
        pre[2, 3] += APPROACH
        q = solve(
            arms.model,
            c,
            arms.q(d),
            f"{c}/fr3_hand_tcp",
            tcp,
            pre,
            seeds(READY, bearing(c, pre[:3, 3])),
        )
        if q is not None and wrist_ok(q):
            pick = (e, T, q)
            break
    if pick is None:
        raise Failed(f"the {c} arm cannot reach the pen either")
    e, T_pick, q_pre = pick
    # Where to set it down: in front, the drawer's side of the middle, clear of the board.
    b = np.array(LEFT_BASE_XYZ[:2])
    u = b / np.linalg.norm(b)
    fwd = np.array([u[1], -u[0]])
    spot = None
    for k in np.linspace(0.0, 0.25, 6):
        for f in (0.40, 0.35, 0.45, 0.30):
            p = b / 2 + f * fwd + k * (base_of(d) - b / 2) / np.linalg.norm(base_of(d) - b / 2)
            r = np.hypot(*(p - base_of(d)))
            if not RELAY_DISTANCE[0] <= r <= RELAY_DISTANCE[1]:
                continue
            if any(tt.inside(poly, p[None], 0.10)[0] for poly in tt.fixtures()):
                continue
            for yaw in np.radians(np.arange(0, 180, 30)):
                a = np.array([math.cos(yaw), math.sin(yaw)])
                T = T_pick.copy()
                T[:3, :3] = ho.frame(np.r_[e * a, 0.0], [0.0, 0.0, -1.0])
                T[:2, 3] = p
                pre = T.copy()
                pre[2, 3] += APPROACH
                q = solve(
                    arms.model,
                    c,
                    arms.q(d),
                    f"{c}/fr3_hand_tcp",
                    tcp,
                    pre,
                    seeds(READY, bearing(c, pre[:3, 3])),
                )
                if q is not None and wrist_ok(q):
                    spot = (p, q)
                    break
            if spot:
                break
        if spot:
            break
    if spot is None:
        raise Failed("no spot both arms reach to pass the pen through")
    p, q_put = spot
    print(
        f"  relay: the {c} arm moves the pen to ({p[0]:.3f}, {p[1]:.3f}), "
        f"{np.hypot(*(p - base_of(d))) * 100:.0f} cm from the {d} base"
    )
    arms.add_table()
    arms.gripper(c, 1.0, "open")
    arms.move(c, q_pre, "over the pen", FREE)
    grasp_pen(arms, c, pen, float(T_pick[2, 3]))
    arms.straight(c, APPROACH, "lift", AWAY)
    q_put_high = raised(arms, c, q_put, CARRY_HIGH)
    carry_up(arms, c, q_put_high, "high over the relay spot", pen, e, tt.TABLE_Z)
    arms.straight(c, -CARRY_HIGH, "straight down over the spot", NEAR)

    arms.straight(c, -(APPROACH - LAST), "down near the table", NEAR)
    arms.straight(c, -(LAST - 0.002), "sets it down", TOUCH)
    arms.gripper(c, 1.0, "lets go")
    arms.straight(c, APPROACH, "lifts away", AWAY)
    arms.move(c, READY, "to ready", FREE)
    md.look_with(arms, d, p, LOOK_PEN)
    joints = {s: arms.q(s) for s in ho.SIDES}
    rgb, depth, K, T, plane = md._frames(arms.app, f"{d}_wrist", arms.model, joints)
    found = find_pen(rgb, depth, K, T, plane, near=p, radius=0.12, base_clearance=0.0)
    arms.move(d, READY, "to ready", FREE)
    if found is None or not found["found_tip"]:
        raise Failed("the drawer's camera does not find the pen at the relay spot")
    if not 0.11 < found["length"] < 0.17:
        tip, a = np.asarray(found["tip"]), np.asarray(found["axis"])
        found["length"], found["button"] = 0.14, (tip - a * 0.14).tolist()
    print(
        f"  {d} wrist: tip ({found['tip'][0]:.3f}, {found['tip'][1]:.3f}), button "
        f"({found['button'][0]:.3f}, {found['button'][1]:.3f})"
    )
    st["pen"] = found
    st.pop("plan", None)
    save(st)
    draw_scene(arms, st)


def stage_back(arms: Arms) -> None:
    """The reverse of stage_touch: straight up, straight across, back where it was clicked."""
    st = load()
    d = st["drawer"]
    back = np.asarray(st["tcp_before_paper"])
    here = arms.tcp(d)
    arms.straight(d, float(back[2] - here[2]), "straight up off the paper", AWAY)
    here = arms.tcp(d)
    arms.along(
        d,
        [float(back[0] - here[0]), float(back[1] - here[1]), 0.0],
        "slides the pen back to be clicked",
        CARRY,
    )
    arms.move(d, st["q_before_paper"], "settles where it was clicked", CARRY)


def stage_putdown(arms: Arms) -> None:
    st = load()
    d, plan = st["drawer"], st["plan"]
    lift_to(arms, d, tt.TABLE_Z + 0.25, "straight up")
    for up in (CARRY_HIGH, 0.10, 0.06):  # as high over the spot as the arm reaches
        try:
            q_high = raised(arms, d, plan["q_pick_pre"], up)
            break
        except Failed:
            continue
    else:
        raise Failed("no pose over where the pen was")
    carry_up(
        arms,
        d,
        q_high,
        f"{up * 100:.0f} cm over where the pen was",
        st["pen"],
        plan["e"],
        max(tt.TABLE_Z, st["paper"]["top_z"]),
    )
    arms.straight(d, -up, "straight down over the spot", NEAR)
    arms.straight(d, -(APPROACH - LAST), "down near the table", NEAR)
    arms.straight(d, -(LAST - 0.002), "sets it down", TOUCH)
    arms.gripper(d, 1.0, "lets go")
    arms.straight(d, APPROACH, "lifts away", AWAY)
    arms.move(d, READY, "to ready", FREE)
    st.pop("held", None)
    save(st)


def stage_flip(arms: Arms) -> None:
    """The camera had the pen's ends the wrong way round: swap tip and button (and the
    plan, which depended on them, is dropped)."""
    st = load()
    pen = st["pen"]
    pen["tip"], pen["button"] = pen["button"], pen["tip"]
    pen["axis"] = (-np.asarray(pen["axis"])).tolist()
    st.pop("plan", None)
    save(st)
    print(
        f"  swapped: tip ({pen['tip'][0]:.3f}, {pen['tip'][1]:.3f}), button ({pen['button'][0]:.3f}, {pen['button'][1]:.3f})"
    )
    draw_scene(arms, st)


STAGES = {
    "flip": stage_flip,
    "look": stage_look,
    "relay": stage_relay,
    "plan": stage_plan,
    "pick": stage_pick,
    "click": stage_click,
    "touch": stage_touch,
    "draw": stage_draw,
    "back": stage_back,
    "putdown": stage_putdown,
}
ALL = ["look", "relay", "plan", "pick", "click", "touch", "draw", "back", "click", "putdown"]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("stages", nargs="+", choices=[*STAGES, "all", "scene"])
    args = ap.parse_args()
    names = ALL if args.stages == ["all"] else args.stages
    arms = Arms(dry=False)
    try:
        for name in names:
            print(f"== {name}")
            t0 = time.monotonic()
            draw_scene(arms, load()) if name == "scene" else STAGES[name](arms)
            print(f"   {name} done in {time.monotonic() - t0:.0f} s")
    except Failed as e:
        print(f"STOPPED: {e}")
    finally:
        arms.close()


if __name__ == "__main__":
    main()
