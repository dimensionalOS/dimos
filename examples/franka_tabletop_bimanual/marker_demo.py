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

"""Marker demo: two FR3s uncap a marker, draw a smiley on paper, recap it, put it down.

The right arm picks the marker lying on the table by its cap (the black end) and holds
it up in the middle. The left arm takes the other end, coming in along the marker, and
pulls straight back: the cap stays in the right arm's fingers. The left arm turns the
marker to point down, feels for the paper through its joint torques, and draws a
smiley. Then it goes back the way it came and pushes the marker into the cap, lets go,
and the right arm puts the marker back where it found it.

Nothing about the marker or paper is given: both are found with the cameras, wherever
they are (the paper within the left arm's reach, the marker within the right's).

    ~/dimos-franka/.venv/bin/python marker_demo.py look      # find marker and paper (moves to look)
    ~/dimos-franka/.venv/bin/python marker_demo.py plan      # no motion: grasp, handover, smiley
    ~/dimos-franka/.venv/bin/python marker_demo.py pick | present | take | uncap
    ~/dimos-franka/.venv/bin/python marker_demo.py touch | draw | recap | release | place
    ~/dimos-franka/.venv/bin/python marker_demo.py all       # everything, in order

Each stage saves to marker_state.json and the next picks up from it.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

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
import tabletop as tt

HERE = Path(__file__).parent
STATE = HERE / "marker_state.json"

CAP_GRIP_IN = 0.022  # the right arm grips the cap this far in from its end, m
# The left arm's TCP this far in from the marker's back end, m: past the narrow waist
# behind the end plug, so the pads close on the full-width body (at 1.8 cm they slid off).
BACK_GRIP_IN = 0.04
TAKE_FROM = 0.10  # the left arm lines up this far back along the marker, m
UNCAP_PULL = 0.11  # the left arm pulls the marker this far out of the cap, m
PRESENT_FORWARD = (0.42, 0.36)  # marker centre in front of the middle of the bases, m
PRESENT_HEIGHT = (0.32, 0.38)  # marker above the table, m
HEADINGS = (0, -25, 25, -50, 50)  # marker turned from pointing straight at the left arm, deg
ROLLS = (0, 90, 180, 270)  # the left hand turned about the marker, deg
ASIDE_UP, ASIDE_OUT = 0.10, 0.10  # the right arm takes the cap up and away from the marker, m
MARKER_BASE_CLEARANCE = 0.22  # the tripod looks for the marker this far from the bases, m

# The marker is thin (~10 mm): grip it with the fingertips just over the table, at the
# height both arms measured the table by touch (the wrist camera read it 2.7 mm high).
MARKER_TIP_CLEARANCE = 0.001  # m
# Speeds by what a move does (shares of the planner's joint speed, or of 10 cm/s straight).
FREE = 1.0  # empty-handed moves through free space
CARRY = 0.7  # moves holding the marker, cap or pen
AWAY = 0.6  # straight moves away from contact (lifts, backing out)
NEAR = 0.6  # straight approaches, down to the last few cm
TOUCH = 0.25  # the last few cm into a grasp, the pull, the recap
LAST = 0.03  # m of an approach taken at TOUCH speed

LOOK_MARKER = 0.30  # right wrist camera above the marker, m
LOOK_PAPER = 0.42  # left wrist camera above the paper, m

FACE_RADIUS = 0.045  # m, shrunk to fit the paper
PEN_LIFT = 0.012  # pen above the paper between strokes, m
PRESS = 0.001  # pen pressed this far below where it first touched, m
DRAW_SPEED = 0.04  # m/s along the paper
TOUCH_FORCE = 3.0  # N along the motion that counts as touching
TOUCH_SPEED = 0.06  # share of the linear speed while feeling for contact
PUSH_FORCE = 8.0  # N: the cap is on
PUSH_EXTRA = 0.004  # m past where the cap came off, at most


def load() -> dict:
    return json.loads(STATE.read_text()) if STATE.exists() else {}


def save(st: dict) -> None:
    STATE.write_text(json.dumps(st, indent=1))


# ---------------------------------------------------------------- perception


def _pixels_on_plane(K, T, n, d, lift, shape):
    """Each pixel's ray, onto the plane n.p = d + lift: world points (H, W, 3) and which hit."""
    H, W = shape
    v, u = np.mgrid[0:H, 0:W]
    rays = np.stack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones_like(u, float)], -1)
    R, o = T[:3, :3], T[:3, 3]
    dirs = rays.reshape(-1, 3) @ R.T
    t = (d + lift - o @ n) / (dirs @ n)
    return (o + dirs * t[:, None]).reshape(H, W, 3), (t > 0).reshape(H, W)


def _frames(app, cam: str, model, joints=None):
    mod = getattr(app, cam)
    info = mod.peek_stream("camera_info", 5.0)
    K = np.asarray(info.K).reshape(3, 3)
    rgb = np.asarray(mod.peek_stream("color_image", 5.0).data)
    depth = np.asarray(mod.peek_stream("depth_image", 5.0).data).astype(float) / 1000.0
    T = tt.world_from_optical(cam, joints or {}, model)
    n, d = tt.fit_table(tt.world_points(depth, K, T))
    return rgb, depth, K, T, (n, d)


def heights(depth, K, T, plane) -> np.ndarray:
    """Each pixel's height above the table from its depth, m; NaN where there is none."""
    n, d = plane
    H, W = depth.shape
    v, u = np.mgrid[0:H, 0:W]
    z = depth
    P = np.stack([(u - K[0, 2]) / K[0, 0] * z, (v - K[1, 2]) / K[1, 1] * z, z], -1).reshape(-1, 3)
    h = (P @ T[:3, :3].T + T[:3, 3]) @ n - d
    h[(z.ravel() < 0.15) | (z.ravel() > 2.5)] = np.nan
    return h.reshape(H, W)


def find_marker(
    rgb, depth, K, T, plane, near=None, radius=0.15, base_clearance=MARKER_BASE_CLEARANCE
):
    """The marker lying on the table: its cap (dark) and body (white) ends, from colour,
    each pixel put on a plane at the marker's axis height."""
    from dimos.robot.manipulators.franka.tabletop_bimanual import LEFT_BASE_XYZ

    n, d = plane
    P, hit = _pixels_on_plane(K, T, n, d, 0.008, rgb.shape[:2])
    xy = P[..., :2]
    ok = hit.copy()
    h = heights(depth, K, T, plane)
    ok &= ~(h > 0.04)  # on the table, not a finger or cable held up in view
    if near is not None:
        ok &= np.hypot(*(xy - near[:2]).transpose(2, 0, 1)) < radius
    if base_clearance:
        for b in (np.zeros(2), np.array(LEFT_BASE_XYZ[:2])):
            ok &= np.hypot(*(xy - b).transpose(2, 0, 1)) > base_clearance
        for poly in tt.fixtures():
            ok &= ~tt.inside(poly, xy.reshape(-1, 2), tt.COLOUR_FIXTURE_MARGIN).reshape(ok.shape)
    lab = cv2.cvtColor(rgb, cv2.COLOR_RGB2LAB).astype(float)
    ref = np.median(lab[ok], axis=0)
    L, A, B = lab[..., 0], lab[..., 1] - 128, lab[..., 2] - 128
    dark = ok & (L < min(80.0, ref[0] - 50))
    dark = cv2.morphologyEx(dark.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)) > 0
    count, labels = cv2.connectedComponents(dark.astype(np.uint8))
    best = None
    for k in range(1, count):
        pts = xy[labels == k]
        if len(pts) < 15:
            continue
        c = pts.mean(0)
        evals, evecs = np.linalg.eigh(np.cov((pts - c).T))
        a = evecs[:, 1]
        length = np.ptp((pts - c) @ a)
        width = np.ptp((pts - c) @ np.array([-a[1], a[0]]))
        if not (0.025 < length < 0.09 and width < 0.035):
            continue
        score = len(pts) - (0 if near is None else 2000 * np.hypot(*(c - near[:2])))
        if best is None or score > best[0]:
            best = (score, pts, c, a)
    if best is None:
        return None
    _, cap, c_cap, a0 = best
    off_line = np.abs((xy - c_cap) @ np.array([-a0[1], a0[0]]))
    bright = (
        ok
        & (L > ref[0] + 25)
        & (np.abs(A) < 14)
        & (np.abs(B) < 18)
        & (np.hypot(*(xy - c_cap).transpose(2, 0, 1)) < 0.16)
        & (off_line < 0.012)
    )
    body = xy[bright]
    if len(body) < 10:
        return None
    pts = np.r_[cap, body]
    c = pts.mean(0)
    evals, evecs = np.linalg.eigh(np.cov((pts - c).T))
    a = evecs[:, 1]
    s_cap, s_body = (cap - c) @ a, (body - c) @ a
    if np.median(s_cap) < np.median(s_body):
        a, s_cap, s_body = -a, -s_cap, -s_body
    cap_end = c + a * np.percentile(s_cap, 99)
    back_end = c + a * np.percentile(s_body, 1)
    across = (cap - c) @ np.array([-a[1], a[0]])
    cap_start = c + a * np.percentile(s_cap, 1)
    return dict(
        cap_end=cap_end.tolist(),
        back_end=back_end.tolist(),
        axis=a.tolist(),
        length=float(np.linalg.norm(cap_end - back_end)),
        cap_length=float(np.linalg.norm(cap_end - cap_start)),
        diameter=float(np.clip(np.ptp(np.percentile(across, [3, 97])), 0.012, 0.022)),
        table_z=float((d - n[0] * c[0] - n[1] * c[1]) / n[2]),
    )


def find_paper(rgb, depth, K, T, plane, near=None, radius=0.4):
    """The biggest white sheet on the table: its centre, size, turn, and its top's height."""
    from dimos.robot.manipulators.franka.tabletop_bimanual import LEFT_BASE_XYZ

    n, d = plane
    lab = cv2.cvtColor(rgb, cv2.COLOR_RGB2LAB).astype(float)
    L, A, B = lab[..., 0], lab[..., 1] - 128, lab[..., 2] - 128
    P0, hit = _pixels_on_plane(K, T, n, d, 0.0, rgb.shape[:2])
    ok = hit & ~(heights(depth, K, T, plane) > 0.03)  # on the table, not the white arms
    if near is not None:
        ok &= np.hypot(*(P0[..., :2] - near[:2]).transpose(2, 0, 1)) < radius
    for b in (np.zeros(2), np.array(LEFT_BASE_XYZ[:2])):
        ok &= np.hypot(*(P0[..., :2] - b).transpose(2, 0, 1)) > 0.2
    ref = np.median(lab[ok], axis=0)
    white = ok & (L > ref[0] + 25) & (np.abs(A) < 10) & (np.abs(B) < 14)
    white = cv2.morphologyEx(white.astype(np.uint8), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(white)
    # The biggest white blob that is sheet-sized on the table (10-40 cm a side).
    mask = None
    for k in 1 + np.argsort(-stats[1:, cv2.CC_STAT_AREA]):
        m = labels == k
        if m.sum() < 200:
            break
        (_, _), (w0, h0), _ = cv2.minAreaRect(P0[m][:, :2].astype(np.float32))
        if 0.10 < min(w0, h0) and max(w0, h0) < 0.40:
            mask = m
            break
    if mask is None:
        return None
    # Its top's height, from the depth over it.
    v, u = np.nonzero(mask)
    z = depth[v, u]
    good = (z > 0.15) & (z < 2.5)
    pts = np.c_[
        (u[good] - K[0, 2]) / K[0, 0] * z[good],
        (v[good] - K[1, 2]) / K[1, 1] * z[good],
        z[good],
        np.ones(good.sum()),
    ]
    top = float(np.median((T @ pts.T).T[:, :3] @ n - d)) if good.sum() > 50 else 0.0
    P, _ = _pixels_on_plane(K, T, n, d, top, rgb.shape[:2])
    xy = P[mask][:, :2].astype(np.float32)
    (cx, cy), (w, h), ang = cv2.minAreaRect(xy)
    centre = np.array([cx, cy])
    table_z = float((d - n[0] * cx - n[1] * cy) / n[2])
    return dict(
        centre=centre.tolist(),
        size=[float(w), float(h)],
        angle=float(ang),
        top_z=table_z + top,
        area=float(w * h),
    )


def look_with(arms: Arms, side: str, at, height: float) -> None:
    """Put that arm's wrist camera straight down over a point."""
    target = np.eye(4)
    target[:3, :3] = np.diag([1.0, -1.0, -1.0])
    target[:3, 3] = [at[0], at[1], tt.TABLE_Z + height]
    q = solve(
        arms.model,
        side,
        arms.q(ho.other(side)),
        f"{side}/fr3_hand",
        camera_optical(side),
        target,
        seeds(arms.q(side), bearing(side, target[:3, 3])),
        free_yaw=True,
    )
    if q is None:
        raise Failed(f"no pose puts the {side} camera over ({at[0]:.2f}, {at[1]:.2f})")
    arms.move(side, q, "to look", FREE)
    time.sleep(1.0)


def stage_look(arms: Arms) -> None:
    app = arms.app
    tt.refresh_scene(app)
    arms.reset()
    arms.add_table()
    rgb, depth, K, T, plane = _frames(app, "scene", arms.model)
    m0 = find_marker(rgb, depth, K, T, plane)
    p0 = find_paper(rgb, depth, K, T, plane)
    if m0 is None or p0 is None:
        raise Failed(f"the tripod does not see {'the marker' if m0 is None else 'the paper'}")
    print(
        f"tripod: marker {m0['length'] * 100:.1f} cm, cap end at ({m0['cap_end'][0]:.3f}, "
        f"{m0['cap_end'][1]:.3f}); paper {p0['size'][0] * 100:.0f} x {p0['size'][1] * 100:.0f} cm at "
        f"({p0['centre'][0]:.3f}, {p0['centre'][1]:.3f})"
    )
    mid = (np.array(m0["cap_end"]) + np.array(m0["back_end"])) / 2
    look_with(arms, "right", mid, LOOK_MARKER)
    joints = {s: arms.q(s) for s in ho.SIDES}
    rgb_r, depth_r, K_r, T_r, plane_r = _frames(app, "right_wrist", arms.model, joints)
    m = find_marker(rgb_r, depth_r, K_r, T_r, plane_r, near=mid, radius=0.15, base_clearance=0.0)
    if m is None:
        print("  the right wrist camera did not find it; using the tripod's measurement")
        m = m0
    print(
        f"right wrist: marker {m['length'] * 100:.1f} cm long, {m['diameter'] * 1000:.0f} mm thick, "
        f"cap {m['cap_length'] * 100:.1f} cm; cap end ({m['cap_end'][0]:.3f}, {m['cap_end'][1]:.3f}), "
        f"back end ({m['back_end'][0]:.3f}, {m['back_end'][1]:.3f})"
    )
    arms.move("right", READY, "to ready", FREE)
    look_with(arms, "left", p0["centre"], LOOK_PAPER)
    joints = {s: arms.q(s) for s in ho.SIDES}
    rgb_l, depth_l, K_l, T_l, plane_l = _frames(app, "left_wrist", arms.model, joints)
    p = find_paper(rgb_l, depth_l, K_l, T_l, plane_l, near=np.array(p0["centre"]), radius=0.3) or p0
    print(
        f"left wrist: paper {p['size'][0] * 100:.1f} x {p['size'][1] * 100:.1f} cm at "
        f"({p['centre'][0]:.3f}, {p['centre'][1]:.3f}), its top {p['top_z'] * 1000:.0f} mm up"
    )
    arms.move("left", READY, "to ready", FREE)
    save({"marker": m, "paper": p})
    draw_scene(arms, {"marker": m, "paper": p})


def draw_scene(arms: Arms, st: dict) -> None:
    """The tripod image with the marker, its cap end, and the smiley to be drawn."""
    rgb, _, K, T, _ = _frames(arms.app, "scene", arms.model)
    img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR).copy()
    Tinv = np.linalg.inv(T)

    def px(p):
        q = Tinv @ np.r_[p, 1.0]
        return tuple(int(v) for v in (K @ (q[:3] / q[2]))[:2])

    m = st["marker"]
    z = m["table_z"] + m["diameter"] / 2
    cv2.line(img, px(np.r_[m["back_end"], z]), px(np.r_[m["cap_end"], z]), (0, 255, 0), 2)
    cv2.circle(img, px(np.r_[m["cap_end"], z]), 5, (0, 0, 255), -1)
    for stroke in smiley(st["paper"]):
        pts = [px(np.r_[p, st["paper"]["top_z"]]) for p in stroke]
        cv2.polylines(img, [np.array(pts)], False, (255, 0, 255), 2)
    cv2.imwrite(str(HERE / "marker_scene.png"), img)
    print("saved marker_scene.png (green: marker, red dot: its cap end, magenta: the smiley)")


# ---------------------------------------------------------------- the smiley


def smiley(paper: dict) -> list[list[np.ndarray]]:
    """Strokes (world x, y) of a smiley at the paper's centre, upright for someone at the
    front of the table looking toward the arms (its top toward the bases)."""
    r = min(FACE_RADIUS, 0.38 * min(paper["size"]))
    c = np.asarray(paper["centre"])
    up, right = np.array([-1.0, 0.0]), np.array([0.0, 1.0])

    def at(u, v):
        return c + u * right + v * up

    def arc(cu, cv, rad, a0, a1, n):
        return [at(cu + rad * math.cos(t), cv + rad * math.sin(t)) for t in np.linspace(a0, a1, n)]

    full = (math.pi / 2, math.pi / 2 + 2 * math.pi)
    return [
        arc(0, 0, r, *full, 41),
        arc(-0.35 * r, 0.3 * r, 0.004, *full, 9),
        arc(0.35 * r, 0.3 * r, 0.004, *full, 9),
        arc(0, 0.05 * r, 0.6 * r, math.radians(200), math.radians(340), 15),
    ]


# ---------------------------------------------------------------- planning


def marker_in_hand(m: dict) -> tuple[np.ndarray, float]:
    """The marker's frame in the right TCP's frame at the pick (x toward the cap, z up,
    origin mid-axis), and the TCP's height above the table there."""
    up = max(m["diameter"] / 2, FINGERTIP_REACH + MARKER_TIP_CLEARANCE)
    T = np.eye(4)
    T[:3, :3] = np.diag([1.0, -1.0, -1.0])
    T[:3, 3] = [-(m["length"] / 2 - CAP_GRIP_IN), 0.0, up - m["diameter"] / 2]
    return T, up


def pick_pose(m: dict) -> np.ndarray:
    a = np.r_[m["axis"], 0.0]
    g = np.asarray(m["cap_end"]) - np.asarray(m["axis"]) * CAP_GRIP_IN
    _, up = marker_in_hand(m)
    T = np.eye(4)
    T[:3, :3] = ho.frame(a, [0.0, 0.0, -1.0])
    T[:3, 3] = [g[0], g[1], tt.TABLE_Z + up]
    return T


def left_on_marker(m: dict, roll: float, back: float = 0.0) -> np.ndarray:
    """The left TCP in the marker's frame: on its axis near the back end, approaching
    along it toward the cap, turned ``roll`` about it; ``back`` further out along it."""
    z = np.array([1.0, 0.0, 0.0])
    x = Rotation.from_rotvec(z * math.radians(roll)).apply([0.0, 0.0, 1.0])
    T = np.eye(4)
    T[:3, :3] = ho.frame(x, z)
    T[:3, 3] = [-m["length"] / 2 + BACK_GRIP_IN - back, 0.0, 0.0]
    return T


def handover(arms: Arms, m: dict, q_r0, q_l0) -> dict | None:
    """Where the right arm holds the marker up and how the left arm takes its back end."""
    from dimos.robot.manipulators.franka.separation_guard import SeparationGuard
    from dimos.robot.manipulators.franka.tabletop_bimanual import LEFT_BASE_XYZ

    guard = SeparationGuard()
    guard._load_world()
    b = np.array(LEFT_BASE_XYZ[:2])
    u = b / np.linalg.norm(b)  # right base -> left base
    fwd = np.array([u[1], -u[0]])
    T_hand_m, _ = marker_in_hand(m)
    T_m_hand = np.linalg.inv(T_hand_m)
    tcp = np.eye(4)
    found = []
    t0 = time.monotonic()
    for f in PRESENT_FORWARD:
        for h in PRESENT_HEIGHT:
            for hd in HEADINGS:
                if len(found) >= 3 or time.monotonic() - t0 > 40:
                    break
                centre = np.r_[b / 2 + f * fwd, tt.TABLE_Z + h]
                c_, s_ = math.cos(math.radians(hd)), math.sin(math.radians(hd))
                body = np.array([c_ * u[0] - s_ * u[1], s_ * u[0] + c_ * u[1]])  # cap -> back end
                T_w_m = np.eye(4)
                T_w_m[:3, :3] = ho.frame(np.r_[-body, 0.0], [0.0, 0.0, 1.0])
                T_w_m[:3, 3] = centre
                T_r = T_w_m @ T_m_hand
                q_r = solve(
                    arms.model,
                    "right",
                    q_l0,
                    "right/fr3_hand_tcp",
                    tcp,
                    T_r,
                    seeds(q_r0, bearing("right", centre)),
                )
                if q_r is None or not wrist_ok(q_r):
                    continue
                for roll in ROLLS:
                    T_l = T_w_m @ left_on_marker(m, roll)
                    q_l = solve(
                        arms.model,
                        "left",
                        q_r,
                        "left/fr3_hand_tcp",
                        tcp,
                        T_l,
                        seeds(q_l0, bearing("left", centre)),
                    )
                    if q_l is None or not wrist_ok(q_l):
                        continue
                    # The line it comes in along, and the pull back out of the cap.
                    path, prev, ok = [], q_l, True
                    for back in (TAKE_FROM, TAKE_FROM * 0.5, 0.0, UNCAP_PULL):
                        T_k = T_w_m @ left_on_marker(m, roll, back)
                        q_k = solve(
                            arms.model, "left", q_r, "left/fr3_hand_tcp", tcp, T_k, [prev, q_l]
                        )
                        if q_k is None or not wrist_ok(q_k):
                            ok = False
                            break
                        path.append(q_k)
                        prev = q_k
                    if not ok:
                        continue
                    low = []
                    for qq in path:
                        arms.model.set(q_r, qq)
                        low.append(
                            min(lowest_part(arms.model, "right"), lowest_part(arms.model, "left"))
                        )
                    if min(low) < MIN_PART_HEIGHT:
                        continue
                    if (
                        min(
                            guard._gaps("right", list(q_r), list(qq)).slack(guard.config)
                            for qq in path
                        )
                        < 0.005
                    ):
                        continue
                    arms.model.set(q_r, q_l)
                    if not arms.model.world.is_collision_free(arms.model.ctx):
                        continue
                    gaps = guard._gaps("right", list(q_r), list(q_l))
                    turns = np.r_[np.abs(q_r - q_r0), np.abs(path[0] - q_l0)]
                    found.append(
                        dict(
                            cost=float(turns.max() + 0.2 * turns.sum()),
                            forward=f,
                            height=h,
                            heading=hd,
                            roll=roll,
                            T_w_m=T_w_m.tolist(),
                            q_right=q_r.tolist(),
                            q_left_pre=path[0].tolist(),
                            q_left=q_l.tolist(),
                            q_left_out=path[-1].tolist(),
                            along=T_l[:3, 2].tolist(),
                            grippers=gaps.grippers,
                            arms=gaps.arms,
                        )
                    )
                    break
    print(f"  {len(found)} handovers found in {time.monotonic() - t0:.0f} s")
    return min(found, key=lambda c: c["cost"]) if found else None


def pen_pose(arms: Arms, paper: dict, height: float, q_r) -> np.ndarray | None:
    """Left arm straight down over the paper's centre, TCP ``height`` above its top."""
    T = np.eye(4)
    T[:3, :3] = np.diag([1.0, -1.0, -1.0])
    T[:3, 3] = [*paper["centre"], paper["top_z"] + height]
    q = solve(
        arms.model,
        "left",
        q_r,
        "left/fr3_hand_tcp",
        np.eye(4),
        T,
        seeds(READY, bearing("left", T[:3, 3])),
        free_yaw=True,
    )
    return q if q is not None and wrist_ok(q) else None


def stage_plan(arms: Arms) -> None:
    st = load()
    m, paper = st["marker"], st["paper"]
    if m["diameter"] > ho.MAX_OPENING - 0.01:
        raise Failed("the marker is too thick for the Hand")
    T = pick_pose(m)
    pre = T.copy()
    pre[2, 3] += APPROACH
    tcp = np.eye(4)
    q_pre = solve(
        arms.model,
        "right",
        READY,
        "right/fr3_hand_tcp",
        tcp,
        pre,
        seeds(READY, bearing("right", pre[:3, 3])),
    )
    if q_pre is None:
        raise Failed("the right arm cannot reach over the cap")
    print(f"pick: the cap at ({T[0, 3]:.3f}, {T[1, 3]:.3f}), TCP {T[2, 3] * 1000:.0f} mm up")
    print("searching handovers ...")
    h = handover(arms, m, READY, READY)
    if h is None:
        raise Failed("no handover found")
    print(
        f"handover: marker {h['forward'] * 100:.0f} cm out, {h['height'] * 100:.0f} cm up, turned "
        f"{h['heading']} deg; left hand rolled {h['roll']} deg; grippers {h['grippers'] * 100:.1f} cm, "
        f"arms {h['arms'] * 100:.1f} cm apart"
    )
    q_pen = pen_pose(arms, paper, (m["length"] - BACK_GRIP_IN) + 0.03, READY)
    if q_pen is None:
        raise Failed("the left arm cannot hold a pen over the paper")
    st["plan"] = {
        "T_pick": T.tolist(),
        "q_pick_pre": q_pre.tolist(),
        "handover": h,
        "q_pen": q_pen.tolist(),
    }
    save(st)
    print("saved the plan")


# ---------------------------------------------------------------- feeling for contact


def jacobian(arms: Arms, side: str, q) -> np.ndarray:
    """d(TCP position)/dq, 3 x 7, by finite differences on the model."""
    other_q = arms.q(ho.other(side))

    def p(qq):
        arms.model.set(qq, other_q) if side == "right" else arms.model.set(other_q, qq)
        return arms.model.pose(f"{side}/fr3_hand_tcp")[:3, 3]

    q = np.asarray(q, float)
    p0, J = p(q), np.zeros((3, 7))
    for i in range(7):
        dq = np.zeros(7)
        dq[i] = 1e-4
        J[:, i] = (p(q + dq) - p0) / 1e-4
    return J


def torques(arms: Arms, side: str, n: int = 1) -> np.ndarray:
    """The arm's measured joint torques (tau_J), mean of n readings. The connection's
    joint_state stream also carries the Hand's width (no torques): those are skipped."""
    mod = arms.app.get_module(f"{side}_arm")
    rows = []
    while len(rows) < n:
        msg = mod.peek_stream("joint_state", 2.0)
        if msg is None:  # the arm stopped sending (a fault): say so instead of crashing
            raise Failed(f"no joint readings from the {side} arm (has it faulted?)")
        effort = list(msg.effort or [])
        if len(effort) >= 7:
            rows.append(np.asarray(effort[:7], float))
    return np.mean(rows, 0)


def push_until(
    arms: Arms, side: str, d, force: float, speed: float, what: str
) -> tuple[bool, float]:
    """Move the TCP straight by d (m), stopping early once the force against the motion
    passes ``force`` (N), from the joint torques. Returns (touched, how far it went)."""
    d = np.asarray(d, float)
    dist = float(np.linalg.norm(d))
    unit = d / dist
    J = jacobian(arms, side, arms.q(side))
    tau0 = torques(arms, side, 15)
    start, refused = arms.tcp(side), arms.refused()
    print(f"  {side} {what}: up to {dist * 1000:.0f} mm, stopping at {force:.0f} N")
    r = arms.mm.move_linear(
        dx=float(d[0]),
        dy=float(d[1]),
        dz=float(d[2]),
        planning_group=side,
        speed_scale=speed,
        blocking=False,
    )
    if not r.plan.succeeded:
        raise Failed(f"{side} {what}: no straight path ({r})")
    touched, peak, t0 = False, 0.0, time.monotonic()
    while time.monotonic() - t0 < 30.0:
        F = np.linalg.lstsq(J.T, torques(arms, side) - tau0, rcond=None)[0]
        along = abs(float(F @ unit))
        peak = max(peak, along)
        if along > force:
            arms.mm.cancel()
            touched = True
            break
        if float((arms.tcp(side) - start) @ unit) > dist - 0.0005:
            break
    time.sleep(0.5)
    went = float((arms.tcp(side) - start) @ unit)
    print(
        f"  {side} {what}: {'touched' if touched else 'no contact'} after {went * 1000:.1f} mm "
        f"(peak {peak:.1f} N)"
    )
    arms.check(side, refused)
    return touched, went


# ---------------------------------------------------------------- stages


def stage_pick(arms: Arms) -> None:
    st = load()
    m, plan = st["marker"], st["plan"]
    arms.add_table()
    arms.gripper("right", 1.0, "open")
    arms.move("right", plan["q_pick_pre"], "above the cap", FREE)
    arms.straight("right", -(APPROACH - LAST), "down near the cap", NEAR)
    arms.straight("right", -LAST, "down to the cap", TOUCH)
    w = arms.gripper("right", 0.0, "close on the cap")
    print(f"  right Hand stopped at {w * 1000:.1f} mm (marker about {m['diameter'] * 1000:.0f} mm)")
    if not 0.5 * m["diameter"] < w < m["diameter"] + 0.01:
        arms.gripper("right", 1.0, "open (missed)")
        arms.straight("right", APPROACH, "back up")
        raise Failed("the right Hand is not holding the marker")
    arms.straight("right", APPROACH, "lift", AWAY)
    st["held"] = True
    save(st)


def stage_present(arms: Arms) -> None:
    st = load()
    h = st["plan"]["handover"]
    arms.move("right", h["q_right"], "holds the marker up", CARRY)


def stage_take(arms: Arms) -> None:
    st = load()
    h, m = st["plan"]["handover"], st["marker"]
    arms.gripper("left", 1.0, "open")
    along = np.asarray(h["along"])
    arms.move("left", h["q_left_pre"], "lines up behind the marker", FREE)
    arms.along("left", (TAKE_FROM - LAST) * along, "comes in along the marker", NEAR)
    arms.along("left", LAST * along, "closes in on the end", TOUCH)
    w = arms.gripper("left", 0.0, "close on the back end")
    print(f"  left Hand stopped at {w * 1000:.1f} mm")
    if not 0.4 * m["diameter"] < w < m["diameter"] + 0.01:
        arms.gripper("left", 1.0, "open (missed)")
        arms.along("left", -TAKE_FROM * np.asarray(h["along"]), "backs out")
        raise Failed("the left Hand is not holding the marker")
    st["q_left_grip"] = arms.q("left")
    st["tcp_left_grip"] = arms.tcp("left").tolist()
    save(st)


def stage_retake(arms: Arms) -> None:
    """The left arm lost the marker (the right still holds it up): open, back off, and
    take it again with the left poses re-solved for the present BACK_GRIP_IN."""
    st = load()
    h, m = st["plan"]["handover"], st["marker"]
    T_w_m = np.array(h["T_w_m"])
    q_r = arms.q("right")
    arms.at("right", h["q_right"])
    arms.gripper("left", 1.0, "open")
    tcp = np.eye(4)
    prev = arms.q("left")
    sol = {}
    for name, back in (("q_left_out", UNCAP_PULL), ("q_left_pre", TAKE_FROM), ("q_left", 0.0)):
        T = T_w_m @ left_on_marker(m, h["roll"], back)
        q = solve(arms.model, "left", q_r, "left/fr3_hand_tcp", tcp, T, [prev, READY])
        if q is None or not wrist_ok(q):
            raise Failed(f"the left arm cannot reach {name}")
        sol[name], prev = q.tolist(), q
    h.update(sol)
    save(st)
    print(f"  left re-solved to grip {BACK_GRIP_IN * 100:.1f} cm in from the back end")
    stage_take(arms)


def stage_uncap(arms: Arms) -> None:
    st = load()
    h = st["plan"]["handover"]
    w_r0, w_l0 = arms.finger("right"), arms.finger("left")
    arms.along(
        "left", -UNCAP_PULL * np.asarray(h["along"]), "pulls the marker out of the cap", TOUCH
    )
    w_r, w_l = arms.finger("right"), arms.finger("left")
    print(
        f"  fingers: right {w_r0 * 1000:.1f} -> {w_r * 1000:.1f} mm, left {w_l0 * 1000:.1f} -> {w_l * 1000:.1f} mm"
    )
    if w_r < 0.004:
        raise Failed("the right Hand closed on nothing: the cap came away with the marker")
    if w_l < 0.004:
        raise Failed("the left Hand closed on nothing: the marker slipped out")
    st["q_left_out"] = arms.q("left")
    st["tcp_left_out"] = arms.tcp("left").tolist()
    save(st)
    aside = ASIDE_OUT * np.asarray(h["along"]) + np.array([0.0, 0.0, ASIDE_UP])
    arms.along("right", aside, "takes the cap up and out of the way", AWAY)
    st["aside"] = aside.tolist()
    save(st)


def stage_touch(arms: Arms) -> None:
    st = load()
    m, paper = st["marker"], st["paper"]
    q_pen = st["plan"]["q_pen"]
    arms.move("left", q_pen, "points the pen down over the paper", CARRY)
    # The pen tip is somewhere between a short and a long guess below the TCP.
    longest = m["length"] - BACK_GRIP_IN
    above = arms.tcp("left")[2] - paper["top_z"] - longest
    print(
        f"  the tip is at most {longest * 100:.1f} cm below the TCP: at least {above * 1000:.0f} mm above the paper"
    )
    arms.along(
        "left", [0.0, 0.0, -(above - 0.01)], "down to 1 cm over the paper (longest pen)", NEAR
    )
    touched, went = push_until(
        arms, "left", [0.0, 0.0, -0.07], TOUCH_FORCE, TOUCH_SPEED, "feels for the paper"
    )
    if not touched:
        raise Failed("the pen never touched the paper")
    z_touch = float(arms.tcp("left")[2])
    arms.straight("left", PEN_LIFT, "lifts the pen", AWAY)
    st["z_touch"] = z_touch
    st["tip_below_tcp"] = float(z_touch - paper["top_z"])
    save(st)
    print(
        f"  the pen touches with the TCP {z_touch * 1000:.1f} mm up; its tip "
        f"{st['tip_below_tcp'] * 1000:.0f} mm below the TCP"
    )


def stage_draw(arms: Arms) -> None:
    st = load()
    strokes = smiley(st["paper"])
    z_draw = st["z_touch"] - PRESS
    start = arms.tcp("left")
    # Over the first stroke's start, pen up, then every stroke: down, along, up, across.
    first = strokes[0][0]
    arms.along(
        "left",
        [first[0] - start[0], first[1] - start[1], (z_draw + PEN_LIFT) - start[2]],
        "over the first stroke",
        NEAR,
    )
    p0 = arms.tcp("left")
    way = []
    for stroke in strokes:
        s = stroke[0]
        way.append((s[0] - p0[0], s[1] - p0[1], 0.0))
        way += [(p[0] - p0[0], p[1] - p0[1], -PEN_LIFT) for p in stroke]
        e = stroke[-1]
        way.append((e[0] - p0[0], e[1] - p0[1], 0.0))
    way = [
        w
        for i, w in enumerate(way)
        if np.linalg.norm(np.subtract(w, way[i - 1] if i else (0, 0, 0))) > 1e-4
    ]
    print(
        f"  left draws: {len(strokes)} strokes, {len(way)} waypoints, pen pressed {PRESS * 1000:.1f} mm"
    )
    refused = arms.refused()
    r = arms.mm.move_linear_path(
        waypoints=way,
        planning_group="left",
        speed_scale=1.0,
        max_linear_speed=DRAW_SPEED,
        blend_deviation=0.0005,
        blocking=True,
        timeout=180.0,
    )
    time.sleep(0.5)
    print(f"  left draws: {getattr(r, 'execution', r)}")
    arms.check("left", refused)
    if "COMPLETED" not in str(getattr(r, "execution", r)):
        raise Failed(f"drawing: {r}")
    arms.move("left", st["plan"]["q_pen"], "lifts the pen", CARRY)


def stage_recap(arms: Arms) -> None:
    st = load()
    h = st["plan"]["handover"]
    along = np.asarray(h["along"])
    arms.move("left", st["q_left_out"], "back to where it left the cap", CARRY)
    arms.along("right", -np.asarray(st["aside"]), "brings the cap back", NEAR)
    arms.at("right", h["q_right"])
    arms.at("left", st["q_left_out"])
    back = np.asarray(st["tcp_left_grip"]) - arms.tcp("left")
    print(
        f"  {np.linalg.norm(back) * 1000:.0f} mm back to where the cap was on, "
        f"{math.degrees(math.acos(np.clip(back @ along / np.linalg.norm(back), -1, 1))):.1f} deg off the marker's line"
    )
    arms.along(
        "left", back - 0.01 * along, "pushes the marker back into the cap (to 1 cm short)", TOUCH
    )
    touched, went = push_until(
        arms, "left", (0.01 + PUSH_EXTRA) * along, PUSH_FORCE, 0.05, "seats the cap"
    )
    print(f"  ended {(went - 0.01) * 1000:+.1f} mm from where the cap was on")


def stage_release(arms: Arms) -> None:
    st = load()
    h = st["plan"]["handover"]
    arms.gripper("left", 1.0, "lets go of the marker")
    arms.along("left", -TAKE_FROM * np.asarray(h["along"]), "backs out", AWAY)
    arms.move("left", READY, "to ready", FREE)


def stage_place(arms: Arms) -> None:
    st = load()
    plan = st["plan"]
    arms.move("right", plan["q_pick_pre"], "over where the marker was", CARRY)
    arms.straight("right", -(APPROACH - LAST), "down near the table", NEAR)
    arms.straight("right", -(LAST - 0.002), "sets it down", TOUCH)
    arms.gripper("right", 1.0, "lets go")
    arms.straight("right", APPROACH, "lifts away", AWAY)
    arms.move("right", READY, "to ready", FREE)
    st.pop("held", None)
    save(st)


STAGES = {
    "look": stage_look,
    "plan": stage_plan,
    "pick": stage_pick,
    "present": stage_present,
    "take": stage_take,
    "retake": stage_retake,
    "uncap": stage_uncap,
    "touch": stage_touch,
    "draw": stage_draw,
    "recap": stage_recap,
    "release": stage_release,
    "place": stage_place,
}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("stages", nargs="+", choices=[*STAGES, "all", "scene"])
    args = ap.parse_args()
    names = list(STAGES) if args.stages == ["all"] else args.stages
    arms = Arms(dry=False)
    try:
        for name in names:
            print(f"== {name}")
            t0 = time.monotonic()
            if name == "scene":
                draw_scene(arms, load())
            else:
                STAGES[name](arms)
            print(f"   {name} done in {time.monotonic() - t0:.0f} s")
    except Failed as e:
        print(f"STOPPED: {e}")
    finally:
        arms.close()


if __name__ == "__main__":
    main()
