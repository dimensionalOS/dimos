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

"""Shuffle loop for the piper-x-localize blueprint: scan the table from left, home and right, pick
every object in order, place it at a random free spot within reach, repeat.

A failed pick of an object is tried again from a new scan, up to --tries picks per object per
round; then the loop goes on to the next object.
usage: piper_x_shuffle.py OUT_DIR [--labels "golf ball" "tennis ball" ...] [--tries N] [--cycles N]
                          [--dry] [--seed S]
Stop: touch OUT_DIR/STOP (finishes the current place, returns home, exits).
"""

import argparse
import json
import math
from pathlib import Path
import random
import time
from typing import Any

import numpy as np
from numpy.typing import NDArray

from dimos.porcelain.dimos import Dimos

LABELS = [
    "white toothpaste tube",
    "black roll of tape",
    "tennis ball",
    "black marker",
    "golf ball",
    "red pet bottle cap",
]
# Base turn from home per view; home faces +x, +0.6 turns the camera towards +y (left, seen from behind the arm).
VIEWS = {"left": 0.6, "home": 0.0, "right": -0.6}
# Where a place spot is sampled: distance from the base (m) and bearing (deg, 0 is +x, where
# home faces). Below FAR_SIDE_BEARING the distance stops at FAR_SIDE_R.
PLACE_R = (0.20, 0.37)
PLACE_BEARING = (-45.0, 45.0)
FAR_SIDE_BEARING, FAR_SIDE_R = -20.0, 0.30
PLACE_CLEAR = 0.04
PLACE_MIN_MOVE = 0.06
PLACE_TRIES = 8
PLACE_LIFT = 0.0
SAME_OBJECT = 0.04
MERGE_RADIUS = 0.06
GATE_MAX_S = 3.0
HELD = (0.04, 0.95)
# Seconds of memory one scan reads.
SCAN_WINDOW_S = 2.0

ap = argparse.ArgumentParser()
ap.add_argument("out", type=Path)
ap.add_argument("--dry", action="store_true")
ap.add_argument("--cycles", type=int, default=25, help="rounds over all objects")
ap.add_argument("--labels", nargs="+", default=LABELS, help="objects to locate and shuffle")
ap.add_argument(
    "--tries", type=int, default=2, help="picks of one object per round before moving on"
)
ap.add_argument("--seed", type=int, default=int(time.time()))
args = ap.parse_args()
LABELS = args.labels
args.out.mkdir(parents=True, exist_ok=True)
STOP = args.out / "STOP"
STOP.unlink(missing_ok=True)
rng = random.Random(args.seed)
jsonl = open(args.out / "shuffle.jsonl", "a")
human = open(args.out / "shuffle.log", "a")


def log(event: str, **kv: Any) -> None:
    rec = {"t": round(time.time(), 3), "event": event, **kv}
    jsonl.write(json.dumps(rec, default=str) + "\n")
    jsonl.flush()
    line = time.strftime("%H:%M:%S ") + event + " " + " ".join(f"{k}={v}" for k, v in kv.items())
    human.write(line + "\n")
    human.flush()
    print(line, flush=True)


app = Dimos.connect()
skills: Any = app.ManipulationSkills
pnp: Any = app.PickAndPlaceModule
loc: Any = app.ManipulationLocalize
man: Any = app.ManipulationModule
seq = 0


def msg_of(r: Any) -> str:
    return str(getattr(r, "message", r))


def state() -> Any:
    return man.get_state().groups["manipulator"]


def head() -> float | None:
    span = loc.index_span()
    return None if span is None else float(span[1])


def move_to(view: str, why: str) -> tuple[bool, float, float]:
    t0 = time.perf_counter()
    joints = list(HOME)
    joints[0] += VIEWS[view]
    m = msg_of(skills.move_to_joints(",".join(f"{j:.4f}" for j in joints)))
    dt = time.perf_counter() - t0
    ok = "COMPLETED" in m
    if not ok:
        log("move_failed", why=why, view=view, msg=m[:160], s=round(dt, 3))
    return ok, dt, time.time()


def gate(h_before: float | None) -> float | None:
    """Wait for a frame captured after the move completed; None when none came."""
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < GATE_MAX_S:
        h = head()
        if h is not None and (h_before is None or h > h_before):
            return time.perf_counter() - t0
        time.sleep(0.02)
    log("gate_timeout", s=GATE_MAX_S)
    return None


def scan_once(prompts: list[str]) -> tuple[list[dict[str, Any]], list[str], float]:
    t0 = time.perf_counter()
    out: list[dict[str, Any]] = []
    unlifted: list[str] = []
    for o in loc.scan(prompts, SCAN_WINDOW_S):
        pts = np.asarray(loc.get_pointcloud(o["id"]).points_f32(), np.float64).reshape(-1, 3)
        if len(pts) == 0:
            unlifted.append(o["label"])
            continue
        lo, hi = pts.min(0), pts.max(0)
        out.append(
            {
                "id": o["id"],
                "label": o["label"],
                "center": ((lo + hi) / 2).round(4).tolist(),
                "size": (hi - lo).round(4).tolist(),
                "top": round(float(np.percentile(pts[:, 2], 95)), 4),
                "score": round(float(o["score"]), 3),
                "n": len(pts),
            }
        )
    return out, unlifted, time.perf_counter() - t0


def scan(prompts: list[str]) -> tuple[list[dict[str, Any]], list[str], float]:
    """One localize scan, retried once at once on a stack error."""
    for attempt in range(2):
        try:
            return scan_once(prompts)
        except Exception as exc:
            log("scan_error", attempt=attempt, error=str(exc).splitlines()[0][:200])
    return [], [], 0.0


def xy(c: Any) -> NDArray[np.float64]:
    return np.asarray(c[:2], np.float64)


def resolve_conflicts(found: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """One object per place: across labels, centres within SAME_OBJECT keep the higher score."""
    keep: list[dict[str, Any]] = []
    for f in sorted(found, key=lambda f: -f["score"]):
        clash = next(
            (k for k in keep if np.linalg.norm(xy(k["center"]) - xy(f["center"])) < SAME_OBJECT),
            None,
        )
        if clash is None:
            keep.append(f)
        else:
            log("label_conflict", kept=clash["label"], dropped=f["label"], at=f["center"][:2])
    return keep


def look(view: str, cycle: int, why: str) -> dict[str, Any] | None:
    """Move to a view, wait for a fresh frame, scan every label. The scan's ids are the only pickable ones
    afterwards, so the result is the new latest: its best hit per label."""
    global seq
    ok, t_move, h0 = move_to(view, why)
    if not ok:
        return None
    t_gate = gate(h0)
    if t_gate is None:
        return None
    found, unlifted, t_scan = scan(LABELS)
    seq += 1
    for f in found:
        f["view"], f["seq"] = view, seq
        log(
            "seen",
            cycle=cycle,
            seq=seq,
            view=view,
            label=f["label"],
            id=f["id"],
            center=f["center"],
            size=f["size"],
            top=f["top"],
            score=f["score"],
            n=f["n"],
        )
    best: dict[str, dict[str, Any]] = {}
    for f in sorted(found, key=lambda f: -f["score"]):
        best.setdefault(f["label"], f)
    hits = {f["label"]: f for f in resolve_conflicts(list(best.values()))}
    log(
        "scan",
        cycle=cycle,
        seq=seq,
        view=view,
        why=why,
        found=sorted(hits),
        unlifted=unlifted,
        move_s=round(t_move, 2),
        gate_s=round(t_gate, 2),
        scan_s=round(t_scan, 2),
    )
    return {"seq": seq, "view": view, "hits": hits, "found": found}


def survey(cycle: int, world: dict[str, dict[str, Any]]) -> dict[str, Any]:
    """Scan every label from left, home and right; map each label to its best sighting and its view."""
    sightings: list[dict[str, Any]] = []
    latest: dict[str, Any] = {"seq": None, "view": None, "hits": {}}
    for view in VIEWS:
        scanned = look(view, cycle, "survey")
        if scanned is None:
            continue
        sightings += scanned["found"]
        latest = scanned
    if latest["seq"] is None:
        raise SystemExit(
            "every survey move failed; the arm cannot plan, stopped instead of spinning"
        )
    merged: dict[str, list[dict[str, Any]]] = {}
    for f in sightings:
        same = [
            m
            for m in merged.get(f["label"], [])
            if np.linalg.norm(xy(m["center"]) - xy(f["center"])) < MERGE_RADIUS
        ]
        if same:
            if f["score"] > same[0]["score"]:
                merged[f["label"]][merged[f["label"]].index(same[0])] = f
        else:
            merged.setdefault(f["label"], []).append(f)
    best: list[dict[str, Any]] = []
    for label, cands in merged.items():
        cands.sort(key=lambda f: -f["score"])
        if len(cands) > 1:
            log(
                "duplicate_label",
                label=label,
                kept=cands[0]["center"][:2],
                others=[c["center"][:2] for c in cands[1:]],
            )
        best.append(cands[0])
    for f in resolve_conflicts(best):
        world[f["label"]] = {**f, "cycle": cycle}
        log(
            "object",
            cycle=cycle,
            label=f["label"],
            center=f["center"],
            size=f["size"],
            top=f["top"],
            score=f["score"],
            n=f["n"],
            view=f["view"],
        )
    for label in LABELS:
        if world.get(label, {}).get("cycle") != cycle:
            log("missing", cycle=cycle, label=label, last=world.get(label, {}).get("center"))
    return latest


def reachable(p: NDArray[np.float64]) -> bool:
    r = float(np.linalg.norm(p))
    b = math.degrees(math.atan2(p[1], p[0]))
    if not PLACE_BEARING[0] <= b <= PLACE_BEARING[1]:
        return False
    return PLACE_R[0] <= r <= (FAR_SIDE_R if b < FAR_SIDE_BEARING else PLACE_R[1])


def sample_place(
    label: str, world: dict[str, dict[str, Any]], size: list[float]
) -> NDArray[np.float64] | None:
    own = xy(world[label]["center"])
    half = 0.5 * max(size[0], size[1])
    for _ in range(500):
        r = rng.uniform(*PLACE_R)
        b = math.radians(rng.uniform(*PLACE_BEARING))
        p = np.array([r * math.cos(b), r * math.sin(b)])
        if not reachable(p) or np.linalg.norm(p - own) < PLACE_MIN_MOVE:
            continue
        others = [o for k, o in world.items() if k != label]
        if all(
            np.linalg.norm(p - xy(o["center"]))
            >= PLACE_CLEAR + half + 0.5 * max(o["size"][0], o["size"][1])
            for o in others
        ):
            return p
    return None


def place(
    label: str,
    world: dict[str, dict[str, Any]],
    grasp_z: float,
    pick_xy: NDArray[np.float64],
    size: list[float],
    cycle: int,
) -> None:
    """Random free spot within reach; planner refusals try the next sample; last resort is the pick spot.
    When every one is refused, the same again from home."""
    t0 = time.perf_counter()
    z = grasp_z + PLACE_LIFT
    tries = 0
    for attempt in range(2):
        sampled = [sample_place(label, world, size) for _ in range(PLACE_TRIES)]
        targets = [t for t in sampled if t is not None] + [pick_xy]
        for i, p in enumerate(targets):
            tries += 1
            m = msg_of(pnp.place_at(float(p[0]), float(p[1]), float(z)))
            # Released, but the retract after it stopped: the object is down at p all the same.
            if m.startswith("Place complete") or "after releasing" in m:
                world[label] = {
                    **world[label],
                    "center": [
                        round(float(p[0]), 4),
                        round(float(p[1]), 4),
                        world[label]["center"][2],
                    ],
                }
                log(
                    "place",
                    cycle=cycle,
                    label=label,
                    at=[round(float(p[0]), 4), round(float(p[1]), 4), round(z, 4)],
                    tries=tries,
                    fallback=i == len(targets) - 1,
                    s=round(time.perf_counter() - t0, 2),
                    retract_failed=not m.startswith("Place complete"),
                )
                return
            if "to release the object" in m:
                log("open_failed", label=label, msg=m[:160], gripper=state().gripper_position)
                raise SystemExit("gripper did not open at the place pose; stopped there")
            log(
                "place_refused",
                cycle=cycle,
                label=label,
                at=[round(float(p[0]), 3), round(float(p[1]), 3)],
                msg=m[:120],
            )
        if attempt == 0:
            move_to("home", f"place {label} from home")
    raise SystemExit(f"could not place {label} anywhere; holding it")


def pick_and_place(
    label: str, world: dict[str, dict[str, Any]], latest: dict[str, Any], cycle: int, attempt: int
) -> tuple[dict[str, Any], bool]:
    """One pick_object call. From the latest scan when it holds the label; otherwise from a scan at the
    view that saw it in the survey. Returns the latest scan and whether the pick failed."""
    source = "latest"
    hit = latest["hits"].get(label)
    if hit is None:
        source = world[label]["view"]
        scanned = look(source, cycle, f"pick {label}")
        if scanned is not None:
            latest = scanned
            hit = latest["hits"].get(label)
    if hit is None:
        log("skip", cycle=cycle, label=label, why=f"not in the scan from {source}")
        return latest, False
    # From here the object is picked, pushed or moved: the scan no longer holds it.
    latest["hits"].pop(label, None)
    common = dict(
        cycle=cycle,
        label=label,
        attempt=attempt,
        id=hit["id"],
        view=hit["view"],
        seq=hit["seq"],
        source=source,
        center=hit["center"],
        size=hit["size"],
        top=hit["top"],
        score=hit["score"],
        n=hit["n"],
    )
    t0 = time.perf_counter()
    try:
        res = pnp.pick_object(hit["id"])
    except Exception as exc:
        log(
            "pick_error",
            **common,
            error=str(exc).splitlines()[0][:200],
            s=round(time.perf_counter() - t0, 2),
        )
        return latest, True
    m, md = msg_of(res), dict(getattr(res, "metadata", {}) or {})
    dt = time.perf_counter() - t0
    if not m.startswith("Pick complete"):
        log("pick_failed", **common, msg=m[:200], meta=md, s=round(dt, 2))
        g = state().gripper_position
        if g is not None and HELD[0] < g < HELD[1]:
            raise SystemExit(
                f"the pick of {label} stopped holding it (opening {g:.2f}); its grasp height is unknown"
            )
        return latest, True
    g = pnp.get_grasp_candidates().candidates[int(md["rank"])].pose
    grasp = [
        round(float(v), 5)
        for v in (
            g.position.x,
            g.position.y,
            g.position.z,
            g.orientation.x,
            g.orientation.y,
            g.orientation.z,
            g.orientation.w,
        )
    ]
    log(
        "pick",
        **common,
        rank=md["rank"],
        grasp_score=round(float(md["score"]), 4),
        grasp=grasp,
        candidates=md.get("candidates"),
        rejected=md.get("contact_rejected"),
        s=round(dt, 2),
    )
    place(label, world, g.position.z, np.array([g.position.x, g.position.y]), hit["size"], cycle)
    return latest, False


def startup() -> None:
    """Refuse to start holding something: its grasp height is unknown, so it cannot be put down safely."""
    g = state().gripper_position
    if g is not None and HELD[0] < g < HELD[1]:
        raise SystemExit(
            f"the gripper is holding something (opening {g:.2f}); put it down by hand first"
        )


def wait_ready(limit: float = 120.0) -> None:
    t0 = time.perf_counter()
    while head() is None:
        if time.perf_counter() - t0 > limit:
            raise SystemExit("perception never became ready")
        time.sleep(0.5)


try:
    log(
        "start",
        seed=args.seed,
        dry=args.dry,
        labels=LABELS,
        views=VIEWS,
        cycles=args.cycles,
        tries=args.tries,
    )
    wait_ready()
    startup()
    HOME = list(state().joint_presets["home"].position)
    world: dict[str, dict[str, Any]] = {}
    if args.dry:
        survey(0, world)
        raise SystemExit(0)
    for cycle in range(1, args.cycles + 1):
        tc = time.perf_counter()
        latest = survey(cycle, world)
        for label in LABELS:
            if world.get(label, {}).get("cycle") != cycle:
                log("skip", cycle=cycle, label=label, why="not seen in the survey")
                continue
            for attempt in range(1, args.tries + 1):
                latest, failed = pick_and_place(label, world, latest, cycle, attempt)
                if not failed or STOP.exists():
                    break
            else:
                log("gave_up", cycle=cycle, label=label, tries=args.tries)
            if STOP.exists():
                break
        log("cycle_done", cycle=cycle, s=round(time.perf_counter() - tc, 1))
        if STOP.exists():
            log("stop_requested")
            break
    move_to("home", "end")
finally:
    app.stop()
