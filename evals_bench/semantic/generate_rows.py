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

"""Generate semantic-map VQA rows from detections.json (run once; frozen data).

Ground truth is derived here, independently of the evo target: detections are
clustered per class within MERGE_RADIUS_M, weak classes (<MIN_SIGHTINGS
sightings) and non-indoor-plausible COCO classes are dropped, and four named
zones come from the odom-track bounding-box quadrants.

Families
  presence   : did the map record a class — yes/no rows with priors
               anti-correlated with truth, plus 4-way "which one was recorded"
               MCQs (one mapped class among three verified-absent ones)
  count      : global object count per class (numeric, band +-max(1, 20%);
               classes where a generic 2-3 guess lands in-band are skipped)
  zonecount  : object count per class within a named quadrant (band 1: one-off
               answers get zero credit; empty zones skipped)
  nearest    : which class is nearest the robot at time t (MCQ)
  egoside    : is an object ahead/behind/left/right of the robot at time t
               (4-way MCQ; picked bearings sit >=EGO_MARGIN_DEG inside a band)
  compass    : 8-way compass relation between two objects >=3 m apart (MCQ)
  objdist    : horizontal distance between two objects >=3 m apart (numeric,
               band max(1.0, 25%); VQASynth-inspired; round 2 diversifies
               anchors beyond the alphabetically-first class)
  robotdist  : robot-to-object horizontal distance at time t (numeric, same
               band rule; timestamps >=15 s apart; VQASynth-inspired)
  recall     : list every mapped class, full map + zone-scoped (set-F1)
  within     : list classes with an instance within R m of an anchor; R
               chosen so no cluster sits within 1.5 m of the boundary (set-F1)
  nextto     : nearest class to an object, runner-up margin >=1.5 m (MCQ)
  between    : the one class in the corridor between two anchors (MCQ;
               exactly one qualifying cluster, corridor otherwise clear)

Set-answer rows carry a ``vocab`` (mapped + verified-absent classes) for
reply parsing; scored by set-F1 in benchmark.py. No bare yes/no proximity
binaries (blind-gate variance — nextto MCQ covers the relation).

Context parity (see benchmark.py header): truth always uses the FULL
detection span; time-conditioned rows (nearest/egoside/robotdist) carry an
odom_window that ends exactly at the question timestamp.
"""

from __future__ import annotations

from collections import defaultdict
import json
import math
from pathlib import Path

DATASET = "go2_bigoffice"
MERGE_RADIUS_M = 1.5
MIN_SIGHTINGS = 3
COMPASS_MIN_PAIR_DIST = 3.0  # grounding error ~1-2 m; closer pairs are noise
NEAREST_MARGIN_M = 1.0  # nearest class must beat runner-up by this margin
EGO_AHEAD_DEG = 40.0  # |rel bearing| <= this: ahead; >= 180-this: behind
EGO_MARGIN_DEG = 15.0  # picked bearings sit this far inside a band boundary
COMPASS_NAMES = [
    "east",
    "northeast",
    "north",
    "northwest",
    "west",
    "southwest",
    "south",
    "southeast",
]
# COCO classes implausible indoors — detector noise on an office run.
NON_INDOOR = {
    "car",
    "truck",
    "bus",
    "train",
    "airplane",
    "boat",
    "motorcycle",
    "bicycle",
    "traffic light",
    "fire hydrant",
    "stop sign",
    "parking meter",
    "bird",
    "cat",
    "dog",
    "horse",
    "sheep",
    "cow",
    "elephant",
    "bear",
    "zebra",
    "giraffe",
    "frisbee",
    "skis",
    "snowboard",
    "kite",
    "surfboard",
    "skateboard",
    "sports ball",
    "baseball bat",
    "baseball glove",
    "tennis racket",
}
# Moving objects break the static-map odom-grounding convention.
NON_STATIC = {"person"}
# Presence probes are chosen so an office prior anti-correlates with truth:
# "yes" cases are classes a blind guesser would not expect mapped in an office,
# "no" cases are office-plausible classes verified absent from raw detections.
PRESENT_SURPRISES = ["bed", "refrigerator", "teddy bear"]
ABSENT_CANDIDATES = ["couch", "sink"]  # hardest "no" probes: blind said yes to these
# 4-way forced choice (chance 0.25): the one mapped class is the option an
# office prior ranks LAST; the three distractors are verified-absent classes.
RECORDED_MCQS = [
    ("teddy bear", ["mouse", "teddy bear", "dining table", "cell phone"]),
    ("bed", ["clock", "oven", "bed", "scissors"]),
]


def cluster(detections: list[dict]) -> list[dict]:
    """Greedy same-class clustering within MERGE_RADIUS_M of the running mean."""
    clusters: list[dict] = []
    for det in sorted(detections, key=lambda d: d["ts"]):
        home = None
        for c in clusters:
            if c["class_name"] != det["class_name"]:
                continue
            cx, cy = c["sx"] / c["n"], c["sy"] / c["n"]
            if math.hypot(det["x"] - cx, det["y"] - cy) <= MERGE_RADIUS_M:
                home = c
                break
        if home is None:
            clusters.append(
                {
                    "class_name": det["class_name"],
                    "sx": det["x"],
                    "sy": det["y"],
                    "n": 1,
                }
            )
        else:
            home["sx"] += det["x"]
            home["sy"] += det["y"]
            home["n"] += 1
    for c in clusters:
        c["x"], c["y"] = c["sx"] / c["n"], c["sy"] / c["n"]
    return clusters


def compass_of(dx: float, dy: float) -> str:
    return COMPASS_NAMES[round(math.atan2(dy, dx) / (math.pi / 4)) % 8]


def robot_pose_at(detections: list[dict], t: float) -> dict:
    return min(detections, key=lambda d: abs(d["ts"] - t))


def main() -> None:
    here = Path(__file__).parent
    detections = json.loads((here / "detections.json").read_text())
    # "no" presence probes must be classes the detector NEVER saw — a faithful
    # map of the raw detections must agree the class is absent.
    ever_detected = {d["class_name"] for d in detections}
    detections = [d for d in detections if d["class_name"] not in NON_INDOOR | NON_STATIC]

    clusters = [c for c in cluster(detections) if c["n"] >= MIN_SIGHTINGS]
    by_class: dict[str, list[dict]] = defaultdict(list)
    for c in clusters:
        by_class[c["class_name"]].append(c)
    singles = {name: cs[0] for name, cs in by_class.items() if len(cs) == 1}
    print(f"{len(clusters)} clusters across {len(by_class)} classes")
    for name, cs in sorted(by_class.items(), key=lambda kv: -len(kv[1])):
        print(f"  {name:15s} {len(cs)} objects, sightings {[c['n'] for c in cs]}")

    # Zones: quadrants of the odom-track bounding box, +x east / +y north.
    xs = [d["x"] for d in detections]
    ys = [d["y"] for d in detections]
    mx, my = round((min(xs) + max(xs)) / 2, 1), round((min(ys) + max(ys)) / 2, 1)
    zones = {
        "northwest area": lambda x, y: x < mx and y >= my,
        "northeast area": lambda x, y: x >= mx and y >= my,
        "southwest area": lambda x, y: x < mx and y < my,
        "southeast area": lambda x, y: x >= mx and y < my,
    }
    zone_clause = (
        (
            f"The map is divided into four zones at the point (x={mx}, y={my}), "
            "with +x east and +y north: the northwest area (x<{mx}, y>={my}), northeast "
            "area (x>={mx}, y>={my}), southwest area (x<{mx}, y<{my}), and southeast area "
            "(x>={mx}, y<{my})."
        )
        .replace("{mx}", str(mx))
        .replace("{my}", str(my))
    )

    rows: list[dict] = []

    def add(row_id: str, family: str, q: str, a, *, ctx: str = "objects", **extra) -> None:
        if "vocab" in extra:
            kind = "set"
        elif isinstance(a, (int, float)) and "choices" not in extra:
            kind = "numeric"
        else:
            kind = "mcq"
        rows.append(
            {"id": row_id, "family": family, "type": kind, "q": q, "a": a, "ctx": ctx}
            | extra
            | {"dataset": DATASET}
        )

    # -- presence: priors anti-correlated with truth so blind guessing loses
    present = [n for n in PRESENT_SURPRISES if n in by_class]
    absent = [n for n in ABSENT_CANDIDATES if n not in ever_detected]
    for name in present + absent:
        truth = "yes" if name in by_class else "no"
        add(
            f"sm_presence_{name.replace(' ', '_')}",
            "presence",
            f"Did the robot's semantic object map record at least one "
            f"{name} anywhere in the mapped area? Answer with exactly one word: yes or no.",
            truth,
            choices=["yes", "no"],
        )
    for truth, options in RECORDED_MCQS:
        assert truth in by_class
        assert all(o not in ever_detected for o in options if o != truth)
        add(
            f"sm_presence_which_{truth.replace(' ', '_')}",
            "presence",
            f"Exactly one of these object classes was actually recorded in the "
            f"robot's semantic object map: {', '.join(options)}. Which one? "
            f"Answer with exactly one of: {', '.join(options)}.",
            truth,
            choices=list(options),
        )

    popular = sorted(by_class, key=lambda n: -sum(c["n"] for c in by_class[n]))

    # -- global counting per class (skip rows a generic small-count guess can hit)
    for name in popular:
        count = len(by_class[name])
        band = max(1, round(0.2 * count))
        if count < 5 and any(abs(g - count) <= band for g in (2, 3)):
            continue  # a blind "2 or 3" would land in-band
        add(
            f"sm_count_{name.replace(' ', '_')}",
            "count",
            f"Based on the semantic object map shown, how many distinct "
            f"{name} objects are in the mapped area? Answer with a single number.",
            count,
            band=band,
        )

    # -- zone counting: per zone, quiz the class most represented there
    for zone_name, inside in zones.items():
        name = max(popular[:6], key=lambda n: sum(1 for c in by_class[n] if inside(c["x"], c["y"])))
        count = sum(1 for c in by_class[name] if inside(c["x"], c["y"]))
        if count == 0:
            continue  # empty zone — a truth of 0 is prior-guessable
        add(
            f"sm_zonecount_{zone_name.split()[0]}_{name.replace(' ', '_')}",
            "zonecount",
            f"{zone_clause} How many distinct {name} objects are in the "
            f"{zone_name}? Answer with a single number.",
            count,
            band=1.0,
        )

    # -- nearest-class MCQ at sampled timestamps
    choices = sorted(by_class)
    duration = max(d["ts"] for d in detections)
    nearest_added = 0
    for t in [duration * f / 20 for f in range(1, 20)]:
        if nearest_added >= 5:
            break
        pose = robot_pose_at(detections, t)
        dists = {
            name: min(math.hypot(c["x"] - pose["x"], c["y"] - pose["y"]) for c in cs)
            for name, cs in by_class.items()
        }
        ranked = sorted(dists.items(), key=lambda kv: kv[1])
        if ranked[1][1] - ranked[0][1] < NEAREST_MARGIN_M:
            continue  # ambiguous — skip timestamp
        if any(
            r["family"] == "nearest" and abs(r["odom_window"][1] - pose["ts"]) < 15.0 for r in rows
        ):
            continue  # keep quizzed timestamps well separated
        nearest_added += 1
        add(
            f"sm_nearest_t{pose['ts']:g}",
            "nearest",
            f"You are the robot; your current pose is the last odom observation "
            f"shown. Based on the semantic object map, which object class is "
            f"horizontally nearest to you? Answer with exactly one of: "
            f"{', '.join(choices)}.",
            ranked[0][0],
            choices=choices,
            ctx="objects+odom",
            odom_window=[round(max(0.0, pose["ts"] - 0.5), 2), pose["ts"]],
        )

    # -- egocentric ahead/behind/left/right at sampled timestamps
    # (single-instance classes, rotating so one object doesn't dominate)
    ego_added = 0
    single_names = sorted(singles)
    for i, t in enumerate([duration * f for f in (0.15, 0.3, 0.45, 0.6, 0.75, 0.9)]):
        if ego_added >= 6:
            break
        pose = robot_pose_at(detections, t)
        rotation = single_names[i % len(single_names) :] + single_names[: i % len(single_names)]
        for name in rotation:
            c = singles[name]
            bearing = math.atan2(c["y"] - pose["y"], c["x"] - pose["x"])
            rel = math.atan2(math.sin(bearing - pose["yaw"]), math.cos(bearing - pose["yaw"]))
            # 4-way bands on |rel|: ahead <=40, behind >=140, else side by sign;
            # only pick bearings >=EGO_MARGIN_DEG inside a band boundary.
            rel_deg = abs(math.degrees(rel))
            if rel_deg <= EGO_AHEAD_DEG - EGO_MARGIN_DEG:
                truth = "ahead"
            elif rel_deg >= 180.0 - EGO_AHEAD_DEG + EGO_MARGIN_DEG:
                truth = "behind"
            elif (
                EGO_AHEAD_DEG + EGO_MARGIN_DEG <= rel_deg <= 180.0 - EGO_AHEAD_DEG - EGO_MARGIN_DEG
            ):
                # positive relative bearing -> object on the left
                truth = "left" if rel > 0 else "right"
            else:
                continue  # too close to a band boundary — ambiguous
            add(
                f"sm_egoside_t{pose['ts']:g}_{name.replace(' ', '_')}",
                "egoside",
                f"You are the robot; your current pose is the last odom "
                f"observation shown, and you face your direction of heading "
                f"(the odom yaw). Based on the semantic object map, is the "
                f"{name} ahead of you, behind you, on your left, or on your "
                f"right? Answer with exactly one word: ahead, behind, left, "
                f"or right.",
                truth,
                choices=["ahead", "behind", "left", "right"],
                ctx="objects+odom",
                odom_window=[round(max(0.0, pose["ts"] - 0.5), 2), pose["ts"]],
            )
            ego_added += 1
            break  # one object per timestamp

    # -- object<->object compass relations (single-instance classes, >=3 m apart)
    names = sorted(singles)
    compass_added = 0
    for i, a in enumerate(names):
        for b in names[i + 1 :]:
            if compass_added >= 5:
                break
            ca, cb = singles[a], singles[b]
            if math.hypot(ca["x"] - cb["x"], ca["y"] - cb["y"]) < COMPASS_MIN_PAIR_DIST:
                continue
            add(
                f"sm_compass_{a.replace(' ', '_')}_{b.replace(' ', '_')}",
                "compass",
                f"Based on the semantic object map (world frame, +x east, +y "
                f"north), in which compass direction is the {a} from the {b}? "
                f"Answer with exactly one word: "
                f"{', '.join(COMPASS_NAMES)}.",
                compass_of(ca["x"] - cb["x"], ca["y"] - cb["y"]),
                choices=list(COMPASS_NAMES),
            )
            compass_added += 1

    # -- object<->object distances (VQASynth-inspired; single-instance, >=3 m
    # apart so the odom-grounding error stays small relative to truth)
    objdist_added = 0
    objdist_pairs: set[tuple[str, str]] = set()
    for i, a in enumerate(names):
        for b in names[i + 1 :]:
            if objdist_added >= 4:
                break
            ca, cb = singles[a], singles[b]
            dist = math.hypot(ca["x"] - cb["x"], ca["y"] - cb["y"])
            if dist < COMPASS_MIN_PAIR_DIST:
                continue
            add(
                f"sm_objdist_{a.replace(' ', '_')}_{b.replace(' ', '_')}",
                "objdist",
                f"Based on the semantic object map, how far apart horizontally "
                f"are the {a} and the {b}, in meters? Answer with a single number.",
                round(dist, 2),
                band=max(1.0, round(0.25 * dist, 2)),
            )
            objdist_pairs.add((a, b))
            objdist_added += 1

    # -- robot-to-object distances at sampled timestamps (VQASynth-inspired;
    # single-instance classes, quizzed timestamps >=15 s apart)
    robotdist_added = 0
    for t in [duration * f for f in (0.2, 0.5, 0.8, 0.35, 0.65)]:
        if robotdist_added >= 3:
            break
        pose = robot_pose_at(detections, t)
        if any(
            r["family"] == "robotdist" and abs(r["odom_window"][1] - pose["ts"]) < 15.0
            for r in rows
        ):
            continue
        name = single_names[robotdist_added % len(single_names)]
        c = singles[name]
        dist = math.hypot(c["x"] - pose["x"], c["y"] - pose["y"])
        add(
            f"sm_robotdist_t{pose['ts']:g}_{name.replace(' ', '_')}",
            "robotdist",
            f"You are the robot; your current pose is the last odom observation "
            f"shown. Based on the semantic object map, how far are you "
            f"horizontally from the {name}, in meters? Answer with a single "
            f"number.",
            round(dist, 2),
            band=max(1.0, round(0.25 * dist, 2)),
            ctx="objects+odom",
            odom_window=[round(max(0.0, pose["ts"] - 0.5), 2), pose["ts"]],
        )
        robotdist_added += 1

    # -- objdist round 2 (appended so pre-existing row positions are stable):
    # diversify anchors — round 1 concentrates on the alphabetically-first
    # class; one new pair per other anchor
    for a in names:
        if objdist_added >= 8:
            break
        if a == names[0]:
            continue
        for b in names:
            pair = tuple(sorted((a, b)))
            if b == a or pair in objdist_pairs:
                continue
            ca, cb = singles[a], singles[b]
            dist = math.hypot(ca["x"] - cb["x"], ca["y"] - cb["y"])
            if dist < COMPASS_MIN_PAIR_DIST:
                continue
            add(
                f"sm_objdist_{a.replace(' ', '_')}_{b.replace(' ', '_')}",
                "objdist",
                f"Based on the semantic object map, how far apart horizontally "
                f"are the {a} and the {b}, in meters? Answer with a single number.",
                round(dist, 2),
                band=max(1.0, round(0.25 * dist, 2)),
            )
            objdist_pairs.add(pair)
            objdist_added += 1
            break  # one new pair per anchor

    # vocabulary for set-answer parsing: mapped classes + verified-absent
    # probes so hallucinated extras cost F1 precision. The probe list is
    # office-plausible COCO classes (things a blind prior WOULD list) filtered
    # to those never detected — the larger it is, the lower the F1 floor for
    # exhaustive or prior-based listing.
    office_probes = [
        "backpack",
        "bowl",
        "cell phone",
        "clock",
        "couch",
        "dining table",
        "handbag",
        "keyboard",
        "microwave",
        "mouse",
        "oven",
        "remote",
        "scissors",
        "sink",
        "suitcase",
        "toaster",
        "umbrella",
        "vase",
        "wine glass",
    ]
    set_vocab = choices + sorted(
        {p for p in office_probes if p not in ever_detected}
        | set(absent)
        | {o for t, opts in RECORDED_MCQS for o in opts if o != t}
    )
    list_clause = "Answer with a comma-separated list of class names."

    # -- recall: list every mapped class (full map + two zone-scoped variants)
    add(
        "sm_recall_all",
        "recall",
        f"List every distinct object class recorded in the robot's semantic "
        f"object map. {list_clause}",
        sorted(by_class),
        vocab=set_vocab,
    )
    recall_zones = 0
    for zone_name, inside in zones.items():
        if recall_zones >= 2:
            break
        zone_classes = sorted(
            n for n, cs in by_class.items() if any(inside(c["x"], c["y"]) for c in cs)
        )
        if not (2 <= len(zone_classes) <= len(by_class) - 2):
            continue  # empty or near-total zones are prior-guessable
        add(
            f"sm_recall_{zone_name.split()[0]}",
            "recall",
            f"{zone_clause} List every distinct object class recorded in the "
            f"{zone_name}. {list_clause}",
            zone_classes,
            vocab=set_vocab,
        )
        recall_zones += 1

    # -- within-radius set listing; anchor+R chosen so NO cluster sits within
    # 1.5 m of the boundary (odom-grounding hysteresis, asserted by the skip)
    within_added = 0
    for x in single_names:
        if within_added >= 3:
            break
        ax, ay = singles[x]["x"], singles[x]["y"]
        others = [c for c in clusters if c is not singles[x]]
        dists = [math.hypot(c["x"] - ax, c["y"] - ay) for c in others]
        best: tuple[int, float, list[str]] | None = None  # (len(truth), -R, truth)
        for radius in (2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0):
            if any(abs(d - radius) < 1.5 for d in dists):
                continue  # a cluster rides the boundary — ambiguous under grounding error
            truth = sorted(
                {c["class_name"] for c, d in zip(others, dists, strict=True) if d < radius}
            )
            if not truth:
                continue  # empty truth is prior-guessable ("none")
            if best is None or (len(truth), -radius) > (best[0], best[1]):
                best = (len(truth), -radius, truth)
        if best is not None:
            radius = -best[1]
            add(
                f"sm_within_{x.replace(' ', '_')}_{radius:g}m",
                "within",
                f"Based on the semantic object map, list every object class other "
                f"than {x} with at least one instance within {radius:g} meters "
                f"horizontally of the {x}. {list_clause}",
                best[2],
                vocab=set_vocab,
            )
            within_added += 1

    # -- nextto: nearest class to an object (MCQ; runner-up margin >= 1.5 m)
    nextto_added = 0
    for x in single_names:
        if nextto_added >= 3:
            break
        ax, ay = singles[x]["x"], singles[x]["y"]
        ranked = sorted(
            (min(math.hypot(c["x"] - ax, c["y"] - ay) for c in cs), n)
            for n, cs in by_class.items()
            if n != x
        )
        if ranked[1][0] - ranked[0][0] < 1.5:
            continue  # ambiguous under grounding error
        opts = [n for n in choices if n != x]
        add(
            f"sm_nextto_{x.replace(' ', '_')}",
            "nextto",
            f"Based on the semantic object map, which object class is "
            f"horizontally nearest to the {x} (other than {x} itself)? "
            f"Answer with exactly one of: {', '.join(opts)}.",
            ranked[0][1],
            choices=opts,
        )
        nextto_added += 1

    # -- between: exactly one cluster in the corridor between two anchors
    # (perp < 1.5 m, projection in the middle 60%; no other cluster within
    # 2.5 m of the corridor, else the pair is skipped)
    between_added = 0
    for i, a in enumerate(single_names):
        for b in single_names[i + 1 :]:
            if between_added >= 2:
                break
            ca, cb = singles[a], singles[b]
            vx, vy = cb["x"] - ca["x"], cb["y"] - ca["y"]
            seg2 = vx * vx + vy * vy
            if seg2 < COMPASS_MIN_PAIR_DIST**2:
                continue
            inside_corridor = []
            near_corridor = 0
            for c in clusters:
                if c is ca or c is cb:
                    continue
                s = ((c["x"] - ca["x"]) * vx + (c["y"] - ca["y"]) * vy) / seg2
                perp = abs((c["x"] - ca["x"]) * vy - (c["y"] - ca["y"]) * vx) / math.sqrt(seg2)
                if not 0.2 <= s <= 0.8:
                    continue
                if perp < 1.5:
                    inside_corridor.append(c)
                elif perp < 2.5:
                    near_corridor += 1
            if len(inside_corridor) != 1 or near_corridor:
                continue
            mid = inside_corridor[0]["class_name"]
            if mid in (a, b):
                continue
            opts = [n for n in choices if n not in (a, b)]
            add(
                f"sm_between_{a.replace(' ', '_')}_{b.replace(' ', '_')}",
                "between",
                f"Based on the semantic object map, which single object class "
                f"lies between the {a} and the {b}? Answer with exactly one of: "
                f"{', '.join(opts)}.",
                mid,
                choices=opts,
            )
            between_added += 1

    out = here / "rows.json"
    out.write_text(json.dumps(rows, indent=2) + "\n")  # matches pretty-format-json hook
    families = defaultdict(int)
    for r in rows:
        families[r["family"]] += 1
    print(f"\nwrote {len(rows)} rows -> {out}")
    for fam, n in families.items():
        print(f"  {fam:10s} {n}")
    for r in rows:
        print(json.dumps(r))


if __name__ == "__main__":
    main()
