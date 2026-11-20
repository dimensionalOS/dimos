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

"""Run every rl_deploy policy through its scripted sequences headless and print pass / fail.

    python -m dimos.simulation.m20_policy_sim.run_all [--only name ...] [--json results.json]

Policies are read from M20_POLICY_DIR (default ~/.cache/dimos/m20_policies); see extract_policies.py.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor
import json
import math
import os
from typing import Any

import numpy as np

from dimos.simulation.m20_policy_sim.config import load_config, policy_dir
from dimos.simulation.m20_policy_sim.controller import Controller
from dimos.simulation.m20_policy_sim.sequences import (
    Segment,
    Sequence,
    entry_segments,
    run_sequence,
    stand_only_segments,
    walk_segments,
)
from dimos.simulation.m20_policy_sim.world import Terrain, World

WALKERS = (
    "basic_flat",
    "basic_plat",
    "basic_stair",
    "basic_backknee",
    "navi_charge",
    "navi_flat",
    "navi_orienteer",
    "stand_policy",
    "height_gap",
    "extre",
    "step_move",
    "unreferenced_m20_38",
    "unreferenced_rough_6",
)
STANDERS = ("finetune_stand", "six_axis_stand")
CLIMBERS = ("basic_stair", "navi_orienteer", "height_gap", "navi_flat")
# dock scenario: base origin this far in front of the dock wall when docked, P gain (1/s) and speed cap (m/s)
DOCK = dict(docked_clearance_m=0.45, approach_gain=1.0, approach_speed=0.3)
STAIRS = Terrain(kind="stairs", distance=1.0, height=0.08, depth=0.35, steps=3)


def climbed(world: World, log: list[dict[str, Any]]) -> dict[str, tuple[float, bool]]:
    top = STAIRS.height * STAIRS.steps
    x_top = STAIRS.distance + STAIRS.depth * (STAIRS.steps - 1) + 0.4
    stand_z = np.mean([r["pos"][2] for r in log if r["seg"] == "stand"])
    z_gain = max(r["pos"][2] for r in log[-100:]) - stand_z
    best_x = max(r["pos"][0] for r in log)
    return {"reached_top_x": (best_x, best_x > x_top), "z_gain_m": (z_gain, z_gain > 0.8 * top)}


def pose_checks(cfg: dict[str, Any], runner: str):
    """The stand policies follow the decoded pose inputs: height lowers the body, pitch / roll tilt it."""
    params = cfg["runners"][runner]["command_vector"]
    roll_gain = math.degrees(
        next(e["scale"] for e in params if isinstance(e, dict) and e.get("name") == "roll")
    )
    pitch_gain = math.degrees(
        next(e["scale"] for e in params if isinstance(e, dict) and e.get("name") == "pitch")
    )

    def checks(world: World, log: list[dict[str, Any]]) -> dict[str, tuple[float, bool]]:
        def mean(seg: str, key: str, index: int | None = None) -> float:
            rows = [r for r in log if r["seg"] == seg]
            rows = rows[len(rows) // 2 :]
            return float(np.mean([r[key][index] if index is not None else r[key] for r in rows]))

        lowered = mean("stand", "pos", 2) - mean("lower", "pos", 2)
        pitch = math.degrees(mean("pitch", "pitch"))
        roll = math.degrees(mean("roll", "roll"))
        return {
            "lowered_m": (lowered, lowered > 0.1),
            "pitch_deg": (pitch, abs(abs(pitch) - 0.8 * pitch_gain) < 5.0),
            "roll_deg": (roll, abs(abs(roll) - 0.8 * roll_gain) < 4.0),
        }

    return checks


def stairs_sequence(cfg: dict[str, Any], runner: str) -> Sequence:
    segs = [
        *entry_segments(cfg, runner),
        Segment("stand", 2.0, "rl_control"),
        Segment("climb", 12.0, "rl_control", (0.5, 0, 0)),
        Segment("stop", 2.0, "rl_control"),
    ]
    return Sequence(f"stairs:{runner}", runner, segs, terrain=STAIRS, checks=climbed)


def dock_sequence(cfg: dict[str, Any]) -> Sequence:
    """NaviCharge backs onto a dock 1 m behind, holds, then drives off forward."""
    terrain = Terrain(kind="dock", dock_distance=1.0)
    docked_x = -terrain.dock_distance + DOCK["docked_clearance_m"]
    gain, vmax = DOCK["approach_gain"], DOCK["approach_speed"]

    def toward(x_goal: float):
        def steer(world: World) -> np.ndarray:
            pos, quat = world.base_pose()
            w, _, _, z = quat
            yaw = 2 * math.atan2(z, w)
            vx = float(np.clip(gain * (x_goal - pos[0]) * math.cos(yaw), -vmax, vmax))
            wz = float(np.clip(-2.0 * yaw, -0.3, 0.3))
            vy = float(np.clip(-gain * pos[1], -0.1, 0.1))
            return np.array([vx, vy, wz])

        return steer

    segs = [
        *entry_segments(cfg, "navi_charge"),
        Segment("stand", 2.0, "rl_control"),
        Segment("dock", 10.0, "rl_control", steer=toward(docked_x)),
        Segment("docked", 3.0, "rl_control"),
        Segment("undock", 8.0, "rl_control", steer=toward(0.0)),
        Segment("lie_down", 5.0, "lie_down"),
    ]

    def checks(world: World, log: list[dict[str, Any]]) -> dict[str, tuple[float, bool]]:
        docked = [r for r in log if r["seg"] == "docked"]
        undocked = [r for r in log if r["seg"] == "undock"][-1]
        dock_err = abs(np.mean([r["pos"][0] for r in docked]) - docked_x)
        lat = max(abs(r["pos"][1]) for r in docked)
        return {
            "dock_x_error_m": (dock_err, dock_err < 0.05),
            "dock_y_error_m": (lat, lat < 0.05),
            "undock_x_error_m": (abs(undocked["pos"][0]), abs(undocked["pos"][0]) < 0.1),
        }

    return Sequence("dock_undock:navi_charge", "navi_charge", segs, terrain=terrain, checks=checks)


def gait_switch_sequence(cfg: dict[str, Any]) -> Sequence:
    """Enter on NaviCharge, then change gait through every walking runner while moving slowly."""
    order = (
        "navi_flat",
        "navi_orienteer",
        "extre",
        "step_move",
        "basic_stair",
        "basic_plat",
        "basic_flat",
        "basic_backknee",
        "basic_flat",
        "basic_stair",
        "navi_charge",
        "navi_flat",
    )
    segs = entry_segments(cfg, "navi_charge")
    for k, name in enumerate(order):
        gait = cfg["runners"][name]["gaits"][0]
        segs.append(Segment(f"{k}:{name}", 3.0, "rl_control", (0.2, 0, 0), gait=gait))
    segs.append(Segment("lie_down", 5.0, "lie_down"))
    return Sequence("gait_switch", "navi_charge", segs)


# HeightGap's height-map image convention (height_map_nav's grid, offset, orientation) is not in rl_deploy
HEIGHT_MAP_UNKNOWN = (
    "height map convention not decoded: the policy stands but walks / climbs wrongly"
)
EXPECTED_FAILURES = {
    "walk:height_gap": HEIGHT_MAP_UNKNOWN,
    "stairs:height_gap": HEIGHT_MAP_UNKNOWN,
    "switch:basic_flat->navi_flat": "BasicFlat's tall sim stance (0.59 m) is out of NaviFlat's distribution",
    "switch:basic_plat->basic_backknee": "BackKnee flips the hind knees; switching to it from BasicPlat while walking falls (standing works)",
}


def switch_sequence(cfg: dict[str, Any], src: str, dst: str) -> Sequence:
    """Enter rl_control on src's gait, walk, then change to dst's gait while walking."""
    segs = [
        *entry_segments(cfg, src),
        Segment("walk", 3.0, "rl_control", (0.2, 0, 0)),
        Segment("switched", 4.0, "rl_control", (0.2, 0, 0), gait=cfg["runners"][dst]["gaits"][0]),
        Segment("stop", 2.0, "rl_control"),
        Segment("lie_down", 5.0, "lie_down"),
    ]
    return Sequence(f"switch:{src}->{dst}", src, segs)


def all_sequences(cfg: dict[str, Any]) -> list[Sequence]:
    seqs = [Sequence(f"walk:{r}", r, walk_segments(cfg, r)) for r in WALKERS]
    seqs += [
        Sequence(f"stand:{r}", r, stand_only_segments(cfg, r), checks=pose_checks(cfg, r))
        for r in STANDERS
    ]
    seqs += [stairs_sequence(cfg, r) for r in CLIMBERS]
    seqs += [
        dock_sequence(cfg),
        gait_switch_sequence(cfg),
        switch_sequence(cfg, "basic_flat", "navi_flat"),
        switch_sequence(cfg, "basic_plat", "basic_backknee"),
    ]
    for seq in seqs:
        seq.expected_failure = EXPECTED_FAILURES.get(seq.name, "")
    return seqs


def run_one(name: str) -> dict[str, Any]:
    cfg = load_config()
    seq = next(s for s in all_sequences(cfg) if s.name == name)
    controller = Controller(cfg, policy_dir())
    result = run_sequence(cfg, seq, controller.tick)
    result["transitions"] = [(round(t, 3), a, b) for t, a, b in controller.transitions]
    return result


def summary_line(r: dict[str, Any]) -> str:
    parts = []
    for e in r["segments"]:
        if "error" in e:
            weak = "!" + ",".join(e["weak"]) if e.get("weak") else ""
            parts.append(f"{e['seg']} err={e['error']}{weak}")
    for k, (v, ok) in r["checks"].items():
        parts.append(f"{k}={v}{'' if ok else ' FAIL'}")
    fell = f" FELL at {r['fell_at']:.2f}s" if r["fell_at"] is not None else ""
    if r["expected_failure"]:
        verdict = "XPASS" if r["passed"] else "XFAIL"
        parts.append(f"(known: {r['expected_failure']})")
    else:
        verdict = "PASS" if r["passed"] else "FAIL"
    return f"{verdict:<5} {r['sequence']:<34}{fell}  " + "  ".join(parts)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--only", nargs="*", help="sequence names or substrings")
    parser.add_argument("--json", help="write full results here")
    parser.add_argument("--jobs", type=int, default=min(8, os.cpu_count() or 1))
    args = parser.parse_args()
    if not policy_dir().exists():
        raise SystemExit(
            f"no policies in {policy_dir()}: run extract_policies.py first (see its docstring)"
        )
    names = [s.name for s in all_sequences(load_config())]
    if args.only:
        names = [n for n in names if any(o in n for o in args.only)]
    with ProcessPoolExecutor(args.jobs) as pool:
        results = list(pool.map(run_one, names))
    for r in results:
        print(summary_line(r))
    passed = sum(r["passed"] for r in results)
    unexpected = [r["sequence"] for r in results if not r["passed"] and not r["expected_failure"]]
    print(
        f"{passed}/{len(results)} sequences passed, {len(results) - passed - len(unexpected)} known failures"
    )
    if args.json:
        with open(args.json, "w") as f:
            json.dump(results, f, indent=1)
    raise SystemExit(1 if unexpected else 0)


if __name__ == "__main__":
    main()
