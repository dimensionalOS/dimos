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

"""Scripted sequences (lie -> stand up -> rl_control stand -> walk -> stop -> lie down, stairs, dock / undock) and
their scoring: falls, velocity tracking error, final state."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
import math
from typing import Any, TypedDict

import numpy as np

from dimos.simulation.m20_policy_sim.config import runner_for_gait
from dimos.simulation.m20_policy_sim.controller import (
    JointCommand,
    UserCommand,
    leg_pose_for_height,
    roll_pitch,
)
from dimos.simulation.m20_policy_sim.runner import RobotState, projected_gravity
from dimos.simulation.m20_policy_sim.world import Terrain, World

# (state, command) -> joint command, called every 1 ms: the Python reference controller, or a bridge to another one
Drive = Callable[[RobotState, UserCommand], JointCommand]
# closed-loop axes from the world (for docking), physical (vx, vy, wz)
Steer = Callable[[World], np.ndarray]
# a sequence's pass / fail checks over its log: name -> (value, passed)
Checks = Callable[[World, list[dict[str, Any]]], dict[str, tuple[float, bool]]]

# the base must be at least this high in the rl_control stand segment (lying is about 0.11 m)
MIN_STAND_HEIGHT = 0.3
# a fall: tilt above this, or the trunk touching the ground, while the policy is in control
FALL_TILT_RAD = math.radians(60)


@dataclass
class Segment:
    name: str
    seconds: float
    target: str  # UserCommand.target
    velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)  # physical vx, vy, wz
    steer: Steer | None = None
    params: dict[str, float] = field(default_factory=dict)
    track: bool = False  # score velocity tracking over the second half
    gait: int | None = None  # switch gait code from this segment on
    runner: str | None = None


@dataclass
class Sequence:
    name: str
    runner: str
    segments: list[Segment]
    terrain: Terrain = field(default_factory=Terrain)
    # extra pass checks on the finished world: name -> (value, ok)
    checks: Checks | None = None
    # max |error| / |command| allowed per tracked axis for a pass
    tracking_tolerance: float = 0.5
    # a known failure and why: reported as XFAIL, not counted against the run
    expected_failure: str = ""


def nearest_entry_gait(cfg: dict[str, Any], runner: str) -> int:
    """The entry gait whose runner stands closest to `runner`'s default pose, so the switch starts in distribution."""
    rl = cfg["states"]["rl_control"]
    pose = np.array(cfg["runners"][runner]["default_pose"])
    best, best_d = rl["entry_fallback_gait"], math.inf
    for gait in rl["entry_gaits"]:
        name = runner_for_gait(cfg, gait)
        if name is None:
            continue
        d = float(np.linalg.norm(np.array(cfg["runners"][name]["default_pose"]) - pose))
        if d < best_d - 1e-9:
            best, best_d = gait, d
    return int(best)


def entry_segments(cfg: dict[str, Any], runner: str) -> list[Segment]:
    """Lying -> stand_up -> rl_control entry as rl_deploy allows it, then the runner's own gait (or a forced runner).

    rl_control only starts on its entry gaits (others become BasicFlat after the finetune_stand_15kg stand-in), so a
    runner with another gait is reached by a gait change once rl_control runs, as an operator would.
    """
    spec = cfg["runners"][runner]
    rl = cfg["states"]["rl_control"]
    own = spec["gaits"][0] if spec["gaits"] else None
    entry = own if own in rl["entry_gaits"] else nearest_entry_gait(cfg, runner)
    return [
        Segment("lying", 2.0, "passive", gait=entry),
        Segment("stand_up", 3.2, "rl_control"),
        Segment("stand_in", 4.0, "rl_control"),
        Segment(
            "switch",
            1.0,
            "rl_control",
            gait=own if own is not None else entry,
            runner=None if own else runner,
        ),
    ]


def walk_segments(
    cfg: dict[str, Any], runner: str, speed: float = 0.5, lateral: float = 0.3, turn: float = 0.5
) -> list[Segment]:
    return [
        *entry_segments(cfg, runner),
        Segment("stand", 4.0, "rl_control", track=True),
        Segment("vx", 6.0, "rl_control", (speed, 0, 0), track=True),
        Segment("vy", 6.0, "rl_control", (0, lateral, 0), track=True),
        Segment("wz", 6.0, "rl_control", (0, 0, turn), track=True),
        Segment("stop", 3.0, "rl_control", track=True),
        Segment("lie_down", 5.0, "lie_down"),
    ]


def stand_only_segments(cfg: dict[str, Any], runner: str) -> list[Segment]:
    """The stand policies hold a body pose from params: height in [0, 1] (lower), roll / pitch in [-1, 1]."""
    return [
        *entry_segments(cfg, runner),
        Segment("stand", 4.0, "rl_control", track=True),
        Segment("lower", 4.0, "rl_control", params={"height": 0.8}, track=True),
        Segment("pitch", 4.0, "rl_control", params={"height": 0.0, "pitch": 0.8}, track=True),
        Segment("roll", 4.0, "rl_control", params={"pitch": 0.0, "roll": 0.8}, track=True),
        Segment("level", 3.0, "rl_control", params={"roll": 0.0}, track=True),
        Segment("lie_down", 5.0, "lie_down"),
    ]


def run_sequence(
    cfg: dict[str, Any], seq: Sequence, drive: Drive, world: World | None = None
) -> dict[str, Any]:
    world = world or World(seq.terrain, torque_limit=np.tile(cfg["robot"]["torque_limit"], 4))
    world.reset(leg_pose_for_height(cfg["robot"], cfg["states"]["lie_down"]["lie_height"], 0.0))
    spec = cfg["runners"][seq.runner]
    cmd = UserCommand()
    log: list[dict[str, Any]] = []
    fell_at = None
    dt = 1.0 / cfg["timing"]["control_hz"]
    t = 0.0
    hm = next((b for b in spec["obs"] if b["kind"] == "height_map"), None)
    heights = None
    for seg in seq.segments:
        cmd.target = seg.target
        cmd.params.update(seg.params)
        if seg.gait is not None:
            cmd.gait = seg.gait
        if seg.runner is not None:
            cmd.runner = seg.runner
        # physical velocity -> the normalized axes of the runner that will be active
        active = (
            cmd.runner
            or runner_for_gait(cfg, cmd.gait)
            or cfg["states"]["rl_control"]["fallback_runner"]
        )
        command_max = np.array(cfg["runners"][active]["command_max"], np.float64)
        for k in range(round(seg.seconds / dt)):
            velocity = seg.steer(world) if seg.steer else np.array(seg.velocity, np.float64)
            cmd.axes = np.divide(velocity, command_max, out=np.zeros(3), where=command_max > 0)
            s = world.state()
            if hm is not None:
                # refreshed at the policy rate; a real height map arrives slower than that
                if k % cfg["timing"]["policy_decimation"] == 0 or heights is None:
                    heights = world.height_map(hm["rows"], hm["cols"], **HEIGHT_MAP_GRID)
                s.height_map = heights
            jc = drive(s, cmd)
            world.step(jc)
            t += dt
            if k % 20 == 0:
                g = projected_gravity(s.quat)
                tilt = math.acos(max(-1.0, min(1.0, -g[2])))
                pos, _ = world.base_pose()
                roll, pitch = roll_pitch(s.quat)
                rec = dict(
                    roll=roll,
                    pitch=pitch,
                    t=t,
                    seg=seg.name,
                    cmd=velocity.tolist(),
                    v=world.base_velocity_body().tolist(),
                    pos=pos.tolist(),
                    tilt=tilt,
                    trunk_contact=world.body_contacts_ground(),
                )
                log.append(rec)
                in_policy = seg.target == "rl_control" and seg.name not in ("lying", "stand_up")
                if in_policy and fell_at is None and (tilt > FALL_TILT_RAD or rec["trunk_contact"]):
                    fell_at = t
        if fell_at is not None:
            break
    return score(seq, log, fell_at, world)


# HeightGap's map as rl_deploy's HeightsSubCallback filters it [C]; the grid spacing, orientation and offset are not
# in the binary (height_map_nav makes the image) [I]
class HeightMapGrid(TypedDict):
    """World.height_map's grid arguments past rows and cols."""

    spacing_x: float
    spacing_y: float
    offset: float
    min_value: float
    fill: float
    min_row: int
    front_first: bool


HEIGHT_MAP_GRID = HeightMapGrid(
    spacing_x=0.05,
    spacing_y=0.05,
    offset=0.1,
    min_value=0.18,
    fill=0.6,
    min_row=20,
    front_first=False,
)


def score(
    seq: Sequence, log: list[dict[str, Any]], fell_at: float | None, world: World
) -> dict[str, Any]:
    segments = []
    ok = fell_at is None
    for name in dict.fromkeys(r["seg"] for r in log):
        rows = [r for r in log if r["seg"] == name]
        half = rows[len(rows) // 2 :]
        cmd = np.array(half[-1]["cmd"]) if half else np.zeros(3)
        v = np.mean([r["v"] for r in half], 0) if half else np.zeros(3)
        entry = dict(
            seg=name,
            cmd=np.round(cmd, 3).tolist(),
            true=np.round(v, 3).tolist(),
            max_tilt_deg=round(math.degrees(max(r["tilt"] for r in rows)), 1),
            z=round(float(np.mean([r["pos"][2] for r in half])), 3) if half else None,
            roll_deg=round(math.degrees(float(np.mean([r["roll"] for r in half]))), 1)
            if half
            else None,
            pitch_deg=round(math.degrees(float(np.mean([r["pitch"] for r in half]))), 1)
            if half
            else None,
        )
        segment_def = next(s for s in seq.segments if s.name == name)
        if segment_def.track and segment_def.steer is None:
            err = np.abs(v - cmd)
            entry["error"] = np.round(err, 3).tolist()
            for axis in range(3):
                bound = max(seq.tracking_tolerance * abs(cmd[axis]), 0.1 if axis < 2 else 0.15)
                if err[axis] > bound:
                    entry.setdefault("weak", []).append("xyz"[axis] if axis < 2 else "wz")
        segments.append(entry)
    completed = [s.name for s in seq.segments] == [e["seg"] for e in segments]
    checks = seq.checks(world, log) if seq.checks and completed else {}
    stand = [r["pos"][2] for r in log if r["seg"] == "stand"]
    if stand:
        z = float(np.mean(stand[len(stand) // 2 :]))
        checks["stand_z_m"] = (z, z > MIN_STAND_HEIGHT)
    ok = ok and completed and all(v[1] for v in checks.values())
    return dict(
        expected_failure=seq.expected_failure,
        sequence=seq.name,
        runner=seq.runner,
        passed=ok,
        fell_at=fell_at,
        segments=segments,
        checks={k: (round(float(v[0]), 3), bool(v[1])) for k, v in checks.items()},
    )
