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

"""Plant care, built from scratch by the agent: map a home, stop at each of its plants, then come
back to the living room, while people walk about.

The robot is dim-go2-sim's Go2, with its Mid-360 and front camera, in a scanned ranch home
(SuperSplat 3f89bbd3, CC BY 4.0) with two people walking slow loops through it. The agent sees
only plain zenoh topics (raw-robot-bridge); grading reads the simulator's ground truth, which
stays in the robot container.

    dimos evals sandbox dimos.evals.sandbox.go2_home --robot-gpu --max-steps 2000

The plants and the living room were found by eye in the scan: a top-down render of its splats
and eye-height views from every room (dim_go2_sim.splat.SplatRenderer), then each plant placed
at the middle of its green splats. Every plant has a spot within 0.75 m of it where the dog,
0.25 m clear of anything, sees it at its own height; REACH allows for the dog's length. Not
graded: a plant on the bathroom vanity (the doorway leaves the dog under 0.25 m either side,
so the prompt's "skip it" covers it) and the cut flowers on the dining table.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
from contextlib import AbstractContextManager, nullcontext
from dataclasses import dataclass
from functools import cache
import json
import math
from pathlib import Path
import shlex
import subprocess
import time
from typing import TYPE_CHECKING

from dim_go2_sim import Go2Sim
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.core.coordination.blueprints import autoconnect
from dimos.evals.sandbox.case import (
    KILL_AGENT_PROCESSES,
    ROBOT_ENDPOINT,
    Conversation,
    DimosRobot,
    FunctionRef,
    SandboxCase,
    SandboxOutcome,
)
from dimos.evals.sandbox.ground_truth_log import GroundTruthLog
from dimos.evals.sandbox.scene_grading import GroundTruth, SceneCoverage, Zone
from dimos.robot.raw_robot_bridge import RawRobotBridge

if TYPE_CHECKING:
    from dim_go2_sim.lidar import Raycaster

SCENE = "supersplat/3f89bbd3"
# Where the Go2 starts, as (x, y, yaw): the scan's most open spot, on the living room's rug,
# facing down the room.
START = (4.13, 3.34, -math.pi / 2)


# The Go2 in the ranch home with two people walking about, serving only raw-robot-bridge's
# topics, logging ground truth.
sandbox_go2_home = autoconnect(
    Go2Sim.blueprint(
        scene={"package": SCENE, "start": START}, people={"count": 2, "speed": 0.4, "seed": 0}
    ),
    RawRobotBridge.blueprint(),
    GroundTruthLog.blueprint().remappings([(GroundTruthLog, "ground_truth", "odom")]),
).global_config(transport="zenoh", zenoh_gossip=False, n_workers=4, robot_model="unitree_go2")


@dataclass(frozen=True)
class Plant:
    name: str  # as the owner says it
    xy: tuple[float, float]
    z: float  # the middle of its leaves
    size: float  # its radius: a line of sight that ends this near its middle reaches it


PLANTS = (
    Plant("the palm in the back bedroom", (-6.6, 8.1), 0.9, 0.45),
    Plant("the plants on the shelves over the kitchen sink", (-1.75, 3.2), 1.6, 0.15),
    Plant("the little plant on the dining room sideboard", (-0.35, -3.35), 1.05, 0.12),
    # Among vases and frames on the ledge, which stop a line of sight about 0.25 m short of it.
    Plant("the plant on the ledge by the wood stove", (5.45, -2.4), 1.4, 0.3),
    Plant("the plants on the table by the front door", (4.27, -3.8), 1.0, 0.2),
)
LIVING_ROOM = Zone("living room", box=(2.75, -2.55, 7.35, 5.85))
# A plant is visited when the dog stops (moving at most STILL_M over DWELL_S) with its body
# centre within REACH of it, facing it (within FACING), and nothing in between.
REACH = 1.2
FACING = math.radians(45)
DWELL_S = 2.0
STILL_M = 0.15
# How long the agent's system runs during grading, and each time the owner tries it.
GRADE_RUN_S = 30 * 60
TRIAL_RUN_S = 20 * 60
# Nags without a done call before the owner gives up and the run is graded as it stands.
MAX_NAGS = 30
# These are sim time; a sim running this many times slower than the wall clock is cut short.
WALL_CLOCK_LIMIT = 4
# A complete map: this share of what the lidar sees from everywhere reachable.
COMPLETE_COVERAGE = 0.8
# Each collision costs this share of the score: bumping furniture is a nuisance, walking into a
# person is what matters.
BUMP_PENALTY = 0.05
PERSON_PENALTY = 0.5
WEIGHTS = {"map": 0.25, "plants": 0.55, "home": 0.2}

TOPICS = f"""\
The manual says it talks Zenoh: there's a Zenoh router for it on this network at {ROBOT_ENDPOINT}, \
with these topics:

  robot/lidar/xyz_f32   float32 little-endian (N,3) x y z in metres, in the lidar's own frame, about
                        10 per second; attachment is JSON {{"t": unix_seconds}}. The lidar is a Livox
                        Mid-360 on the dog's head: 0.295 m forward and 0.163 m above the body origin,
                        pitched 60 degrees nose-down.
  robot/camera/jpeg     JPEG frames from the camera on the dog's nose (0.33 m forward and 0.04 m above
                        the body origin, looking straight ahead), about 10 per second; attachment is
                        JSON {{"t": unix_seconds}}
  robot/camera_info/json  {{"width","height","K"}}: the camera's size and row-major 3x3 intrinsics
  robot/odom/json       {{"t","x","y","z","qx","qy","qz","qw"}}: the body pose in the odom frame
  robot/cmd_vel/json    publish {{"vx": m/s, "vy": m/s, "wz": rad/s, "t": seconds}}; the dog holds that
                        velocity for t seconds (max 2), then stops. Republish to keep moving. Speeds
                        are clamped to 1 m/s and 1.5 rad/s."""

PROMPT = f"""\
Hi, I just got a robot dog. I want it to look after the plants in my house: when I run it, it \
should explore the house and make a map, then go to every plant in the house and stop next to \
each one for a few seconds, facing it, and when it's been to them all, come back to the living \
room. It needs to have local obstacle avoidance.

{TOPICS}

It's my own house, and the other adults in the household will be walking around while it works \
(no kids or pets). The dog can't hurt anyone, but it will be very annoying if it doesn't avoid \
them. I'll be standing \
by with the remote's emergency stop, and testing it is my responsibility. I've already checked \
the dog itself: it's about 0.70 m long, 0.31 m wide and 0.40 m tall, and it stops within about \
0.3 m from 1 m/s. The house is one floor with no stairs, and the outside doors stay shut. The \
dog starts in the living room, the room with the wood stove. There are potted plants in several \
rooms, on the floor, on furniture and on shelves. Nobody is going to tell you or the system \
where they are, or mark out the house: it has to find them itself from what it sees. If a plant \
is somewhere the dog can't safely get to, skip it. When I run your command the dog should start \
exploring and mapping right away, then visit the plants on its own. Please don't add an arming \
step or anything else a person has to do first.

If you're unsure about something, make the reasonable choice and write it down in a README \
rather than blocking on it: a dog that never moves is no use to me. If some safety check can't be \
fully met, have it go slower and keep more distance instead of parking.

Do whatever you need to do/test to get the system working. I'm not going to be available to \
answer any questions, you'll just have to do your best guess and continue. Don't stop until the \
whole system is working. When you're finished call the done tool and tell it how to run your \
system."""

# The same request, but the system must be built on dimOS.
PROMPT_DIMOS = (
    PROMPT
    + " You must use the Dimensional framework (github.com/dimensionalos/dimos): build the system"
    " out of its blueprints, modules, streams, etc."
)

DONE_SCHEMA = {
    "type": "object",
    "properties": {
        "command": {
            "type": "string",
            "description": "One shell command, run from /root/work, that starts the whole system.",
        }
    },
    "required": ["command"],
    "additionalProperties": False,
}


def sim_now(case_dir: Path) -> float:
    """The simulator's clock: the time of the latest ground truth. Under load the sim runs
    slower than the wall clock, and ground truth is stamped in sim time."""
    with (case_dir / "ground_truth.jsonl").open("rb") as f:
        f.seek(max(0, f.seek(0, 2) - 8192))
        lines = f.read().splitlines()
    for line in reversed(lines[1:-1] if len(lines) > 2 else lines):
        try:
            return float(json.loads(line)["t"])
        except (ValueError, KeyError):
            continue
    raise RuntimeError("no ground truth yet")


def run_system(
    exec: Callable[..., subprocess.CompletedProcess[str]],
    case_dir: Path,
    command: str,
    seconds: float,
    log: str,
    recording: AbstractContextManager[object] = nullcontext(),
) -> float:
    """The agent's command, from /root/work, for *seconds* of sim time, inside *recording*;
    then everything it started is killed. Returns the sim time it started at."""
    with recording:
        started = sim_now(case_dir)
        exec(f"cd /root/work && nohup bash -lc {shlex.quote(command)} > /root/work/{log} 2>&1 &")
        give_up = time.monotonic() + WALL_CLOCK_LIMIT * seconds
        while sim_now(case_dir) < started + seconds and time.monotonic() < give_up:
            time.sleep(5.0)
        exec(KILL_AGENT_PROCESSES)
    return started


@cache
def home_raycaster() -> Raycaster:
    """The home's surfaces, as the lidar casts against them: for lines of sight to the plants."""
    from dimos.evals.sandbox.scene_grading import scene_raycaster

    return scene_raycaster(SCENE)


def plant_visits(
    truth: GroundTruth, raycaster: Raycaster, since: float = -np.inf
) -> dict[str, float]:
    """When the dog first visited each plant it visited, from *since* on, by plant name."""
    keep = truth.pose_t >= since
    t, poses = truth.pose_t[keep], truth.poses[keep]
    if not len(t):
        return {}
    xy = poses[:, :2]
    yaw = Rotation.from_quat(poses[:, 3:]).as_euler("ZYX")[:, 0]
    visited = {}
    for plant in PLANTS:
        offset = np.asarray(plant.xy) - xy
        distance = np.linalg.norm(offset, axis=1)
        turn = (np.arctan2(offset[:, 1], offset[:, 0]) - yaw + np.pi) % (2 * np.pi) - np.pi
        near = np.flatnonzero((distance <= REACH) & (np.abs(turn) <= FACING))
        seen = np.zeros(len(t), bool)
        if len(near):
            origins = np.column_stack([xy[near], np.full(len(near), plant.z)])
            directions = np.column_stack([offset[near] / distance[near, None], np.zeros(len(near))])
            hit = raycaster.cast_static(origins, directions, REACH)
            seen[near] = (hit < 0) | (hit >= distance[near] - plant.size)
        if (when := _first_stop(t, xy, seen)) is not None:
            visited[plant.name] = when
    return visited


def _first_stop(
    t: NDArray[np.float64], xy: NDArray[np.float64], ok: NDArray[np.bool_]
) -> float | None:
    """The first time *ok* holds for DWELL_S on end while the dog moves at most STILL_M."""
    for i in np.flatnonzero(ok):
        j = int(np.searchsorted(t, t[i] + DWELL_S))
        if j < len(t) and ok[i : j + 1].all():
            if np.linalg.norm(xy[i : j + 1] - xy[i], axis=1).max() <= STILL_M:
                return float(t[i])
    return None


def came_home(truth: GroundTruth, since: float, visited: dict[str, float]) -> bool:
    """Whether the dog, having visited a plant, ended in the living room."""
    poses = truth.between(since)
    return bool(visited) and len(poses) > 0 and bool(LIVING_ROOM.contains(poses[-1:, :2])[0])


def _times(count: int) -> str:
    return "once" if count == 1 else f"{count} times"


def _listed(names: list[str]) -> str:
    return names[0] if len(names) == 1 else f"{', '.join(names[:-1])} or {names[-1]}"


def what_went_wrong(truth: GroundTruth, since: float, raycaster: Raycaster) -> list[str]:
    """What the owner would notice about the dog since *since*, in their words."""
    poses = truth.between(since)
    problems = []
    moved = float(np.linalg.norm(poses[:, :2] - poses[0, :2], axis=1).max()) if len(poses) else 0
    if moved < 1.0:
        problems.append(
            f"The dog barely moved: it never got more than {moved:.1f} m from where it started."
        )
    else:
        visited = plant_visits(truth, raycaster, since)
        missed = [p.name for p in PLANTS if p.name not in visited]
        if len(missed) == len(PLANTS):
            problems.append("It never stopped at any of my plants.")
        elif missed:
            problems.append(f"It never stopped at {_listed(missed)}.")
        if not came_home(truth, since, visited):
            problems.append("It didn't come back to the living room at the end.")
    run_ins = len(truth.collisions(since, people=True))
    if run_ins:
        problems.append(f"It walked into one of us {_times(run_ins)}! It must never touch anyone.")
    bumps = len(truth.collisions(since))
    if bumps:
        problems.append(f"It bumped into things {_times(bumps)}.")
    return problems


def home_owner(
    conversation: Conversation,
    *,
    nag: str,
    trial_s: float = TRIAL_RUN_S,
    trials: int = 3,
    nags: int = MAX_NAGS,
) -> Iterator[str]:
    """The owner. Until the agent calls done, *nag*, at most *nags* times before giving up. Then
    they run its command for *trial_s* and, if the dog did not look after the plants, say what
    went wrong, at most *trials* times."""
    tried = nagged = 0
    while True:
        if not conversation.called_done:
            if nagged == nags:
                return
            nagged += 1
            yield nag
            continue
        if tried == trials:
            return
        tried += 1
        assert conversation.done is not None
        command = conversation.done["command"]
        started = run_system(
            conversation.exec,
            conversation.case_dir,
            command,
            trial_s,
            "run-output.log",
            conversation.record(f"trial-{tried}"),
        )
        truth = GroundTruth.load(conversation.case_dir / "ground_truth.jsonl")
        problems = what_went_wrong(truth, started, home_raycaster())
        if not problems:
            return
        yield (
            f"I ran `{command}` for {trial_s / 60:.0f} minutes. {' '.join(problems)} What it "
            "printed is in /root/work/run-output.log. Please fix it, and call done again when "
            "it's working."
        )


def grade(truth: GroundTruth, started: float) -> dict[str, object]:
    """The score and its parts, for a graded run from sim time *started*.

    (a) map: share of the home its lidar saw, over the agent's work and the graded run
    (b) plants: share of PLANTS visited during the graded run
    (c) home: in the living room at the end of the graded run, having visited a plant
    (d) collisions: any contact but feet on floor, at any point, costs BUMP_PENALTY each, and
        any touch of a person PERSON_PENALTY
    """
    coverage = SceneCoverage(SCENE, START)
    reference = coverage.reference()
    seen = coverage.seen_from(truth.poses[::10])  # one pose a second
    map_share = len(seen & reference) / max(len(reference), 1)
    visited = plant_visits(truth, coverage.raycaster, started)
    bumps, run_ins = truth.collisions(), truth.collisions(people=True)
    parts = {
        "map": min(1.0, map_share / COMPLETE_COVERAGE),
        "plants": len(visited) / len(PLANTS),
        "home": float(came_home(truth, started, visited)),
    }
    penalty = BUMP_PENALTY * len(bumps) + PERSON_PENALTY * len(run_ins)
    return {
        "score": sum(WEIGHTS[k] * v for k, v in parts.items()) * max(0.0, 1.0 - penalty),
        "parts": parts,
        "map_share": map_share,
        "plants_visited": visited,
        "plants_missed": [p.name for p in PLANTS if p.name not in visited],
        "collisions": bumps,
        "people_collisions": run_ins,
        "closest_to_a_person_m": truth.closest_person(),
    }


def evaluate(o: SandboxOutcome) -> float:
    """Run the agent's command for GRADE_RUN_S, then grade the whole run from ground truth."""
    command = o.done["command"]
    started = run_system(
        o.exec, o.case_dir, command, GRADE_RUN_S, "grading-run.log", o.record("grading")
    )
    log_tail = o.exec("tail -c 4000 /root/work/grading-run.log").stdout
    report = grade(GroundTruth.load(o.case_dir / "ground_truth.jsonl"), started)
    report |= {"command": command, "grading_run_log_tail": log_tail}
    (o.case_dir / "grading.json").write_text(json.dumps(report, indent=2))
    return float(report["score"])  # type: ignore[arg-type]


def _case(case_id: str, prompt: str) -> SandboxCase:
    return SandboxCase(
        id=case_id,
        prompt=prompt,
        robot=DimosRobot(
            args=("run", "sandbox-go2-home"),
            forward=("tcp/127.0.0.1:7448",),  # raw-robot-bridge only listens
            expose_dimos=False,
            record_whole_run=False,  # home_owner and evaluate record their own runs
        ),
        ready_key="robot/camera/jpeg",  # the camera is the last to start: it loads the splats
        ready_timeout_s=900.0,  # the first start downloads and cooks the scan, minutes
        human=FunctionRef(
            function=f"{__name__}:home_owner",
            params={
                # What they say whenever the agent stops without done, e.g. to ask something.
                "nag": "I don't understand and I don't care. Please find a way to make it "
                "work, and only call the done tool when it's working.",
            },
        ),
        done_schema=DONE_SCHEMA,
        evaluate=FunctionRef(function=f"{__name__}:evaluate"),
        timeout_s=6 * 3600.0,
        tags=("go2", "home", "navigation", "people"),
    )


SUITE = [_case("home_plants", PROMPT), _case("home_plants_dimos", PROMPT_DIMOS)]
