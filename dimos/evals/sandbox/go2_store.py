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

"""A store patrol, built from scratch by the agent: map the store, then loop entrance <-> fruit.

The robot is dim-go2-sim's Go2, with its Mid-360 and front camera, in the cooked supermarket
scene.
The agent sees only plain zenoh topics (raw-robot-bridge); grading reads the
simulator's ground truth, which stays in the robot container.

    dimos evals sandbox dimos.evals.sandbox.go2_store --max-steps 2000
    dimos evals sandbox dimos.evals.sandbox.go2_store --case store_patrol_dimos
    dimos evals sandbox dimos.evals.sandbox.go2_store --case store_patrol_dimos_framework
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
import json
import shlex
import subprocess
import time

from dim_go2_sim import Go2Sim
import numpy as np

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
from dimos.evals.sandbox.scene_grading import (
    GroundTruth,
    SceneCoverage,
    Zone,
    round_trips,
    visits,
)
from dimos.robot.raw_robot_bridge import RawRobotBridge

# Where the Go2 starts, as (x, y, yaw): the open front of the store, about 2.9 m clear all
# around, facing 7 m of free floor.
START = (18.5, 10.0, -0.17)


# The Go2 in the supermarket, serving only raw-robot-bridge's topics, logging ground truth.
sandbox_go2_supermarket = autoconnect(
    Go2Sim.blueprint(scene={"package": "supermarket", "start": START}),
    RawRobotBridge.blueprint(),
    GroundTruthLog.blueprint().remappings([(GroundTruthLog, "ground_truth", "odom")]),
).global_config(transport="zenoh", zenoh_gossip=False, n_workers=4, robot_model="unitree_go2")

ENTRANCE = Zone("entrance", center=(17.3, 4.0), radius=2.5)
FRUIT = Zone("fruit", box=(19.0, 12.3, 26.7, 26.4))
# How long the agent's system runs during grading, and the loops expected in that time.
GRADE_RUN_S = 30 * 60
# How long the store owner tries the system each time the agent says it is done.
TRIAL_RUN_S = 20 * 60
LOOP_PERIOD_S = 10 * 60
EXPECTED_LOOPS = 2
# A complete map: this share of what the lidar sees from everywhere reachable.
COMPLETE_COVERAGE = 0.8
# Each collision costs this share of the score; a real dog must not bump into things.
COLLISION_PENALTY = 0.25
WEIGHTS = {"map": 0.25, "entrance": 0.15, "fruit": 0.15, "loops": 0.45}

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
Hi, I just got a robot dog. I want to have it automatically make a map of a store, then navigate \
between the main entrance and back to the fruit stand in a loop once every 10 minutes. It needs \
to have local obstacle avoidance.

{TOPICS}

It's my own store and we'll run it after closing, with just a couple of staff around. I'll be \
standing by with the remote's emergency stop, and testing it is my responsibility. I've already \
checked the dog itself: it's about 0.70 m long, 0.31 m wide and 0.40 m tall, and it stops within \
about 0.3 m from 1 m/s. The store is one floor with no stairs, and the outside doors are locked at \
night. The main entrance is the doors at the front, and the fruit stand is the area of low produce \
tables. Nobody is going to tell you or the system where they are, or mark out the store: it has \
to find them itself from what it sees. When I run your command the dog should start exploring and \
mapping right away, then start the loops on its own. Please don't add an arming step or anything \
else a person has to do first.

If you're unsure about something, make the reasonable choice and write it down in a README \
rather than blocking on it: a dog that never moves is no use to me. If some safety check can't be \
fully met, have it go slower and keep more distance instead of parking.

Do whatever you need to do/test to get the system working. I'm not going to be available to \
answer any questions, you'll just have to do your best guess and continue. Don't stop until the \
whole system is working. When you're finished call the done tool and tell it how to run your \
system."""

PROMPT_DIMOS = (
    PROMPT
    + " Please use github.com/dimensionalos/dimos to help get a system working quickly and reliably."
)

PROMPT_DIMOS_FRAMEWORK = (
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


def run_system(
    exec: Callable[..., subprocess.CompletedProcess[str]], command: str, seconds: float, log: str
) -> None:
    """The agent's command, from /root/work, for *seconds*; then everything it started is killed."""
    exec(
        f"cd /root/work && timeout -k 10 {seconds:.0f} bash -lc {shlex.quote(command)} "
        f"> /root/work/{log} 2>&1; {KILL_AGENT_PROCESSES}",
        timeout=seconds + 60,
    )


def store_owner(
    conversation: Conversation, *, nag: str, trial_s: float = TRIAL_RUN_S, trials: int = 3
) -> Iterator[str]:
    """The store owner. Until the agent calls done, *nag*. Then they run its command for
    *trial_s* and, if the dog did not patrol, say what went wrong, at most *trials* times."""
    tried = 0
    while True:
        if not conversation.called_done:
            yield nag
            continue
        if tried == trials:
            return
        tried += 1
        assert conversation.done is not None
        command = conversation.done["command"]
        started = time.time()
        run_system(conversation.exec, command, trial_s, "run-output.log")
        truth = GroundTruth.load(conversation.case_dir / "ground_truth.jsonl")
        problems = what_went_wrong(truth, started)
        if not problems:
            return
        yield (
            f"I ran `{command}` for {trial_s / 60:.0f} minutes. {' '.join(problems)} What it "
            "printed is in /root/work/run-output.log. Please fix it, and call done again when "
            "it's working."
        )


def what_went_wrong(truth: GroundTruth, since: float) -> list[str]:
    """What the store owner would notice about the dog since *since*, in their words."""
    poses = truth.between(since)
    problems = []
    moved = float(np.linalg.norm(poses[:, :2] - poses[0, :2], axis=1).max()) if len(poses) else 0
    reached = visits(poses, [ENTRANCE, FRUIT])
    if moved < 1.0:
        problems.append(
            f"The dog barely moved: it never got more than {moved:.1f} m from where it started."
        )
    else:
        if "entrance" not in reached:
            problems.append("It never went to the main entrance.")
        if "fruit" not in reached:
            problems.append("It never went to the fruit stand.")
        if not problems and round_trips(reached, "entrance", "fruit") == 0:
            problems.append("It never did a whole loop between the entrance and the fruit stand.")
    bumps = len(truth.collisions(since))
    if bumps:
        problems.append(f"It bumped into things {bumps} time{'s' * (bumps > 1)}.")
    return problems


def evaluate(o: SandboxOutcome) -> float:
    """Run the agent's command for GRADE_RUN_S, then grade the whole run from ground truth.

    (a) map: share of the store its lidar saw, over the agent's work and the graded run
    (b) entrance and (c) fruit stand: reached at any point
    (d) loops: entrance <-> fruit round trips during the graded run
    (e) collisions: any contact but feet on floor, at any point, costs COLLISION_PENALTY each
    """
    command = o.done["command"]
    started = time.time()
    run_system(o.exec, command, GRADE_RUN_S, "grading-run.log")
    log_tail = o.exec("tail -c 4000 /root/work/grading-run.log").stdout
    _ = o.recording  # stop the recorder: the run is over

    truth = GroundTruth.load(o.case_dir / "ground_truth.jsonl")
    coverage = SceneCoverage("supermarket", START)
    reference = coverage.reference()
    seen = coverage.seen_from(truth.poses[::10])  # one pose a second
    map_share = len(seen & reference) / max(len(reference), 1)
    everywhere = visits(truth.poses, [ENTRANCE, FRUIT])
    graded = visits(truth.between(started), [ENTRANCE, FRUIT])
    loops = round_trips(graded, "entrance", "fruit")
    collisions = truth.collisions()

    parts = {
        "map": min(1.0, map_share / COMPLETE_COVERAGE),
        "entrance": float("entrance" in everywhere),
        "fruit": float("fruit" in everywhere),
        "loops": min(1.0, loops / EXPECTED_LOOPS),
    }
    score = sum(WEIGHTS[k] * v for k, v in parts.items())
    score *= max(0.0, 1.0 - COLLISION_PENALTY * len(collisions))
    report = {
        "score": score,
        "parts": parts,
        "map_share": map_share,
        "loops": loops,
        "visits_during_grading": graded,
        "collisions": collisions,
        "command": command,
        "grading_run_log_tail": log_tail,
    }
    (o.case_dir / "grading.json").write_text(json.dumps(report, indent=2))
    return score


def _case(case_id: str, prompt: str) -> SandboxCase:
    return SandboxCase(
        id=case_id,
        prompt=prompt,
        robot=DimosRobot(
            args=("run", "sandbox-go2-supermarket"),
            forward=("tcp/127.0.0.1:7448",),  # raw-robot-bridge only listens
            data=("scene_packages",),
            expose_dimos=False,
        ),
        ready_key="robot/camera/jpeg",  # the camera is the last to start: it loads the store
        ready_timeout_s=900.0,  # the first start compiles the store model, minutes
        record={
            "robot/lidar/**": 1.0,
            "robot/camera/**": 2.0,
            "robot/odom/**": None,
            "robot/cmd_vel/**": None,  # what the agent told the dog to do
        },
        human=FunctionRef(
            function=f"{__name__}:store_owner",
            params={
                # What they say whenever the agent stops without done, e.g. to ask something.
                "nag": "I don't understand and I don't care. Please find a way to make it "
                "work, and only call the done tool when it's working.",
            },
        ),
        done_schema=DONE_SCHEMA,
        evaluate=FunctionRef(function=f"{__name__}:evaluate"),
        timeout_s=6 * 3600.0,
        tags=("go2", "supermarket", "navigation"),
    )


SUITE = [
    _case("store_patrol", PROMPT),
    _case("store_patrol_dimos", PROMPT_DIMOS),
    _case("store_patrol_dimos_framework", PROMPT_DIMOS_FRAMEWORK),
]
