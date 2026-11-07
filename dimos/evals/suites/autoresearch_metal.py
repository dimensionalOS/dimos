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

"""One unattended autoresearch session per task on the Metal arm in MuJoCo.

The case prompt is the ASPIRE actor loop: read the task and scene, submit
immutable candidate programs, run them on dev trials, read the traces and
keyframes, repair by failure mode, search with several candidates when
stuck, freeze one. The lab scores the frozen candidate on held-out trials
and the grader reads that rate from ``session.json``. The run needs
``ANTHROPIC_API_KEY`` and Pi on PATH; the keyword guard denies every way
around the lab's tools:

    dimos evals run dimos.evals.suites.autoresearch_metal --agent dimos.evals.agents.pi \\
        --set provider=anthropic --set model=<model> --set max_steps=600 \\
        --exclude shell,porcelain,sdk,topic,select,pick_object,place_at
"""

from __future__ import annotations

from datetime import datetime
import json
import re
from typing import Any

from dimos.autoresearch.session import Budget, Session
from dimos.evals.environments.autoresearch import AutoresearchEnvironment
from dimos.evals.types import EvalCase, Outcome, Step, Suite, Trajectory

TOOLS = ("task", "scene", "submit", "run", "stop_run", "results", "leaderboard", "freeze", "note")

# Tool calls mentioning these are denied: the ways past the lab's tools into the stack.
EXCLUDED_KEYWORDS = ("shell", "porcelain", "sdk", "topic", "select", "pick_object", "place_at")

BUDGET = Budget(iterations=3, candidates_per_round=3, dev_trials=3, heldout_trials=5, threshold=0.6)

TASKS = ("lift the cube",)

REPORT_NAME = "report.json"

PROGRAM_API = """\
The program API. A candidate is a Python file defining run(robot, task); `import math` is the
only import allowed (checked on submit, enforced when it runs). task.text is the task in
words. The program never sees the judge or any score. Motion calls block until done and raise
RobotCallError (with a .status) when planning or execution fails; catch it to try another way.
  robot.arm.joints()                      joint positions, radians
  robot.arm.pose()                        end-effector position (m) and xyzw quaternion
  robot.arm.move_joints(positions)        planned move to joint positions
  robot.arm.move_pose(xyz, rpy_or_quat)   planned move; orientation optional
  robot.arm.move_linear(dx, dy, dz)       straight-line translation, metres
  robot.arm.home()                        the home preset
  robot.gripper.open() / .close()         fully open or closed
  robot.gripper.set(x), .range()          the gripper joint at x; range() gives (closed, open)
  robot.scene.objects(words)              detected objects: id, name, position (m), size (m)
  robot.observe()                         saves a keyframe, returns its path
  robot.log(text)                         a line in the trace
Example:
  import math

  def run(robot, task):
      \"\"\"Top-down grasp at the cube's centre. May fail if the fingers land on an edge.\"\"\"
      cube = robot.scene.objects(["cube"])[0]
      x, y, z = cube.position
      robot.gripper.open()
      robot.arm.move_pose([x, y, z + 0.10], [math.pi, 0.0, 0.0])
      robot.arm.move_linear(dz=-0.08)
      robot.gripper.close()
      robot.arm.move_linear(dz=0.10)
"""


def actor_prompt(task: str, budget: Budget) -> str:
    """The actor's instructions for ``task``: the lab's tools, the loop rules and the program API."""
    return f"""\
You are the actor of an autoresearch session: write a program that makes a robot arm in
simulation "{task}", and improve it from evidence until one version is good enough.

The lab's tools: task(), scene(), submit(program, hypothesis), run(candidate, split),
stop_run(), results(candidate), leaderboard(), freeze(candidate), note(title, text).
Nothing else may move the robot: no shell into the stack, no topics, no SDK, no other skills.
From a shell, write the program to a file and build the submit call's JSON with Python's json
module, so newlines and quotes survive.

The loop:
1. Call task() first, then scene() for the layout and its keyframe image, before writing code.
2. Write run(robot, task) with a docstring stating your hypothesis (why this will work) and the
   failure you expect. submit(program, hypothesis) stores it as immutable candidate NN;
   run(NN, "dev") starts its {budget.dev_trials} dev trials and returns at once ("started").
   Poll results(NN) every few seconds until every trial has a record.
3. Before changing anything, read the trial's trace (trace.jsonl: every robot call with its
   arguments, status, result and duration) and its keyframes (PNG before and after each motion
   call); their paths are in the results, under the session directory. Repair by failure mode:
   say what went wrong in the world, then fix that. Never change things by trial and error.
4. When stuck (the same failure twice, or no idea what is wrong), propose
   {budget.candidates_per_round} distinct candidates in one round, seeded from the top three on
   leaderboard(), at least one structurally different (another approach direction, grasp or
   ordering), each with its own hypothesis. Run them all on dev.
5. Candidates are immutable: a fix is a new submit, never an edit. stop_run() halts a run you
   no longer need.
6. When a fix is reusable, note(title, text) it: the failure signature, when to apply it, the
   repair and a code sketch.
7. Stop when a candidate's dev pass rate reaches {budget.threshold:g}, or after
   {budget.iterations} rounds. Then freeze(NN) exactly one candidate, your choice, and say why
   in your final message. The lab scores it on held-out trials; do not wait for that.

{PROGRAM_API}"""


def heldout_rate(outcome: Outcome) -> float:
    """The frozen candidate's held-out pass rate from ``session.json``, which must be
    scored. Also writes ``report.json`` in the session: leaderboard, iteration count
    and tokens and seconds per iteration (one iteration is one submitted candidate)."""
    session_dir = outcome.artifacts["session"]
    info = json.loads((session_dir / "session.json").read_text())
    if info.get("status") != "scored":
        raise RuntimeError(f"session {session_dir} is {info.get('status')!r}, not scored")
    session = Session(session_dir)
    frozen = session.frozen()
    if frozen is None:
        raise RuntimeError(f"session {session_dir} is scored but froze no candidate")
    rate = float(info["heldout_rate"])
    per_iteration = iterations(outcome.trajectory)
    report = {
        "task": info["task"],
        "frozen": frozen,
        "heldout_rate": rate,
        "threshold": info["budget"]["threshold"],
        "leaderboard": [score.model_dump() for score in session.leaderboard()],
        "iterations": len(per_iteration),
        "per_iteration": per_iteration,
    }
    (session_dir / REPORT_NAME).write_text(json.dumps(report, indent=2))
    return rate


def iterations(trajectory: Trajectory) -> list[dict[str, Any]]:
    """Tokens and seconds per submitted candidate: the model turns from the
    previous ``submit`` call up to and including this one, in submission order."""
    rows: list[dict[str, Any]] = []
    prompt = completion = 0
    started: str | None = None
    for step in trajectory.steps:
        if step.source != "agent":
            continue
        started = started or step.timestamp
        if step.metrics:
            prompt += step.metrics.prompt_tokens
            completion += step.metrics.completion_tokens
        if calls_tool(step, "submit"):
            latency = step.extra.latency_s if step.extra else 0.0
            rows.append(
                {
                    "candidate": len(rows) + 1,
                    "prompt_tokens": prompt,
                    "completion_tokens": completion,
                    "seconds": seconds_between(started, step.timestamp) + latency,
                }
            )
            prompt = completion = 0
            started = None
    return rows


def calls_tool(step: Step, tool: str) -> bool:
    """Whether the step called the lab tool natively or as ``dimos mcp call <tool>`` from a shell."""
    pattern = re.compile(rf"\bmcp call {re.escape(tool)}\b")
    return any(
        call.function_name == tool or bool(pattern.search(json.dumps(call.arguments)))
        for call in step.tool_calls or ()
    )


def seconds_between(start: str, end: str) -> float:
    return (datetime.fromisoformat(end) - datetime.fromisoformat(start)).total_seconds()


def case(task: str, budget: Budget = BUDGET) -> EvalCase:
    return EvalCase(
        id=re.sub(r"[^a-z0-9]+", "_", task.lower()).strip("_"),
        inputs=actor_prompt(task, budget),
        environment=AutoresearchEnvironment(task=task, budget=budget),
        grade=heldout_rate,
        timeout_s=2 * 3600.0,
        threshold=budget.threshold,
        tags=frozenset({"mujoco", "manipulation", "autoresearch"}),
    )


SUITE: Suite = [case(task) for task in TASKS]
