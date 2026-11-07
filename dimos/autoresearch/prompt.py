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

"""What the actor is told: the ASPIRE loop over the lab's tools, the program
API, and the words that mark a way around the tools."""

from __future__ import annotations

from dimos.autoresearch.session import Budget

# Tool calls mentioning these are denied: the ways past the lab's tools into the stack.
EXCLUDED_KEYWORDS = ("shell", "porcelain", "sdk", "topic", "select", "pick_object", "place_at")

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
  robot.scene.clear(object_id, margin)    leave that object, plus margin (m, default 0.03) on
                                          every side, out of the obstacle map: the map treats
                                          every object as an obstacle, so clear each object
                                          you will touch or put something into; clears add up
  robot.scene.restore()                   put every cleared object back in the map
  robot.observe()                         saves a keyframe, returns its path
  robot.log(text)                         a line in the trace
Example:
  import math

  def run(robot, task):
      \"\"\"Top-down grasp at the cube's centre. May fail if the fingers land on an edge.\"\"\"
      cube = robot.scene.objects(["cube"])[0]
      x, y, z = cube.position
      robot.scene.clear(cube.id)
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
