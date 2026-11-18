# Autoresearch

A coding agent writes a program against a fixed robot API; the lab runs it on
fixed trials, logs every call, and the agent repairs or replaces it until it
freezes one candidate. This package is the loop's core: the session on disk,
the program contract and runner, the judges and the recorder hook. The DimOS
module, the MCP tools and the blueprint that put a robot behind it come
separately.

## A session on disk

`~/.local/state/dimos/autoresearch/<name>/` (`SESSIONS_DIR / name`):

```
session.json          task, budget (iterations, candidates per round, dev and
                      held-out trial counts, threshold), model, status (idle,
                      running, frozen, scored) and, once scored, the frozen
                      candidate's held-out pass rate
candidates/NN/        one directory per submitted program, never changed
  program.py          the candidate
  hypothesis.md       what the agent expects it to fix and where it may fail
  results.json        one record per trial: split, index, success, stage,
                      note, status, duration_s, trace path, start_ts (the
                      recorder's time at start, which joins the trial to its
                      exported episode), episode index, backend
  trials/<split>-<k>/ trace.jsonl, keyframes/*.png, program.log, outcome.json
library/*.md          notes the agent keeps for later sessions
frozen                the number of the final candidate
```

`Session.create(dir, task, budget, model)` starts one; `Session(dir)` opens
it. `submit(program, hypothesis)` validates and stores a candidate and returns
its number; `record_trial`, `results` and `leaderboard` keep score;
`freeze(NN)` names the final candidate, after which nothing can be submitted.
A trial that errored before its record was written (a judge that could not
answer, a backend that could not connect) can be run again; one on the record
cannot.

## The program contract

```python
import math  # the only allowed import


def run(robot, task):
    cube = robot.scene.objects(["cube"])[0]
    x, y, z = cube.position
    robot.scene.clear(cube.id)
    robot.gripper.open()
    robot.arm.move_pose([x, y, z + 0.10], [math.pi, 0.0, 0.0])
    robot.arm.move_linear(dz=-0.08)
    robot.gripper.close()
    robot.arm.move_linear(dz=0.10)
```

`task.text` is the task in words. The program never sees the judge or any
score. The import rule is checked on submit and enforced when the program
runs: its builtins have no `open`, `exec`, `eval` or `compile`, and an
import of anything but `math` raises `ImportError`. Motion calls block and
raise `RobotCallError` (with a `status`) when planning or execution fails;
the program may catch it and try another way.

| `robot.` | Does | Backed by (live) |
|---|---|---|
| `arm.joints()` | joint positions, radians | `Arm.joints` |
| `arm.pose()` | end-effector position (m) and xyzw quaternion | `Arm.pose` |
| `arm.move_joints(positions)` | planned move to joint positions | `Arm.move_joints` |
| `arm.move_pose(xyz, rpy or quaternion)` | planned move; orientation optional | `Arm.move_pose` |
| `arm.move_linear(dx, dy, dz)` | straight-line translation, metres | `Arm.move_linear` |
| `arm.home()` | the home preset | `Arm.home` |
| `gripper.open()` / `close()` | fully open or closed | `Arm.open_gripper` / `close_gripper` |
| `gripper.set(x)` | the gripper joint at `x`, URDF units; `gripper.range()` gives (closed, open) | `Arm.set_gripper_position` |
| `scene.objects(words)` | detected objects: id, name, position, size | the object-registration module's `scan_scene` |
| `scene.clear(object_id, margin=0.03)` | leave that object, plus `margin` metres on every side, out of the obstacle map: the map treats every object as an obstacle, so clear each object you will touch or put something into; clears add up | the map's box filter `set_box` |
| `scene.restore()` | put every cleared object back in the map | the box filter `clear_box` |
| `observe()` | saves a keyframe, returns its path | an `Observer` |
| `log(text)` | a line in the trace | the trace |

Every call is appended to `trace.jsonl` as one JSON line: time, call, args,
status (`OK`, `PLANNING_FAILED`, `EXECUTION_FAILED`, `TIMEOUT`, `RAISED`), a
short result, duration, and for motion calls the keyframe saved before and
after.

## Running a program

`run_program(program_path, trial_dir, task, timeout_s, backend_factory, env)`
runs the candidate in a fresh `python -I` process (no `PYTHONPATH`, no current
directory on the path; only the DimOS checkout is added) and kills it at the
timeout. `env`, when given, is the child's whole environment, so the lab can
hand it the bus settings `Dimos.connect()` needs. The child gets four
arguments: the program path, the trial directory, the task text and
`backend_factory`, a `"package.module:function"` string. It imports that
function and calls it with the trial directory to get its `RobotBackend`:

- `dimos.autoresearch._test_helpers:fake_backend` for tests: moves are
  bookkeeping, every word finds one object, frames are blank PNGs.
- a live stack: a function that returns `DimosBackend.connect(observer,
  GripperRange(closed, open))`, which attaches through `Dimos.connect()`,
  `Arm.from_app` and the object-registration module. That module must run
  with `target_frame="world"`, the frame the arm plans in; `scene.objects()`
  refuses any other frame. The observer writes the camera frame as a PNG.

The child's prints go to `program.log`; `outcome.json` says whether `run`
completed, raised (with the traceback, also when the module body fails or
the program calls `exit()`) or was killed. A child that dies before writing
an outcome, because the backend could not connect or the kill came before
the backend was up, raises in the parent instead of counting as a failed
trial.

## One trial

`run_trial(session, NN, split, index, world, judge, episodes,
backend_factory, timeout_s)`:

1. `world.reset(seed)`, the seed fixed by split and index, so every candidate
   sees the same scenes; `world.snapshot()` before.
2. `episodes.command("start")` on the learning-stack recorder.
3. `run_program` with the timeout.
4. `world.snapshot()` after; `judge.verdict(before, after)`.
5. `save` on success, `discard` otherwise; the saved episode's index goes on
   the record, with the recorder's time at `start` as `start_ts`.
6. `session.record_trial(NN, record)`.

Judges are pure functions of the two snapshots (`LiftJudge`: the body rose at
least 5 cm; `PlaceJudge`: the body ended inside a box; `PlaceAtJudge`: within
5 cm of where another body stood), or `OperatorJudge`, which asks the person
at the robot. A `Snapshot` is each tracked body's pose, which is what the
MuJoCo sim publishes for its tracked bodies. `NoEpisodes` stands in for the
recorder on a stack without one.

## The lab module

`AutoresearchModule` (`module.py`) puts a robot behind the loop. Its config
names the session directory, the task, the budget and the judge; `start()`
creates the session and homes the arm. The actor's tools are its skills,
served over MCP: `task`, `scene`, `submit`, `run`, `stop_run`, `results`,
`leaderboard`, `freeze` and `note`. `run` and `freeze` run the trials on a
background thread and return at once; `results` is polled. The world is
`MujocoWorld` (`world.py`): a seeded reset places the cube through
`SimSceneSpec`, drives the arm home through the controller's trajectory task
(no plan, so it works from any pose), waits for it to hold still and for an
obstacle map of the new bench, then puts back whatever the last program
cleared from that map and waits for fresh poses; a snapshot stops the arm and
waits for it to hold still. Programs run on `live.py:backend`,
which attaches to the stack and takes keyframes from its `color_image`
stream. `TruthSceneModule` (`truth_scene.py`) answers scene scans from the
simulator's true poses so no detector is needed. The lab publishes an
`AutoresearchStatus` document (`status.py`) on `autoresearch_status`, and
`session.json` goes idle, running, frozen, scored with the held-out rate.
The blueprint is `autoresearch-metal-sim`.

## Run the actor from the lab

`start_actor(model=None, provider=None)` is an RPC on the lab, not a skill, so
the actor never sees it: it runs the coding agent on a thread of the lab
itself, and a Desktop app button can run a whole session with no terminal.
The agent is Pi (`dimos.evals.agents.pi`) with the prompt in `prompt.py`,
pointed at this stack's MCP server, on the config's `model` and
`actor_provider` unless the call names others; `actor_max_steps` and
`actor_timeout_s` bound the run. The provider's key must be in the lab's
environment: put it in the `.env` of the dimos directory, which `dimos run`
loads. `anthropic` reads `ANTHROPIC_API_KEY`, `openai` reads
`OPENAI_API_KEY`, `google` reads `GEMINI_API_KEY`. The Pi process gets the
lab interpreter's `bin` directory first on its PATH, so a stack launched by
Desktop, whose service environment has no venv on PATH, still lets Pi run
`dimos mcp call`. Without the key, without `pi` on PATH, or without `dimos`
there or on PATH, the actor fails at once and the status says why.
`stop_actor()` kills the Pi process; the session stays as it is, so a
trial loop the actor started runs on. In the status document `actor` goes
idle, running, then done (the actor returned; freeze from the app if it did
not) or failed with `actor_error`, beside `actor_model` and `actor_provider`.
The trajectory (`trajectory.json`, the ATIF document), Pi's raw requests, its
system prompt and its stderr land under `<session>/actor/`.
