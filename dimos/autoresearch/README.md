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
                      held-out trial counts, threshold), model, status
candidates/NN/        one directory per submitted program, never changed
  program.py          the candidate
  hypothesis.md       what the agent expects it to fix and where it may fail
  results.json        one record per trial: split, index, success, stage,
                      note, status, duration_s, trace path, episode index
  trials/<split>-<k>/ trace.jsonl, keyframes/*.png, program.log, outcome.json
library/*.md          notes the agent keeps for later sessions
frozen                the number of the final candidate
```

`Session.create(dir, task, budget, model)` starts one; `Session(dir)` opens
it. `submit(program, hypothesis)` validates and stores a candidate and returns
its number; `record_trial`, `results` and `leaderboard` keep score;
`freeze(NN)` names the final candidate, after which nothing can be submitted.

## The program contract

```python
import math  # the only allowed import


def run(robot, task):
    cube = robot.scene.objects(["cube"])[0]
    x, y, z = cube.position
    robot.gripper.open()
    robot.arm.move_pose([x, y, z + 0.10], [math.pi, 0.0, 0.0])
    robot.arm.move_linear(dz=-0.08)
    robot.gripper.close()
    robot.arm.move_linear(dz=0.10)
```

`task.text` is the task in words. The program never sees the judge or any
score. Motion calls block and raise `RobotCallError` (with a `status`) when
planning or execution fails; the program may catch it and try another way.

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
| `observe()` | saves a keyframe, returns its path | an `Observer` |
| `log(text)` | a line in the trace | the trace |

Every call is appended to `trace.jsonl` as one JSON line: time, call, args,
status (`OK`, `PLANNING_FAILED`, `EXECUTION_FAILED`, `TIMEOUT`, `RAISED`), a
short result, duration, and for motion calls the keyframe saved before and
after.

## Running a program

`run_program(program_path, trial_dir, task, timeout_s, backend_factory)` runs
the candidate in a fresh `python -I` process (no `PYTHONPATH`, no current
directory on the path; only the DimOS checkout is added) and kills it at the
timeout. The child gets four arguments: the program path, the trial
directory, the task text and `backend_factory`, a `"package.module:function"`
string. It imports that function and calls it with the trial directory to get
its `RobotBackend`:

- `dimos.autoresearch.program:fake_backend` for tests: moves are bookkeeping,
  every word finds one object, frames are blank PNGs.
- a live stack: a function that returns `DimosBackend.connect(observer,
  GripperRange(closed, open))`, which attaches through `Dimos.connect()`,
  `Arm.from_app` and the object-registration module. The observer writes the
  camera frame as a PNG.

The child's prints go to `program.log`; `outcome.json` says whether `run`
completed, raised (with the traceback) or was killed. A child that dies before
writing an outcome, e.g. because the backend could not connect, raises in the
parent instead of counting as a failed trial.

## One trial

`run_trial(session, NN, split, index, world, judge, episodes,
backend_factory, timeout_s)`:

1. `world.reset(seed)`, the seed fixed by split and index, so every candidate
   sees the same scenes; `world.snapshot()` before.
2. `episodes.command("start")` on the learning-stack recorder.
3. `run_program` with the timeout.
4. `world.snapshot()` after; `judge.verdict(before, after)`.
5. `save` on success, `discard` otherwise; the saved episode's index goes on
   the record.
6. `session.record_trial(NN, record)`.

Judges are pure functions of the two snapshots (`LiftJudge`: the body rose at
least 5 cm; `PlaceJudge`: the body ended inside a box), or `OperatorJudge`,
which asks the person at the robot. A `Snapshot` is each tracked body's pose
plus the gripper value, which is what the MuJoCo sim publishes for its tracked
bodies. `NoEpisodes` stands in for the recorder on a stack without one.
