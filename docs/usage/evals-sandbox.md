# Sandbox evals

A sandbox eval puts the agent in a container that holds nothing but the agent: Ubuntu, the
[Pi](https://github.com/earendil-works/pi) coding agent and a provider API key. The robot
lives in a second container on the same Docker network, reachable as one Zenoh router at
`tcp/robot:7447`. The agent learns about it only from the prompt, and cannot tell whether
the router fronts a blueprint in that container or a real robot.

```text
        docker compose project, one per case
 ┌──────────────────────────────────────────────────────────────────────────┐
 │  agent                         robot                                     │
 │  Ubuntu + pi ── zenoh ───────▶ zenoh router :7447                        │
 │   ▲   │                         └─ dimos --record <args>  (a blueprint,  │
 │   │   │ model calls                on zenoh) ── recordings/memory.db     │
 │   │   │                            or: dials a real robot's router       │
 │   │   ▼                                                                  │
 │   │  llm-proxy ── saves each request/response ── provider API            │
 └───┼──────────────────────────────────────────────────────────────────────┘
     │ docker exec: pi --mode json <prompt>      (from `dimos evals sandbox`)
     │ docker exec: evaluate's commands
```

## A case

A case is data: a pydantic model that round-trips through JSON, so a suite can be a Python
module exporting `SUITE` or a JSON file of cases (`dimos evals sandbox <suite> --json` prints
one). The two things a case does, answering the agent and grading it, are functions it names
with `FunctionRef("module:function", params)`; `params` are JSON keyword arguments.

```python skip
from dimos.evals.sandbox.case import DimosRobot, FunctionRef, SandboxCase, SandboxOutcome
from dimos.memory.store.sqlite import SqliteStore


def evaluate(o: SandboxOutcome) -> float:
    with SqliteStore(path=str(o.recording)) as store:
        commands = store.stream("cmd_vel").to_list()
    script_ran = o.exec("python3 /root/work/drive.py --check").returncode == 0
    return float(bool(commands) and script_ran and o.done["distance_m"] > 0.3)


SUITE = [
    SandboxCase(
        id="drive_forward",
        prompt="A robot is reachable over Zenoh at tcp/robot:7447. ...",
        robot=DimosRobot(args=("run", "unitree-go2")),
        ready_key="dimos/odom/**",
        done_schema={
            "type": "object",
            "properties": {"distance_m": {"type": "number"}},
            "required": ["distance_m"],
        },
        evaluate=FunctionRef(function=f"{__name__}:evaluate"),
        human=FunctionRef(function="dimos.evals.sandbox.human:until_done", params={"times": 3}),
        timeout_s=600.0,
    ),
]
```

- `prompt` is the user's first message. Describe the endpoints there.
- `robot=DimosRobot(args=...)` runs `dimos <args>` in the robot container with
  `DIMOS_TRANSPORT=zenoh` and every zenoh session dialing the router, so the blueprint's own
  topics (`dimos/<stream>/<msg type>`) are what the agent sees. `forward=` adds endpoints the
  router dials itself, for a peer that only listens, e.g. `raw-robot-bridge` at
  `tcp/127.0.0.1:7448`, which serves plain JSON/bytes topics under `robot/`.
  `record_whole_run=False` drops `--record`: the blueprint is recorded only while the case's
  human or `evaluate` holds `record(label)` open (below). With `expose_dimos=False` the
  blueprint's own dimOS topics run under a zenoh namespace (`evals/<run>-<case>/dimos/...` on
  the wire; `--topic-namespace` sets it, `''` turns it off), so they can't cross another dimOS
  on the same machine, e.g. a sim you have running. The topics a bridge serves the agent keep
  their keys (`robot/...`), so a prompt can name them.
- `ready_key` is a zenoh key expression the robot must publish before the agent starts.
- `done_schema` is the JSON schema of the `done` tool's arguments. Pi validates each call;
  an invalid call is an error the agent sees and can retry, and a valid call ends the agent's
  turn. A run that never makes a valid call scores 0 and `evaluate` is not called.
- `human` pretends to be the user. It is a generator function, `human(conversation, **params)`,
  that the runner asks for the next message every time the agent stops. A yielded string is
  the user's next message, in the same Pi session; returning ends the conversation. The
  conversation, not the done tool, decides when the agent is finished:
  - `conversation.called_done` / `conversation.done`: whether the agent's latest turn ended
    with an accepted done call, and its arguments.
  - `conversation.turns`, `conversation.exec("cmd")` (e.g. to try the agent's system),
    `conversation.case_dir`.
  - `with conversation.record("trial-1") as db:` records the robot's blueprint into
    `recordings/trial-1/memory.db` in the case directory while the block runs:
    `dimos.evals.sandbox.record`, started beside the blueprint (in the robot container, with
    its environment) and stopped with SIGINT at the end of the block, taps the same transports
    as `dimos --record`. It yields None for a robot that isn't a blueprint in the sandbox. Each
    label is used once.

  ```python skip
  def until_done(conversation, *, message="You have not called done yet...", times=3):
      for _ in range(times):
          if conversation.called_done:
              return
          yield message
  ```

  That is the default (`dimos.evals.sandbox.human:until_done`). `while True: yield "..."` keeps
  the agent going until the timeout or the step budget. A human can also run what the agent
  built and answer "that didn't work, please fix it" until it does.
- `timeout_s` covers the whole conversation, every agent turn.
- `evaluate(o, **params)` runs after the human ends the conversation, with the robot and agent
  containers still up:
  - `o.done`: the last accepted done arguments.
  - `o.recording`: the robot blueprint's `dimos --record` store (`memory.db`): every one of its
    streams at full rate, what it published and what the agent sent it, by stream name; open it
    with `dimos.memory.store.sqlite.SqliteStore` (see [memory](/dimos/memory/intro.md)).
    None for a robot that isn't a blueprint in the sandbox, or with `record_whole_run=False`.
  - `o.record("grading")`: as `conversation.record`, e.g. around the graded run.
  - `o.exec("cmd")`: a `bash -lc` command in the agent container, returning
    `subprocess.CompletedProcess`.
  - `o.trajectory`: the ATIF trajectory, every turn, with the human's messages as user steps.

`dimos/evals/sandbox/example.py` is a runnable example on a mock wheeled base.

## Running

```bash skip
docker build -f docker/evals_sandbox/agent.Dockerfile -t dimensional/evals-agent docker/evals_sandbox
docker build -f docker/evals_sandbox/robot.Dockerfile -t dimensional/evals-robot .

export OPENAI_API_KEY=...
dimos evals sandbox dimos.evals.sandbox.example --model gpt-5.6-luna --max-steps 100
```

Each run writes `~/.local/state/dimos/evals/sandbox-*/`: `manifest.json`, `results.jsonl`,
`summary.json`, and per case `trajectory.json`, `raw/` (exact model payloads),
`recordings/<run id>/memory.db` (the robot's `dimos --record`) or `recordings/<label>/memory.db`
and `record.log` (each `record(label)`),
`work/` (the agent's working directory, copied out before its container is removed),
`case.json` (the case as run), `harness/session/` (Pi's own session), `compose.json`, `containers.log` and `pi-stderr.txt`.
`results.jsonl` gains a line as each case ends. The robot image is rebuilt from the
checkout, so rebuild it after changing dimOS.

`--resume <run dir> --message "..."` continues each case's session from an earlier run: a
fresh robot, the agent's `/root/work` copied back in, and the message as its next user turn.

## A real robot

The same router runs anywhere dimOS does. On the robot (or the machine that drives it):

```bash skip
dimos evals robot -- run unitree-go2        # router on tcp/0.0.0.0:7447, blueprint behind it
```

Then point the sandbox at it. Every case's `DimosRobot` is replaced by a robot container
that only forwards to that router, so the agent dials the same `tcp/robot:7447` and sees the
same keys:

```bash skip
dimos evals sandbox dimos.evals.sandbox.example --robot tcp/192.168.123.18:7447
```

`--listen` moves the router off 7447 when the robot already uses that port (a Go2 running
`go2-dds` is itself a router there). A robot on the eval machine itself is
`tcp/host.docker.internal:<port>`.

`--robot-on-host` does that per case: each case's blueprint runs on this machine, behind its
own router on 7447, started and stopped with the case, writing ground truth to the case
directory (its recording lands in this checkout's `recordings/`). Use it when the robot needs
the GPU, which Docker on a Mac doesn't have; the agent stays in its container.

```bash skip
dimos evals sandbox dimos.evals.sandbox.go2_home --robot-on-host --max-steps 2000
```

On Linux with an NVIDIA GPU and the NVIDIA Container Toolkit, `--robot-gpu` keeps the robot
in its container and gives that container the GPU (`gpus: all`, with graphics, for Vulkan):

```bash skip
dimos evals sandbox dimos.evals.sandbox.go2_home --robot-gpu --max-steps 2000
```

## What the agent can and cannot see

- It has a shell as root in its own container, internet access, and the provider key its
  harness needs. It has no dimOS checkout, no suite files and no grader.
- It can resolve the `llm-proxy` hostname on the network. The router's admin
  space is off, so zenoh does not list who is connected behind it.
- Pi's harness files (`/opt/harness`) are mounted into the agent container. The done
  result is read from Pi's event stream, not from a file, so editing them changes nothing.

## The home plants cases

`dimos/evals/sandbox/go2_home.py` has two cases on
[dim-go2-sim](https://pypi.org/project/dim-go2-sim/)'s Go2 (a rolling Mid-360 and a front
camera) in a scanned ranch home ([SuperSplat 3f89bbd3](https://superspl.at/scene/3f89bbd3),
CC BY 4.0) with two people walking slow loops through it: map the house, stop at every plant
facing it, then come back to the living room, without bumping into anyone. `home_plants_dimos`
is the same case, its prompt with one more sentence: the system must be built on the
Dimensional framework, out of its blueprints, modules and streams.

```bash skip
dimos evals sandbox dimos.evals.sandbox.go2_home --robot-gpu --max-steps 2000   # both
dimos evals sandbox dimos.evals.sandbox.go2_home --case home_plants_dimos --robot-gpu --max-steps 2000
```

- The robot container runs `sandbox-go2-home` with `--hide-dimos`: the agent sees only
  raw-robot-bridge's `robot/*` topics, never the simulator's ground truth. Its dimOS topics are
  under the run's zenoh namespace.
- The robot is recorded only while the agent's system runs: each of the owner's trials
  (`recordings/trial-<n>/memory.db`) and the graded run (`recordings/grading/memory.db`), every
  stream of the blueprint at full rate, ground truth and people included: about 11 GB an hour,
  nine tenths of it the lidar. Nothing is recorded while the agent writes code.
- `GroundTruthLog` writes the true pose, every contact (`person` when the dog touches someone)
  and the people's poses to the case directory, which only the grader reads.
- The five graded plants (`PLANTS`) and the living room (`LIVING_ROOM`) were placed by eye in
  renders of the scan. A plant counts as visited when the dog stops for 2 s within 1.2 m of it,
  facing it, with nothing in between.
- The human is the owner (`home_owner`). Until the agent calls done they say "I don't
  understand and I don't care..."; after each done they run its command for 20 minutes of sim
  time and, unless the dog stopped at every plant, ended in the living room and touched nothing
  and no one, tell it what they saw ("It never stopped at the palm in the back bedroom.", "It
  walked into one of us once!") and ask for a fix, at most 3 times. Its output is
  `/root/work/run-output.log`.
- `evaluate` then runs the agent's command for 30 minutes of sim time (the sim can run slower
  than the wall clock when the machine is loaded; ground truth is in sim time) and scores map
  coverage (the share of the home its lidar saw, from the true poses, against what is visible
  from every reachable spot; 0.25), the share of plants visited during that run (0.55) and
  ending in the living room (0.2), less 0.25 of the score for each bump into the house and 0.5
  for each time it touched a person. `grading.json` has the breakdown, with the closest the
  dog came to a person.
- `dim-go2-sim`'s `Go2Sim` module is the whole robot, and the people: `cmd_vel` in, lidar,
  camera, odometry, contacts and the people's poses out. The first start downloads and cooks
  the scan (31 MB, under a minute) into the robot cache. Without a GPU the camera renders with
  Mesa's software Vulkan, at about 2 frames a second.
