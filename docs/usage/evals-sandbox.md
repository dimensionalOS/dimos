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
 │   ▲   │                         └─ dimos <args>  (any blueprint, on zenoh)│
 │   │   │ model calls                or: dials a real robot's router       │
 │   │   ▼                                                                  │
 │   │  llm-proxy ── saves each request/response ── provider API            │
 │   │                                                                      │
 │   │                            recorder ── subscribes to ** ── zenoh.db  │
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


def evaluate(o: SandboxOutcome) -> float:
    commands = [s.decode() for s in o.recording.samples("robot/cmd_vel/json")]
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
- `ready_key` is a zenoh key expression the robot must publish before the agent starts.
- `record` picks what the recorder keeps: `{"robot/lidar/**": 1.0, "robot/odom/**": None}`
  keeps lidar at most once a second, odometry whole, and drops every other key. Leave it
  unset to keep everything, which for a lidar is gigabytes an hour.
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
  - `o.recording`: every zenoh sample of the run, both directions
    (`samples(key_expr)`, `last(key_expr)`, `keys()`; `Sample.decode()` gives a dimOS message
    when the key names its type, parsed JSON, or bytes).
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
`summary.json`, and per case `trajectory.json`, `raw/` (exact model payloads), `zenoh.db`,
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

## What the agent can and cannot see

- It has a shell as root in its own container, internet access, and the provider key its
  harness needs. It has no dimOS checkout, no suite files and no grader.
- It can resolve the `llm-proxy` and `recorder` hostnames on the network. The router's admin
  space is off, so zenoh does not list who is connected behind it.
- Pi's harness files (`/opt/harness`) are mounted into the agent container. The done
  result is read from Pi's event stream, not from a file, so editing them changes nothing.
