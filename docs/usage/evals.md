# Evals

An eval case is an environment, an instruction, and a grader. Nothing else.
Which agent runs it is the run's business, so the same case is benchmarked
under a bare model, the shipped robot agent, or an external agent CLI without
changing, and scores stay comparable for years.

- **Environment**: what exists. `Dataset` (a frozen memory recording, or the
  part of it the task is about), `ImageFile` (a standalone image), or `Sim`
  (a live simulator: launches a blueprint, records everything it publishes).
- **Agent**: how the instruction reaches a model and how the model acts.
  `QuestionAnswer` encodes the whole recording into one prompt; `Blind`
  never looks; `Pi` is a coding agent over the recording as a file and, on a
  live case, the robot's tools; `McpClientAgent` drives the blueprint's own
  `McpClient`.
- **Grade**: one function over the outcome, the agent's trajectory (its final
  answer, steps, tokens) and the artifacts the environment produced (the
  recording). Runs once, after the agent finishes.

Memory is the only sensor channel: agents read `env.recording` and nothing
else; graders open the recording artifact.

## Quick start (CLI)

```bash
# two documentation cases against the go2_short recording (needs OPENAI_API_KEY)
dimos evals run dimos.evals.suites.examples --agent dimos.evals.agents.question_answer

# same cases with observations withheld (the guessing ablation)
dimos evals run dimos.evals.suites.examples --agent dimos.evals.agents.blind

# same cases, the Pi coding agent over the recording as a file (needs pi on PATH)
dimos evals run dimos.evals.suites.examples --agent dimos.evals.agents.pi

# list suites
dimos evals list
```

Every run writes to `~/.local/state/dimos/evals/run-*/`:

| file | what |
|---|---|
| `results.jsonl` | one row per case: score, steps, tokens, seconds, `ended_by`, trajectory path |
| `summary.json` | mean/pass rate/errors plus the **agent that ran** (its class and every constructor argument: model, prompt, `max_steps`, `modules`, ...) and the git sha |
| `<case_id>/trajectory.json` | every tool available to the agent and one `Step` per model call (message, tool calls with results, tokens, latency) |
| `<case_id>/raw/NNN-request.json`, `NNN-response.json` | the exact payload sent to and received from the provider for every call |

To generate deterministic image questions from recordings, see
[Visual Question Answering](/docs/usage/vqa.md).

## Your first eval, end to end

Build a tiny recording. Any memory store works, since this is the same API the
robot's Recorder uses (see `dimos/memory/intro.md` for the full Stream API):

```python session=evals ansi=false no-result
import os
from pathlib import Path

os.environ["DIMOS_LOG_LEVEL"] = "WARNING"  # keep doc output stable

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import make_vector3

Path("/tmp/evals_intro.db").unlink(missing_ok=True)
store = SqliteStore(path="/tmp/evals_intro.db")
odom = store.stream("odom", PoseStamped)
for i in range(20):
    odom.append(
        PoseStamped(position=make_vector3(float(i), 2.5, 0.0),
                    orientation=Quaternion(0, 0, 0, 1), frame_id="world"),
        ts=1000.0 + i,
    )
store.stop()
```

A case is one Python literal. `Dataset.select` is a tuple of callables that
receive the opened `Store` and return the `Stream`s the recording holds for
this case: anything the Stream API expresses (windows, filters, single
frames). `grade` reads the agent's final answer:

```python session=evals ansi=false no-result
from dimos.evals.environments.dataset import Dataset
from dimos.evals.scorers import first_number, within
from dimos.evals.types import EvalCase

case = EvalCase(
    id="how_far",
    inputs="How far along x did you travel, in meters?",
    environment=Dataset("/tmp/evals_intro.db", select=(lambda s: s.streams.odom,)),
    # model text -> float, graded: 1.0 exact, linear to 0 at ±1m
    grade=lambda o: within(1.0)(19.0, first_number(o.trajectory.final_answer)),
)
```

Pick an agent and run. `QuestionAnswer` puts `agent_encode()` of every
selected observation (at most 8 per stream, spread evenly) in front of the
question and makes one model call. `chat_model=` injects any LangChain chat
model. Here it is a canned fake so this document runs offline; drop it to use
the agent's `model` with the production construction:

```python session=evals ansi=false
from langchain_core.language_models.fake_chat_models import FakeListChatModel
from dimos.evals.agents.question_answer import QuestionAnswer
from dimos.evals.runner import EvalRunner, summarize

agent = QuestionAnswer(chat_model=FakeListChatModel(responses=["about 19 meters"]))
result = EvalRunner().run([case], agent)[0]
print(f"score={result.score} passed={result.passed} answer={result.final_answer!r} steps={result.steps}")
s = summarize([result])
print(f"n={s.n} mean={s.mean_score} pass_rate={s.pass_rate} errors={s.errors}")
```

```results
score=1.0 passed=True answer='about 19 meters' steps=1
n=1 mean=1.0 pass_rate=1.0 errors=0
```

That's the whole loop: environment -> agent -> trajectory + artifacts ->
grade -> run dir.

## Agents

An agent is a module defining one dataclass: everything it decides is a
constructor argument (`model`, `system_prompt`, `max_steps`, `frames_per_stream`,
`modules`), and `summary.json` records all of them. `--agent` names the
module; `--set field=value` sets a field. Values that parse as JSON (`10`,
`null`, `true`) are decoded; everything else remains text. The same two flags
reach an agent from another package.

```python session=evals ansi=false
from dimos.evals.cli import load_agent
from dimos.evals.module import list_agents

for module in list_agents():
    a = load_agent(module)
    print(f"{module.rsplit('.', 1)[1]:16} {type(a).__name__:15} modules={a.modules!r} max_steps={getattr(a, 'max_steps', None)}")
```

```results
blind            Blind           modules='' max_steps=None
mcp_client       McpClientAgent  modules='' max_steps=None
pi               Pi              modules='' max_steps=40
question_answer  QuestionAnswer  modules='' max_steps=None
```

- The world is on the case: `Sim.blueprint` is the robot stack plus
  `McpServer`, and its skill containers **are** the tool set (there is no tool
  filter). An agent's `modules` are appended at launch (`dimos run <blueprint>
  <modules>`): the shipped agent adds `unitree-go2-agentic`, the whole
  composite, and `autoconnect` dedups the base it shares with the case. To
  compare two tool sets on one task, run the suite twice with a different
  `--set modules=...`; each `trajectory.json` records the tools exposed.
- `max_steps` caps model calls on the agents that loop; `timeout_s` on the
  case caps wall-clock. Tokens and cost are recorded on the trajectory and
  ranked, never capped. A limit an agent can't honor is not a parameter it
  has: `QuestionAnswer(max_steps=3)` and `McpClientAgent(model=...)` are
  `TypeError`s.
- How the recording reaches the model is the agent, not a parameter:
  `agent_encode()` is called in `QuestionAnswer` and nowhere else. A different channel is a different agent class.
- `Pi` (`dimos.evals.agents.pi`) is the [Pi coding agent](https://pi.dev)
  run headless with the run dir as its working directory: the recording is a
  file it opens from Python, and a live robot's tools are listed in its prompt
  and called through `dimos mcp call` from `bash` (Pi has no MCP client; its
  authors say to give it CLI tools, and dimos ships one). Provider traffic
  goes through a local recording proxy, so its `raw/` looks like every other
  agent's. Needs `pi` on PATH and `OPENAI_API_KEY`; runs on any environment.
- An agent is any object implementing the `Agent` protocol (`modules`,
  `available_tools`, `preflight`, `run`), in-repo or from another package. Every implementation
  saves each provider request/response whole under `<case_id>/raw/`.

A mismatched pair fails **in preflight**, before a sim boots or a model call
is paid for, with both sides named:

| case environment | agent | preflight raises |
|---|---|---|
| `Dataset` / `ImageFile` | non-empty `modules` | a frozen recording launches nothing |
| `Dataset` / `ImageFile` | `McpClientAgent` | agent needs a running `McpClient`; the environment has no robot |
| `Sim(attach=True)` | `McpClientAgent` with `modules=""` and no dimos running | nothing to attach to |

Nothing runs blind unless the agent is `Blind()`: an agent that encodes the
recording and finds nothing to encode raises instead of sending an empty
observation.

## Scoring

Scores are floats in `[0, 1]`; `passed = score >= threshold`. Scorers are
plain functions that compose inside `grade`:

```python session=evals ansi=false
from dimos.evals.scorers import choice, exact, first_number, ramp, within, yes_no

print(exact("yes", "yes"), within(2.0)(10.0, 11.0), ramp(1.0, band=2.0))
print(first_number("around 12.5 m"), yes_no("Yes, clearly."), choice(["chairs", "sofas"])("Mostly chairs."))
```

```results
1.0 0.5 0.5
12.5 yes chairs
```

- `exact`: equality. Pair with a parser (`yes_no`, `choice(options)`, `int`) so
  formatting noise doesn't fail a correct answer.
- `within(band)`: graded numeric credit. 1.0 exact, 0.5 halfway, 0 outside.
- `ramp(distance, band)`: same ramp over meters. Msg types support arithmetic,
  so physical graders stay one-liners.
- `judge(rubric)`: LLM-as-judge with partial credit, wrapping the
  langchain/openevals standard.
- `o.trajectory` carries `steps`, `input_tokens`, `output_tokens`, `ended_by`:
  "under N tokens" or "did not hit `max_steps`" as a pass criterion is the
  grader reading it.

A grader that raises (an unparseable reply, a missing stream) makes that case
an **error**, not a score; the run continues.

## Live environments

`Sim` launches `dimos --simulation dimsim --dimsim-scene <scene> --record run
<blueprint> <modules>`, waits for MCP, runs the case's `setup`, and hands out the
recording that `--record` writes (`recordings/<run-id>/memory.db`). The run
ends when the agent finishes or `timeout_s` hits, the environment stops, and
`grade` reads the recording once. `recording(o)` opens it:

```python session=evals ansi=false no-result
from dimos.evals.environments.sim import Sim
from dimos.evals.scorers import ramp
from dimos.evals.types import EvalCase, recording
from dimos.msgs.geometry_msgs.Vector3 import Vector3

BED = Vector3(-3.567, -1.332, 0.0)


def ended_near_bed(o):
    store = recording(o)
    try:
        p = store.streams.odom.last().data.position
    finally:
        store.stop()
    return ramp((BED - p).length(), band=2.0)


go_to_bed = EvalCase(
    id="go_to_bed",
    inputs="go to the bed",
    environment=Sim(
        blueprint="unitree-go2 mcp-server unitree-skill-container",
        simulator="dimsim",
        scene="apartment",
    ),
    grade=ended_near_bed,
    timeout_s=180.0,
)
```

```bash
dimos evals run dimos.evals.suites.dimsim_house --agent dimos.evals.agents.mcp_client --set modules=unitree-go2-agentic
```

The recording holds the whole history, so "never left the zone" is `min` over
the `odom` stream in the grader, and "coverage per meter driven" fuses the
recorded `lidar` and integrates `odom`. `Sim(attach=True)` drives an
already-running dimos (start it with `--record`) instead of launching one.

## Running

- **CLI**: `dimos evals run <dotted.suite> --agent <agent-module> [--set model=gpt-4o] [--tags nav] [--limit 5]`
- **Python**: `EvalRunner().run(SUITE, agent, tags=frozenset({"encoding"}))`
- **pytest**: suites are importable lists. Use
  `@pytest.mark.parametrize("case", SUITE)` and assert on `passed`
  (gate live-model tests with `skipif_no_openai`).
- **MCP**: the `EvalModule` skills `run_evals` / `list_eval_suites` /
  `list_eval_agents` return the summary + run dir, so a coding agent can run
  evals, grep trajectories, edit prompts/encodings, and run again.
- **Blind ablation**: run every new suite with
  `--agent dimos.evals.agents.blind` once before trusting it. A case that still
  passes blind is guessable; fix its distractors. `summary.json` records which
  agent ran, so blind and sighted runs never get mixed up.
- **Preflight**: before anything runs, every case is checked against the
  agent. A missing stream fails with `"No stream 'lidar'. Available: [...]"`,
  a mismatched environment/agent pair with what's missing. Errors are per-case;
  one broken case never kills a run.
