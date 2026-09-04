# Evals

## Terminology

- **Case**: a discrete scenario being tested. This specifies the prompt sent to the agent, the environment, and scoring functions for the final world state and agent response.
- **Environment**: this is either a dataset, a live simulation, or an image file being passed to the agent.
- **Suite**: a collection of cases.
- **Agent**: this is a wrapper that might contain an entire agent loop, a single request to an llm provider, and it may contain tools. Agents include our mcp client as well as a simple single-turn question/answer with no tools.

## Quick start (CLI)

```bash
# two documentation cases against the go2_short recording (needs OPENAI_API_KEY)
dimos evals run dimos.evals.suites.examples --agent dimos.evals.agents.question_answer

# same cases with observations withheld (the guessing ablation)
dimos evals run dimos.evals.suites.examples --agent dimos.evals.agents.blind

# list suites
dimos evals list
```

Every runner invocation writes one `~/.local/state/dimos/evals/run-*/` directory.

| file | what |
|---|---|
| `manifest.json` | immutable, versioned run inputs: source, ordered selected case IDs, explicit agent arguments, runner settings, and Git state |
| `results.jsonl` | one row per case: score, steps, tokens, seconds, `ended_by`, trajectory path |
| `summary.json` | aggregate outcomes |
| `<case_id>/trajectory.json` | the run in Harbor's [ATIF](https://www.harborframework.com/docs/agents/trajectory-format) format
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

Agents live in `dimos/evals/agents/`, one per file. `--agent` takes the
module path; `--set field=value` sets one of its fields, e.g.
`--set max_steps=20`.

**Tools.** The case's blueprint decides the tool set: `Sim.blueprint`
is the robot stack plus `McpServer`, and its skill containers are the tools.
There is no tool filter. The agent's `modules` string is appended to the
launch command (`dimos run <blueprint> <modules>`), and `autoconnect` dedups
anything shared with the case, so `--set modules=unitree-go2-agentic` adds
the whole shipped agentic stack. On a `Dataset` case the agent's `modules`
are the whole launched stack (`dimos run <modules>`, no simulator or robot
underneath): a tool surface over the frozen recording, torn down with the
case. `Dataset(mcp_url=...)` attaches an already-running dimos instead. To
compare two tool sets on one task, run the suite twice with different
`--set modules=...`; each `trajectory.json` records the tools exposed.

**Limits.** `max_steps` caps model calls on agents that support it. The
case's `timeout_s` caps wall-clock time; the agent returns what it has by
then, marked `timeout`. Tokens and cost have no cap; both are recorded on
the trajectory and ranked.

**Observation encoding.** Each agent class hard-codes how the recording
reaches the model. `QuestionAnswer` calls `agent_encode()`; no other agent
does. Sending observations a different way means writing a new agent class.

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
- `o.trajectory` carries `steps`, `final_metrics`, `extra.ended_by`:
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
  passes blind is guessable; fix its distractors. `manifest.json` records which
  agent ran, so blind and sighted runs never get mixed up.
- **Preflight**: before anything runs, every case is checked against the
  agent. A missing stream fails with `"No stream 'lidar'. Available: [...]"`,
  a mismatched environment/agent pair with what's missing. Errors are per-case;
  one broken case never kills a run.

Direct `EvalRunner.run(cases, agent)` calls still write a manifest, but mark source and agent
provenance unavailable because arbitrary objects do not identify importable factories. The shipped
CLI and MCP entry points provide that provenance explicitly.
