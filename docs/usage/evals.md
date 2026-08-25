# Evals

Evals measure what an agent (or a bare model, or a single skill) can do with
the robot's memory. Two kinds:

- **Passive** — the world is a frozen memory recording. Deterministic, cheap,
  repeatable. Run these constantly.
- **Interactive** — a live robot or sim; actions change the world; scoring
  samples the live memory store while the agent works.

memory is the source of truth for everything an eval sees: context selectors
return real `Stream`s, and interactive scoring reads a real `Store`.

## Quick start (CLI)

```bash
# two documentation cases against the go2_short recording (needs OPENAI_API_KEY)
dimos evals run dimos.evals.suites.examples

# same questions with observations withheld — the guessing ablation
dimos evals run dimos.evals.suites.examples --blind

# list available suites
dimos evals list
```

Each run prints a per-case table and writes `results.jsonl`, `summary.json`,
and per-case transcripts to `~/.local/state/dimos/evals/run-*/`.

To generate deterministic image questions from recordings, see
[Visual Question Answering](/docs/usage/vqa.md).

## Your first eval, end to end

Build a tiny recording (any memory store works — this is the same API the
robot's Recorder uses; see `dimos/memory/intro.md` for the full Stream API):

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
```

```python session=evals ansi=false
print(odom.summary())
```

```results
Stream("odom"): 20 items, 1970-01-01 00:16:40 — 1970-01-01 00:16:59 (19.0s, 1.00 Hz, 1.68 KiB)
```

A passive eval is one Python literal. `context` is a tuple of callables that
receive the opened `Store` and return the mem2 `Stream`s the model may see —
anything the Stream API expresses (windows, filters, single frames) works, and
the runner evenly downsamples each selected stream to `context_budget`
observations before encoding:

```python session=evals ansi=false no-result
from dimos.evals.scorers import first_number, within
from dimos.evals.types import PassiveEval

case = PassiveEval(
    id="how_far",
    inputs="How far along x did you travel, in meters?",
    expected=19.0,
    parse=first_number,          # model text -> float
    score=within(1.0),           # graded: 1.0 exact, linear to 0 at ±1m
    context=(lambda s: s.streams.odom,),
    dataset="/tmp/evals_intro.db",   # a mem2 name ("go2_short") or a path
)
```

Run it. `chat_model=` injects any LangChain chat model — here a canned fake so
this document runs offline; drop the argument to use the production model
config (`gpt-5.6-luna`, same construction as the deployed `McpClient`):

```python session=evals ansi=false
from langchain_core.language_models.fake_chat_models import FakeListChatModel
from dimos.evals.runner import EvalRunner, summarize

runner = EvalRunner(chat_model=FakeListChatModel(responses=["about 19 meters"]))
result = runner.run([case])[0]
print(f"score={result.score} passed={result.passed} outputs={result.outputs!r}")
s = summarize([result])
print(f"n={s.n} mean={s.mean_score} pass_rate={s.pass_rate} errors={s.errors}")
```

```results
score=1.0 passed=True outputs='about 19 meters'
n=1 mean=1.0 pass_rate=1.0 errors=0
```

That's the whole loop: dataset -> context streams -> encoded prompt -> model
-> parse -> score -> artifacts.

## Scoring

Scores are floats in `[0, 1]`; `passed = score >= threshold`. Scorers are
plain functions `(expected, got) -> float` — a custom heuristic is a lambda,
not a class:

```python session=evals ansi=false
from dimos.evals.scorers import choice, exact, first_number, ramp, within, yes_no

print(exact("yes", "yes"), within(2.0)(10.0, 11.0), ramp(1.0, band=2.0))
print(first_number("around 12.5 m"), yes_no("Yes, clearly."), choice(" Chairs. "))
```

```results
1.0 0.5 0.5
12.5 yes chairs
```

- `exact` — equality (the default). Pair with a parser (`yes_no`, `choice`,
  `int`) so formatting noise doesn't fail a correct answer.
- `within(band)` — graded numeric credit: 1.0 exact, 0.5 halfway, 0 outside.
- `ramp(distance, band)` — same ramp over meters; msg types support
  arithmetic, so physical scorers stay one-liners:
  `lambda s: ramp((GOAL - s.streams.odom.last().data.position).length(), band=0.5)`
- `judge(rubric)` — LLM-as-judge with partial credit, wrapping the
  langchain/openevals standard (`inputs`/`reference_outputs` convention, so
  external VQA benchmarks map on natively).

Interactive evals score a *series* (one sample per `interval_s`); `aggregate`
reduces it:

```python session=evals ansi=false
from dimos.evals.scorers import final, floor, mean

print(final([0.2, 0.9]), floor([0.4, 0.2, 0.8]), mean([0.0, 1.0]))
```

```results
0.9 0.2 0.5
```

`final` = "where did it end up", `floor` = "never left the zone",
`mean` = "how good was it throughout".

## Interactive evals

The case names its environment (reproducibility); `score` reads the **live**
store the robot's Recorder writes, sampled every `interval_s`:

```python session=evals ansi=false no-result
from dimos.evals.scorers import final, ramp
from dimos.evals.types import InteractiveEval
from dimos.msgs.geometry_msgs.Vector3 import Vector3

BED = Vector3(-3.567, -1.332, 0.0)

go_to_bed = InteractiveEval(
    id="go_to_bed",
    inputs="go to the bed",
    score=lambda s: ramp((BED - s.streams.odom.last().data.position).length(), band=2.0),
    aggregate=final,
    interval_s=2.0,
    timeout_s=180.0,
    blueprint="unitree-go2-agentic go2-memory",
    simulator="dimsim",
    scene="apartment",
)
```

```bash
dimos evals run dimos.evals.suites.dimsim_house --live-db recording_go2.db
```

The result carries the full `(t, score)` series — "reached the bed at t=50s
and stayed" and "grazed it at the deadline" score differently under `floor`
vs `final`.

## Running

- **CLI**: `dimos evals run <dotted.suite> [--tags nav --blind --limit 5 --model gpt-4o]`
- **Python**: `EvalRunner(...).run(SUITE, tags=frozenset({"encoding"}))`
- **pytest**: suites are importable lists —
  `@pytest.mark.parametrize("case", SUITE)` and assert on `passed`
  (gate live-model tests with `skipif_no_openai`).
- **MCP**: the `EvalModule` skills `run_evals` / `list_eval_suites` return the
  summary + run dir, so a coding agent can run evals, grep transcripts, edit
  prompts/encodings, and run again.
- **Blind ablation**: `EvalRunner(blind=True)` withholds all observations. A
  case that still passes blind is guessable — fix its distractors. Run every
  new suite sighted and blind once before trusting it.
- **Preflight**: before anything runs, every case is checked against the rig —
  a missing stream fails with `"No stream 'lidar'. Available: [...]"`, a case
  needing MCP/sim fails with what's missing. Errors are per-case; one broken
  case never kills a run.
