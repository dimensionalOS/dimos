# Evals

## Terminology

- **Case**: a discrete scenario being tested. This specifies the prompt sent to the agent, the environment, and scoring functions for the final world state and agent response.
- **Environment**: this is either a dataset, a live simulation, or an image file being passed to the agent.
- **Suite**: a collection of cases.
- **Agent**: this is a wrapper that might contain an entire agent loop, a single request to an llm provider, and it may contain tools. Agents include our mcp client, pi, as well as a simple single-turn question/answer with no tools.

## Quick start (CLI)

```bash skip
# two documentation cases against the go2_short recording (needs OPENAI_API_KEY)
dimos evals run dimos.evals.suites.examples --agent dimos.evals.agents.question_answer

# same cases with observations withheld (the guessing ablation)
dimos evals run dimos.evals.suites.examples --agent dimos.evals.agents.blind

# same cases, the Pi coding agent over the recording as a file (needs pi on PATH)
dimos evals run dimos.evals.suites.examples --agent dimos.evals.agents.pi

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

## Baseline Bash versus dimcode + DimOS

For the complete-stack comparison, use `PiAdapter` with a sandbox against `DimcodeAdapter`
with the **same model**. The primary pair is Astra with Bash versus Astra in
dimcode with DimOS. This measures the combined product/robotics-stack effect;
it does not isolate the harness contribution.

```bash skip
dimos evals run dimos.evals.suites.examples --agent dimos.evals.agents.pi \
  --allow bash,grep --set sandbox=true \
  --set model=gpt-6-astra --set thinking=medium \
  --set max_steps=12 --set max_output_tokens=4096

dimos evals run dimos.evals.suites.examples --agent dimos.evals.agents.dimcode \
  --set model=gpt-6-astra --set thinking=medium \
  --set max_steps=12 --set max_output_tokens=4096
```

`PiAdapter` uses Pi's stock loop, provider support, tracing, limits and cleanup.
The allowlist exposes only Pi's Bash and grep tools. Linux bubblewrap isolates their filesystem,
processes, environment and network. The shell can read selected observations in
`/input` and write `/workspace`; host homes, DimOS source, virtual environments,
credentials, MCP, and host services are unavailable. The ordinary system tools
under `/usr` are read-only, with `/usr/local` hidden. Bash, grep, coreutils and
system Python remain available. Record the host system package versions when
freezing a pilot; this is not a portable pinned container image.

The baseline receives selected point coordinates/colors as CSV, camera frames
as lossless PNG and primitive observations as JSON, with timestamps and hashes.
Selected PNGs are also attached to its initial model message because the allowed
tools do not read images. No `agent_encode` summaries, labels, semantic tags or
original database are exported. Dimcode receives the same selected observations
through the DimOS store and retains its production tools. Representation and
image-delivery differences are part of this stack comparison and must be reported.

The sandbox supports Bash and grep (including either alone or neither), and rejects
other tools, skills, modules and MCP endpoints. Missing or
unsupported isolation fails preflight; there is no unrestricted fallback. This
adapter currently supports recordings only. Live tasks need a separately bounded
vendor SDK/robot connection available to the baseline, without DimOS. A blocked
robot interface is an unsupported case, not a baseline failure.

The primary benchmark uses only these two Pi-based adapters. Using the DimOS
CLI in the baseline violates the experiment's access policy.
Dimcode's current adapter still needs grader and unrelated-data isolation before
publication runs. Neither arm may receive task-specific solutions or hidden truth.

### Shared Pi runtime

Use the same model, reasoning setting, output cap, case selection and timeout
for each harness. Pi and dimcode use Pi's provider SDKs and built-in model
registry; model capabilities and prices are not redefined by DimOS. Pin the
same Pi version in both installations. The integration uses Pi 0.85.1 and
dimcode's local gateway protocol (`0.1.0-next.2` / `0.1.0-next.3`).
`DimcodeAdapter` extends `PiAdapter`, overriding gateway startup, session
configuration and event transport. Provider setup, budgets, tracing and
cleanup remain shared. No second model loop or SDK wrapper is introduced.

To compare another model, change both commands together: `model=gpt-5.6-sol`,
or `provider=anthropic` and `model=claude-fable-5-1`. Fable requires
`ANTHROPIC_API_KEY`, independently of OpenAI access.

`cli=` selects a specific Pi or dimcode executable. `OPENAI_BASE_URL` and
`ANTHROPIC_BASE_URL` optionally override the upstream endpoints. API keys stay
in process environment variables; the generated provider config contains
variable references. Request traces omit authentication headers.

Both adapters get fresh home/config/state directories and a copy of only the
selected recording observations. Pi retains its stock system prompt with
shared case guidance appended. Dimcode retains its production prompt, skills,
MCP integration and rendering tool; shared case guidance accompanies the user
instruction. Each case starts and stops its own dimcode gateway and session.
An existing personal gateway is never attached. Dimcode retains its production
skills; its tools can be restricted with the shared allowlist.

Freeze code/data, record runtime versions, balance execution order, repeat
each case and retain failed trials before interpreting the comparison.

### Tool selection

`allowed_tools` belongs to the shared `AgentConfig`. Use `--allow bash,grep`
on `dimos evals run` or `PiAdapter(allowed_tools=("bash", "grep"))` in Python.
Omit the option to keep the adapter's native defaults; `--allow ""` disables all
tools. Exact tool names are required. Unknown names, duplicates and conflicting
`--allow` / `--set allowed_tools` inputs are errors. The manifest retains the
requested allowlist, and raw requests retain the actual provider schemas.

Pi and dimcode share a Pi extension that selects active tools and blocks excluded
calls. Pi also passes the selection to its CLI. The dimcode adapter verifies that
the extension applied the selection before sending the first prompt. Native MCP
names are those advertised by dimcode, including its endpoint prefix and suffix.
The production MCP-client adapter does not yet implement filtering and rejects
explicit allowlists; single-call agents accept only an empty list or defaults.
Adapters must enforce selection or reject it, never silently ignore it.

Tool selection is independent of filesystem and network access. `--allow bash`
still permits any program the shell can reach. For the no-DimOS baseline, also
use `--set sandbox=true`; this currently supports only Pi with recorded inputs.
The sandbox is a composed isolation helper, not another agent subclass.

### Reading the metrics

- `model_turns`: assistant model responses represented in the trajectory.
  `steps` also includes the initial user instruction.
- `request_attempts`: recorded HTTP requests, including retries. A failed
  attempt can have no assistant response, so this can exceed `model_turns`.
- `tool_calls`: calls requested by those model responses.
- `agent_duration_s`: adapter execution, including recording export, provider
  setup and agent cleanup. `duration_s` additionally includes environment
  startup, settling, teardown and grading.
- Prompt tokens include cache reads and writes; completion tokens include
  reasoning when reported by the provider. `cached_tokens` means cache reads.
- Pi/dimcode `cost_usd` is Pi's registry-based estimate, not a billing receipt.
  Unknown costs remain null (or omitted in ATIF), never a fabricated zero.
  Totals cover recorded assistant responses; interrupted requests may have
  unreported usage. Keep raw attempts when auditing spend.

Errors stay in the summary denominator. An agent error preserves completed
steps, returns `ended_by=error`, and cannot pass. The runner saves a returned
trajectory before environment teardown so teardown failures retain evidence.
Raw requests preserve the actual tools/schemas used on each call; the ATIF
agent metadata includes the latest recorded definition for each tool name.

## Your first eval, end to end

Build a tiny SQLite recording using the same Store/Stream API as the robot's
Recorder (see `dimos/memory/intro.md` for the full Stream API):

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

A case is one Python literal. `Dataset(..., select=...)` takes a tuple of callables that
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
selected observation (by default, at most 8 per stream, spread evenly) in front of the
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
score=1.0 passed=True answer='about 19 meters' steps=2
n=1 mean=1.0 pass_rate=1.0 errors=0
```

That's the whole loop: environment -> agent -> trajectory + artifacts ->
grade -> run dir.

To run your cases through the CLI, put them in an importable module with
`SUITE: Suite = [case]`, importing `Suite` from `dimos.evals.types`.

## Agents

Agents live in `dimos/evals/agents/`, one per file. `--agent` takes the
module path; `--set field=value` sets an agent option, e.g.
`--set frames_per_stream=16` for `QuestionAnswer`. In Python, pass the same
options to the constructor: `QuestionAnswer(frames_per_stream=16)`.
CLI values are parsed as JSON when possible; quote lists as shown below.

**Tools.** The case's blueprint decides the tool set: `Sim(blueprint=...)`
is the robot stack plus `McpServer`, and its skill containers are the tools.
There is no tool filter. The agent's `modules` list is appended to the
launch command (`dimos run <blueprint> <modules>`), and `autoconnect` dedups
anything shared with the case, so `--set 'modules=["unitree-go2-agentic"]'` adds
the whole shipped agentic stack. On a `Dataset` case the agent's `modules`
are the whole launched stack (`dimos run <modules>`, no simulator or robot
underneath), torn down with the case. Configure those modules to read the
recording if their tools need it. `Dataset(mcp_url=...)` attaches an
already-running dimos instead. To
compare two tool sets on one task, run the suite twice with different
`--set modules=...`; each `trajectory.json` records the tools exposed.

**Limits.** The case's `timeout_s` sets the time budget for the agent and
subsequent motion settling. `McpClientAdapter` returns what it has when its
wait expires, marked `timeout`; `QuestionAnswer` and `Blind` rely on the
model provider's timeout. Environment startup has a separate
`launch_timeout_s`. Pi and dimcode bound model HTTP requests with `max_steps`
(including retries), and optionally cap output per request with `max_output_tokens`.
There is no aggregate token or dollar cap; usage is recorded when the agent supplies it.

**Observation encoding.** Each agent class hard-codes how the recording
reaches the model. `QuestionAnswer` calls `agent_encode()`; no other agent
does. Sending observations a different way means writing a new agent class.

## Scoring

Scores are floats in `[0, 1]`; `passed = score >= threshold`, the case's own
pass bar (`EvalCase.threshold`, default 1.0). Scorers are
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
        blueprint=["unitree-go2", "mcp-server", "unitree-skill-container"],
        simulator="dimsim",
        scene="apartment",
    ),
    grade=ended_near_bed,
    timeout_s=180.0,
)
```

```bash
dimos evals run dimos.evals.suites.dimsim_house --agent dimos.evals.agents.mcp_client_adapter --set 'modules=["unitree-go2-agentic"]'
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
