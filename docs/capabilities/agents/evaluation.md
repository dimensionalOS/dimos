---
title: "Agent evaluations"
---

DimOS runs complete **Evaluations**. An Evaluation owns its inputs, protocol,
scoring, aggregation, and native result semantics. The shared framework resolves
the Evaluation, supplies CodePolicy agent sessions, and records an immutable
Evaluation Run.

```text
Evaluation Run Specification
          |
          v
       Evaluation ------ dataset / cases / native harness
          |
          v
 CodePolicy Runtime ----- Pi + one persistent python_exec tool
          |
          v
   Evaluation Run ------- status / native result / artifacts
```

This is deliberately not a universal scorer. The built-in frozen integer QA
Evaluation uses OpenEvals internally. A third-party benchmark should instead call
its own harness and return that harness's native result without rescoring it.

## Setup

From a source checkout, install the Python runtime and build the Pi extension:

```bash
uv sync --extra agents
npm ci --prefix packages/pi-code-policy-extension
npm run build --prefix packages/pi-code-policy-extension
export OPENAI_API_KEY=...
```

The initial runtime profile is `code-policy-v1`: Pi 0.80.10, model
`gpt-5.6-luna`, medium thinking, and exactly one `python_exec` MCP tool. Pi is the
profile's driver, not a user-selectable evaluation runtime.

## Run specification and CLI

An Evaluation Run Specification binds an Evaluation configuration to the agent
configuration:

```json
{
  "schema_version": "1.0",
  "evaluation": {
    "name": "frozen-integer-qa",
    "config": {"case": "case.json"}
  },
  "agent": {
    "profile": "code-policy-v1",
    "model": "gpt-5.6-luna",
    "thinking_level": "medium"
  }
}
```

Evaluation-owned relative paths are resolved from the specification directory.
Run the included smoke specification with:

```bash
uv run dimos eval run \
  dimos/benchmark/short_horizon_qa/cases/demo_go2_hongkong_office-room-count-smoke/run.json \
  --output=/tmp/dimos-eval-smoke
```

The operational CLI settings stay thin:

| Option | Default | Purpose |
| --- | --- | --- |
| `--api-key-env` | `OPENAI_API_KEY` | Name of the environment variable containing the API key. |
| `--output` | required | Atomically publish the run to this directory. |
| `--json` | off | Print the complete Evaluation Run as JSON. |
| `--quiet` | off | Suppress live progress on stderr. |

The API key is passed only to Pi. It is not written to the specification, run
record, prompt evidence, subprocess arguments, or Python kernel environment.

## Results, artifacts, and exit status

`--output` must be absent or empty. DimOS builds the run in a temporary sibling
directory and publishes it atomically as:

```text
run.json
runtime/session-0001/
  runtime-system.txt
  evaluation-protocol.txt
  task-input.txt
  assembled-user-message.txt
  prompt-assembly.json
  pi-transcript.jsonl       # when Pi emits one
  stderr.log                # when nonempty
```

`run.json` contains only universal infrastructure status—`completed`, `failed`,
or `cancelled`—plus the Evaluation's summary, opaque native result or artifact
reference, and artifact metadata. A native score of `false` can still be a
successfully completed run.

| Exit | Meaning |
| --- | --- |
| `0` | The Evaluation completed, regardless of native semantic score. |
| `1` | Evaluation or agent infrastructure failed after execution started. |
| `2` | Specification, discovery, configuration, credential, or output preflight failed. |
| `130` | The user cancelled the Evaluation. |

## Implement an Evaluation

An Evaluation is the only public semantic extension point:

```python
class MyEvaluation:
    name = "my-evaluation"
    config_model = MyEvaluationConfig

    def run(self, config, context):
        with context.agent.open_session(environment) as session:
            outcome = session.run(
                evaluation_protocol="Return one answer per benchmark rules.",
                task_input=sample.question,
            )
        native_result = my_existing_harness.score(outcome.final_text)
        return EvaluationReport(
            summary=(...),
            native_result=InlineNativeResult(value=native_result),
        )
```

Built-ins are registered lazily inside DimOS. External distributions expose an
Evaluation object through the `dimos.evaluations` entry-point group:

```toml
[project.entry-points."dimos.evaluations"]
my-evaluation = "my_package.evaluation:my_evaluation"
```

An installed external Evaluation is addressed as
`<canonical-distribution-name>.my-evaluation`. Keep benchmark datasets, sample
loops, success checks, and aggregation in the Evaluation or native harness. Do
not translate them into a universal DimOS case or scorer.

## Prompt ownership

The versioned runtime profile owns system instructions, the `python_exec` tool
surface, Pi flags, and deterministic assembly. Every Evaluation supplies two
immutable strings: its Evaluation Protocol and its Task Input. Their owners and
SHA-256 hashes are recorded separately even though Pi receives them together in
one user message. There are no prompt-template settings.

## Trust boundary

CodePolicy executes agent-authored Python in a persistent Jupyter kernel. It is
trusted and **unsandboxed**. Frozen Memory2 SQLite connections are read-only, but
Python can still access other host files and processes. Run only trusted agents,
or place the entire evaluation command in an OS sandbox or container.
