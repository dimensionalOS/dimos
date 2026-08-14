---
title: "CodePolicy evaluation"
---

# CodePolicy evaluation

CodePolicy evaluations separate exploration from measured execution.

```text
Pi + Python REPL                           native Evaluation
      │                                           │
      ├── submit_policy(policy) ── fresh debug trial
      │                         ◀── outcome, logs, Memory2, artifacts
      │
      └── last submitted policy ── held-out trials without Pi
                                                  │
                                      privileged native scorer
```

The existing `dimos evals` command and its passive and interactive cases remain
unchanged. Complete third-party benchmarks use the additive `Evaluation` plugin
interface and the `dimos eval` command.

## Evaluation ownership

An `Evaluation` owns its native environment, cases, seeds, blueprint lifecycle,
step or real-time horizon, privileged scorer, aggregation, and native result.
External packages register an Evaluation through the `dimos.evaluations` entry
point group. The shared runner resolves the plugin, provides a fixed CodePolicy
runtime, and publishes one immutable result directory.

The policy sees every capability exposed by its running blueprint through
`Dimos`: modules, RPCs, skills, and streams. The scorer may use simulator-only
state, such as true poses and task predicates. Scorer state never enters the
policy process, trial logs, or Memory2 recording.

## Exploration

The fixed `code-policy-v1` profile runs Pi with `gpt-5.6-luna`, medium thinking,
and one tool: `python_exec`. The persistent Python REPL contains `Dimos` and
`submit_policy`.

```python
def policy(app: Dimos) -> None:
    target = app.peek_stream("detections")
    app.skills.move_to(target.position)


trial = submit_policy(policy)
trial.outcome
trial.read_logs(module="MotionPlanner", tail=100)
memory = trial.open_memory()
list(memory.streams)
list(trial.artifacts.iterdir())
```

The callable must have the exact synchronous signature shown above. Each
accepted submission starts a fresh debug environment and a fresh policy-only
blueprint. The Evaluation chooses the debug-case sequence. Pi may submit five
trials; its last accepted callable becomes the task-level policy artifact.

`TrialRun` describes a stopped run. It exposes debug success and reward, policy
errors, DimOS logs, a read-only Memory2 store, primitive traces, and recorded
artifacts. It does not expose simulator state or scorer internals.

## Held-out execution

The Evaluation reuses one task-level policy across all held-out cases. For each
case it starts a fresh environment and policy-only blueprint, waits for DimOS to
be ready, and invokes the serialized callable in a clean process:

```python
execution = context.runtime.execute(policy, timeout_s=case.timeout_s)
```

Pi and the production `McpClient` are absent from measured execution. The task
horizon starts immediately before `policy(app)` runs, so blueprint startup and
model generation do not consume real-time or simulator-step budgets.

## CLI

Install the agent dependencies and build the Pi extension:

```bash
uv sync --extra agents
npm --prefix packages/pi-code-policy-extension install
npm --prefix packages/pi-code-policy-extension run build
```

Run an installed Evaluation from a strict JSON specification:

```json
{
  "schema_version": "1.0",
  "evaluation": {
    "name": "vendor-evals.benchmark-name",
    "config": {}
  }
}
```

```bash
dimos eval run specification.json --output evaluation-run
```

The run specification has no agent or policy-mode fields. The runtime profile
is fixed and recorded in `evaluation-run/run.json`.
