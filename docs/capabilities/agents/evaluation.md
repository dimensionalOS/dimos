---
title: "Agent evaluation runtimes"
---

# Agent evaluation runtimes

Evaluations select the runtime that matches the benchmark condition.

```text
Pi + Python REPL                           native Evaluation
      │                                           │
      ├── submit_policy(policy) ── fresh debug trial
      │                         ◀── immutable candidate + Trial Evidence
      │
      └── freeze_policy(candidate) ── held-out trials without Pi
                                                  │
                                      privileged native scorer
```

`code-policy-v1` uses the flow above. `live-agent-v1` keeps Pi and the
persistent Python workspace in the measured runtime loop:

```text
Pi + persistent Python ── observations and DimOS RPCs ── native Evaluation
          │                                                   │
          └──────── explicit terminal action ─────────────────┘
```

The existing `dimos evals` command and its passive and interactive cases remain
unchanged. Complete benchmarks use the `Evaluation` interface and the
`dimos eval` command.

## Evaluation ownership

An `Evaluation` owns its native environment, cases, seeds, blueprint lifecycle,
step or real-time horizon, privileged scorer, aggregation, and native result.
Evaluations are explicit in-repo built-ins. Each declares `code-policy-v1` or
`live-agent-v1`. The shared runner constructs that runtime and publishes one
immutable result directory.

The policy sees every capability exposed by its running blueprint through
`Dimos`: modules, RPCs, skills, and streams. The scorer may use simulator-only
state, such as true poses and task predicates. Scorer state never enters the
policy process, trial logs, or Memory2 recording.

## Exploration

The `code-policy-v1` profile runs the model and thinking level pinned by the run
specification with one tool: `python_exec`. The persistent Python REPL contains
`Dimos`, `submit_policy`, and `freeze_policy`.

```python
def policy(app: Dimos) -> None:
    target = app.peek_stream("detections")
    app.skills.move_to(target.position)


candidate = submit_policy(policy)
candidate.evidence.summary
candidate.evidence.logs(module="MotionPlanner", tail=100)
memory = candidate.evidence.open_memory()
list(memory.streams)
list(candidate.evidence.artifacts.iterdir())
freeze_policy(candidate)
```

The callable must have the exact synchronous signature shown above. Each
accepted submission starts a fresh debug environment and a fresh policy-only
blueprint. The Evaluation chooses the debug-case sequence. Pi may submit five
candidates and must explicitly freeze one. Freezing is irreversible, closes
submissions, and is the only way to produce the task-level policy artifact.

`TrialEvidence` gives a bounded summary before exposing the underlying stopped
`TrialRun`, filtered DimOS logs, selected frames, policy output, a read-only
Memory2 store, and raw artifacts on demand. Missing evidence remains unavailable;
the harness does not diagnose from incomplete facts or expose scorer internals.

## Held-out execution

The Evaluation reuses one task-level policy across all held-out cases. For each
case it starts a fresh environment and policy-only blueprint, waits for DimOS to
be ready, and loads the serialized callable in a clean process behind a start
gate:

```python
execution = context.runtime.prepare(
    policy,
    memory_path=recording_path,
    startup_timeout_s=30.0,
)
evaluator.start_trial()
execution.start()
terminal = evaluator.wait_for_terminal()
policy_result = execution.finish()
```

Pi and the production `McpClient` are absent from measured execution. The task
horizon starts immediately before `policy(app)` runs, so blueprint startup and
model generation do not consume real-time or simulator-step budgets.

## CLI

Start the rootless Podman service and verify the required GPU runtime:

```bash
systemctl --user enable --now podman.socket
nvidia-smi -L

uv run dimos eval check --workspace "$PWD/.container-eval/check"
```

Run an installed Evaluation from a strict JSON specification:

```json
{
  "schema_version": "2.0",
  "runtime": {
    "model": "gpt-5.6-luna",
    "thinking_level": "medium"
  },
  "evaluation": {
    "name": "vendor-evals.benchmark-name",
    "config": {}
  }
}
```

```bash
dimos eval run specification.json --output evaluation-run
```

`dimos eval run` always builds the current checkout into the locked evaluation
image and executes that immutable image ID. There is no host execution mode. The
specification directory is mounted read-only; task-manifest references must be
relative to it. Outputs are staged on a private host-backed mount and published
atomically after the container exits, including valid failed or cancelled run
records.

The in-repo LIBERO-PRO smoke case runs each debug submission and the scored
trial in a fresh sibling Podman container and a fresh DimOS blueprint. The same
outer evaluation image also runs VLN-CE; each Evaluation retains ownership of its
native simulator image and scoring.

```bash
dimos eval run \
  dimos/benchmark/libero_pro/cases/goal-task-0-single-trial/evaluation.json \
  --output /tmp/dimos-libero-pro-smoke \
  --json --quiet
```

A completed trial exits successfully whether its native score is `0.0` or
`1.0`. Infrastructure and policy execution failures remain failed runs.

Every debug submission and the scored trial records the two public camera
streams side by side at 20 FPS. Debug videos live beside their corresponding
trial diagnostics; the scored video is also listed in the evaluation report:

```text
runtime/exploration-0001/submission-0001/trial/trial.mp4
scored-trial/trial.mp4
```

The left half is `agentview` and the right half is `robot0_eye_in_hand`. The
video contains the post-settling initial observation followed by one frame for
each simulator policy tick.

The run specification pins the model condition but cannot switch runtime
profiles. The Evaluation owns that choice, and `evaluation-run/run.json`
records both the requested condition and resolved profile.

## Live-agent execution

The `live-agent-v1` profile prepares one Pi session and one persistent Python
workspace before the evaluator's start gate. The workspace provides `app` for
ordinary DimOS RPCs and `app.memory` for read-only public observations. Pi can make
repeated `python_exec` calls while the native environment advances. The native
terminal condition or evaluator timeout stops Pi and closes the workspace.

VLN-CE R2R uses this profile because navigation requires new decisions as
observations arrive. Its checked-in case shares a 600-second wall-clock horizon
between Habitat and Pi and ends only when the agent calls
`app.VlnceConnection.submit_route()` or the horizon expires. See
`dimos/benchmark/vlnce_r2r/README.md` for its condition and evidence contract.
