## Context

The archived `generate-dimsim-benchmark-tasks` change now produces frozen public tasks and private contracts under `dimos.benchmark.dimsim`. DimOS therefore lacks the next vertical slice: select one generated public navigation task, give it to the intended Pi agent, let the full DimOS stack act in DimSim, let DimSim privately execute the generated contract, and retain enough evidence to understand the result.

Audit of the merged implementation found four execution-blocking contract gaps: stopped-state parameters are absent from `NavigateContract`, destination feasibility uses robot-center rather than robot-footprint distance, semantic-profile revision is not an explicit source field, and `load_full_release()` does not enforce its claimed completeness and one-to-one invariants. This change corrects those narrow generation seams before consuming a release. It does not reopen question selection, add scenes, or move evaluation into the generator.

Three existing implementations inform this change without being merged:

- `cc/feat/code-as-policy-interface` at `06a77a26c` provides a generic persistent Jupyter-backed `python_exec` skill, full `Dimos` and observation-memory handles, bounded transcripts, and timeout recovery, but its blueprint and recorder are manipulation-specific and its kernel lacks an evaluation-attempt reset contract.
- DimSim PR 3249 at `2f13260068f3264ef6382dd511a571c4ddb5f40a` demonstrates working closed-loop DimSim robot I/O, authoritative reset, correlated evaluation, private distance rubrics, abort, and cleanup, but it is based on older module architecture, has broad conflicts with current code, and drives an internal agent rather than Pi code policy.
- The spatial Pi baseline provides useful Pi SDK session and native-evidence patterns, but its adapter, hardened Podman topology, static-map tools, typed-answer submission, scheduler, and publication model are intentionally unsuitable for this local live-robot smoke.

The current `dimos.benchmark.runtime` package remains the low-level simulator/runtime-sidecar layer. This change introduces a separate agent-evaluation layer that can attach either to a native DimOS simulator blueprint or, later, to a runtime-sidecar-backed robot stack.

The initial deployment is deliberately trusted and local. A developer manually starts one DimSim evaluation blueprint, then runs one foreground evaluation command. There is no process or filesystem isolation and no fairness or anti-cheating claim.

## Goals / Non-Goals

**Goals:**

- Prove one real Pi-driven navigation episode against an already running DimSim + DimOS stack.
- Give Pi exactly one model-facing tool, `python_exec`, while allowing generated Python to use all normally deployed DimOS skills and RPCs through `app`.
- Preserve a Python namespace and agent-visible observation memory within an attempt and clear policy state between attempts.
- Keep simulator lifecycle and native scoring behind a narrow backend interface.
- Preserve the public/private task boundary and make native DimSim state authoritative for reset and success.
- Separate valid evaluation completion from whether the agent passed the task.
- Produce a minimal private attempt directory sufficient to inspect prompt, session, code, tool activity, native result, lifecycle, and terminal outcome.
- Implement current-compatible behavior rather than merging either historical reference branch.

**Non-Goals:**

- Launching or stopping DimSim or DimOS from the runner.
- Sandboxing agent-written Python or proving oracle secrecy against an adversarial local process.
- Retrying, resuming, scheduling, parallelizing, sharding, or aggregating attempts.
- Publishing scores, maintaining a ledger, producing leaderboard-comparable results, or building a review UI.
- Selecting the permanent simulator or supporting a second simulator in this change.
- Expanding beyond one generated destination task, one apartment profile, one deterministic episode binding, and one Pi condition.
- Replacing the existing low-level benchmark runtime protocol or hardened spatial Pi evaluator.
- Merging, rebasing, or cherry-picking PR 3249 or `cc/feat/code-as-policy-interface`.

## Decisions

### 1. Separate generic agent evaluation from robot runtime

The new package lives under `dimos/benchmark/agent_eval/`. It owns task dispatch, Pi execution, episode lifecycle, artifacts, and normalized outcomes. A backend owns simulator readiness, reset, native evaluation, result waiting, cancellation, and runner-owned cleanup.

```text
public task ───────────────┐
                          ▼
                  LocalAgentEvalRunner
                  ├─ Pi session → python_exec
                  ├─ lifecycle/events/artifacts
                  └─ normalized outcome
                          │
                 AgentEvalBackend
                          │
                  DimSim native adapter
                  ├─ authoritative reset
                  ├─ private rubric
                  └─ correlated result

running DimOS blueprint
  sensors → perception → mapping/spatial memory → navigation → robot commands
                     ▲
              CodePolicyModule
              app + memory
```

The generic backend protocol is intentionally narrow:

```python
wait_ready(...)
reset(episode, attempt_id, operation_id) -> ResetReceipt
start_evaluation(episode, attempt_id, operation_id) -> EvaluationHandle
wait_result(handle, deadline) -> NativeResult
cancel(handle)
close()
```

Episode references and native results remain backend-owned. The generic layer normalizes only identity, infrastructure completion, task result, terminal reason, duration, metrics map, and artifact references.

Alternative considered: extend `dimos.benchmark.runtime.BenchmarkEpisodeConfig` into the agent evaluator. Rejected because that package describes motor/runtime compatibility and sidecar launch material; agent sessions, semantic tasks, prompts, and evidence are a different lifecycle.

### 2. Attach to manually started services

The first command verifies and attaches to an existing DimOS MCP endpoint and DimSim evaluation endpoint. It never assumes ownership of those processes. Normal completion, failure, and interruption cancel only the active Pi/evaluation work and leave the stack alive.

The initial CLI is a dedicated module entrypoint:

```text
uv run python -m dimos.benchmark.agent_eval run --config <path>
```

The resolved strict config includes the completed release root, selected task ID, output root, endpoints, Pi authentication binding, model/thinking configuration, infrastructure timeouts, 180-second episode timeout, and backend-specific options. It does not expose scheduler or publication configuration.

Alternative considered: add a top-level `dimos benchmark` command immediately. Deferred until the local runner contract survives a real episode.

### 3. Use a Pi-side one-tool façade, not another MCP server

The attached MCP server may expose all deployed skills. The Pi host adapter lists them only to validate and record the server state, selects the reference-compatible `python_exec` schema, registers exactly that tool with Pi, and refuses any other outgoing tool name.

A second MCP server would add deployment and readiness complexity but would not enforce capabilities against an unsandboxed Python kernel. The intended constraint is model-interface shape, not denial of robot capabilities: generated code receives the full `app.skills` and `app.get_module` interface.

Private reset and evaluation controls remain outside the DimOS agent module graph rather than being hidden by a nominal MCP allowlist.

### 4. Port the generic code-policy module behavior selectively

The generic module, focused tests, dependency declarations, and general documentation are copied from reference commit `06a77a26c` without its xArm recorder, xArm blueprint, manipulation prompts, or unrelated refactors.

The port preserves:

- lazy persistent Jupyter kernel;
- synchronous bounded `python_exec`;
- preloaded full `Dimos` client as `app`;
- preloaded recorder store as `memory`;
- single-cell execution lock;
- bounded stdout, stderr, final expression, and traceback;
- interrupt/recover/restart behavior;
- explicit warning that remote RPC work may outlive a timed-out cell.

It adds a non-skill `reset_session()` RPC. Reset destroys any live kernel and returns a fresh session identity. Each execution record is bound to that identity. Namespace persists across calls within an attempt and never across attempts.

The DimSim evaluation blueprint supplies a recorder configured for current agent-visible navigation streams. Exact stream names are selected from the current spatial blueprint and frozen in the resolved attempt manifest. No evaluator or oracle stream is connected to this recorder.

### 5. Port PR 3249 behavior, not its module structure

The DimSim implementation is rewritten against current interfaces using PR 3249 as a behavioral test oracle. Only these behaviors are brought forward:

- functional current sensor and odometry publication;
- motion commands affecting the authoritative simulated body;
- reset request/acknowledgement with actual pose and cleared motion;
- reset generation and operation correlation;
- initial-predicate rejection;
- native evaluation start/result correlation;
- private rubric monitoring;
- timeout, abort, stale-message handling, and cleanup.

The current perception, mapping, spatial-memory, navigation, manipulation, visualization, and blueprint APIs remain authoritative. The PR's internal agent driver, internal `McpClient`, custom agent-output path, duplicate task runner, duplicate spatial modules, and historical blueprint composition are not ported.

### 6. Compose an external-Pi DimSim evaluation blueprint

The manually launched blueprint composes the current working DimSim robot/spatial stack with:

- the selected navigation observation recorder;
- `CodePolicyModule`;
- `McpServer`;
- current normal skill containers.

It deliberately omits `McpClient`. External Pi is the only LLM agent. The complete server inventory is allowed to exist for normal development and for calls made by generated code; only Pi's registered inventory is restricted to `python_exec`.

### 7. Select one task from a completed generated release

The runner accepts a release root and task ID, loads it through the canonical DimSim bundle API, and selects exactly one joined `PublicTask`, `TaskContract`, and `ExpectedOutcome`. The smoke configuration must select a `destination` task with a `NavigateContract` and terminal navigation outcome. It rejects incomplete manifests, count mismatches, duplicate IDs, missing or extra joins, incompatible public/contract/outcome shapes, identity-payload mismatches, and unsupported source/profile/policy revisions.

The runner imports the merged generation models; it does not define a second public-task projection, task-contract schema, checked-in bathtub fixture, or copied scene oracle. Evaluation-specific records hold only the episode binding, lifecycle, and evidence. The exact selected public record is retained as `task.v1.json`; the private contract and its release/source digests are referenced in the private manifest.

The episode binding adds only execution context that is intentionally not part of semantic task identity: the 180-second attempt deadline and the deterministic reset pose selected by the contract's compatible scene profile. It may not replace or override the target, metric, threshold, stopped-state parameters, or provenance in the generated contract.

### 8. Require authoritative reset before agent dispatch

Preparation is:

```text
reserve attempt and lock
→ validate MCP and backend readiness
→ validate the one-tool schema
→ reset code-policy session
→ request authoritative simulator reset
→ validate actual pose and reset generation
→ export a fresh private oracle view and match scene, upstream, profile, reset, and content revisions to the selected contract
→ verify initial predicate false
→ start private native evaluation
→ create Pi session
→ start episode clock and dispatch public task
```

Readiness and reset use separate infrastructure timeouts and do not consume the episode budget. All reset and evaluation messages carry attempt and operation identities; stale messages cannot advance the active lifecycle.

### 9. Execute the generated stateful navigation predicate

The generated `NavigateContract` is extended to carry the versioned metric, threshold, linear-speed tolerance, angular-speed tolerance, and stationary dwell duration required by its controlled language. Destination distance is the minimum world-frame 2-D surface distance from the configured robot footprint to the target's outer footprint. Generation feasibility and runtime evaluation use this same predicate.

Passing through the region does not succeed. The dwell clock advances only with monotonic simulated time while every contract condition holds and resets otherwise. Ordinary collision physics remains authoritative, and the first smoke adds no separate collision-failure rule. Duration and readily available path length may be retained as metrics but do not affect pass/fail.

The evaluator never publishes target binding, distance, thresholds, dwell progress, or success state through agent-visible streams, prompt content, or `python_exec` results.

### 10. Keep one Pi session alive for the episode

The runner creates one fresh Pi SDK session with the configured robot-specific system prompt and exact public task. It uses the established credential handling and native session/prompt evidence patterns, but not the static spatial adapter's hard-coded tools or `submit_answer`.

If a Pi turn ends while robot motion is active, the evaluator waits. When motion is idle and the task remains incomplete, it continues the same session with exactly:

```text
Continue working on the task.
```

No evaluator feedback is added. Two consecutive turns without `python_exec` end as a task failure. A generous loop-safety ceiling prevents pathological zero-work continuations; the episode deadline remains the primary budget.

### 11. Treat simulator state as authoritative

The episode terminates on the first correlated native success, episode timeout, or unrecoverable infrastructure failure. Pi text never establishes success.

On success or timeout the runner first records the terminal state, then best-effort aborts Pi, interrupts any active code cell, cancels the native evaluator, and calls the current navigation/motion cancellation path. It records when an already-issued remote RPC could not be cancelled.

Timeout after a healthy episode is a completed task failure. Readiness, reset, incompatible schema, session setup, lost connection, corrupt/mismatched native result, or required artifact failure is an infrastructure failure.

### 12. Separate attempt completion from task success

`outcome.v1.json` records:

- `attempt_status`: `completed` or `failed`;
- `task_result`: `passed`, `failed`, or `not_evaluated`;
- terminal reason and failure stage;
- attempt, task, code-policy-session, Pi-session, backend-run, and operation identities;
- wall and monotonic durations;
- native result reference and digest;
- safe metric summary;
- required-artifact completeness.

Exit zero means a valid completed evaluation, whether the agent passed or failed. Nonzero means infrastructure or artifact failure.

### 13. Retain a minimal private attempt directory

Every attempt reserves a fresh opaque directory and never overwrites or resumes:

```text
<output-root>/<attempt-id>/
├── attempt-manifest.v1.json
├── task.v1.json
├── events.jsonl
├── pi-session/<session-id>.jsonl
├── pi-prompt/system.txt
├── pi-prompt/initial.txt
├── code-policy-calls.jsonl
├── dimsim-result.v1.json
└── outcome.v1.json
```

The manifest freezes resolved configuration, release and selected-record digests, source revisions including semantic-profile revision, trust mode, endpoints without credentials, tool and observation inventories with digests, task identity, and dependency versions. Events are append-only and use UTC for inspection plus monotonic time for ordering/durations. The terminal outcome is written atomically and is never replaced.

Valid partial evidence survives failure or interruption. A best-effort failure outcome identifies missing artifacts rather than fabricating them. No public/private publication, score ledger, comprehensive log capture, or review bundle exists in this phase.

### 14. Test generically, validate DimSim manually

Automated tests use fake Pi, code-policy, and backend components to cover lifecycle ordering, single-tool registration, reset correlation, initial-predicate rejection, continuation, terminal states, interruption, locking, partial evidence, and oracle non-propagation. Code-policy and DimSim protocol/rubric components receive focused unit tests.

The real end-to-end test is documented and manually invoked because it requires a running browser simulator, model credentials, and the full stack. Change completion requires one infrastructure-valid artifact bundle; agent task success is not required.

## Risks / Trade-offs

- **[Risk] PR 3249 behavior depends on obsolete APIs.** → Port small behaviors in testable layers and prove current sensor/control/reset/result flow before building the complete runner.
- **[Risk] Current DimSim support cannot drive the normal spatial stack reliably.** → Make closed-loop robot I/O the first implementation gate; stop rather than compensating with a parallel historical navigation stack.
- **[Risk] The generator's globally pinned upstream DimSim revision cannot support the required closed loop.** → Probe that exact revision first. If a newer upstream revision is required, update the explicit compatibility policy and regenerate the release; never evaluate a release against a silently different upstream.
- **[Risk] Generic `SceneClient` becomes a benchmark-policy container.** → Keep reset and scoring policy in the DimSim evaluation adapter and expose only the smallest typed simulator-control primitives from the generic client.
- **[Risk] A timed-out Python cell leaves robot RPC work running.** → Invoke current navigation cancellation during every terminal path and record incomplete cancellation as evidence.
- **[Risk] The observation recorder accidentally captures private evaluator state.** → Define an explicit agent-visible stream inventory, validate it before Pi starts, and test that evaluator fields do not reach memory or prompts.
- **[Risk] A generated release is structurally valid JSON but internally inconsistent.** → Harden the canonical full-release loader and reject the release before reset unless manifest, identities, discriminated record shapes, identity payload, provenance, and expected outcome agree.
- **[Risk] The live reset no longer matches the generation snapshot or semantic profile.** → Export a fresh private oracle view after reset and require its source revisions and digest to match the selected task contract before Pi starts.
- **[Risk] Full `app.get_module` access could reach privileged controls.** → Keep reset/evaluator controls outside the reachable DimOS module graph; document that local unsandboxed execution is trusted and not a security boundary.
- **[Risk] A stationary threshold is unstable under simulation jitter.** → Pin velocity tolerances and a simulated-time dwell policy, unit-test boundary behavior, and retain final metric evidence.
- **[Risk] One fixed episode overfits infrastructure to DimSim.** → Keep backend values opaque and the generic contract narrow; require a second backend only in a later change after this vertical slice exposes real needs.
- **[Risk] Copying code-policy code creates temporary divergence from its source branch.** → Record the reference commit, preserve focused tests, minimize changes, and reconcile only after the source work lands cleanly.
- **[Trade-off] Manual process startup weakens provenance.** → Record attached runtime identities and versions now; automatic process ownership is intentionally deferred.
- **[Trade-off] No isolation means oracle secrecy is cooperative.** → Make only a logical separation claim and do not publish benchmark-comparability results from this phase.

## Migration Plan

1. Port and test the generic code-policy module and add attempt-scoped reset without touching manipulation-specific branch changes.
2. Prove the current DimSim sensor, actuator, reset, and native-result path against the current spatial stack using focused tests.
3. Add the external-Pi evaluation blueprint and freeze its agent-visible observation inventory.
4. Add the generic runner, strict config, one-tool Pi adapter, lifecycle store, and artifact models using fake components first.
5. Harden canonical release validation, select the generated destination contract, and add the DimSim backend adapter.
6. Run the documented real local smoke and inspect the resulting artifact bundle.

Rollback removes the new evaluation package, blueprint, code-policy module port, minimal current-compatible DimSim evaluation bridge, and the narrow contract/provenance/validation corrections. Existing runtime sidecars, hardened spatial evaluation, generated question set, and normal robot blueprints remain unchanged.

## Open Questions

- Resolve the exact current spatial blueprint and stream names for camera, odometry, map/spatial memory, and navigation status during the first implementation gate.
- Resolve the current authoritative navigation/motion cancellation RPC and verify its behavior after a code-policy timeout.
- Choose and version the linear/angular stationary tolerances and simulated-time dwell duration in the generated navigation predicate policy; the start pose and robot footprint come from the compatible scene profile rather than smoke configuration.
- Determine the smallest current-compatible portion of the DimSim browser bridge that must change for correlated reset and evaluation, without restoring PR 3249's older module structure.
- Verify whether the generation-pinned upstream DimSim revision supports the required closed-loop primitives; if not, identify the smallest explicit revision-policy update and release regeneration needed before implementation proceeds.
