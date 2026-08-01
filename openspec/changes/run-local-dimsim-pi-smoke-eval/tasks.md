## 1. Current-API Integration Gates

- [x] 1.1 Map the code-policy reference commit `06a77a26c` to current dependencies and record the exact generic files and tests to port, excluding all xArm blueprint, recorder, prompt, and refactor changes
- [x] 1.2 Map PR 3249 head `2f13260068f3264ef6382dd511a571c4ddb5f40a` behaviors to current DimSim, robot, spatial, navigation, manipulation, visualization, and blueprint APIs without merging or cherry-picking it
- [x] 1.3 Run a focused current-stack probe to identify the authoritative camera, odometry, map/spatial-memory, navigation-status, motion-command, and motion-cancellation interfaces
- [x] 1.4 Audit the selected generated destination record and pin only evaluation-owned configuration; source scene, target binding, start pose, robot footprint, metric, threshold, stopped-state policy, and provenance MUST come from the generated contract and compatible profile
- [x] 1.5 Add a failing closed-loop integration gate that demonstrates current placeholder/no-op DimSim sensor or motion behavior before porting the fix
- [x] 1.6 Add failing generation-contract tests for explicit profile revision, stopped-state parameters, robot-footprint destination distance, and strict full-release validation before implementing their fixes
- [x] 1.7 Verify the generation-pinned upstream DimSim revision can supply the required closed-loop primitives; if not, update the explicit compatibility policy and regenerate the release rather than evaluating across an unrecorded revision change

## 2. Generic Code-Policy Execution

- [x] 2.1 Port the generic Jupyter-backed `CodePolicyModule` and focused tests from the reference commit while preserving bounded output, execution locking, timeout recovery, and trusted-runtime documentation
- [x] 2.2 Add only the required generic agent-runtime dependencies and update the lockfile without incorporating unrelated reference-branch dependency changes
- [x] 2.3 Add stable code-policy session identities and bind every `python_exec` execution record to the active identity
- [x] 2.4 Implement and test the non-skill `reset_session()` RPC that destroys the current kernel, clears namespace state, and returns a fresh session receipt
- [x] 2.5 Test persistence of imports, helpers, variables, and mutations within one session and absence of that state after reset
- [x] 2.6 Test successful, exception, busy, timeout-with-preserved-namespace, timeout-with-restart, and module-stop behavior
- [x] 2.7 Add structured code-policy execution records containing source, timestamps, monotonic duration, Jupyter identity, bounded output, transcript, and interrupt/restart state

## 3. Current-Compatible DimSim Closed Loop

- [x] 3.1 Implement current-compatible DimSim RGB, depth/point-cloud, and odometry publication needed by the selected spatial stack
- [x] 3.2 Implement current-compatible motion-command handling that updates the authoritative simulated robot and satisfy the closed-loop gate from task 1.5
- [x] 3.3 Add the attempt/operation-correlated reset request, authoritative-body reset, residual-motion clearing, pose/odometry publication, reset generation, and reset acknowledgement
- [x] 3.4 Add reset validation tests for stale identities, scene mismatch, pose mismatch, stale generation, residual motion, and timeout
- [x] 3.5 Implement the private contract-driven robot-footprint surface-distance plus velocity-dwell rubric using the generated navigation predicate and monotonic simulated time
- [x] 3.6 Test distance, footprint, velocity, dwell reset, pass-through, threshold boundary, and initially satisfied cases
- [x] 3.7 Implement correlated evaluation start, exactly-once native result, timeout/failure-stage reporting, stale-message rejection, idempotent abort, and cleanup
- [x] 3.8 Test successful, timed-out, malformed, stale, duplicate, cancelled, and interrupted native evaluation lifecycles

## 4. External-Pi DimSim Evaluation Blueprint

- [x] 4.1 Add a recorder for the exact current agent-visible camera, odometry, map/spatial-memory, and navigation-status streams identified in task 1.3
- [x] 4.2 Add tests proving the recorder excludes reset, private task contract, rubric, oracle, and native evaluator streams
- [x] 4.3 Compose a manually launchable current DimSim spatial evaluation blueprint with the recorder, `CodePolicyModule`, `McpServer`, and normal current skill containers
- [x] 4.4 Verify by blueprint inspection and tests that the composition contains no internal `McpClient`, historical agent driver, duplicate spatial stack, or obsolete manipulation/visualization modules
- [x] 4.5 Regenerate the blueprint registry through the repository-owned generation test if the new evaluation blueprint is registered for CLI discovery

## 5. Generic Evaluation Models and Attempt Store

- [x] 5.1 Reuse the canonical DimSim `PublicTask`, `TaskContract`, and `ExpectedOutcome` models and add separate strict models only for smoke configuration, resolved configuration, backend episode reference, reset receipt, evaluation handle, native-result reference, lifecycle event, attempt manifest, and normalized outcome
- [x] 5.2 Define the narrow simulator backend protocol for readiness, reset, evaluation start, result wait, cancel, and runner-owned cleanup without importing DimSim types into the generic package
- [x] 5.3 Implement exclusive attempt reservation with opaque identities, non-overwriting directories, and a local lock that prevents a second active attempt against the same target
- [x] 5.4 Implement append-only `events.jsonl` with UTC timestamps, monotonic ordering/durations, attempt/operation correlation, and safe payload validation
- [x] 5.5 Implement canonical JSON writing, relative artifact references and digests, partial-evidence retention, and non-overwriting atomic `outcome.v1.json`
- [x] 5.6 Test completed-pass, completed-fail, not-evaluated infrastructure failure, interrupted, missing-artifact, existing-attempt, concurrent-attempt, and outcome-write-failure states

## 6. One-Tool Pi Code-Policy Adapter

- [x] 6.1 Extract or reuse the existing Pi SDK session creation, authentication, model resolution, subscription, abort, disposal, native-session retention, and prompt-sidecar behavior without importing the static-map tool broker
- [x] 6.2 Implement MCP readiness and inventory inspection that locates and validates the reference-compatible `python_exec` schema and records the complete observed inventory
- [x] 6.3 Implement the host broker that registers exactly `python_exec` with Pi, forwards only that tool to the attached MCP server, and rejects every other tool name
- [x] 6.4 Retain each forwarded code-policy call and result in `code-policy-calls.jsonl` with the active Pi and code-policy session identities
- [x] 6.5 Test missing, malformed, changed, duplicate, and additional MCP tools plus successful one-tool forwarding and forbidden-tool rejection
- [x] 6.6 Test that credential contents never enter protocol frames, prompts, manifests, sessions, call records, or diagnostics while the safe authentication binding is recorded

## 7. Single-Episode Runner

- [x] 7.1 Implement the exact preparation order: validate/select the generated task, reserve attempt, verify endpoints, validate tool schema, reset code policy, reset DimSim, validate the reset against a fresh private oracle view and the initially false predicate, start native evaluation, create Pi, start the episode clock, and dispatch the public task
- [x] 7.2 Implement one-session turn execution that waits for active robot motion and issues only the neutral continuation after motion becomes idle
- [x] 7.3 Implement the two-consecutive-no-policy-call failure and a generous pathological-loop safety ceiling subordinate to the episode deadline
- [x] 7.4 Implement success, healthy episode timeout, Pi/code-policy recovery, unrecoverable infrastructure failure, and user-interruption terminal paths
- [x] 7.5 Implement terminal cleanup ordering that records the native terminal state before aborting Pi, interrupting code policy, cancelling evaluation, and invoking current navigation/motion cancellation
- [x] 7.6 Record when a timed-out or interrupted cell may have left remote RPC work active and verify motion cancellation behavior
- [x] 7.7 Implement exit-zero semantics for infrastructure-valid task pass/fail and nonzero semantics for infrastructure or required-artifact failure
- [x] 7.8 Test that stale reset/result messages, Pi final text, evaluator metrics, and task-private fields cannot incorrectly determine or leak task success

## 8. Generated Release, DimSim Backend, and CLI

- [x] 8.1 Implement the DimSim backend adapter over the current-compatible reset and native evaluation protocol, retaining the raw native result unchanged
- [x] 8.2 Extend the canonical oracle/task provenance with explicit semantic-profile revision and propagate it into stable digests and generated records
- [x] 8.3 Extend `NavigateContract` and its predicate-policy identity with linear/angular stopped tolerances and stationary dwell duration; make the generation feasibility gate use robot-footprint-to-target surface distance
- [x] 8.4 Harden `load_full_release()` to reject incomplete/count-mismatched manifests, duplicate IDs, non-bijective joins, incompatible public/contract/outcome shapes, identity-payload mismatches, digest mismatches, and unsupported revisions
- [x] 8.5 Add strict smoke-config loading for release root, task ID, output root, endpoints, Pi auth/model/thinking settings, infrastructure timeouts, episode timeout, and DimSim backend options
- [x] 8.6 Select exactly one generated destination triple, derive its reset pose from the compatible semantic profile, and reject any attempt to override contract-owned predicate fields
- [x] 8.7 After reset, acquire a fresh private oracle view and require its scene, upstream, profile, reset, and content revisions to match the selected task source before Pi dispatch
- [x] 8.8 Add `python -m dimos.benchmark.agent_eval run --config <path>` with concise human output and stable exit codes
- [x] 8.9 Add a fake backend and fake Pi/code-policy bindings so the complete runner lifecycle and artifact set can execute in fast automated tests
- [x] 8.10 Verify the finalized attempt contains the exact selected public task, release/contract source references and digests, manifest, events, native Pi session and prompts, code-policy calls, raw DimSim result, and normalized outcome with consistent identities and digests

## 9. Documentation and Real Smoke

- [x] 9.1 Document the trusted unsandboxed boundary, logical-only oracle separation, manually started service prerequisite, and absence of fairness or leaderboard claims
- [x] 9.2 Document how to generate or locate a completed release, select its destination task ID, launch the DimSim evaluation blueprint, verify readiness, run the bathtub command, inspect artifacts, and leave the attached stack running
- [x] 9.3 Run focused code-policy, DimSim protocol/rubric, one-tool adapter, fake-backend, runner lifecycle, oracle non-propagation, and artifact tests
- [x] 9.4 Run repository formatting, lint, type checks, blueprint-generation checks when applicable, and the relevant fast pytest subset
- [x] 9.5 Execute one real local Pi bathtub attempt and retain a complete infrastructure-valid artifact directory regardless of whether the task passes
- [x] 9.6 Review the real attempt for prompt/tool/oracle separation, correlation, cancellation, native result integrity, artifact completeness, and actionable implementation gaps before declaring the change complete
