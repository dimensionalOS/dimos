## Context

`CodePolicyRuntimeSession.run()` currently launches Pi once against a persistent Jupyter kernel and returns Pi's final text. Pi can make many `python_exec` calls during that invocation, but the runtime neither identifies an authored program nor validates it after Pi finishes. The frozen integer QA evaluation consequently parses `ANSWER: <integer>` from prose.

The existing session already provides the useful substrate: an environment-specific kernel bootstrap, one trusted `python_exec` MCP tool, a persistent authoring namespace, fixed prompt assembly, Pi transcript capture, and evaluation-owned native scoring. The change should deepen that session rather than create another agent runtime.

## Goals / Non-Goals

**Goals:**

- Make direct-answer and policy-authoring behavior explicit on the existing session.
- Let an agent iteratively author a zero-argument `policy()` in the persistent REPL.
- Turn one successful, self-contained notebook cell into a reproducible source artifact.
- Validate the candidate in a clean kernel and return its JSON value to the evaluation.
- Give the agent bounded feedback for mechanical failures while keeping benchmark correctness hidden.
- Exercise the path end to end with frozen integer QA.

**Non-Goals:**

- Live DimOS control, simulation episodes, reset semantics, containers, SimConnection, LIBERO, or other third-party benchmark integration.
- Selecting a best candidate or feeding native benchmark scores back to the agent.
- Persisting or serializing arbitrary live Python function objects.
- A registry or class hierarchy for user-defined evaluation schemes.
- Policies generic across multiple QA cases; the first version authors one policy per evaluation case.

## Decisions

### Expose two operations on the existing session

Replace `run()` with `answer()` and `author_policy()`. Each session still accepts exactly one top-level operation. `answer()` retains the current one-Pi-invocation behavior and returns `AgentOutcome`. `author_policy()` owns the author/validate/repair loop and returns a distinct `PolicyOutcome`.

Explicit methods avoid a mode flag with a union return type and make evaluation code state which product it scores. A general scheme registry was rejected because only two concrete behaviors are required.

### Treat a notebook cell as the policy artifact

`CodePolicySession` will retain an in-memory record for every `python_exec` request, including source and execution status. After an authoring invocation ends, the runtime scans successful records from newest to oldest and selects the latest cell whose module-level AST contains a synchronous `def policy()` with no parameters.

The entire cell is the candidate so it may contain imports, constants, and helper functions alongside `policy()`. The authoring prompt requires all dependencies of the final policy to be in that cell. Capturing source avoids the fragility of `inspect.getsource()` and avoids attempting to serialize a live function or notebook namespace.

### Validate by clean replay

For each candidate, the runtime creates a temporary `CodePolicySession` with the same `CodePolicySessionConfig`, executes the candidate cell, invokes `policy()`, and serializes the return value as JSON. The temporary session is stopped after validation.

Clean replay proves that the source is self-contained and does not accidentally depend on variables from earlier authoring cells. JSON is the initial result boundary because it is deterministic, sufficient for frozen QA, easy to record, and does not introduce a policy-object abstraction. A candidate is mechanically valid only when source replay, invocation, and JSON serialization all succeed.

### Use independent Pi invocations for bounded repair

`author_policy()` accepts a positive `max_rounds`, initially set to three by frozen integer QA. All rounds connect to the same authoring MCP server and therefore the same persistent kernel. Each Pi invocation gets its own working directory and transcript artifact.

The first prompt contains the evaluation protocol and task input. If candidate capture or clean replay fails, the next prompt repeats that immutable context and includes the candidate source, the mechanical validation error, and an instruction to redefine `policy()`. This does not require Pi conversation-resume support; continuity comes from the persistent Python namespace plus explicit feedback.

The first mechanically valid candidate ends authoring. If no valid candidate exists after the round budget, `author_policy()` returns an invalid `PolicyOutcome` rather than raising an infrastructure exception. This lets the evaluation record an ordinary semantic failure.

### Keep native scoring outside the runtime

The runtime reports only candidate validity and execution result. It never receives an oracle, calls a benchmark scorer, selects a best result, or includes native correctness in a repair prompt.

Frozen integer QA will require the valid policy result to be a JSON integer and will compare it with the existing oracle through its native exact-match path. A missing, invalid, or non-integer result is scored as a normal benchmark failure. The evaluation does not reopen authoring after scoring.

### Record policy-specific evidence

The runtime will retain existing prompt and Pi evidence and add:

- one transcript per authoring round;
- the selected candidate cell as `policy.py`, when present;
- a JSON validation history containing round number, candidate presence, validation status, bounded error text, and duration;
- the final JSON policy result when validation succeeds.

Temporary Pi working directories and clean validation kernels remain implementation details and are removed when the session closes.

## Risks / Trade-offs

- **A successful but semantically wrong result cannot be repaired** → This is intentional: hidden native scores are evaluated once and never returned to authoring.
- **A self-contained-cell requirement may make policies longer** → State the rule in the authoring prompt and return a precise clean-replay error when earlier notebook state was required.
- **Independent Pi invocations do not retain conversational context** → Repeat the immutable task, candidate source, and validation error; preserve the authoring kernel across rounds.
- **Clean replay adds kernel startup latency** → Limit it to one validation per authoring round and keep the initial scope to frozen evaluation environments.
- **JSON excludes arbitrary Python results** → Accept the deliberate narrow boundary now; later live policies can use side effects and return JSON `null` without changing source capture.
- **Execution history contains agent-authored code** → Keep it in evaluation-owned artifacts under the existing trusted-code boundary and credential filtering.

## Migration Plan

1. Replace the session protocol's `run()` with `answer()` and `author_policy()` and add policy outcome models.
2. Rename current runtime behavior and all existing in-repository callers/test doubles to `answer()`.
3. Add execution recording, candidate selection, clean validation, repair rounds, and artifacts.
4. Move frozen integer QA to `author_policy(max_rounds=3)` and native scoring of the returned JSON integer.
5. Update agent evaluation documentation and run focused evaluation, QA, and CodePolicy tests.

The change is local and unreleased; rollback is a normal source revert. No compatibility alias or data migration is required.

## Open Questions

None for the frozen VQA implementation. Live policy side effects and episode lifecycle will be designed with the later simulation integration.
