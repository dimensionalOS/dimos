## 1. Session Contracts

- [x] 1.1 Add protocol tests for mutually exclusive `answer()` and `author_policy()` operations, positive round limits, and valid/invalid policy outcomes.
- [x] 1.2 Replace `CodePolicySessionHandle.run()` with `answer()` and `author_policy()`, add the JSON policy outcome and validation record models, and rename all current in-repository callers and test doubles to `answer()`.

## 2. Candidate Capture

- [x] 2.1 Add unit tests covering successful and failed execution records, newest-candidate selection, module-level zero-argument `policy()` detection, helper-containing cells, and rejected async or parameterized policies.
- [x] 2.2 Record source and status for every `python_exec` call in `CodePolicySession` and implement AST-based selection of the latest successful conforming policy cell.

## 3. Clean Validation

- [x] 3.1 Add kernel-level tests for clean replay success, missing authoring-state dependencies, policy exceptions, timeout recovery, and non-JSON return values using frozen memory fixtures.
- [x] 3.2 Implement temporary clean-kernel replay with the same session configuration, zero-argument policy invocation, JSON result decoding, bounded errors, duration recording, and unconditional kernel cleanup.

## 4. Bounded Authoring Flow

- [x] 4.1 Add runtime tests for first-round success, mechanical feedback followed by repair, persistent authoring namespace, first-valid-candidate freezing, exhausted rounds as an invalid semantic outcome, and top-level session reuse rejection.
- [x] 4.2 Refactor the existing one-shot Pi launch into shared round execution used by `answer()` and `author_policy()`, with isolated per-round working directories and cumulative tool-call and duration accounting.
- [x] 4.3 Implement deterministic repair prompts containing the original protocol and task, candidate source when present, and mechanical validation error without any native score or oracle data.
- [x] 4.4 Record per-round transcripts, selected `policy.py`, validation history JSON, and successful policy result JSON while preserving existing prompt evidence and cleanup behavior.

## 5. Frozen Integer QA Integration

- [x] 5.1 Update frozen integer QA tests to cover valid integer results, semantically wrong integers without retry, invalid policy exhaustion, and valid non-integer results as native failures.
- [x] 5.2 Change frozen integer QA to call `author_policy(max_rounds=3)`, score only a valid JSON integer through the existing exact-match oracle, and report policy status and authoring rounds in its summary.
- [x] 5.3 Replace the frozen QA direct-answer prompt with the self-contained zero-argument `policy()` contract and remove obsolete final-text integer parsing from that evaluation path.

## 6. Documentation and Verification

- [x] 6.1 Update the agent evaluation documentation with the two explicit schemes, frozen VQA policy flow, mechanical-repair boundary, policy artifacts, and deferred simulator scope.
- [x] 6.2 Run focused CodePolicy core/server, evaluation runtime/runner, and frozen integer QA tests and resolve all failures.
- [x] 6.3 Run formatting, lint, and strict type checks for every changed Python file.
