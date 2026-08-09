## Why

The evaluation runtime currently scores only an agent's final text, so an agent that finishes with invalid or non-executable work cannot repair it after the runtime observes the failure. Evaluations also need a first-class way to evaluate authored code while retaining direct-answer evaluation for benchmarks whose native product is text.

## What Changes

- Add a policy-authoring evaluation scheme in which the agent defines a zero-argument `policy()` function in the existing persistent Python REPL.
- Capture the latest candidate policy source, replay it in a clean kernel with the same environment, and return its JSON-serializable result for native benchmark scoring.
- Allow a bounded number of authoring rounds when policy capture or clean execution reports a mechanical failure, without exposing hidden benchmark scores to the agent.
- Keep direct-answer and policy-authoring evaluations as two explicit operations on the existing CodePolicy session.
- Convert frozen integer QA to exercise policy authoring and score the policy return value instead of parsing the agent's final prose.
- Record policy source, validation attempts, transcripts, and execution evidence as evaluation artifacts.
- **BREAKING**: Replace the ambiguous session `run()` operation with explicit `answer()` and `author_policy()` operations, and update in-repository evaluations and test doubles accordingly.
- Defer live simulation, benchmark reset, SimConnection, container, and LIBERO integration to later changes.

## Capabilities

### New Capabilities

- `code-policy-authoring`: Explicit direct-answer and policy-authoring session behavior, candidate capture, clean validation, bounded mechanical repair, policy artifacts, and frozen QA consumption of policy results.

### Modified Capabilities

None.

## Impact

- Affects the evaluation session protocols and outcomes in `dimos/benchmark/evaluation/`.
- Affects Python execution and MCP session plumbing in `dimos/agents/code_policy_core.py` and `dimos/agents/code_policy_server.py`.
- Changes the built-in frozen integer QA evaluation from final-text parsing to policy-result scoring.
- Requires updates to evaluation runtime, runner, QA, and documentation tests.
- Adds no dependencies and does not change third-party benchmark lifecycle semantics.
