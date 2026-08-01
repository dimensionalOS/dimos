## Why

DimOS can generate objective DimSim benchmark tasks, but it cannot yet execute one through the intended Pi code-as-policy agent path and retain a trustworthy, inspectable episode result. A single local navigation smoke is needed to prove the end-to-end evaluation boundary before investing in isolation, scheduling, more scenes, or a long-term simulator.

## What Changes

- Add trusted simulation-only code-policy execution, based on the isolated behavior of `cc/feat/code-as-policy-interface`, with one `python_exec` skill, persistent state within an attempt, full deployed DimOS skill/RPC access, agent-visible recorded observations, and explicit session reset between attempts.
- Add a local, simulator-neutral agent-evaluation runner that attaches to an already running stack, creates a fresh Pi session exposing only `python_exec`, coordinates one episode, and separates infrastructure completion from task success.
- Harden the merged DimSim release contract at the evaluation boundary: encode stopped-state semantics, preserve semantic-profile provenance, validate releases strictly, and make generation-time reachability use the same robot-footprint distance predicate as runtime scoring.
- Add a DimSim navigation evaluation backend by porting only the current-compatible sensor/control, authoritative reset, correlated result, contract-driven rubric, abort, and cleanup behavior demonstrated by PR 3249.
- Select the destination task from one completed generated release and run its exact public instruction: “Go to the bathtub and stop within 1 meter of its outer edge.”
- Retain the exact task and configuration, native Pi session and prompts, code-policy calls, lifecycle events, native DimSim result, and normalized terminal outcome in a fresh attempt directory.
- Add fake-backend and focused component tests plus one documented manual end-to-end smoke.
- Explicitly defer process isolation, automatic stack launch, retries, parallelism, scheduling, publication, aggregate reporting, scene expansion, and simulator selection.

## Capabilities

### New Capabilities

- `agent-code-policy-execution`: Trusted persistent Python policy execution over deployed DimOS skills, RPCs, and agent-visible recorded observations, with per-attempt session isolation.
- `local-agent-evaluation-runner`: Single-attempt local Pi execution, simulator-backend lifecycle coordination, evidence retention, and normalized infrastructure/task outcomes.
- `dimsim-navigation-evaluation-backend`: Current-compatible DimSim robot I/O, authoritative reset, private navigation rubric evaluation, correlated results, and the canonical bathtub smoke episode.

### Modified Capabilities

- `dimsim-benchmark-task-generation`: Complete the generated navigation contract and enforce release validation needed for trustworthy execution.
- `dimsim-scene-oracle-view`: Carry the semantic-profile revision explicitly so an evaluator can verify the live scene/profile binding.

## Impact

- Adds a generic agent-evaluation package under `dimos/benchmark/agent_eval/` while keeping the existing low-level `dimos.benchmark.runtime` and hardened spatial Pi scheduler contracts unchanged.
- Updates the merged DimSim generation models, geometry gate, and full-release loader only where required to make the generated destination task executable without evaluator-owned guesswork.
- Adds the generic code-policy module and focused dependencies from the reference branch without merging its manipulation-specific blueprint changes.
- Updates the current DimSim integration and its vendored/runtime evaluation bridge only where required for one closed-loop episode; PR 3249 remains a behavioral reference and is not merged or cherry-picked.
- Adds a DimSim evaluation blueprint composed from current perception, mapping, spatial-memory, and navigation modules, an observation recorder, `CodePolicyModule`, and `McpServer`, without an internal `McpClient`.
- Introduces a private local attempt-artifact layout and a dedicated module CLI; it makes no security, fairness, leaderboard, or cross-simulator comparability claim.
