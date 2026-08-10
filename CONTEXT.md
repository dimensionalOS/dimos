# Evaluation

The language used to describe how DimOS measures task-performing behavior across recorded, simulated, and live environments.

## Language

**Evaluation**:
A complete benchmark integration that owns its environment lifecycle, cases, scoring, aggregation, and native result semantics. Third-party evaluations retain their original harness rather than translating it into DimOS scoring primitives.
_Avoid_: Runtime, universal scorer

**Evaluation Case**:
A fixed task, environment, budget, and scoring definition. It does not select the behavior being evaluated.
_Avoid_: Policy configuration, run

**Evaluation Run**:
One attempt of an evaluation case by a selected policy. Different policies can attempt the same case without changing its definition.
_Avoid_: Case, suite

**Runtime**:
The injected DimOS facility that attaches to a system launched by an evaluation and executes the selected policy. It does not own the benchmark environment or scoring.
_Avoid_: Evaluation, benchmark harness

**Policy**:
The callable produced during exploration and replayed without an agent during evaluation. Its canonical signature is `policy(app: Dimos) -> None`.
_Avoid_: Agent, exploration transcript, execution mode

**Policy Artifact**:
The serialized callable and human-readable source captured when `submit_policy(policy)` is called. One artifact is produced per benchmark task and reused across its held-out evaluation cases or seeds. REPL outputs and the agent transcript are separate exploration evidence.
_Avoid_: Exploration transcript, policy source

**Exploration Stage**:
The unscored stage in which an agent uses a persistent Python REPL and calls `submit_policy(policy)` to run complete debug trials in fresh environments and blueprints. Model latency does not consume the evaluation horizon.
_Avoid_: Evaluation rollout, scoring

**Evaluation Stage**:
The measured stage in which the Policy Artifact executes without an agent against a reset or held-out environment. The native benchmark owns its real-time or step horizon and privileged scoring.
_Avoid_: Agent session, policy generation

**Policy Environment**:
The capabilities exposed by the fresh policy-only DimOS blueprint while a policy runs. Simulated, live, and replay-backed evaluations all pass the policy a connected `Dimos` application; completed trials additionally expose their Memory2 recording read-only through `TrialRun`.
_Avoid_: Agent tools, scorer context

**Evaluation Oracle**:
The evaluator-only source of truth used to score a policy attempt. In simulation it contains privileged state, such as true poses and object identities, that the Policy Environment cannot access.
_Avoid_: Runtime memory, perception output

**Agent**:
The model-backed participant in the Exploration Stage that issues code through the runtime's Python REPL and produces a Policy Artifact. It is absent from the Evaluation Stage and does not receive the evaluation oracle.
_Avoid_: Evaluator, runner
