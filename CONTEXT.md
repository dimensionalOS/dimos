# Evaluation

The language used to describe how DimOS measures task-performing behavior across recorded, simulated, and live environments.

## Language

**Evaluation**:
A complete benchmark integration that owns its environment lifecycle, cases, scoring, aggregation, and native result semantics. Third-party evaluations retain their original harness rather than translating it into DimOS scoring primitives.
_Avoid_: Runtime, universal scorer

**Comparable LIBERO-PRO Evaluation**:
An Evaluation that preserves LIBERO-PRO's tasks, observations, action semantics, initial states, native success, and reporting while declaring any policy-runtime deviations that affect direct score comparison.
_Avoid_: Official LIBERO-PRO evaluation, protocol-identical score

**Unified Real-Time Policy**:
A Policy Artifact that uses the same ordinary DimOS interface in simulation and on a real robot while the environment continues advancing independently of policy computation latency.
_Avoid_: Benchmark-stepped policy, simulator-specific policy

**Real-Time Horizon**:
The fixed number of simulator control ticks available to a Unified Real-Time Policy after startup and settling complete. Every tick consumes the horizon whether or not the policy produces a new command.
_Avoid_: Policy-call budget, wall-clock timeout

**LIBERO-PRO Trial**:
One scored policy episode against a selected LIBERO-PRO task and fresh initial state, producing the benchmark's native result and a side-by-side MP4 of the public camera observations. A trial validates the integration but is not by itself a comparable aggregate benchmark score.
_Avoid_: LIBERO-PRO benchmark result, aggregate score

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
The serialized callable and human-readable source captured when `submit_policy(policy)` is called. One artifact is produced per benchmark task and reused across fresh evaluation episodes or seeds of that task. REPL outputs and the agent transcript are separate exploration evidence.
_Avoid_: Exploration transcript, policy source

**Exploration Stage**:
The unscored training stage in which an agent receives the selected benchmark task's exact instruction, uses a persistent Python REPL, and calls `submit_policy(policy)` to run complete debug trials of that task in fresh environments and blueprints. It is part of the evaluated code-as-policy system, while model latency remains outside the evaluation horizon.
_Avoid_: Evaluation rollout, scoring

**Task-Aligned Exploration**:
An Exploration Stage whose debug trials and scored Evaluation Stage use the same benchmark suite, task identity, and task instruction. Episodes and initial states are fresh, but the policy is not asked to transfer to an undisclosed task.
_Avoid_: Hidden-task evaluation, unperturbed training suite

**Debug Trial**:
One unscored policy attempt created by `submit_policy()`, owning a fresh complete DimOS blueprint and a fresh episode of the selected benchmark task. It returns a stopped `TrialRun` snapshot containing the outcome, logs, Memory2 recording, and artifacts.
_Avoid_: Shared simulator reset, partial run, evaluation rollout

**Evaluation Stage**:
The measured stage in which the Policy Artifact executes without an agent in a fresh episode of the same suite and task disclosed during Task-Aligned Exploration. The task instruction is unchanged, the episode initial state is fresh, and the evaluation owns the declared real-time horizon while native privileged scoring remains benchmark-owned and unavailable to the policy.
_Avoid_: Agent session, policy generation

**Policy Environment**:
The capabilities exposed by the fresh policy-only DimOS blueprint while a policy runs. Simulated, live, and replay-backed evaluations all pass the policy a connected `Dimos` application; completed trials additionally expose their Memory2 recording read-only through `TrialRun`.
_Avoid_: Agent tools, scorer context

**Policy Interface**:
The non-privileged robot observations, state, commands, and operational health that a simulator or real robot connection contributes to the complete Policy Environment.
_Avoid_: Normal API, evaluator interface

**Evaluation Control Interface**:
The evaluator-only capabilities for environment configuration, reset, initial-state selection, clock control, native terminal state, and scoring. It is not part of the Policy Environment.
_Avoid_: Privileged API, robot interface

**Evaluation Oracle**:
The evaluator-only source of truth used to score a policy attempt. In simulation it contains privileged state, such as true poses and object identities, that the Policy Environment cannot access.
_Avoid_: Runtime memory, perception output

**Agent**:
The model-backed participant in the Exploration Stage that issues code through the runtime's Python REPL and produces a Policy Artifact. It is absent from the Evaluation Stage and does not receive the evaluation oracle.
_Avoid_: Evaluator, runner
