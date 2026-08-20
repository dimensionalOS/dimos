## Context

The code-policy runtime currently gives one Pi agent a persistent Python workspace, `submit_policy(policy)`, five fresh debug trials, and raw `TrialRun` handles. `_SubmissionManager.last_policy` silently selects the last accepted submission for evaluation. LIBERO-PRO compensates with a long protocol containing API facts and manipulation strategy, while the existing autoresearch panel runs four representative cases and writes a DimOS-specific aggregate.

The desired research subject is not a prompt or skillbook. It is the task-independent exploration harness that shapes how the agent submits candidates, receives execution evidence, and selects a final policy. Evo already provides experiment worktrees, branching lineage, shared traces, concurrency, gates, and a GEPA-inspired per-task Pareto frontier, so DimOS only needs a research target and a standard benchmark adapter.

## Goals / Non-Goals

**Goals:**

- Align manipulation-agent behavior through a deterministic executable harness rather than a growing behavioral prompt.
- Preserve every submitted policy as an immutable candidate with navigable evidence.
- Require intentional, auditable final-policy selection.
- Give the agent concise evidence first and deterministic access to complete raw artifacts on demand.
- Run the current four-case LIBERO-PRO panel through unmodified Evo and expose both combined and per-case native scores.
- Keep panel definitions data-driven and immutable within an Evo run.

**Non-Goals:**

- Adding verifier, diagnoser, or judge model calls.
- Evolving task instructions, perception, grasp generation, planning, robot control, benchmark assets, or native scoring.
- Forking, patching, vendoring, subclassing, or reproducing Evo.
- Claiming statistically robust benchmark improvement from the four-case integration pilot.
- Adding multi-seed task scoring in the first pilot; that is the next measurement layer after the integration is proven.

## Decisions

### Isolate the Manipulation Agent Harness as Evo's research target

The harness will live in a focused evaluation module, separate from LIBERO task manifests, the native evaluator, and robot primitives. Runtime and kernel code will transport harness operations but will not contain manipulation strategy. Evo will be initialized with this harness module as its target and will execute the normal DimOS panel command in each experiment worktree.

Alternatives rejected:

- An editable skillbook or prompt: improvements would be prompt coaching rather than reusable execution structure.
- Arbitrary repository optimization: the pilot would not isolate whether the agent harness improved.
- A DimOS-native search controller: it would duplicate Evo's experiment tree, scheduling, and observability.

### Replace last-submission-wins with immutable candidates and explicit freezing

`submit_policy()` will return a `PolicyCandidate` containing a stable candidate ID, its immutable policy source/serialization, Debug Trial, compact evidence summary, and drilldown handle. The harness will keep all candidates for the exploration. `freeze_policy(candidate)` will select exactly one candidate, close further submissions, and make that candidate the `ExplorationOutcome.policy`. Exploration that ends without a frozen candidate is invalid.

There is no compatibility fallback to the last submission and no automatic score-based selection. Sparse debug success cannot deterministically identify the most robust policy, and diagnostic submissions must not overwrite earlier candidates.

### Present Trial Evidence through progressive disclosure

The candidate returned to the Python workspace will expose a compact, bounded summary: candidate ID, native debug outcome, execution status, duration, remaining budget, initial and terminal visual evidence when available, and an index of evidence categories. On-demand methods will provide timelines, filtered logs, selected frames, policy output, read-only Memory2, and raw artifact paths.

Evidence selection will use timestamps, typed statuses, module identity, and artifact metadata only. Missing structured evidence is represented as unknown or unavailable; the harness does not infer physical causes. Full raw artifacts remain accessible.

Alternatives rejected:

- Returning every artifact automatically: it overwhelms the agent context and obscures salient failures.
- Model-written diagnoses: they introduce hidden behavioral prompts and another stochastic research subject.

### Keep task instructions minimal and keep capability contracts out of strategy prose

The runtime system instruction will state the callable contract, submission budget, evidence workflow, and explicit freeze requirement. LIBERO supplies the exact task instruction and non-privileged capability surface. The existing long manipulation recipes will be removed rather than moved to another prompt. Typed APIs, docstrings, harness summaries, and Trial Evidence provide the operational guidance.

### Use Evo's inline benchmark contract without an Evo library dependency

The panel command will honor `EVO_RESULT_PATH`, `EVO_TRACES_DIR`, and `EVO_EXPERIMENT_ID` directly. It will atomically publish one result with a numeric macro score and a task map keyed by Evaluation Case ID. It will write one trace per case containing the native outcome, failure category, runtime identity, panel hash, and paths to the DimOS artifacts needed for diagnosis.

The current four cases remain the pilot panel. Each case is one Evo task; individual debug or scored seeds are trace details, not frontier dimensions. Evo's standard `pareto_per_task` strategy can therefore retain branches that specialize on different manipulation cases while the top-level macro score ranks overall performance.

### Make panel composition configurable between runs and immutable within a run

`ResearchPanel` will require a non-empty set of unique case IDs and safe relative specification paths, without enforcing exactly four cases or one hard-coded family each. The runner will hash the resolved panel and case specifications and stamp the hash into the result and traces. One invocation resolves the panel once; an Evo run is initialized against that fixed panel path and hash.

Changing panel membership creates a new research baseline. Outcomes with different panel hashes are not comparable.

### Distinguish low policy score from invalid measurement

A completed policy attempt with native failure is a valid task score of zero. Policy exceptions are recorded as task failures when the Evaluation produces a valid native result. Preflight, infrastructure, missing-result, or malformed-result conditions make the Evo benchmark command fail non-zero rather than publishing a misleading aggregate.

## Risks / Trade-offs

- **Existing logs may not expose a reliable primitive timeline** → Begin implementation with a trace-quality spike over recorded failed trials. Add narrow typed event recording at the policy-capability boundary only where timestamps and statuses cannot be reconstructed.
- **The four-case pilot is noisy and coarse** → Label it as integration evidence, retain per-case traces, and add one-policy/multi-seed measurement only after Evo demonstrates useful search behavior.
- **Minimal instructions may initially regress performance** → Preserve API contracts and typed discovery, compare against the current baseline, and let failures identify missing harness affordances rather than restoring strategy prose.
- **Evo agents can explore changes outside the nominal target** → Use Evo's normal target, experiment diffs, gates, and review workflow; do not modify Evo to enforce a custom sandbox in the pilot.
- **Parallel LIBERO cases may contend for local ports, GPU services, or caches** → Let Evo size concurrency from the benchmark resource profile and start conservatively when the environment exposes singleton resources.
- **Evo version drift can change experiment semantics** → Record the installed Evo version and generated workspace configuration with each research run; verify the public contract when upgrading.

## Migration Plan

1. Add the deterministic candidate/evidence models and trace-quality tests without changing final selection.
2. Add the freeze endpoint and kernel helper, switch `ExplorationOutcome.policy` to the frozen candidate, and remove `last_policy` fallback in the same change.
3. Replace the LIBERO coaching protocol with the minimal harness/task contract and update prompt-evidence assertions.
4. Generalize the panel schema and add Evo inline output alongside the existing human-readable panel result.
5. Document and execute a four-case baseline using an unmodified Evo installation and the built-in per-task Pareto frontier.

Rollback is a normal source revert before adoption. There is no runtime data migration and no backward-compatible last-submission mode.

## Open Questions

- Do existing module logs and Memory2 timestamps provide enough typed events to locate failure-adjacent visual evidence, or is a small policy-capability event recorder required? Resolve with the first implementation spike before finalizing the evidence index.
