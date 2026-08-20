## Why

DimOS can run isolated code-policy trials and aggregate a small LIBERO-PRO panel, but the exploration agent receives a large behavioral recipe, raw trial artifacts, and a last-submission-wins policy lifecycle. We need a measurable, task-independent manipulation harness that aligns the agent through deterministic affordances and evidence, and we need to expose that harness to unmodified Evo so its built-in branching autoresearch can improve it across manipulation tasks.

## What Changes

- Add a deterministic Manipulation Agent Harness that manages immutable policy candidates, compact Trial Evidence, on-demand artifact drilldown, submission budget, and explicit final-policy freezing.
- Replace the long manipulation coaching protocol with a minimal task instruction and harness contract; keep perception, grasping, planning, robot control, benchmark cases, and native scoring fixed.
- **BREAKING**: replace last-accepted-submission selection with an explicit `freeze_policy(candidate)` requirement; exploration without a frozen candidate is invalid.
- Adapt the existing configurable LIBERO-PRO research panel to emit Evo's standard numeric score, per-case task scores, and per-task traces while preserving native DimOS artifacts.
- Use unmodified upstream Evo through its public CLI/plugin and inline benchmark contract, including its built-in GEPA-inspired `pareto_per_task` frontier; do not fork, patch, vendor, or reproduce Evo orchestration in DimOS.
- Start with the current four-case panel as an integration pilot. Treat its four binary results as pilot evidence rather than a statistically robust benchmark claim.

## Capabilities

### New Capabilities

- `manipulation-agent-harness`: Deterministic code-policy exploration, Trial Evidence navigation, immutable policy candidates, and explicit policy freezing.
- `evo-manipulation-autoresearch`: Evo-compatible manipulation panel measurement, task-level traces, configurable frozen panel composition, and unmodified Evo orchestration.

### Modified Capabilities

None.

## Impact

- Affects the code-policy runtime and submission API in `dimos/benchmark/evaluation/`, the exploration kernel in `dimos/agents/code_policy_core.py`, and LIBERO-PRO evaluation prompts and tests.
- Extends the existing `dimos/benchmark/libero_pro/autoresearch.py` panel runner and its checked-in cases with Evo-compatible output.
- Adds a focused harness research target and documentation for initializing an Evo workspace; Evo remains an external development tool rather than a DimOS runtime dependency.
- Existing callers that rely on the final submitted policy being selected implicitly must use explicit freezing.
