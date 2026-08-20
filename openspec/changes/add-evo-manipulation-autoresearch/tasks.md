## 1. Validate the Evidence Boundary

- [x] 1.1 Build a fixture-based trace-quality test from representative successful, policy-error, planning-failure, and infrastructure-failure Debug Trials and inventory which timelines, statuses, and visual timestamps are recoverable from current logs and Memory2.
- [x] 1.2 Implement the smallest typed policy-capability event recording needed for evidence fields that the trace-quality test proves cannot be reconstructed, without recording privileged evaluator state. (The spike proved no additional event recording is needed.)

## 2. Implement the Deterministic Harness Model

- [x] 2.1 Add focused `PolicyCandidate`, `TrialEvidence`, evidence-summary, and evidence-category models with immutable IDs and bounded agent encodings.
- [x] 2.2 Implement deterministic Trial Evidence indexing for outcomes, event timelines, filtered module logs, policy output, artifact references, read-only Memory2, and available initial, terminal, or failure-adjacent frames.
- [x] 2.3 Add unit tests proving progressive disclosure, bounded summaries, unavailable-evidence behavior, non-privileged content, and access to complete raw artifacts.

## 3. Replace Submission Selection with Explicit Freezing

- [x] 3.1 Refactor the submission manager to preserve every accepted policy as an immutable candidate, enforce the existing submission budget, and remove `last_policy` selection.
- [x] 3.2 Add the authenticated freeze operation to the code-policy server and exploration kernel, accepting only a candidate from the current exploration.
- [x] 3.3 Make freezing irreversible, reject submissions after freezing, and resolve `ExplorationOutcome.policy` exclusively from the frozen candidate.
- [x] 3.4 Add lifecycle tests for multiple candidates, freezing an earlier candidate, missing freeze, invalid candidate IDs, repeated freeze, post-freeze submission, exhausted budget, and clean frozen-policy execution.

## 4. Minimize Manipulation Instructions

- [x] 4.1 Replace the LIBERO-PRO strategy-heavy evaluation protocol with the exact task input, callable contract, submission/evidence workflow, non-privileged capability discovery, and explicit freeze requirement.
- [x] 4.2 Update prompt-evidence recording and tests to prove manipulation recipes, skillbook content, privileged state, and secondary verifier or diagnoser instructions are absent.
- [x] 4.3 Add an end-to-end mocked exploration test in which the Agent submits candidates, drills into Trial Evidence, freezes one candidate, and produces the expected Policy Artifact.

## 5. Add the Evo-Compatible Research Panel

- [x] 5.1 Generalize `ResearchPanel` to any non-empty collection of unique case IDs with safe relative specifications, removing the exact-four-cases and hard-coded-family requirements.
- [x] 5.2 Resolve and hash the panel plus case specifications once per invocation, and record the panel hash in the normal panel result.
- [x] 5.3 Add an inline Evo result publisher that honors `EVO_RESULT_PATH`, `EVO_TRACES_DIR`, and `EVO_EXPERIMENT_ID`, atomically publishes the macro score and per-case task map, and requires no Evo Python dependency.
- [x] 5.4 Emit one diagnostic Evo trace per Evaluation Case with native outcome, stable failure category, runtime identity, panel hash, and references to DimOS policy, log, video, Memory2, and result artifacts.
- [x] 5.5 Separate valid native or policy failure from invalid measurement so preflight, infrastructure, missing-result, and malformed-result conditions exit non-zero without publishing a valid aggregate.
- [x] 5.6 Add tests for configurable panels, hash changes, four-case score publication, atomic duplicate-writer rejection, non-Evo invocation, per-case traces, valid zero scores, and invalid measurement exits.

## 6. Document and Verify the Bare-Bones Evo Pilot

- [x] 6.1 Document installation and standard `evo init`/optimize commands using the harness target, existing four-case development panel, inline instrumentation, recorded Evo version, and built-in `pareto_per_task` frontier.
- [x] 6.2 State explicitly that Evo is an unmodified external development tool and that the four binary case outcomes are integration evidence rather than a statistically robust benchmark result.
- [x] 6.3 Run the focused code-policy and LIBERO autoresearch unit tests, the Evo contract check with a fake local evaluator, and repository formatting/type checks for all touched files.
- [ ] 6.4 When credentials and simulator services are available, run one upstream Evo four-case baseline and verify the dashboard receives the combined score, per-case Pareto dimensions, diagnostic traces, experiment diff, and harness artifact references.
