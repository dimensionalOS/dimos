# Technical Benchmark Results: r31 Forbidden-Only Optimized

## Abstract

This report documents the completed `spatial-real-pilot-full-r31-forbidden-optimized` Pi benchmark using manual inspection of public records only. The run covered 1,170 independent tasks with one `visualization-forbidden` condition and 10 workers. 1,160 tasks completed and 10 terminated with safe execution failures, for a completion rate of 99.15%. No public score, ledger, model answer, or oracle answer is available in the permitted record set; therefore this report makes no claim about task accuracy or correctness. It reports execution completion, public-input/evidence coverage, and infrastructure reliability only.

## Experiment definition and integrity

| Field | Value |
|---|---|
| Experiment | `spatial-real-pilot-full-r31-forbidden-optimized` |
| Executor | Pi |
| Jobs / workers | 1,170 / 10 |
| Condition | `visualization-forbidden` only |
| Observation | `reconciled` |
| Corpus fingerprint | `3963e9e6a08ad664b084b683b2f1762fa0df4e951198a3c1774f93f61fec3264` |
| Plan digest | `3ea885e06447b705f8ecceb523bbbc1016b50d41b8f826ea6cfee3968403d927` |
| Selected-inputs digest | `abff4a5746b3dd405192b22537d2f68c97e89cc22508b34f2a81e5696afa094c` |
| Executor / snapshot fingerprint | `e85c0c4db8667b193556d0b78b5c0d3479558b98f14397d4d7eb92530c1eae34` |
| Model fingerprint | `b4a06019e8181bfcdbae4b70daa027f7e4d5ef16097d33380d90ece789b87d6e` |
| Prompt fingerprint | `78ae2fe2819f7737e531bee7f6439c04c6a3570ab2c602e16b07c3ff86635b19` |
| Tools fingerprint | `22fedba12e7b451f64d399ab4b71a6a560a557ec00f53dcb350436a323b659cb` |
| Runner-image fingerprint | `71b9392ba502522d1ece0a117602800dd8c0d1af75d2cbed0d36ddca78a907ca` |
| Limits fingerprint | `16678c654ada19d70dde2193276c8db8c0475001bb68da82248b7ee1b4fc712d` |
| Worker fingerprint | `1f926c4bbc64a4e6d1ab73225f8849c9910c0eae1a53c705e46e6ddee12123b6` |

The experiment manifest and plan report 1,170 planned cases and 1,170 jobs. Public runtime case, staging, provenance, inventory, and evidence-manifest records were present for all 1,170 jobs. The public runtime log was also inspected for operational signals.

## Coverage and task inventory

| Dimension | Publicly observed inventory |
|---|---:|
| Scenes | 30 |
| Trajectories | 30 |
| Questions | 390 |
| Variants | 3 (`clean`, `noisy-01`, `noisy-02`) |
| Answer types | 2 (`boolean`, `integer`) |
| Predicates / contracts | 7 |
| Query geometries | 5 |
| Splits | 2 (`development`, `held-out`) |

The case manifests expose the following predicate/contract families: `direct-neighbor-count`, `direct-room-connection`, `eligible-room-count`, `in-place-rotation`, `pose-occupancy`, `same-room`, and `straight-translation`. Public case manifests expose contract and geometry fields; no private answer material was used.

## Benchmark task catalog and public examples

The public case manifests expose seven task families. The table lists the public answer domain and contract parameters; parameter values are intentionally not reproduced as answers.

| Task family | Answer domain | Public contract parameters |
|---|---|---|
| `same-room` | boolean | `first_marker_id`, `second_marker_id`, `kind` |
| `direct-room-connection` | boolean | `first_marker_id`, `second_marker_id`, `kind` |
| `direct-neighbor-count` | integer | `marker_id`, `kind` |
| `eligible-room-count` | integer | `kind` |
| `pose-occupancy` | boolean | `footprint_policy_version`, `kind` |
| `straight-translation` | boolean | `distance_m`, `footprint_policy_version`, `kind` |
| `in-place-rotation` | boolean | `footprint_policy_version`, `yaw_delta_rad`, `kind` |

### Question-only public examples

The following are representative public question texts. **Answers are omitted:** oracle answers and agent/model answers are unavailable in the permitted public record set.

1. “Are the two markers in the same room? (variant 1)” — answer omitted.
2. “Do the rooms containing the two markers share a direct opening? (variant 2)” — answer omitted.
3. “How many rooms are directly connected to the marked room? (variant 1)” — answer omitted.
4. “How many eligible rooms are on this floor?” — answer omitted.
5. “Can the robot occupy the marked pose? (variant 2)” — answer omitted.
6. “Can the robot drive straight by the stated distance? (variant 2)” — answer omitted.
7. “Can the robot rotate in place by the stated angle? (variant 1)” — answer omitted.

![Public observed map sample; identifiers and answers omitted.](assets/r31-public-observed-map.png)

*Public observed map sample; identifiers and answers omitted.*

### Agent-response availability

Agent transcripts and model answers are private and were not inspected. No accuracy or correctness is claimed absent a sanctioned public score export; the reported rates remain completion/reliability coverage only.

## Aggregate execution outcomes

| Outcome | Count | Rate |
|---|---:|---:|
| Succeeded | 1,160 | 99.15% |
| Failed | 10 | 0.85% |
| Pending | 0 | 0.00% |
| Running | 0 | 0.00% |
| Interrupted | 0 | 0.00% |
| Cancelled | 0 | 0.00% |
| Total | 1,170 | 100.00% |

### Public task-dimension breakdowns

Rates are success rates within each safely observable dimension bucket. For high-cardinality dimensions, identical-count profiles are grouped rather than printing hundreds of opaque identifiers.

#### Variant

| Variant | Succeeded | Failed | Total | Success rate |
|---|---:|---:|---:|---:|
| clean | 385 | 5 | 390 | 98.72% |
| noisy-01 | 387 | 3 | 390 | 99.23% |
| noisy-02 | 388 | 2 | 390 | 99.49% |

#### Answer type

| Answer type | Succeeded | Failed | Total | Success rate |
|---|---:|---:|---:|---:|
| boolean | 897 | 3 | 900 | 99.67% |
| integer | 263 | 7 | 270 | 97.41% |

#### Predicate and contract

| Predicate / contract | Succeeded | Failed | Total | Success rate |
|---|---:|---:|---:|---:|
| direct-neighbor-count | 175 | 5 | 180 | 97.22% |
| direct-room-connection | 179 | 1 | 180 | 99.44% |
| eligible-room-count | 88 | 2 | 90 | 97.78% |
| in-place-rotation | 179 | 1 | 180 | 99.44% |
| pose-occupancy | 180 | 0 | 180 | 100.00% |
| same-room | 179 | 1 | 180 | 99.44% |
| straight-translation | 180 | 0 | 180 | 100.00% |

#### Query geometry

| Geometry | Succeeded | Failed | Total | Success rate |
|---|---:|---:|---:|---:|
| in-place-rotation | 179 | 1 | 180 | 99.44% |
| markers | 533 | 7 | 540 | 98.70% |
| none | 88 | 2 | 90 | 97.78% |
| pose-occupancy | 180 | 0 | 180 | 100.00% |
| straight-translation | 180 | 0 | 180 | 100.00% |

#### Split

| Split | Succeeded | Failed | Total | Success rate |
|---|---:|---:|---:|---:|
| development | 386 | 4 | 390 | 98.97% |
| held-out | 774 | 6 | 780 | 99.23% |

#### Scene and trajectory profiles

There are 30 scenes and 30 trajectories. Twenty-one scene buckets and 21 trajectory buckets each had 39 succeeded / 0 failed. Eight scene buckets and eight trajectory buckets each had 38 succeeded / 1 failed. One scene bucket and one trajectory bucket each had 37 succeeded / 2 failed. This is a compact public-count profile; opaque IDs are intentionally omitted because they add no technical interpretation.

#### Question profiles

There are 390 question IDs. 381 had 3 succeeded / 0 failed; eight had 2 succeeded / 1 failed; one had 1 succeeded / 2 failed. Question IDs are opaque content identities, so the report preserves the complete count profile without reproducing an unnecessary identifier dump.

## Safe failure taxonomy

The following categories are the known r31 public-safe classification of the 10 failed jobs. Counts are mutually exclusive and sum to the final failed count.

| Safe category | Count | Reliability interpretation |
|---|---:|---|
| Adapter terminal: `max_turns` | 3 | Agent/execution termination; not a correctness judgment |
| Podman sandbox execution timeout | 2 | Infrastructure/runtime reliability |
| Workspace export `ValueError` | 3 | Infrastructure/public-export reliability |
| Unspecified adapter failure | 2 | Adapter/execution reliability |
| **Total** | **10** | |

All 10 failures were non-scoring execution failures. The public runtime log also contains stream-drainer `UnicodeDecodeError` thread exceptions. They are reported as an operational signal only and are not reclassified as additional task failures; the final outcome count remains 10. No raw error payload, model output, transcript, tool audit, or private content was used.

### Reliability versus task quality

The 99.15% figure is completion/reliability coverage, not answer accuracy. The failure taxonomy identifies where execution did not produce a completed public evidence record. It cannot establish whether any completed task answer was correct, incorrect, or semantically valid because sanctioned public scoring and answer exports are absent.

## Cross-run operational context

| Run | Public configuration | Public outcome |
|---|---|---|
| r27 forbidden canary | 1,170 jobs / 5 workers | 1,170 succeeded; 0 failed |
| r29 forbidden canary | 20 jobs / 10 workers | 20 succeeded; 0 failed |
| r30 forbidden canary | 20 jobs / 10 workers | 20 succeeded; 0 failed |
| r31 forbidden optimized | 1,170 jobs / 10 workers | 1,160 succeeded; 10 failed |

The r27/r29/r30 rows are operational comparison context from their public experiment records. Their differing fingerprints reflect separate immutable experiment definitions/material snapshots; they are not interchangeable accuracy baselines.

## Reproducibility and limitations

- Reproduce the immutable run identity from the manifest, plan digest, selected-inputs digest, executor fingerprint, and worker count recorded above.
- Public runtime case and evidence manifests establish public-input/evidence coverage and execution presence, not semantic answer quality.
- No public score/ledger, model answer, oracle answer, transcript, or tool-audit export was inspected or used.
- Duration was not derivable from the permitted manifest/case/evidence records and is not reported.
- Failure categories are intentionally bounded and redacted. They should not be expanded into raw diagnostics without an explicit authorization path.
- Scene, trajectory, question, variant, answer-type, predicate, contract, geometry, and split rates describe completion only.

## Future sanctioned task-quality reporting

A future quality report requires a separately sanctioned public score export containing, at minimum, job identity, public case identity, answer-type-aware correctness result, and an aggregate-safe score schema. That export must be generated at the public boundary without exposing OAuth material, oracle contents, model outputs, transcripts, tool audits, or private scoring evidence. Until then, report completion/reliability coverage only and do not label succeeded jobs as accurate.
