# Pi Spatial Full-1170 Experiment Report

Date: 2026-07-24
Experiment: `spatial-api-key-full-1170-forbidden`
Status: complete with 10 executor failures

## Executive summary

The experiment tested a Pi coding agent on 1,170 spatial question-map
instances. The run covered 390 questions across three map variants: clean,
noisy-01, and noisy-02.

The scheduler completed 1,160 jobs and failed 10, for a 99.1% operational
completion rate. Of the 1,160 scored answers, 895 were correct:

- scored QA accuracy: 77.2%;
- conservative end-to-end accuracy: 76.5%, counting executor failures as
  unsuccessful; and
- category-balanced macro accuracy: 72.3%.

The main result is a capability split. The agent answered local metric questions
well, including pose occupancy, rotation clearance, straight translation, and
same-room membership. It performed poorly when it had to reconstruct rooms and
doorway topology from the point cloud.

The run supports this narrower conclusion:

> A coding agent can derive reliable local collision and motion judgments from
> the supplied point cloud, but its improvised room segmentation does not
> produce a dependable architectural topology.

The result does not establish that the model directly understands occupancy-map
images or has a stable internal map representation. The agent had a programming
environment and wrote case-specific analysis code.

## Experiment setup

### Model and execution

| Field | Value |
| --- | --- |
| Model | `gpt-5.6-luna` |
| Thinking level | `medium` |
| Authentication | OpenAI API key loaded from `.env` |
| Workers | 10 |
| Per-job timeout | 900 seconds |
| Maximum turns | 50 |
| Maximum tool calls | 50 |
| Prompt condition | `visualization-forbidden` |

The agent could call:

- `sandbox_exec`;
- `read_generated_image`, forbidden by the active prompt; and
- `submit_answer`.

Each successful job retained the Pi-native session, tool audit, prediction, and
private score. The agent had no answer-bearing oracle access.

The configured network policy allowed general outbound traffic with heuristic
auditing. The prompt prohibited online information, but this configuration
cannot prove that no online service was used. No generated-image reads occurred.

### Corpus composition

The plan contained 390 unique questions, each paired with three map variants:

| Variant | Planned cases |
| --- | ---: |
| Clean | 390 |
| Noisy-01 | 390 |
| Noisy-02 | 390 |
| **Total** | **1,170** |

Most task categories contained 60 questions and 180 cases. Eligible-room count
contained 30 questions and 90 cases:

| Task | Planned cases |
| --- | ---: |
| Same room | 180 |
| Pose occupancy | 180 |
| In-place rotation | 180 |
| Straight translation | 180 |
| Direct room connection | 180 |
| Direct neighbor count | 180 |
| Eligible room count | 90 |

Because the categories were not equally represented, the micro accuracy gives
less weight to eligible-room counting. The report therefore includes a
category-balanced macro score.

### Immutable identities

| Component | Fingerprint |
| --- | --- |
| Model | `a245c00ae8dadba1d438130465c4e02416d989163beb8b4778aaa9d362c96b8c` |
| Prompt | `8b9248c18457e9356d647e54a9088643c5725adec0c609508c82997c6b03076c` |
| Tools | `c247dda40f401b517b106d99b43b91f92932d40bc865cb644189c0a836001d5e` |
| Corpus | `08b564bb8dc631146e250a3b5b7c17730fd53fe801a4e10be4f84850dd523ea3` |
| Runner image | `186a39cab79e76f2fe8924d34403f279df39134565827f3193a3578d1c56a9ab` |
| Scorer | `2e139b2b0d044c99e11d57dd663f44e830ee9c8a29a0fdee3e3d8097abd4132e` |

## Operational results

| State | Jobs | Rate |
| --- | ---: | ---: |
| Succeeded | 1,160 | 99.1% |
| Failed | 10 | 0.9% |
| Interrupted | 0 | 0.0% |
| Cancelled | 0 | 0.0% |
| Pending | 0 | 0.0% |

The retained session timestamps span approximately 2 hours 55 minutes, from
2026-07-24 05:37 UTC to 08:32 UTC.

Session-reported API usage totaled approximately:

- cost: $88.00;
- output tokens: 4.24 million;
- reasoning tokens: 2.09 million; and
- cache-read tokens: 251.84 million.

The mean reported cost was $0.075 per attempted case.

## QA results

### Accuracy by task

| Task | Scored | Correct | Scored accuracy | Correct / planned |
| --- | ---: | ---: | ---: | ---: |
| Same room | 180 | 177 | 98.3% | 98.3% |
| Pose occupancy | 177 | 171 | 96.6% | 95.0% |
| In-place rotation | 180 | 171 | 95.0% | 95.0% |
| Straight translation | 179 | 161 | 89.9% | 89.4% |
| Direct room connection | 179 | 116 | 64.8% | 64.4% |
| Direct neighbor count | 179 | 89 | 49.7% | 49.4% |
| Eligible room count | 86 | 10 | 11.6% | 11.1% |
| **Overall** | **1,160** | **895** | **77.2%** | **76.5%** |

The approximate 95% Wilson interval for scored QA accuracy is 74.7% to 79.5%.
The category-balanced macro accuracy is 72.3%.

### Accuracy by answer type

| Answer type | Scored | Correct | Accuracy |
| --- | ---: | ---: | ---: |
| Boolean | 895 | 796 | 88.9% |
| Integer | 265 | 99 | 37.4% |

The difference is not merely a formatting effect. Boolean questions mostly ask
about local collision, motion, or membership. Integer questions require
explicit room enumeration or adjacency counting.

### Accuracy by map variant

| Variant | Scored | Correct | Accuracy |
| --- | ---: | ---: | ---: |
| Clean | 388 | 308 | 79.4% |
| Noisy-01 | 385 | 293 | 76.1% |
| Noisy-02 | 387 | 294 | 76.0% |

Noise reduced aggregate accuracy by about three percentage points. It had a
larger effect on noisy-02 rotation and translation:

- in-place rotation: 98.3% on clean and noisy-01, 88.3% on noisy-02;
- straight translation: 91.7% on clean, 91.5% on noisy-01, and 86.7% on
  noisy-02; and
- pose occupancy: approximately 96.6% on all three variants.

Noise was not the primary cause of room-topology failures.

### Development and held-out splits

| Split | Scored | Correct | Accuracy |
| --- | ---: | ---: | ---: |
| Development | 387 | 305 | 78.8% |
| Held-out | 773 | 590 | 76.3% |

The held-out result was 2.5 percentage points lower. The gap is smaller than the
difference between local geometry and topology categories.

## Cross-variant consistency

Of the 390 unique questions, 380 had all three variants scored:

| Outcome across three variants | Questions |
| --- | ---: |
| Correct on all three | 247 |
| Correct on exactly two | 57 |
| Correct on exactly one | 31 |
| Wrong on all three | 45 |

The agent returned the same value for all three variants in 261 of 380 complete
triplets.

Consistency depended strongly on task:

| Task | Complete triplets | All three correct | All three wrong | Mixed |
| --- | ---: | ---: | ---: | ---: |
| Same room | 60 | 57 | 0 | 3 |
| Pose occupancy | 57 | 52 | 0 | 5 |
| In-place rotation | 60 | 51 | 0 | 9 |
| Straight translation | 59 | 45 | 0 | 14 |
| Direct room connection | 59 | 26 | 9 | 24 |
| Direct neighbor count | 59 | 15 | 16 | 28 |
| Eligible room count | 26 | 1 | 20 | 5 |

Eligible-room failures were systematic: 20 of 26 complete triplets were wrong
on clean, noisy-01, and noisy-02. Only one eligible-room question was correct
on all three. This pattern points to a deficient room model rather than noise
alone.

## Error analysis

### Boolean answers defaulted toward `false`

The Boolean confusion counts were:

| Expected | Predicted | Cases |
| --- | --- | ---: |
| `false` | `false` | 432 |
| `true` | `true` | 364 |
| `true` | `false` | 84 |
| `false` | `true` | 15 |

False negatives outnumbered false positives by more than five to one.

Direct-room connection showed the strongest bias. Of 90 expected-true cases,
the agent answered `true` in only 37 and missed 53. Of 89 expected-false cases,
it correctly answered `false` in 79.

Session inspection shows a recurring interpretation: the agent approximates
wall bounds, infers an intermediate room, and returns `false` when it cannot
verify an opening. The current answer contract forces a binary answer and
cannot distinguish a confident negative from uncertainty.

### Integer answers often overcounted rooms

Of 166 wrong integer answers:

- 107 were overcounts;
- 59 were undercounts; and
- the mean absolute error across all integer cases was 1.45.

Eligible-room predictions commonly fell between four and eight, while expected
values usually fell between three and six. Some predictions were much larger,
including values as high as 21.

Agents often varied grid resolution, dilation radius, component-size threshold,
or boundary assumptions until the component count looked plausible. The
resulting number was sensitive to representation details and did not correspond
consistently to the oracle's room definition.

Direct-neighbor counting was less one-sided. Its wrong answers included both
overcounts and undercounts, usually by one room.

## How the agents solved cases

The typical successful workflow was:

1. Inventory `/input` and read the case, schema, inventory, and provenance.
2. Probe `map.lcm` and search the installed DimOS package for a decoder.
3. Decode the map with `PointCloud2.lcm_decode`.
4. Convert the point cloud to NumPy coordinates.
5. Project the points into a two-dimensional grid or inspect coordinate slices.
6. Apply a case-specific distance, clearance, histogram, ASCII-grid, or
   connected-component calculation.
7. Interpret the output and submit one Boolean or integer answer.

Across the 1,160 scored sessions:

| Observable behavior | Sessions |
| --- | ---: |
| Used NumPy | 1,021 |
| Used the supported point-cloud decoder | 974 |
| Used coordinate histograms or frequency analysis | 353 |
| Used distance or clearance calculations | 239 |
| Generated an ASCII-style map | 160 |
| Attempted flood fill or connected components | 131 |
| Read a generated image | 0 |

These method labels come from case-insensitive keyword matching over redacted
tool commands. They are useful aggregate indicators, not perfect semantic
classifications.

### Methods that fit local questions

Distance and clearance calculations align with pose, rotation, and translation
questions. Sessions containing these operations scored 88.7% overall.

Examples include:

- nearest occupied point to a target footprint;
- clearance along a sampled translation corridor;
- obstacle distance during a swept rotation; and
- wall checks between two nearby markers.

These calculations require limited local geometry and do not require the agent
to reconstruct the entire floor plan.

### Methods that failed on room topology

For room questions, agents commonly:

- rounded coordinates into a dense occupancy grid;
- printed an ASCII plan;
- counted frequent x/y coordinates as candidate walls;
- applied dilation or erosion;
- flood-filled free space;
- discarded small components; and
- manually assigned remaining components to rooms.

Flood fill can answer whether two markers share connected free space, which
helps explain the 98.3% same-room result. It does not directly recover semantic
rooms: doorways connect navigable free space, exterior areas can form large
components, and dilation parameters can close real openings or preserve
spurious gaps.

Flood-fill or connected-component sessions scored:

- 35/35 on same-room cases where the method was detected;
- 8/25 on direct-neighbor count;
- 8/21 on direct-room connection; and
- 3/50 on eligible-room count.

The comparison is observational and confounded by task difficulty, but the
within-method task split is consistent with the underlying geometric mismatch.

## Effort, uncertainty, and cost

Across scored sessions, the agent made 18,587 sandbox calls:

- mean: 16.0 calls per case;
- median: 16 calls per case;
- sessions with at least one nonzero command: 852; and
- total nonzero commands: 1,377.

Incorrect answers required more work:

| Measure | Correct | Incorrect |
| --- | ---: | ---: |
| Mean sandbox calls | 15.4 | 18.0 |
| Median duration | 65.7 s | 90.7 s |
| Median API cost | $0.070 | $0.091 |

This is an uncertainty signal, not evidence that more commands cause errors.
Hard topology questions induce more attempts, competing interpretations, and
threshold changes.

The current `submit_answer` schema retains only the typed answer. It cannot
record abstention or a concise evidence summary. The project wishlist now
proposes:

- `true | false | "unsure"` for Boolean answers; and
- a bounded `reasoning` field containing a short evidence summary.

These additions would make uncertainty observable. The reasoning summary would
remain self-reported evidence, not proof of the agent's internal process.

## Executor failures

All 10 failed sessions ran for approximately the full 900-second timeout. Each
ended during a `sandbox_exec` call without submitting an answer. Two adapter
logs explicitly contain `run timeout`; the remaining eight have empty adapter
stderr but the same final unmatched tool-call pattern.

| Case | Task | Variant |
| --- | --- | --- |
| `case-0121` | Pose occupancy | Clean |
| `case-0212` | Direct room connection | Noisy-01 |
| `case-0641` | Direct neighbor count | Noisy-01 |
| `case-0695` | Pose occupancy | Noisy-01 |
| `case-0700` | Eligible room count | Clean |
| `case-0815` | Pose occupancy | Noisy-01 |
| `case-0855` | Eligible room count | Noisy-02 |
| `case-0996` | Eligible room count | Noisy-02 |
| `case-1029` | Eligible room count | Noisy-02 |
| `case-1109` | Straight translation | Noisy-01 |

Four failures were eligible-room cases and three were pose-occupancy cases.
The evidence suggests that individual analysis commands stalled until the
per-case deadline. It does not show a broad API-key or scheduler failure:
1,160 other jobs completed and retained valid sessions.

## Interpretation

### What the run establishes

The run establishes that:

- the API-key execution path works at 1,000-case scale;
- the scheduler and session-retention pipeline completed 99.1% of jobs;
- the coding agent can decode the LCM point cloud without a task-specific map
  tool;
- local metric questions are highly tractable with model-written NumPy code;
- whole-floor topology remains unreliable; and
- errors are structured rather than random.

### What the run does not establish

The run does not establish that:

- the base model understands a rendered occupancy map;
- multimodal map input is ineffective;
- the agent has a persistent symbolic room representation;
- the same approach works on a real Go2 map;
- the tool audit proves that no online access occurred;
- the model's natural-language reasoning faithfully describes its computation;
  or
- the 77.2% micro score represents balanced spatial competence.

The strongest result concerns code-as-action spatial analysis, not one-shot
visual map understanding.

## Recommendations from the retained sessions

The next engineering improvement should target architectural topology rather
than prompting or longer reasoning.

1. Consolidate point-cloud decoding, coordinate conventions, occupancy
   rasterization, and robot-footprint handling into one supported spatial
   inspection layer.
2. Preserve the successful local operations: pose clearance, swept rotation,
   and straight-path checks.
3. Add a deterministic room-and-opening representation instead of asking each
   agent to rediscover room segmentation.
4. Expose room lookup, doorway adjacency, neighbor count, and eligible-room
   enumeration through a consistent interface.
5. Add explicit `unsure` and bounded reasoning-summary fields to submissions so
   uncertainty can be separated from negative predictions and executor
   failures.

The retained sessions do not support spending the next iteration on longer
budgets. Incorrect cases already consumed more time and money without resolving
the topological ambiguity.

## Artifacts and reproducibility

The machine-local experiment artifacts are under:

```text
/tmp/opencode/pi-api-key-full-1170-20260723/
  config/
  private/
  public/
  spatial-api-key-full-1170-forbidden/
```

Important files include:

```text
spatial-api-key-full-1170-forbidden/manifest.json
spatial-api-key-full-1170-forbidden/plan.json
spatial-api-key-full-1170-forbidden/executor.pi.v1.json
spatial-api-key-full-1170-forbidden/jobs/*.json
private/spatial-api-key-full-1170-forbidden/<job>/attempt-1/private/<run>/
public/spatial-api-key-full-1170-forbidden/<job>/attempt-1/public/<run>/
```

The `/tmp` tree is ephemeral and should be copied to durable storage if it must
survive host cleanup.

The broader investigation and tooling wishlist are tracked in
[Spatial Agent Understanding: Progress and Evaluation Plan](/docs/development/spatial_agent_understanding_progress.md).
