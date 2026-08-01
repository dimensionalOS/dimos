# Spatial Agent Understanding: Progress and Evaluation Plan

Status: active investigation, updated 2026-07-23.

## Goal

Determine which spatial representation and interaction contract lets an agent
reliably:

1. ground a robot, target, or semantic object in space;
2. understand free space, obstacles, rooms, and openings;
3. transform between world, robot, image, and map coordinates;
4. return an actionable position and orientation; and
5. preserve those abilities on real robot maps rather than only synthetic data.

The goal is not to maximize one benchmark score by letting a coding agent invent
an arbitrary solution. The benchmark must reveal what the model understands,
which computation or representation supplies the missing capability, and where
the result fails under adversarial controls.

## Current experiment

The current track evaluates a general coding agent with:

- one immutable public case at a time;
- a point-cloud map serialized as `map.lcm`;
- public question and query geometry;
- a writable workspace and bounded shell execution;
- DimOS preinstalled;
- no answer-bearing oracle access;
- no generated-image reads; and
- exactly one typed answer submission.

The completed balanced run used 100 unique questions across 30 held-out
Structured3D-derived scenes. It balanced seven task categories at 14 or 15 cases
each and map variants at 34 clean, 33 noisy-01, and 33 noisy-02 cases.

Operational result:

- 100/100 jobs completed and produced valid scores.
- There were no failed, interrupted, pending, or cancelled jobs.
- This is a 100% execution success rate, not a 100% QA success rate.

QA result:

| Task | Correct | Accuracy |
| --- | ---: | ---: |
| Same room | 13/14 | 92.9% |
| In-place rotation | 12/14 | 85.7% |
| Pose occupancy | 12/14 | 85.7% |
| Straight translation | 12/14 | 85.7% |
| Direct neighbor count | 7/15 | 46.7% |
| Direct room connection | 7/15 | 46.7% |
| Eligible room count | 2/14 | 14.3% |
| **Overall** | **65/100** | **65.0%** |

Boolean questions scored 56/71 (78.9%). Integer questions scored 9/29
(31.0%). Sixteen of the 20 wrong integer answers were overcounts. Fourteen of
the 15 wrong Boolean answers were false negatives.

Map noise was not the dominant variable in this sample:

| Variant | Correct | Accuracy |
| --- | ---: | ---: |
| Clean | 21/34 | 61.8% |
| Noisy-01 | 23/33 | 69.7% |
| Noisy-02 | 21/33 | 63.6% |

The current run is stored in the machine-local, ephemeral directory:

```text
/tmp/opencode/pi-api-key-balanced-100-20260723/
  spatial-api-key-balanced-100-forbidden-audit-v3/
  private-v3/
  public-v3/
```

The committed runner implementation is `2949a16b9`.

## How agents currently solve the cases

The 100 retained sessions show a common sequence:

1. Inventory `/input` and read the case, schema, inventory, and provenance.
2. Probe the binary map and search the installed DimOS package for a decoder.
3. Decode the point cloud, usually with `PointCloud2.lcm_decode`.
4. Project points into 2-D NumPy arrays.
5. Inspect coordinate histograms, wall runs, ASCII grids, nearest-point
   distances, or an improvised flood fill.
6. Interpret the result manually and submit one answer.

Aggregate behavior:

- 1,648 accepted `sandbox_exec` calls and 100 answer submissions;
- zero generated-image reads;
- 83 sessions used the supported `PointCloud2` decoder;
- 80 used coordinate histograms or line-frequency analysis;
- 47 generated an ASCII occupancy representation;
- 29 used an explicit nearest-distance or clearance calculation;
- 11 attempted flood fill, dilation, or connected-component segmentation; and
- 70 sessions had at least one nonzero exploratory command.

Explicit local distance or clearance checks occurred in 29 sessions and 26 were
correct. This method fits pose, rotation, and translation questions. Whole-map
room reconstruction was less reliable. Flood-fill sessions were correct in
4/11 cases, although category difficulty confounds that comparison.

Wrong sessions spent more effort:

| Measure | Correct | Wrong |
| --- | ---: | ---: |
| Mean sandbox calls | 15.7 | 17.9 |
| Median duration | 68 s | 88 s |
| Median API cost | $0.072 | $0.093 |

This is an uncertainty signal, not evidence that extra commands cause errors.
Hard room-topology questions naturally require more exploration.

## Representative questions and submitted answers

These examples report the public question, public query geometry, submitted
answer, private expected answer, and the observable method. They do not treat
the model's prose explanation as proof that it used the map correctly.

### Local occupancy: correct

Question: “Can the robot occupy the marked pose? (variant 1)”

Public query:

```json
{"kind":"pose-occupancy","pose":{"x_m":-6.55504875,"y_m":0.8072461,"yaw_rad":0.0}}
```

Agent answer: `false`. Expected answer: `false`.

The agent decoded the point cloud and examined obstacle clearance near the
specified footprint. This is representative of the stronger local-geometry
workflow.

### Straight motion: correct

Question: “Can the robot drive straight by the stated distance? (variant 2)”

Public query:

```json
{"kind":"straight-translation","start_pose":{"x_m":-1.0235962125,"y_m":3.0150631667,"yaw_rad":3.1415926536}}
```

Agent answer: `true`. Expected answer: `true`.

The agent read the footprint configuration, decoded the point cloud, and
checked the path corridor against nearby wall coordinates.

### Direct neighbor count: incorrect overcount

Question: “How many rooms are directly connected to the marked room?
(variant 1)”

Public marker:

```json
{"x_m":1.0270002397,"y_m":3.1198524063}
```

Agent answer: `2`. Expected answer: `1`.

The agent manually traced wall segments and interpreted an apparent gap as an
additional connection. This is the common room-oversegmentation failure.

### Direct room connection: incorrect false negative

Question: “Do the rooms containing the two markers share a direct opening?
(variant 2)”

Public markers:

```json
[
  {"x_m":-4.2224122,"y_m":3.5211784},
  {"x_m":0.017352539,"y_m":0.679492203}
]
```

Agent answer: `false`. Expected answer: `true`.

The agent inferred an intermediate room from approximate wall bounds and
missed the direct opening. This illustrates why plausible verbal room
descriptions cannot substitute for a verified topology computation.

### Eligible room count: incorrect overcount

Question: “How many eligible rooms are on this floor?”

Agent answer: `9`. Expected answer: `7`.

The agent used 28 shell calls and several whole-map projections. More
exploration did not resolve the ambiguity in room boundaries and eligibility.

## What the current result establishes

The run establishes that a coding agent can often recover enough metric
geometry from the supplied point cloud to answer local collision questions. It
also establishes that the execution and evidence pipeline can retain complete,
inspectable sessions at 100-case scale.

The run does **not** establish that:

- the base model understands an occupancy-map image;
- multimodal input is ineffective;
- the model has a stable internal room representation;
- the same method works on a real Go2 map;
- the model can ground semantic objects such as a couch;
- a prose explanation reflects the computation that produced the answer; or
- the score cannot be achieved from dataset priors or low-resolution cues.

The agent has access to a general programming environment, inspects installed
source, and writes custom algorithms. This is a code-as-action baseline, not a
one-shot map-understanding result.

## Questions raised by the team

The discussion identifies six separate research questions:

1. **Basic map comprehension:** Can the model locate free space, obstacles, and
   the robot's orientation without writing a custom algorithm?
2. **Coordinate grounding:** Can it reliably place a cursor or return a metric
   pose, rather than merely describe an image?
3. **Representation:** Does it perform better with a point cloud, occupancy
   raster, numeric grid, vector geometry, room graph, or another structured
   representation?
4. **Frame and scope:** Should the map be world-aligned or body-aligned, and
   global, local, or multiscale?
5. **Semantic fusion:** Given an RGB/VLM observation such as “couch, three
   meters forward,” can it project the object into a map and return a pose
   beside or behind it with a valid yaw?
6. **Model versus data:** Are failures caused by perception, coordinate
   transforms, map conventions, spatial reasoning, action formatting, or the
   absence of a suitable deterministic tool?

The linked historical context is
[issue #1913](https://github.com/dimensionalOS/dimos/issues/1913) and
[PR #822](https://github.com/dimensionalOS/dimos/pull/822). The PR's
conversation is relevant; its diverged implementation is not the proposed
baseline.

## Evaluation ladder

The investigation should proceed from controlled primitives to real tasks.
Each rung should pass before conclusions are generalized to the next.

### 1. Answer-without-evidence controls

Measure dataset priors and leakage with:

- question only, no map;
- wrong map from another case;
- shuffled marker-map pairing;
- mirrored or rotated map without transforming the query;
- 6×6 and other aggressively downsampled maps; and
- constant-majority and category-specific heuristic baselines.

A method must beat these controls and change its answer when the relevant
geometry changes.

### 2. Visual coordinate grounding primitives

Use synthetic images with exact ground truth:

- select one of six colored rectangles;
- place a cursor at a requested rectangle center;
- return normalized coordinates and pixel coordinates;
- identify whether an arrow points into a wall;
- rotate an arrow to a requested heading; and
- return a target point plus orientation.

Score position error, yaw error, selection accuracy, and consistency under
translation, rotation, scaling, color permutation, and distractors. This
separates visual recognition from coordinate emission.

### 3. Map representation ablation

Hold the question, geometry, model, and split constant while varying only:

- raw point-cloud LCM plus coding;
- decoded numeric point list;
- dense numeric occupancy array;
- top-down PNG with explicit scale, axes, and robot pose;
- SVG or vector wall/opening geometry;
- local metric graph or room/opening graph; and
- structured scene/tree representations analogous to UI accessibility trees.

Compare one-shot answers, model-written code, a documented decoder, and a
fixed deterministic spatial API. This reveals whether gains come from the
representation, the model's programming ability, or the supplied tool.

### 4. Frame and context ablation

Cross:

- world-aligned versus robot/body-aligned maps;
- full global map versus a 5×5 m local crop;
- local plus a global inset;
- pose arrow versus numeric pose metadata; and
- north-up versus heading-up renderings.

Questions should include front/left/right relations, collision during
rotation, hallway direction, and “is the robot facing a wall?” Body-frame
benefits would indicate that coordinate transformation, rather than spatial
recognition, is the limiting factor.

### 5. Semantic object-to-map grounding

Separate object recognition from map reasoning:

1. Provide the object's class, bearing, and range as ground truth.
2. Ask the agent to project it into the map.
3. Ask for “other side,” “beside,” “in front of,” and “sit beside me” poses.
4. Score target position, collision, reachability, side relation, and yaw.

Then replace the ground-truth object observation with RGB/VLM output. Compare:

- RGB only;
- map only with a supplied object projection;
- RGB plus numeric bearing/range;
- RGB plus local map; and
- RGB plus local and global maps.

This identifies whether failure comes from object perception, projection, map
reasoning, or pose generation.

### 6. Real-map validation

Run the same controlled tasks on a real Go2 global map, such as the Hong Kong
office recording, after defining independent ground truth. Start with
human-verifiable local questions before room-count questions:

- Is the robot facing a wall?
- Can it rotate 90 degrees?
- Can it move one meter forward?
- Which side of a marked obstacle is free?
- Can it reach the other side without crossing occupied space?

Real-data evaluation needs explicit annotation provenance and uncertainty
handling. Synthetic geometry can provide exact labels; real lidar maps may not.

## Metrics

Keep the following measures separate:

- execution completion;
- exact QA accuracy by category;
- cursor or target-position error;
- yaw error;
- collision and reachability validity;
- relation correctness, such as the requested side of an object;
- perturbation consistency;
- answer-without-evidence performance;
- tool calls, latency, and cost;
- invalid or nonzero analysis commands; and
- calibrated abstention where the map is genuinely ambiguous.

Report micro accuracy, category-balanced macro accuracy, and confidence
intervals. Preserve per-case predictions so paired representation comparisons
can use paired statistical tests.

## Sharing questions and agent answers

Share a compact review packet for each sampled case:

```text
case_id:
capability:
public question:
public query/markers:
input representation:
agent mode: one-shot | fixed tools | coding agent
submitted answer:
expected answer: reviewer-only when the corpus remains private
score:
observable evidence: commands, derived map/grid, and measurements
position/yaw/collision errors:
counterfactual results: no-map, wrong-map, rotated, or downsampled
failure attribution:
```

The submitted answer and score are authoritative records. The tool transcript
and derived artifacts show what information was available and what computation
occurred. The agent's natural-language explanation is useful for generating a
hypothesis, but it is not faithful-process evidence and should not be presented
as such.

For team discussion, publish a stratified sample rather than only good-looking
successes: at least one correct and one incorrect case from every capability,
plus high-confidence wrong answers, answers that survive a wrong-map control,
and answers that flip under a one-cell or one-door counterfactual. Keep
answer-bearing private corpus files out of the public packet.

## Tooling wishlist

### Allow explicit uncertainty for Boolean answers

Extend `submit_answer` for Boolean questions from `true | false` to
`true | false | "unsure"`. The agent should not be forced to invent a binary
answer when its spatial evidence is incomplete or contradictory.

Record `"unsure"` as an accepted submission and a completed job, but not as a
correct exact-match answer. Report it separately with:

- abstention rate;
- coverage, the fraction answered `true` or `false`;
- selective accuracy, accuracy among non-abstained answers; and
- error rate among non-abstained answers.

Keep `"unsure"` distinct from `null`, a missing submission, timeout, or executor
failure. The tool should provide no correctness feedback after submission. This
change would reveal whether the agent can recognize its topology failures,
especially missed room openings, instead of defaulting to `false`.

### Capture a concise reasoning summary with every answer

Add a required `reasoning` field to `submit_answer` for both Boolean and integer
questions:

```json
{
  "answer": "unsure",
  "reasoning": "The two markers appear separated by a wall, but the point cloud does not resolve whether the observed gap is a doorway."
}
```

Treat this field as a short evidence summary, not a request for hidden
chain-of-thought. It should identify the method used, the decisive observation
or measurement, and any remaining ambiguity. Set a bounded length and reject
empty or structurally invalid submissions.

Persist the summary with the prediction and retained session, but exclude its
content from correctness scoring. Review it alongside tool calls and derived
artifacts to compare claimed evidence with observable computation. A plausible
summary is not proof that the agent used the stated method.

## Immediate next steps

1. Run no-map, wrong-map, and 6×6-map controls on the existing 100 questions.
2. Build the colored-rectangle and arrow-orientation coordinate-grounding set.
3. Define a paired representation matrix for numeric grid, PNG, SVG, and room
   graph inputs.
4. Add local/body-frame and local-plus-global variants.
5. Select a small, independently annotated real Go2 map set.
6. Add semantic projection tasks using supplied object bearing/range before
   adding RGB perception.
7. Inspect failures by perturbation behavior and derived artifacts, not by
   accepting the agent's explanation at face value.
8. Only after these controls stabilize, use iterative agent research to improve
   a reusable spatial tool.

## Research companion

Primary-source notes on the linked GitHub discussion and related evaluation
work belong in:

```text
docs/research/spatial-agent-map-understanding-sources.md
```

That note should support experiment selection; this document remains the
living record of decisions, results, and next steps.
