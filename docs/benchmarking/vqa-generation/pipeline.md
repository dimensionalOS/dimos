---
title: "VQA Generation Pipeline"
---

# VQA Generation Pipeline

## 1. Run Generation

```bash skip
dimos vqa generate \
  --recording <recording.db> \
  --start-index <first-frame> \
  --stop-index <exclusive-last-frame> \
  --stride <sample-every-n-frames> \
  --question-mode constrained|agentic \
  --output <dataset-directory>
```

Flags:

- `--recording`: Memory2 Go2 recording path or named recording.
- `--start-index`, `--stop-index`, `--stride`: sampled recording frames.
- `--question-mode`: `constrained` or `agentic`; default is `constrained`.
- `--min-mask-area-px`: minimum accepted segmentation-mask area; default `128`.
- `--min-foreground-points`: minimum point-cloud support inside a mask; default `3`.
- `--output`: dataset root; frames with a completed `frame.json` marker are skipped, while partial
  frame directories are safely rewritten on rerun.

To generate one frame, set `--start-index` to that frame and `--stop-index` to the next index. For
example, use `--start-index 40 --stop-index 41` for frame 40.

### Generation Specification

`dimos vqa generate --spec <generation.json>` is an alternative to the explicit generation flags.
Do not combine `--spec` with `--recording`, frame bounds, question mode, grounding thresholds, or
output flags. A specification is a reproducible generation request:

```json
{
  "recording": "go2_bigoffice.db",
  "start_index": 0,
  "stop_index": 100,
  "stride": 20,
  "question_mode": "agentic",
  "grounding": {
    "min_mask_area_px": 128,
    "min_foreground_points": 3
  },
  "output": "~/.local/state/dimos/datasets/vqa/go2-bigoffice"
}
```

Generation writes the resolved request, model IDs, and aggregate counts to private
`audit/run.json`. This is an output record, not the input specification.

## 2. Create Questions

### Constrained

The image author inspects the scene and returns structured intents in one JSON array. Invalid JSON
is retried once. It selects only
families likely to be useful for the visible arrangement, rather than expanding every object into
every family. Private grounding still rejects unsupported or ambiguous candidates.

| Family | Sample question | Choices |
|---|---|---|
| Presence | `Is there a chair in the image?` | `yes`, `no` |
| Horizontal direction | `Where is the chair in the image?` | `left`, `center`, `right` |
| Distance threshold | `Is the nearest chair within 3 meters?` | `yes`, `no` |
| Visible count | `How many chairs are visible?` | `0`, `1-2`, `3-4`, `5-7`, `8+` |
| Camera range | `How far is the nearest chair from the camera?` | `under 1 m`, `1 to under 2 m`, `2 to under 4 m`, `4 m or more` |
| Nearest by side | `Which chair is closer, the left or right one?` | `left`, `right` |
| Pairwise left/right | `Is the chair to the left or right of the table?` | `left`, `right` |

### Agentic

The image-only author returns a frozen question with one contract:

- Boolean: `{"kind":"boolean"}`; public choices are `yes`, `no`.
- Choice: `{"kind":"choice","choices":[...]}`; at least two choices.

The agentic oracle is not bound to a constrained family. It can inspect individual detection, mask,
and object handles, choose its own tool sequence, and either return a cited answer or privately
reject the proposal with the missing evidence.

## 3. Pre-Answer Evidence Checks

Presence, visible count, and image direction use valid MoonDream detections directly. Range families
need a detected mask with enough visible 3D point support. Questions are authored from RGB only;
private LiDAR evidence establishes range labels.

- Presence and count: accepted visual detections. `0` is an exhaustive distractor; empty detector
  output is unverified absence and rejects rather than producing a zero label.
- Range: accepted grounded instances only. Failed box segmentation attempts point-prompt fallback.
- Left/right comparison: one grounded instance per named object with clear separation.

Missing, sparse, or ambiguous evidence rejects the question.

## 4. Create Answers

### Constrained

Each deterministic family runs its own fixed sequence. `ground(A)` below is always the same private
primitive chain supplied by the shared family context. The deterministic question answerer only
dispatches the intent to the matching family; the family owns grounding, measurements, quality
gates, and answer selection:

```text
ground(A)
-> detect_objects(A)
-> segment_detections(A)
-> ground_masks(A)
-> accepted grounded A instances, or reject if the family requires an unavailable instance
```

```text
presence(A)
-> detect_objects(A)
-> no valid visual detection: reject
-> one or more visual detections: yes
```

```text
horizontal_direction(A)
-> detect_objects(A)
-> require exactly one valid visual detection
-> bounding-box center: left/center/right
```

```text
within_distance(A, T)
-> ground(A)
-> select_nearest_object(grounded A instances) -> nearest A
-> nearest A range_m <= T: yes/no
```

```text
compare_nearest_by_side(A)
-> ground(A)
-> select_nearest_object(grounded A instances, left) -> nearest left A
-> select_nearest_object(grounded A instances, right) -> nearest right A
-> either side missing or ranges within 0.1 m: reject
-> compare the two range_m values: left/right
```

```text
visible_count(A)
-> detect_objects(A)
-> deduplicate near-identical boxes
-> bucket accepted visual instance count: 0 / 1-2 / 3-4 / 5-7 / 8+
-> empty perception output: reject, never infer 0
```

```text
camera_range(A)
-> ground(A)
-> select_nearest_object(...) -> bucket camera-origin range
```

```text
compare_left_right(A, B)
-> ground(A), ground(B)
-> require exactly one accepted A and B
-> compare camera-frame support centroids
-> separation under 0.1 m: reject -> otherwise choose left/right
```

### Agentic

The private oracle calls only read-only perception and geometry primitives shared with constrained
recipes. It interprets their measurements and selects an answer itself:

| Primitive tool | Input | Output |
|---|---|---|
| `detect_objects` | semantic query | Individual opaque detection IDs, confidence, and citable visual box evidence. |
| `segment_object` | detection ID | Individual opaque mask IDs or an explicit empty-segmentation rejection. |
| `ground_mask` | mask ID | One frame-scoped canonical object ID and citable support evidence. Near-identical masks with matching range reuse the same object ID. |

Opaque IDs chain tool results; raw masks and point-cloud arrays are not exposed to the oracle. The
oracle returns a candidate answer and cited evidence IDs.

## 5. Post-Answer Validation

The candidate is rejected unless:

1. Its normalized answer matches one of the fixed public choices.
2. It cites one or more known evidence IDs.

Tool failures, quality-gate failures, invalid citations, and invalid answer formats are retained as
rejected generation records. The tool-calling oracle currently interprets whether its cited evidence
supports a freeform question without a second model judge.

## 6. Write Dataset Artifacts

The generated root is directly evaluable while retaining private audit state:

```text
cases.jsonl                         public id, image path, question, choices
labels.jsonl                        private id and expected choice
assets/
  frame-000040.jpg                  public rectified image, stored once
audit/
  run.json                          private resolved request and aggregate counts
  frame-000040/
    frame.json                      completion marker, written last
    ground_truth.json               private evidence, measurements, and traces
    cases.json                      resumable per-frame evaluation rows
    labels.json                     resumable per-frame expected answers
```

Each line in `cases.jsonl` is one public evaluation case. Formatted for readability, one record is:

```json
{
  "id": "go2-40-chair-range",
  "image": "assets/frame-000040.jpg",
  "question": "How far is the nearest chair from the camera?",
  "choices": [
    "under 1 m",
    "1 to under 2 m",
    "2 to under 4 m",
    "4 m or more"
  ]
}
```

- `id`: unique case identifier.
- `image`: dataset-relative path to the public image.
- `question`: question shown to the evaluated vision model.
- `choices`: at least two allowed answer strings.

Each line in `labels.jsonl` is the corresponding private correct answer. Formatted for readability,
one record is:

```json
{
  "id": "go2-40-chair-range",
  "answer": "1 to under 2 m"
}
```

- `id`: case identifier matching exactly one `cases.jsonl` row.
- `answer`: one of that case's `choices`.

The files store each record as one JSON object per physical line; the examples above are expanded
only for documentation readability.

Run the generated dataset through the standard evaluation framework:

```bash
dimos evals run dimos.benchmark.vqa.evaluation \
  --dataset <generated-dataset> \
  --model gpt-4o-mini
```

The VQA suite loader reads public `cases.jsonl`, public images, and private `labels.jsonl`. It does
not read generation evidence or point-cloud data.
