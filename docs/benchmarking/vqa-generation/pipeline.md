---
title: "VQA Generation Pipeline"
---

# VQA Generation Pipeline

## 1. Run Generation

```bash
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
- `--output`: dataset root; completed frame directories are skipped on rerun.

`dimos vqa single-frame` accepts the same generation settings and uses `--frame-index` instead of frame bounds.

## 2. Create Questions

### Constrained

The image author proposes visible object queries, then the generator creates these deterministic families for every query:

| Family | Sample question | Choices |
|---|---|---|
| Presence | `Is there a chair in the image?` | `yes`, `no` |
| Horizontal direction | `Where is the nearest chair?` | `left`, `center`, `right` |
| Distance threshold | `Is the nearest chair within 3 meters?` | `yes`, `no` |
| Nearest by side | `Which chair is closer, the left or right one?` | `left`, `right` |

### Agentic

The image-only author returns a frozen question with one contract:

- Boolean: `{"kind":"boolean"}`; public choices are `yes`, `no`.
- Choice: `{"kind":"choice","choices":[...]}`; at least two choices.
- Height: `{"kind":"deferred_height_choice","strategy":"height-window-v1"}`; the question is
  frozen before private geometry, while four public choices are generated from a successful private
  height measurement.

Numeric contracts are rejected. `height-window-v1` selects a local four-choice window from private
measurement against the fixed internal breakpoints `0.1`, `0.2`, `0.6`, `1.0`, and `2.0` meters. For
example, a private height of `0.42 m` produces:

```text
under 0.2 m | 0.2-0.6 m | 0.6-1.0 m | over 1.0 m
```

## 3. Pre-Answer Grounding Checks

For each referenced object:

1. MoonDream detects or point-localizes the object.
2. EdgeTAM produces a mask.
3. Visible calibrated point-cloud samples are projected into the mask.
4. Mask area must meet `--min-mask-area-px`.
5. Point support must meet `--min-foreground-points`.

Height questions also require:

1. Accepted Open3D RANSAC ground plane.
2. Exactly one grounded object and one mask.
3. At least six points inside the object mask.
4. At least four elevated points, with at least 60% of selected points elevated more than `0.02 m` above the plane.

## 4. Create Answers

### Constrained

Each deterministic family runs its own fixed sequence.

```text
presence(A)
-> detect_objects(A) -> detection_id
-> segment_detections(detection_id) -> mask_id
-> ground_masks(mask_id) -> grounded A instances
-> no grounded A: reject
-> one or more grounded A instances: yes
```

```text
horizontal_direction(A)
-> detect_objects(A) -> detection_id
-> segment_detections(detection_id) -> mask_id
-> ground_masks(mask_id) -> grounded A instances
-> select_nearest_object(grounded A instances) -> nearest A
-> nearest A horizontal_direction: left/center/right
```

```text
within_distance(A, T)
-> detect_objects(A) -> detection_id
-> segment_detections(detection_id) -> mask_id
-> ground_masks(mask_id) -> grounded A instances
-> select_nearest_object(grounded A instances) -> nearest A
-> nearest A range_m <= T: yes/no
```

```text
compare_nearest_by_side(A)
-> detect_objects(A) -> detection_id
-> segment_detections(detection_id) -> mask_id
-> ground_masks(mask_id) -> grounded A instances
-> select_nearest_object(grounded A instances, left) -> nearest left A
-> select_nearest_object(grounded A instances, right) -> nearest right A
-> either side missing or tied: reject
-> compare the two range_m values: left/right
```

### Agentic

The private oracle chooses a sequence from the same read-only primitives used by constrained recipes:

| Tool | Input | Output |
|---|---|---|
| `detect_objects` | semantic query | Detection ID and private boxes. |
| `segment_detections` | detection ID | Mask ID and accepted mask count. |
| `ground_masks` | mask ID | Grounded object IDs, range, side, point support, evidence IDs. |
| `select_nearest_object` | object IDs, optional side | Nearest grounded object ID. |
| `fit_ground_plane` | none | Plane ID, plane estimate, residual, inlier support, quality flags. |
| `measure_height` | object ID, plane ID | Measurement ID, private height, uncertainty, provenance, quality flags. |
| `bucket_measurement` | measurement ID | Public answer-conditioned height choices and matching choice. |

Opaque IDs chain tool results; raw masks and point-cloud arrays are not exposed to the oracle. The
oracle returns a candidate answer and cited evidence IDs. Deferred height choices are the sole
exception to frozen public choices: the question remains frozen, but the public options and answer
are deterministically derived from the private measurement.

## 5. Post-Answer Validation

The candidate is rejected unless:

1. Its answer exactly matches the fixed public choices, or the public choices deterministically
   generated from a cited private height measurement.
2. It cites one or more known evidence IDs.
3. A cited height bucket matches the deterministic measurement bucket.
4. The private semantic validator confirms the cited tool output supports the question and answer.

Tool failures, quality-gate failures, invalid citations, invalid answer format, and unsupported claims are retained as rejected generation records.

## 6. Write Dataset Artifacts

Each `frame-*` directory contains:

```text
image.jpg                 public rectified image
frame.json                frame metadata and accepted/rejected counts
ground_truth.json         private tool evidence, checks, answers, and rejections
cases.json                public per-frame image/question/choice rows
labels.json               private per-frame correct-choice rows
```

The dataset root aggregates:

```text
cases.jsonl               public id, image path, question, choices
labels.jsonl              private id and expected choice
```

Each line in `cases.jsonl` is one public evaluation case. Formatted for readability, one record is:

```json
{
  "id": "go2-40-chair-height",
  "image": "frame-000040/image.jpg",
  "question": "How tall is the chair?",
  "choices": [
    "under 0.2 m",
    "0.2-0.6 m",
    "0.6-1.0 m",
    "over 1.0 m"
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
  "id": "go2-40-chair-height",
  "answer": "0.2-0.6 m"
}
```

- `id`: case identifier matching exactly one `cases.jsonl` row.
- `answer`: one of that case's `choices`.

The files store each record as one JSON object per physical line; the examples above are expanded
only for documentation readability.

The shared `point-cloud-vqa` evaluator reads public `cases.jsonl`, public images, and private `labels.jsonl`. It does not read generation evidence or point-cloud data.
