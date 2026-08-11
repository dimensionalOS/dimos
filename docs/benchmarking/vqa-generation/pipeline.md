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

Generation writes the resolved request, model IDs, and aggregate counts to the dataset root's
private `run.json`. This is an output record, not the input specification.

## 2. Create Questions

### Constrained

The image author inspects the scene and returns up to five structured intents. It selects only
families likely to be useful for the visible arrangement, rather than expanding every object into
every family. Private grounding still rejects unsupported or ambiguous candidates.

| Family | Sample question | Choices |
|---|---|---|
| Presence | `Is there a chair in the image?` | `yes`, `no` |
| Horizontal direction | `Where is the nearest chair?` | `left`, `center`, `right` |
| Distance threshold | `Is the nearest chair within 3 meters?` | `yes`, `no` |
| Visible count | `How many chairs are visible?` | `1-2`, `3-4`, `5-7`, `8+` |
| Camera range | `How far is the nearest chair from the camera?` | `under 1 m`, `1 to under 2 m`, `2 to under 4 m`, `4 m or more` |
| Nearest by side | `Which chair is closer, the left or right one?` | `left`, `right` |
| Pairwise left/right | `Is the chair to the left or right of the table?` | `left`, `right` |
| Height comparison | `Which is taller: the chair or the table?` | `chair`, `table` |
| Closest object | `Which object is closest to the chair: table or lamp?` | `table`, `lamp` |
| Door state | `Is the door open or closed?` | `open`, `closed` |
| Forward path | `Is the path directly ahead clear or blocked?` | `clear`, `blocked` |

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

Door-state questions also require one grounded door, a robust plane fit for the door mask, and a
robust plane fit in a narrow ring around that mask. The planes must be either nearly aligned
(`closed`) or clearly rotated (`open`); slightly ajar or otherwise ambiguous doors are rejected.

Closest-object questions require exactly one grounded target and one grounded instance for every
candidate choice. They compare private support-point centroids and reject candidates whose nearest
two distances are within `0.15 m`.

Visible-count questions count only accepted grounded instances, rather than raw detections. Camera-range
questions use the nearest grounded instance's camera-origin Euclidean range. Height comparisons require
one accepted shared ground plane and one successful physical-height measurement per distinct object;
overlapping measurement uncertainty intervals are rejected.

Pairwise left/right questions require exactly one grounded instance for each named object. They compare
their visible support-point centroids in the camera horizontal axis and reject separation under `0.1 m`.

Forward-path questions require a fitted visible ground plane and enough ground support in each third
of the center camera-forward corridor from `0.5-3.0 m`. Supported non-ground points block the
corridor; incomplete ground support or sparse obstacle evidence is rejected.

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

```text
visible_count(A)
-> detect_objects(A) -> segment_detections(...) -> ground_masks(...) -> grounded A instances
-> bucket accepted instance count: 1-2 / 3-4 / 5-7 / 8+
```

```text
camera_range(A)
-> detect_objects(A) -> segment_detections(...) -> ground_masks(...) -> grounded A instances
-> select_nearest_object(...) -> bucket camera-origin range
```

```text
compare_left_right(A, B)
-> ground exactly one A and one B -> compare camera-frame support centroids
-> separation under 0.1 m: reject -> otherwise choose left/right
```

```text
compare_height(A, B)
-> ground exactly one A and one B -> fit_ground_plane()
-> measure_height(A, plane) and measure_height(B, plane)
-> reject overlapping uncertainty intervals -> choose taller A/B
```

### Agentic

The private oracle chooses a sequence from the same read-only primitives used by constrained recipes:

| Tool | Input | Output |
|---|---|---|
| `detect_objects` | semantic query | Detection ID and private boxes. |
| `segment_detections` | detection ID | Mask ID and accepted mask count. |
| `ground_masks` | mask ID | Grounded object IDs, range, side, point support, evidence IDs. |
| `select_nearest_object` | object IDs, optional side | Nearest grounded object ID. |
| `count_grounded_objects` | object IDs | Fixed count bucket and cited grounded instances. |
| `bucket_camera_range` | object ID | Fixed camera-origin range bucket and cited object. |
| `compare_nearest_by_side` | object IDs | Public `left` or `right` choice from nearest grounded objects. |
| `compare_left_right` | two object IDs | Public pairwise `left` or `right` relation, or an ambiguity rejection. |
| `select_closest_object` | target ID, candidate IDs | Candidate nearest to target by private support-point centroids. |
| `fit_ground_plane` | none | Plane ID, plane estimate, residual, inlier support, quality flags. |
| `measure_height` | object ID, plane ID | Measurement ID, private height, uncertainty, provenance, quality flags. |
| `compare_heights` | two object IDs, plane ID | Taller object choice from shared-plane measurements, or rejection. |
| `classify_door_state` | object ID | Public `open` or `closed` choice, or a private geometry rejection. |
| `classify_forward_path` | none | Public `clear` or `blocked` choice, or a private visibility rejection. |
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
run.json                  private resolved generation request and aggregate counts
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
