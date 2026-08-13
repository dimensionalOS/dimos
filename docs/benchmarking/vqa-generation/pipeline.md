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

The image author inspects the scene and returns up to 15 structured intents. It selects only
families likely to be useful for the visible arrangement, rather than expanding every object into
every family. Private grounding still rejects unsupported or ambiguous candidates.

| Family | Sample question | Choices |
|---|---|---|
| Presence | `Is there a chair in the image?` | `yes`, `no` |
| Horizontal direction | `Where is the nearest chair?` | `left`, `center`, `right` |
| Distance threshold | `Is the nearest chair within 3 meters?` | `yes`, `no` |
| Visible count | `How many chairs are visible?` | `0`, `1-2`, `3-4`, `5-7`, `8+` |
| Camera range | `How far is the nearest chair from the camera?` | `under 1 m`, `1 to under 2 m`, `2 to under 4 m`, `4 m or more` |
| Nearest by side | `Which chair is closer, the left or right one?` | `left`, `right` |
| Pairwise left/right | `Is the chair to the left or right of the table?` | `left`, `right` |
| Height comparison | `Which is taller: the chair or the table?` | `chair`, `table` |
| Direct support | `Is the box on the table?` | `yes`, `no` |
| Opening width | `How wide is the doorway?` | `under 0.2 m`, `0.2 to under 0.5 m`, `0.5 to under 0.8 m`, `0.8 m or more` |
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

The agentic oracle is not bound to a constrained family. It can inspect individual detection, mask,
object, and plane handles, choose its own tool sequence, and either return a cited answer or privately
reject the proposal with the missing evidence.

## 3. Pre-Answer Grounding Checks

All referenced objects need a detected mask with enough visible 3D point support. Questions are
authored from RGB only so the public question is visually answerable; private LiDAR evidence may
establish the gold answer but does not make a hidden or ambiguous question acceptable.

- Height: one grounded object and an accepted ground plane.
- Door state: a door plane and nearby surrounding plane with a clear relative angle.
- Closest object: one grounded target and one grounded instance per candidate.
- Count and range: accepted grounded instances only. `0` is an exhaustive distractor; empty detector,
  segmenter, or grounding output is unverified absence and rejects rather than producing a zero label.
- Left/right and height comparison: one grounded instance per named object with clear separation.
- Object on support: contact or clear separation relative to the support plane.
- Opening width: one ground-connected aperture and stable surrounding-wall geometry.
- Forward path: visible ground support across the forward corridor and supported obstacle evidence.

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
-> ground(A)
-> no grounded A: reject
-> one or more grounded A instances: yes
```

```text
horizontal_direction(A)
-> ground(A)
-> select_nearest_object(grounded A instances) -> nearest A
-> nearest A horizontal_direction: left/center/right
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
-> either side missing or tied: reject
-> compare the two range_m values: left/right
```

```text
visible_count(A)
-> ground(A)
-> bucket accepted instance count: 0 / 1-2 / 3-4 / 5-7 / 8+
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

```text
compare_height(A, B)
-> ground(A), ground(B)
-> require exactly one accepted A and B
-> fit_ground_plane()
-> measure_height(A, plane) and measure_height(B, plane)
-> reject overlapping uncertainty intervals -> choose taller A/B
```

```text
object_on_support(A, B)
-> ground(A), ground(B)
-> require exactly one accepted A and B
-> fit_ground_plane() -> ground plane
-> fit_object_surface_plane(B) -> support plane
-> reject non-horizontal support plane
-> measure_object_plane_relation(A, B, support plane, ground normal)
-> clear separation: no -> unsupported or ambiguous geometry: reject -> otherwise yes
```

```text
opening_width(A)
-> detect_objects(A) -> segment_detections(A)
-> require exactly one accepted aperture mask
-> fit_ground_plane() -> ground plane
-> measure_opening_width_from_mask(mask, ground plane)
-> reject non-ground-connected, nonvertical, edge-clipped, or unstable aperture geometry
-> bucket: under 0.2 / 0.2-0.5 / 0.5-0.8 / 0.8 m or more
```

```text
closest_object(A, B, C, ...)
-> ground(A), ground(B), ground(C), ...
-> require exactly one target and one instance for every candidate type
-> select_closest_object(target, candidates) from private support-point centroids
-> reject ties within 0.15 m -> otherwise choose the closest candidate
```

```text
door_state(A)
-> ground(A) -> require exactly one door
-> fit_object_surface_plane(door) -> door plane
-> fit_object_surrounding_plane(door) -> surrounding plane
-> measure_relative_plane_angle(door plane, surrounding plane)
-> nearly aligned: closed; clearly rotated: open; intermediate/failed geometry: reject
```

```text
forward_path()
-> fit_ground_plane()
-> classify_forward_corridor(visible camera points, ground plane)
-> inspect the fixed camera-frame corridor within 2 m and 0.5 m on either side
-> require ground support across each forward distance band
-> coherent elevated cluster: blocked; no supported obstacle: clear; scattered points: reject
```

### Agentic

The private oracle calls only read-only perception and geometry primitives shared with constrained
recipes. It interprets their measurements and selects an answer itself:

| Primitive tool | Input | Output |
|---|---|---|
| `detect_objects` | semantic query | Individual opaque detection IDs and confidence. |
| `segment_detection` | detection ID | Individual opaque mask IDs. |
| `ground_mask` | mask ID | One frame-scoped canonical object ID and citable support evidence. Near-identical masks with matching range reuse the same object ID. |
| `fit_ground_plane` | none | Plane ID, plane estimate, residual, inlier support, quality flags. |
| `get_object_pose` | object ID | Camera-frame grounding evidence: range, side, support count. |
| `fit_object_surface_plane` | object ID | Plane ID from one object's visible support. |
| `fit_mask_surrounding_plane` | mask ID | Plane ID from visible support around a mask. |
| `measure_object_pair_distance` | two object IDs | Private 3D support-centroid distance. |
| `measure_relative_plane_angle` | two plane IDs | Private unsigned angle between accepted planes. |
| `measure_object_plane_relation` | object ID, support ID, support plane, ground plane | Clearance, contact, and projected-separation metrics. |
| `measure_aperture_geometry` | mask ID, ground plane | Private ground-connected aperture span and uncertainty. |
| `measure_forward_corridor` | ground plane | Private floor-support and elevated-obstacle metrics. |
| `measure_height` | object ID, plane ID | Measurement ID, private height, uncertainty, provenance, quality flags. |

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
  "id": "go2-40-chair-height",
  "image": "assets/frame-000040.jpg",
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
