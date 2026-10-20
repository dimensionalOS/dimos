# Visual Question Answering

The VQA tools generate deterministic questions from recorded images and evaluate them through the
shared dimOS evaluation runner.

## Generate a Dataset

Generate from one image in a recording containing `color_image`, `camera_info`, `tf`, and either
`pointlio_lidar` or `lidar`:

```bash skip
dimos evals vqa generate /path/to/calibrated-recording.db --image-index 100
```

Generate from a range. `stop` is exclusive:

```bash skip
dimos evals vqa generate /path/to/calibrated-recording.db --start 0 --stop 100 --stride 10
```

Or keep update frequency with mem `QualityWindow` on `Image.sharpness`
(same path as loop-closure / semantic-peak picking). Optional
`--min-brightness` drops dark frames first. Do not combine with `--stride`:

```bash skip
dimos evals vqa generate /path/to/calibrated-recording.db \
  --start 0 --stop 100 --quality-window 0.5 --min-brightness 0.1
```

Generated datasets default to:

```text
~/.local/state/dimos/datasets/vqa/<recording-stem>-frames
```

Use `--output <directory>` to override that location. The destination must be empty. Generation
requires non-empty `color_image`, `camera_info`, `tf`, and either `pointlio_lidar` or `lidar` streams;
incomplete datasets are rejected at startup. For a valid dataset, point-cloud evidence is available
when image and LiDAR observations are within `0.1` seconds. Individual frames with unmatched LiDAR
observations or unresolvable TF retain only image-based question families.

### Point-cloud frame preparation

Point-cloud question families use an explicit image-aligned frame boundary. After validating the
required dataset streams, the loader in `dimos/evals/vqa/pointcloud_frame.py` reads the selected image
and nearest point cloud, rectifies the image and camera intrinsics, resolves the
point-cloud-to-camera transform from recorded TF, and packages the synchronized observations as a
`PointCloudFrame`.

The range primitive in `dimos/evals/vqa/primitives/range.py` consumes that prepared frame. It
projects valid points into the image, keeps the nearest camera-depth point per pixel, selects points
inside each object mask, and derives robust range statistics. Keeping dataset access and calibration
preparation outside the primitive lets range estimation operate only on explicit geometry.

## Question Families

The image-only question author selects object names and applicable families. It does not produce
answers. Every proposal uses `object_names`; single-object families require one entry, while
`closest_object` requires two to five.

| Family | Choices | Evidence rule |
|---|---|---|
| `presence` | yes, no | At least one Moondream detection |
| `horizontal_direction` | left, center, right | Exactly one detection; use its horizontal center |
| `object_count` | one, two, three, four or more | Count Moondream detections |
| `image_coverage` | Adaptive percentage buckets | Divide EdgeTAM mask pixels by image pixels |
| `largest_visible_area` | Two to five authored object references | Select a mask at least 20% larger than the runner-up |
| `object_distance` | under 1 m, 1 to under 2 m, 2 to under 3 m, 3 m or more | Median LiDAR range inside an EdgeTAM mask |
| `closest_object` | Two to five authored object references | Select the smallest unambiguous LiDAR range |
| `ego_fore_aft` | ahead, behind | Sign on the robot forward axis for a referent: bare unique name, `nearest X`, `left X`, or `right X` |
| `height_order` | Two authored object references | Higher world-`z` with a minimum vertical margin (localize scene) |
| `ego_turn` | left, right, ahead | Bearing to a resolved referent in the robot yaw frame |
| `ego_any_fore_aft` | yes, no | Whether any verified instance of a category lies ahead of the robot |

Proposals without sufficient evidence are rejected and retained in the private audit. If no proposal
can be answered, generation fails without publishing a dataset.

Distance questions require one Moondream detection, one EdgeTAM mask, and at least five projected
LiDAR points. Evidence whose range quartiles cross an answer boundary is rejected.
Closest-object questions accept references such as `left person`, `right person`, and `chair`, which
lets Moondream distinguish repeated categories. They are rejected unless every reference has exactly
one detection and the closest range interval does not overlap any other candidate.

Image-coverage questions choose between two bucket schemes so the measured mask coverage is as far
as possible from the nearest answer boundary. Largest-visible-area questions are rejected unless the
winning EdgeTAM mask contains at least 20% more pixels than the runner-up. Both families work without
point-cloud data.

Localize-backed families (`ego_fore_aft`, `height_order`, `ego_turn`, `ego_any_fore_aft`) are **off by
default**. Enable them with `--with-localize`. Generation then:

1. Discovers object names in a padded time window with prompt-free YOLO-E (or uses optional
   `--localize-query` overrides)
2. Runs Dan localize once with a room-scale Go2 policy (not the xArm desktop defaults) to verify
   those names in 3D
3. Resolves `base_link` pose per selected image and attaches `LocalizeSceneEvidence`
4. Adds deterministic localize proposals for case diversity:
   - **unique** bare names when a category has exactly one verified hit
   - **nearest** `<category>` when multiple hits exist (closest to the robot)
   - **`<anchor> next to <landmark>`** relational referents for `ego_fore_aft` / `ego_turn`
     when a unique nearby pair exists (disambiguates without leaking turn direction)
   - **any-ahead** yes/no for each selected category (`ego_any_fore_aft`)
   - **height_order** pairs among unique categories only

Ego geometry uses shared cones (`θ = atan2(left, forward)`, `0.5 ≤ r ≤ 5` m): ahead
`|θ| ≤ 45°`, behind `|θ| ≥ 135°`, sides rejected for fore/aft; `ego_turn` answers
left / right / ahead / **turn around**. See `experience/ego-spatial-sectors.md`.

Left/right ``<category>`` referents are not proposed (awkward English in the question text;
multi-instance disambiguation uses nearest / next-to instead).

Published localize cases are budgeted to at most **K=5 per frame**. Selection
round-robins `(family, selector)` types across frames so every frame with ego
candidates keeps at least one localize question and later frames are not starved.
Within the target type, under-represented categories win. Override with
`--localize-coverage-k`, or pass `0` to publish every answered localize case.
Image-path families are never budgeted.

Without `--with-localize`, those families are not offered.

```bash skip
dimos evals vqa generate /path/to/calibrated-recording.db \
  --start 180 --stop 280 --stride 20 \
  --with-localize \
  --localize-pad 20 \
  --localize-coverage-k 5
```

## Dataset Layout

```text
dataset/
  cases.jsonl
  labels.jsonl
  assets/
    frame-000100.png
  audit/
    run.json
    frame-000100/
      frame.json
      ground_truth.json
      cases.json
      labels.json
```

`cases.jsonl` and `assets/` are public evaluation inputs. `labels.jsonl` and `audit/` contain private
answers, evidence, rejected proposals, source indices, and timestamps.

## Run Evaluation

```bash skip
dimos evals vqa run ~/.local/state/dimos/datasets/vqa/calibrated-recording-frames \
  --model gpt-4o-mini
```

Evaluation exposes only each public image, question, and fixed choices to the model. The shared runner
writes results under `~/.local/state/dimos/evals/run-*/`.
