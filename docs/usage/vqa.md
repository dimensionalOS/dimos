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

## Create or Edit a Dataset

Open the local editor with a calibrated recording and a new or existing VQA dataset directory:

```bash skip
dimos evals vqa edit /path/to/calibrated-recording.db /path/to/vqa-dataset
```

The interface runs on `http://127.0.0.1:8765` by default; use `--port` to select another local port.
Frame navigation displays raw recorded images. Generating a single frame or a `start`, `stop`,
`stride` range applies the normal rectification and image/point-cloud question pipelines only to the
selected frames. Range generation advances the editor as each frame completes and reports progress
without waiting for the full range to finish.

Recordings with world-frame LiDAR and odometry also display a cached global top-down LiDAR map. The
editor builds the map once at startup, then crops it around the selected frame's synchronized robot
pose. The map panel remains available as an empty status panel when a frame has no synchronized
odometry; incompatible or missing optional map streams do not prevent normal editing.

Questions, choices, and ground-truth answers can be edited, added, or removed. Navigation retains
drafts in the editor process but does not modify dataset files. A nonexistent output directory is
created as a blank workspace; an empty directory is also accepted. **Submit dataset** creates or
updates `cases.jsonl` and `labels.jsonl`, writes rectified assets for edited frames, and preserves all
untouched cases and assets when editing an existing dataset. Partially initialized or nonempty
non-dataset directories are rejected rather than overwritten. An output directory can be open in
only one editor process at a time, preventing concurrent submissions from overwriting each other.

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
