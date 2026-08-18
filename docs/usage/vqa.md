---
title: "Visual Question Answering"
---

The VQA tools generate deterministic questions from recorded images and evaluate them through the
shared dimOS evaluation runner.

## Generate a Dataset

Generate from one image:

```bash skip
dimos evals vqa generate go2_short.db --image-index 100
```

Generate from a range. `stop` is exclusive:

```bash skip
dimos evals vqa generate go2_short.db --start 0 --stop 100 --stride 10
```

Generated datasets default to:

```text
~/.local/state/dimos/datasets/vqa/<recording-stem>-frames
```

Use `--output <directory>` to override that location. The destination must be empty.

Go2 recordings without `camera_info` and `tf` require the explicit fallback profile:

```bash skip
dimos evals vqa generate go2_short.db --image-index 500 --calibration-profile go2
```

Recordings containing only one calibration stream are rejected. The VQA generation configuration
requires image and LiDAR observations to be within `0.1` seconds by default.

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
dimos evals vqa run ~/.local/state/dimos/datasets/vqa/go2_short-frames \
  --model gpt-4o-mini
```

Evaluation exposes only each public image, question, and fixed choices to the model. The shared runner
writes results under `~/.local/state/dimos/evals/run-*/`.
