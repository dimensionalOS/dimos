---
title: "Visual Question Answering"
---

# Visual Question Answering

The VQA tools generate deterministic questions from recorded images and evaluate them through the
shared DimOS evaluation runner.

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

Legacy Go2 recordings without `camera_info` and `tf` require explicit calibration:

```bash skip
dimos evals vqa generate go2_short.db --image-index 500 --calibration-profile go2
```

Image and LiDAR observations must be within `--sync-tolerance` seconds, which defaults to `0.1`.

## Question Families

The image-only question author selects object names and applicable families. It does not produce
answers.

| Family | Choices | Evidence rule |
|---|---|---|
| `presence` | yes, no | At least one Moondream detection |
| `horizontal_direction` | left, center, right | Exactly one detection; use its horizontal center |
| `object_count` | one, two, three, four or more | Count Moondream detections |
| `object_distance` | under 1 m, 1 to under 2 m, 2 to under 3 m, 3 m or more | Median LiDAR range inside an EdgeTAM mask |

Proposals without sufficient evidence are rejected and retained in the private audit. If no proposal
can be answered, generation fails without publishing a dataset.

Distance questions require one Moondream detection, one EdgeTAM mask, and at least five projected
LiDAR points. Evidence whose range quartiles cross an answer boundary is rejected.

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

## Architecture

- `author.py` proposes constrained family inputs from an image.
- `families.py` owns question text, choices, and deterministic answer rules.
- `preprocessing.py` synchronizes and calibrates image and point-cloud frames.
- `primitives/edgetam.py` segments objects and estimates LiDAR-backed range.
- `primitives/moondream.py` supplies private object detections.
- `generate.py` loads frames, reuses models, and writes datasets atomically.
- `suite.py` validates generated artifacts and creates shared evaluation cases.

The initial implementation intentionally defers negative-presence policy, retries, and resume
behavior.
