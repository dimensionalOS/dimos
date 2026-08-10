---
title: "VQA Generation Infrastructure"
---

# VQA Generation Infrastructure

This document describes the software stack behind the point-cloud-grounded VQA benchmark. For the
step-by-step generation procedure and dataset schema, see [Pipeline](/docs/benchmarking/vqa-generation/pipeline.md).

## Design Boundary

The benchmark has two intentionally separate phases:

1. Generation privately uses calibrated point clouds and perception tools to establish labels.
2. Evaluation receives only public images, questions, choices, and private expected-choice labels.

The evaluated vision model never receives point clouds, camera calibration, masks, measurements,
tool traces, or rejected attempts.

```text
recording + calibration + point cloud
                |
                v
       private generation runtime
                |
                +-- public: image.jpg, cases.jsonl
                +-- private: labels.jsonl, ground_truth.json
                |
                v
       image-only evaluation runtime
                |
                v
       run.json + vqa-results.json
```

## Package Layout

```text
dimos/benchmark/vqa/
  models.py                    shared immutable VQA contracts
  generation/
    primitives/
      frame.py                 cached frame-scoped perception runtime
      contracts.py             typed private primitive results
      geometry.py              plane fitting and masked-point helpers
      selection.py             nearest-object selection
      choices.py               deterministic answer-choice resolution
    ground_truth_generator.py  deterministic constrained recipe runner
    oracle_tools.py            opaque-ID and LangChain adapter for primitives
    oracle.py                  bounded tool-calling and evidence validation
    question_agent.py          image-only question author
    dataset.py                 frame records and evaluation export
  evaluation.py                shared point-cloud-vqa Evaluation plugin

dimos/cli/vqa.py              generation CLI commands
```

## Generation Runtime

`dimos vqa generate` and `dimos vqa single-frame` load a frozen Go2 recording frame containing:

- A rectified RGB image.
- A calibrated visible point cloud.
- Camera intrinsics and the point-cloud-to-camera transform.

The generator constructs one `FramePerceptionPrimitives` instance per frame. It owns MoonDream and
EdgeTAM calls, intermediate-result caches, grounded masks, and the accepted ground-plane fit.
Projected visible point-cloud samples establish whether a mask has enough foreground support to
become a grounded object. The generation runtime writes complete frame directories, so multi-frame
generation can skip completed frames after an interrupted run.

### Shared Perception Primitives

Constrained and agentic generation use the same private primitives:

```text
detect_objects(query)
-> segment_detections(detection_id)
-> ground_masks(mask_id)
-> select_nearest_object(object_ids, side)
-> fit_ground_plane()
-> measure_height(object_id, plane_id)
-> bucket_measurement(measurement_id)
```

Constrained generation selects a fixed sequence for each question family. Agentic generation lets
the oracle select a bounded sequence of these tools, passing opaque IDs rather than masks or point
arrays between calls.

## Question And Answer Contracts

The image author sees only RGB and proposes a question plus one of these contracts:

- Boolean: the final public choices are `yes` and `no`.
- Fixed choice: the author supplies at least two public choices.
- Deferred height choice: the author freezes the question, then private geometry deterministically
  creates the public height choices and matching answer after a valid measurement.

The deferred height contract prevents the image author from guessing a numeric range. The private
`height-window-v1` policy derives four local, exhaustive choices around a measured height. The raw
measurement, uncertainty, plane fit, and support remain private.

## Private Validation

Every accepted agentic answer must have valid cited evidence. The deterministic validator checks:

- Cited IDs were emitted by private tools for the current frame.
- The answer is allowed by the resolved public contract.
- A deferred height answer exactly matches the cited measurement bucket.

A private semantic validator then determines whether the cited structured evidence supports the
question and answer. Invalid calls, inadequate geometry, bad citations, and unsupported answers
are retained as private rejected records rather than exported as evaluation cases.

## Dataset Assembly

Each completed frame retains the public `image.jpg`, resumability metadata, private generation
audit, and temporary per-frame case/label rows. `write_dataset_manifest()` aggregates those rows
into root `cases.jsonl` and `labels.jsonl` files.

The root dataset is deliberately minimal:

```text
cases.jsonl     public case ID, image path, question, and choices
labels.jsonl    private case ID and expected choice
frame-*/image.jpg
```

`ground_truth.json` is not an evaluator dependency. It exists solely to make generated labels
auditable.

## Evaluation Runtime

`point-cloud-vqa` is registered in the shared evaluation framework and is invoked through:

```bash
dimos eval run <specification.json> --output <directory>
```

For each case, the evaluator loads the referenced public image, sends the question and choices to
the configured vision model, normalizes an `ANSWER: <choice>` response, and compares it with the
private label. It writes `vqa-results.json` with the expected answer, normalized model answer, raw
model response, and pass/fail result. The shared evaluation runtime also writes its immutable
`run.json` record.

## Operational Dependencies

Generation requires local CUDA support for EdgeTAM and credentials for the image author and, in
agentic mode, the private oracle and semantic validator. Evaluation requires only the selected
vision-model credentials and the exported public images/cases plus private labels; it has no
perception-model, CUDA, recording, or point-cloud dependency.

## Future Work

Evaluation visualization is intentionally separate from generation. The planned report should be
derived from evaluator inputs and `vqa-results.json` only, so it can show public images, questions,
choices, predictions, labels, and pass/fail results without exposing private generation evidence.
