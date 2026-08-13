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
                +-- public: assets/, cases.jsonl
                +-- private: labels.jsonl, audit/
                |
                v
       image-only evaluation runtime
                |
                v
       audit/run.json + vqa-results.json
```

## Package Layout

```text
dimos/benchmark/vqa/
  models.py                    shared immutable VQA contracts
  generation/
    primitives/
      frame.py                 cached frame-scoped perception runtime
      contracts.py             typed private primitive results
      projection.py            calibrated point-cloud-to-image projection
      grounding.py             mask-to-point-cloud object grounding
      geometry.py              plane fitting and masked-point helpers
      selection.py             nearest-object selection
      choices.py               deterministic answer-choice resolution
    deterministic_question_answerer.py  constrained family dispatch
    family_context.py          shared frame state and grounding workflow
    family_common.py           shared deterministic result construction
    families.py                end-to-end deterministic family recipes and answer policy
    oracle_tools.py            opaque-ID and LangChain adapter for primitives
    oracle.py                  bounded tool-calling and evidence validation
    question_agent.py          image-only question author
    specification.py           validated generation-specification schema
    runner.py                  model lifecycle, frame iteration, and publication
    dataset.py                 frame records and evaluation export
  evaluation.py                shared point-cloud-vqa Evaluation plugin

dimos/cli/vqa.py              dependency-light generation CLI adapter
```

## Generation Runtime

`dimos vqa generate` loads each selected frozen Go2 recording frame containing:

- A rectified RGB image.
- A calibrated visible point cloud.
- Camera intrinsics and the point-cloud-to-camera transform.

The generator accepts either explicit CLI flags or a reproducible JSON specification through
`dimos vqa generate --spec <generation.json>`. It writes the resolved generation request and
aggregate counts to `audit/run.json`. The generator constructs one
`FramePerceptionPrimitives` instance per frame. It owns MoonDream and
EdgeTAM calls, intermediate-result caches, grounded masks, and the accepted ground-plane fit.
Projected visible point-cloud samples establish whether a mask has enough foreground support to
become a grounded object. The generation runtime writes complete frame directories, so multi-frame
generation can skip frames whose `frame.json` completion marker was atomically written last after an
interrupted run. Partial directories without that marker are safely rewritten.

### Shared Perception Primitives

Constrained and agentic generation share the same private geometry runtime. Constrained families select a
fixed recipe; the agentic oracle may inspect intermediate results and choose its own next operation. Handles
are immutable and scoped to one frozen frame.

```text
detect_objects(query)
-> segment_detection(detection_id)
-> ground_mask(mask_id)
-> frame-scoped canonical object_id / pose / supported point set
-> fit_ground_plane()
-> fitted planes and reusable geometric measurements
```

Agentic answers may explicitly reject a question when the available primitive results cannot establish it;
the rejection and missing-evidence reason remain private generation audit data.

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

Each public image is stored once under root `assets/`. The corresponding `audit/frame-*` directory
retains an atomically published `frame.json` completion marker, private generation audit, and
resumable per-frame case/label rows. `write_dataset_manifest()` aggregates those rows into root
`cases.jsonl` and `labels.jsonl` files.

The root dataset is deliberately minimal:

```text
cases.jsonl     public case ID, image path, question, and choices
labels.jsonl    private case ID and expected choice
assets/frame-*.jpg
audit/frame-*/frame.json
audit/frame-*/ground_truth.json
```

`ground_truth.json` is not an evaluator dependency. It exists solely to make generated labels
auditable.

## Evaluation Runtime

`point-cloud-vqa` is registered in the shared evaluation framework and is invoked through:

```bash skip
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
