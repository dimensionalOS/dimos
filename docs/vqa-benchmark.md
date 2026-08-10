---
title: "VQA Benchmark"
---

# Point-Cloud-Grounded VQA Benchmark

DimOS generates image-question-multiple-choice VQA cases from frozen robot recordings. Private point-cloud tools establish and validate answers during generation; evaluation sees only public images, questions, choices, and private answer labels.

## Pipeline Flow

```text
Frozen recording -> rectified RGB image + calibrated visible point cloud
        |
        +-- constrained: image object author -> deterministic question families
        |                  -> private grounding -> quality-gate validation
        |
        +-- agentic: image question author -> frozen choice question
                           -> private local oracle tools -> validation
        |
        v
accepted cases.jsonl + private labels.jsonl
        |
        v
point-cloud-vqa Evaluation -> image-only vision model -> exact choice scoring
```

Rejected questions and private evidence remain in each frame's generation record. The evaluator does not load point clouds, calibration, tool traces, overlays, or rejection records.

## Generation Modes

Constrained generation expands visible objects into fixed choice-question families: presence (`yes`/`no`), horizontal direction, distance threshold, and nearest left-versus-right comparison. Additional deterministic question families will be added as their evidence programs mature.

Agentic generation freezes a free-form image-authored question, then a private oracle uses read-only local tools to establish its answer. Height questions use the fixed choices `under 0.5 m`, `0.5-1.0 m`, `1.0-1.5 m`, and `over 1.5 m`; the private tool measures height and maps it deterministically to one choice. Additional oracle tools will be added as new evidence capabilities mature.

The generation models are code-level defaults, not CLI settings. Constrained mode uses the image author when no `--query` is provided; supplying one or more `--query` values selects the deterministic families for those objects.

Geometry quality gates are the private validation step. They reject insufficient point support, ambiguous masks, unreliable ground planes, and incomplete height evidence before a case becomes public.

## Dataset Format

The root evaluation export follows the common image-question-choice benchmark pattern:

```json
{"id":"go2-40-chair-height","image":"frame-000040/image.jpg","question":"How tall is the chair?","choices":["under 0.5 m","0.5-1.0 m","1.0-1.5 m","over 1.5 m"]}
```

`cases.jsonl` contains public rows only. `labels.jsonl` contains the matching private `id` and `answer`. Frame directories retain `ground_truth.json`, which includes the full private generation audit data.

## Generate

```bash
OPENAI_API_KEY="$OPENAI_API_KEY" dimos vqa generate \
  --recording go2_short \
  --start-index 0 --stop-index 100 --stride 20 \
  --question-mode constrained \
  --output ~/.local/state/dimos/datasets/vqa/go2-short
```

## Evaluate

Create an Evaluation Run Specification next to the generated dataset:

```json
{
  "evaluation": {
    "name": "point-cloud-vqa",
    "config": {
      "dataset": "./go2-short",
      "model": "gpt-4o-mini"
    }
  }
}
```

Then run the shared evaluator:

```bash
OPENAI_API_KEY="$OPENAI_API_KEY" dimos eval run vqa-run.json --output /tmp/vqa-evaluation
```

The evaluation writes the shared immutable `run.json` and a VQA-native `vqa-results.json` artifact containing each model response, normalized answer, expected answer, and pass/fail result.
