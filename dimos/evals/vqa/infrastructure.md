# VQA Infrastructure

This package generates standalone visual question-answering datasets from recorded Memory images and
evaluates those datasets through the shared DimOS evaluation runner. See [Pipeline](pipeline.md) for
the execution sequence.

## Package Layout

```text
dimos/evals/vqa/
  author.py                 image-only family proposal author
  cli.py                    dimos evals vqa generate/run commands
  generate.py               generation requests, dataset rows, and orchestration
  families.py               deterministic question-family contracts and recipes
  suite.py                  standalone dataset adapter for the evaluation runner
  primitives/
    moondream.py             Moondream object-detection adapter
```

Tests live beside the code they cover.

## Responsibilities

The question author sees one image and the available family specifications. It proposes only the
family inputs needed to construct a question. It does not choose public answer choices and does not
produce an answer. Its model prompt and proposal parsing live in `author.py`.

A family owns the stable behavior for one kind of question:

- Required proposal fields.
- Public question formatting.
- Public answer choices.
- Required primitive evidence.
- The deterministic rule that maps evidence to an answer.

A primitive exposes perception evidence without defining question semantics. Families depend on
primitive interfaces rather than concrete implementations. The initial `MoondreamObjectDetector`
adapts the existing `MoondreamVlModel` to the object-detection interface.

`generate.py` coordinates the components and converts answered questions into public cases and
private labels. It opens the Memory recording once, owns model lifecycle across all selected frames,
and writes the standalone artifact tree. It does not implement family-specific perception or answer
rules.

`suite.py` loads generated artifacts for evaluation. Evaluation receives only the public image,
question, and choices; the expected answer remains private.

## Current Contracts

`QuestionProposal` is the constrained model-authored output. The available families currently share
one `object_name` input:

```json
{
  "family": "presence",
  "object_name": "chair"
}
```

```json
{
  "family": "horizontal_direction",
  "object_name": "robot"
}
```

```json
{
  "family": "object_count",
  "object_name": "box"
}
```

`FamilyAnswer` contains the rendered question, fixed choices, and privately derived answer. It
requires at least two unique choices and requires the answer to match one of them.

`PublicCase` contains the case ID, dataset-relative image path, question, and choices. `PrivateLabel`
contains the matching case ID and expected answer.

## Design Rules

- Question authors select family inputs; families select choices and answers.
- Private answers come from deterministic family rules over primitive evidence.
- Primitive implementations remain replaceable and do not construct questions.
- Generation reads privileged evidence, while evaluation reads only exported public artifacts.
- Add abstractions only when a second concrete use requires them.
- Single-frame and range selection feed the same frame-generation path.

## Initial Scope

The package supports one image selected with `image_index` or a `[start, stop)` range with `stride`
from the `color_image` Memory stream. An `OpenAIVlModel` proposes any number of useful presence,
horizontal-direction, or object-count inputs per image, and
`MoondreamVlModel` supplies private object detections through the primitive adapter. Each valid family
answer becomes one public case. When at least one case is answered, proposals without sufficient
evidence are skipped and retained in their original order in the private frame audit. If none can be
answered, generation fails without publishing a dataset.

The initial author proposes a clearly visible object, so an empty detector result rejects generation
rather than being exported as evidence of absence. Negative presence labels require a separate
evidence policy.

The standalone suite loader validates case and label IDs, allowed answers, and dataset-relative image
paths before constructing VQA cases for the shared `EvalRunner`.

Resume behavior, author retries, true-negative policy, geometric evidence, and further families
remain intentionally deferred.
