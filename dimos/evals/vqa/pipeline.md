# VQA Pipeline

The first implementation target is one presence question generated from one recorded image and
exported as a standalone evaluation dataset. See [Infrastructure](infrastructure.md) for component
boundaries.

## Generate Command

```bash
dimos evals vqa generate <memory-dataset> \
  --image-index 0 \
  --output <dataset-directory>
```

The generation sequence is:

1. Open the recording with the shared Memory dataset loader.
2. Select one observation from the `color_image` stream by index.
3. Give the image and available family specifications to the question author.
4. Parse the author response into constrained `QuestionProposal` values.
5. Dispatch each proposal to its named deterministic family.
6. Let the family request evidence through its primitive interfaces.
7. Convert each `FamilyAnswer` into a `PublicCase` and `PrivateLabel`.
8. Copy the image and write the standalone dataset artifacts.

Only the image is available to the question author. Primitive evidence and derived answers remain
private.

## Presence Family

The initial proposal is:

```json
{
  "family": "presence",
  "object_name": "chair"
}
```

The family recipe is:

```text
object_name
-> render "Does the image contain any <object_name>?"
-> choices (yes, no)
-> request object detections from ObjectDetector
-> no detections: reject the proposal
-> one or more detections: answer yes
-> return FamilyAnswer
```

The question author does not produce the question text, choices, or answer. The primitive returns
evidence and does not decide how that evidence maps to a public answer.

True-negative authoring and evidence policy are intentionally deferred until the positive presence
path works end to end.

## Dataset Artifacts

The standalone dataset retains the earlier public/private split:

```text
dataset/
  cases.jsonl
  labels.jsonl
  assets/
    frame-000000.jpg
  audit/
    run.json
    frame-000000/
      frame.json
      ground_truth.json
      cases.json
      labels.json
```

A public case has this shape:

```json
{
  "id": "frame-000000-chair-presence",
  "image": "assets/frame-000000.jpg",
  "question": "Does the image contain any chair?",
  "choices": ["yes", "no"]
}
```

The matching private label is:

```json
{
  "id": "frame-000000-chair-presence",
  "answer": "yes"
}
```

`ground_truth.json` retains the proposal, selected answer, object query, detection count, and detected
boxes. `frame.json` records the source image index and timestamp, while `run.json` records the source
Memory dataset and enabled families.

## Run Command

```bash
dimos evals vqa run <dataset-directory> --model <model>
```

The evaluation sequence is:

1. Load and validate `cases.jsonl` and `labels.jsonl`.
2. Resolve each public image under the dataset directory.
3. Ask the selected vision model the public question with its fixed choices.
4. Parse the response as one of those choices.
5. Compare it with the private label through the shared evaluation runner.
6. Write standard evaluation results and summary artifacts.

## Next Steps

1. Run the command against one real recording and inspect its authored proposal and Moondream boxes.
2. Evaluate that generated dataset with one vision model.
3. Define a true-negative evidence policy before balancing presence labels.
4. Add more images, families, and operational behavior only after the first path is understood.
