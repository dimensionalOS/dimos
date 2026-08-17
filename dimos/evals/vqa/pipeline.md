# VQA Pipeline

The current implementation generates constrained questions from selected recorded images and exports
them as a standalone evaluation dataset. See [Infrastructure](infrastructure.md) for component
boundaries.

## Generate Command

```bash
dimos evals vqa generate <memory-dataset> \
  --image-index 0
```

Range mode uses Python-style exclusive stop semantics:

```bash
dimos evals vqa generate <memory-dataset> \
  --start 0 \
  --stop 100 \
  --stride 10
```

`image_index` cannot be combined with range options. Range mode requires `start` and `stop`; `stride`
defaults to one. Generated datasets default to
`~/.local/state/dimos/datasets/vqa/<recording-stem>-frames`; `--output` overrides that location.

The generation sequence is:

1. Open the recording with the shared Memory dataset loader.
2. Normalize single-frame or range selection into ordered frame indices.
3. Reuse one question author and one primitive implementation across selected frames.
4. Give each image and the available family specifications to the question author.
5. Parse each author response into constrained `QuestionProposal` values.
6. Dispatch each proposal to its named deterministic family.
7. Let the family request evidence through its primitive interfaces.
8. Retain unsupported proposals as private rejections without exporting them.
9. Convert each valid `FamilyAnswer` into a `PublicCase` and `PrivateLabel`.
10. Store each image once, write per-frame audit, and aggregate root cases and labels.

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

## Horizontal Direction Family

The proposal is:

```json
{
  "family": "horizontal_direction",
  "object_name": "robot"
}
```

The family recipe is:

```text
object_name
-> request object detections from ObjectDetector
-> require exactly one detection
-> compute the bounding-box center as a fraction of image width
-> [0, 1/3): left
-> [1/3, 2/3): center
-> [2/3, 1]: right
-> render "Which horizontal region contains the visible <object_name>?"
-> choices (left, center, right)
-> return FamilyAnswer
```

Zero detections cannot establish a direction, while multiple detections make the referenced instance
ambiguous. Both cases reject the proposal.

## Object Count Family

The proposal is:

```json
{
  "family": "object_count",
  "object_name": "box"
}
```

The family counts Moondream detections and maps them to the fixed choices `one`, `two`, `three`, and
`four or more`. Zero detections reject the proposal rather than publishing a count. The final bucket
avoids claiming an exact count when the detector reaches its object limit.

## Dataset Artifacts

The standalone dataset retains the earlier public/private split:

```text
dataset/
  cases.jsonl
  labels.jsonl
  assets/
    frame-000000.jpg
    frame-000010.jpg
  audit/
    run.json
    frame-000000/
      frame.json
      ground_truth.json
      cases.json
      labels.json
    frame-000010/
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

`ground_truth.json` retains answered proposals and rejected proposals with their failure reasons.
Answered rows include the selected answer, object query, detection count, and detected boxes.
`frame.json` records each source image index and timestamp, while `run.json` records the source Memory
dataset, selection mode, enabled families, frame count, and aggregate question counts.

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

1. Define a true-negative evidence policy before balancing presence labels.
2. Add segmentation and geometry only when the next family requires them.
