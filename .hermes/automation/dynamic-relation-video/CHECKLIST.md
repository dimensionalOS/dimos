# Dynamic Robot-Relationship Video — Implementation Checklist

## Goal

Generate a playable annotated MP4 from an input video. Once per second, detect the robot and other objects, update robot-to-object `left/right` and `above/below` relationships, and render boxes plus a relationship panel over the full-rate output video.

## Fixed scope

- Separate branch/worktree: `feat/dynamic-robot-relation-video` at `/Users/tian/dimos-worktrees/dynamic-robot-relation-video`.
- Do not modify PR #2989 or its worktree.
- Reuse `ObservationDetector`, `DetectedObject`, and `YoloeObservationDetector`.
- Default robot label: `quadruped robot`; select the highest-confidence matching detection.
- Every other detected object participates; no importance model, question generation, bundles, scoring, replay, or new benchmark schema.
- Refresh detections and relationships once per second; retain the latest overlay between refreshes so the output keeps the source frame rate.
- Use box centers for deterministic horizontal and vertical labels.
- If the robot is absent, render `Robot not detected`; if no other objects exist, render `No other objects detected`.
- Never interpolate motion or claim 3D/world-coordinate relationships.

## Step status

| Step | Behavior | Status |
|---|---|---|
| 00 | Baseline and control-plane verification | COMPLETE |
| 01 | Pure robot-to-object relationship snapshots | PENDING |
| 02 | Full-rate annotated MP4 renderer with one-second refresh | PENDING |
| 03 | CLI, path safety, decode/encode verification | PENDING |
| 04 | Real YOLO-E reference-video acceptance | PENDING |
| 05 | README, focused review, final gates, and PR preparation | PENDING |

## Micro-spec 01 — Relationship snapshots

**Files:** create `relationship_video.py` and `test_relationship_video.py`.

- [ ] RED: highest-confidence configured robot is selected.
- [ ] RED: each non-robot object yields horizontal and vertical relationships from robot to object.
- [ ] RED: missing robot and no-other-object states are explicit.
- [ ] RED: ordering is deterministic by object ID/label.
- [ ] GREEN: add immutable snapshot/relationship records and pure computation.
- [ ] Run focused pytest, Ruff, and mypy for touched files.
- [ ] Focused read-only review.
- [ ] Update progress, commit `feat(benchmark): derive robot relationship snapshots`, push, verify SHA.

## Micro-spec 02 — Annotated MP4 renderer

**Files:** `relationship_video.py`, `test_relationship_video.py`.

- [ ] RED: fake-detector fixture proves refreshes occur at 0s, 1s, 2s rather than every frame.
- [ ] RED: output preserves source FPS, dimensions, and decoded frame count.
- [ ] RED: overlays persist between refreshes and change at the next refresh.
- [ ] GREEN: decode sequentially, call detector once per second, draw boxes/labels/panel, encode MP4.
- [ ] Always close capture, writer, and detector on success/failure.
- [ ] Decode the generated artifact to verify exact frame count.
- [ ] Focused tests/static checks/review.
- [ ] Update progress, commit `feat(benchmark): render dynamic relationship video`, push, verify SHA.

## Micro-spec 03 — CLI and safety

**Files:** `relationship_video.py`, `test_relationship_video.py`, optionally `__main__` wiring only if needed.

- [ ] RED: CLI accepts input, output, robot label, update period, and prompts.
- [ ] RED: reject source/output aliases, symlink output, invalid periods, unreadable video, and writer failure.
- [ ] RED: failed runs leave no partial final output.
- [ ] GREEN: atomic temporary output and clear summary output.
- [ ] Command: `uv run python -m dimos.benchmark.spatiotemporal.relationship_video ...`.
- [ ] Focused tests/static checks/review.
- [ ] Update progress, commit `feat(benchmark): add relationship video CLI`, push, verify SHA.

## Micro-spec 04 — Real YOLO-E acceptance

**Machine-local inputs:**

- Source video: `/Users/tian/dimos-worktrees/replayable-video-relation-evals/assets/simple_demo.mp4`
- Model directory: `/Users/tian/dimos-worktrees/replayable-video-relation-evals/data/models_yoloe`
- Output root: `/Users/tian/dimos-worktrees/dynamic-robot-relation-video/.artifacts/dynamic-robot-relations`

- [ ] Run the real detector with prompts including `quadruped robot` and `refrigerator`.
- [ ] Produce `robot-relationships.mp4` outside tracked paths.
- [ ] Verify the artifact is a real MP4, decodes every expected frame, retains source FPS/dimensions, and has multiple relationship refreshes.
- [ ] Inspect at least three frames at different seconds and confirm overlay changes or explicit missing-robot state.
- [ ] Record exact artifact path, byte size, frame count, duration, and observed labels in progress.
- [ ] Fix only evidence-backed defects using RED-GREEN TDD.
- [ ] Update progress, commit any necessary focused implementation fix, push, verify SHA.

## Micro-spec 05 — Documentation and final handoff

**Files:** package `README.md`, tests only if documentation claims need executable gates.

- [ ] Add one concise command and explain 2D/one-second semantics.
- [ ] Add generated-video output path and honest limitations.
- [ ] Run all `dimos/benchmark/spatiotemporal` tests, Ruff format/check, and package mypy.
- [ ] Independent read-only review for correctness, safety, and overclaims.
- [ ] Confirm final diff does not contain source videos, generated MP4s, model weights, `.venv`, logs, or secrets.
- [ ] Mark `AUTOMATION_STATUS: COMPLETE`, commit `docs(benchmark): document dynamic relationship video`, push, verify SHA.
- [ ] Do not open an upstream PR while #2989 is unmerged; report branch ready for clean follow-up PR preparation.

## Baseline evidence

- Base/head: `a4fb14069c67c0dd7b83ef17c927356bf9183e4b`.
- Fork push dry-run: passed.
- Existing package suite: `127 passed in 3.45s`.
