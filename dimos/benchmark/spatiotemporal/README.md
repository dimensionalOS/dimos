# Spatiotemporal Video-QA Benchmark

This package provides a deterministic, privacy-separated benchmark for asking spatial and temporal questions about tracked objects in video. It includes strict data contracts, real OpenCV decoding, real YOLO-E detection/tracking, deterministic relation and question generation, public/private bundle separation, replay, exact scoring, a TemporalMemory candidate adapter, and a verified end-to-end demo.

The benchmark is designed so a candidate sees only the source video and answer-free public questions. Teacher observations, accepted relations, intervals, oracle answers, and evidence remain private until evaluation.

## What is real, and what is scripted

The end-to-end demo uses:

- a real LFS-backed video;
- real OpenCV decoding and frame sampling;
- real YOLO-E 11s weights and inference;
- real persistent tracker IDs;
- real relation, interval, question, bundle, replay, and scoring code; and
- the real TemporalMemory accumulator, state, entity graph, query lifecycle, readiness, and cleanup paths.

The demo's TemporalMemory VLM fixture is intentionally timestamp-scripted and its final yes/no baseline is a stable hash of the public question. It does **not** inspect image pixels and is **not** presented as a visual QA model. Its purpose is to verify TemporalMemory plumbing and the public/private boundary without requiring cloud credentials. The YOLO-E teacher path is the visually grounded part of the reference demo.

## Architecture

```text
LFS video
   |
   v
OpenCVVideoSampler -----> YoloeObservationDetector -----> canonical ObjectObservation JSONL
                                                            |
                                                            v
                                             spatial RelationFact records
                                                            |
                                                            v
                                             strict RelationInterval records
                                                            |
                                     +----------------------+----------------------+
                                     |                                             |
                                     v                                             v
                              public Question records                      private OracleAnswer records
                                     |                                             |
                                     v                                             v
                              public bundle manifest <---- cryptographic bind ---- oracle manifest
                                     |
                                     v
                    CandidateAnswerer(video + public questions only)
                                     |
                                     v
                                Prediction records
                                     |
                                     +---------------- evaluator only ----------------+
                                                                                       |
                                                                                       v
                                                                           exact EvaluationReport
```

Teacher and candidate execution are deliberately separate:

- `_run_temporal_memory_candidate(...)` accepts only the video, public `Question` records, an ephemeral candidate directory, and duration.
- `_score_candidate(...)` runs in the evaluator boundary and owns `OracleAnswer` records.
- The candidate function never receives an `EvaluationBundle`, observations, facts, intervals, evidence, or oracle answers.

## Package map

| File | Purpose |
|---|---|
| `models.py` | Strict frozen Pydantic v2 records and closed predicate/status enums |
| `utilities.py` | Canonical JSON, schema version, and stable SHA-256 identifiers |
| `relations.py` | Image-plane `left-of`, `right-of`, `above`, and `below` derivation |
| `intervals.py` | Coalescing accepted relation facts into bounded intervals |
| `generation.py` | Spatial and strict temporal question generation |
| `bundles.py` | Canonical public/private artifacts, manifests, loading, and validation |
| `observation_io.py` | Strict canonical observation JSONL read/write |
| `replay.py` | Deterministic observation-to-bundle replay |
| `ports.py` | Frozen detector, generator, replay, and candidate seams |
| `video_adapter.py` | Deterministic OpenCV video sampling |
| `yoloe_adapter.py` | YOLO-E normalization and identity continuity statistics |
| `temporal_memory_answerer.py` | Public-only TemporalMemory candidate lifecycle adapter |
| `runner.py` | Candidate answer parsing and aggregate report construction |
| `scoring.py` | Exact Boolean scoring and evidence-linked diagnostics |
| `demo.py` | One-command real video/YOLO-E/TemporalMemory plumbing demo |
| `test_*.py` | Contract, privacy, determinism, safety, and integration tests |

## Frozen contracts and invariants

### Record strictness

Every benchmark record derives from `StrictFrozenModel`:

- Pydantic v2 strict mode;
- `frozen=True`;
- `extra="forbid"`;
- no implicit type coercion;
- finite numeric values;
- normalized, non-empty strings; and
- deterministic validation of stable identifiers.

Normalized bounding boxes use `[0, 1]` coordinates and must have strictly increasing minima/maxima.

### Supported predicates

Spatial predicates are closed to:

- `left-of`
- `right-of`
- `above`
- `below`

Temporal predicates are closed to:

- `before`
- `after`

Temporal comparisons are strict. Touching or overlapping intervals are neither strictly before nor strictly after.

### Stable identifiers

Question, relation, interval, and bundle IDs are SHA-256-derived stable IDs. Their preimages include the relevant schema version and semantic fields. Question IDs include the episode identity so equivalent questions from different episodes cannot collide.

Do not change stable-ID preimages, enum values, or schema version casually. Such a change is a benchmark schema migration and requires explicit compatibility analysis and regenerated fixtures.

### Relation and interval rules

- Relation subjects and objects must differ.
- Fact evidence cites exactly the fact's sampled frame.
- Evidence IDs are ordered and unique.
- An interval represents one stable relation ID.
- Frame and timestamp endpoints must agree.
- A single-frame interval must also have identical timestamps; a multi-frame interval must span increasing timestamps.
- Facts and intervals are private teacher artifacts.

### Question and oracle rules

- Public questions contain no expected answer or teacher evidence.
- Spatial public questions refer to object IDs and one spatial predicate.
- Temporal public questions refer to stable relation IDs, not private interval payloads.
- Oracle answers carry the expected Boolean and private evidence frame/interval IDs.
- Candidate predictions accept only exact booleans or the exact lowercase strings `yes` and `no`.
- Missing, invalid, correct, and incorrect are distinct scoring statuses.

### Bundle privacy

The public bundle contains only release metadata and answer-free public questions. The private oracle bundle contains observations, relation facts, intervals, answers, and evidence. Both manifests share one `bundle_id`; the oracle manifest is cryptographically bound to the exact public manifest SHA-256.

Logical replay hashing is root-independent: writing the same logical bundle under different directories must produce the same manifests and logical digest.

## Prerequisites

Run from the repository root with the project environment available through `uv`.

### 1. Git LFS

Install Git LFS, initialize it for this repository, and materialize the video and YOLO-E archive:

```bash
git lfs version
git lfs install --local
git lfs pull --include='assets/simple_demo.mp4,data/.lfs/models_yoloe.tar.gz'
```

Verify that the files are binaries rather than ~130-byte pointer files:

```bash
python3 - <<'PY'
from pathlib import Path
for name in ("assets/simple_demo.mp4", "data/.lfs/models_yoloe.tar.gz"):
    path = Path(name)
    print(name, path.stat().st_size, "bytes")
    assert path.stat().st_size > 1024, f"{name} is still an LFS pointer"
PY
```

Reference sizes are approximately 51 MB for `simple_demo.mp4` and 185 MB for `models_yoloe.tar.gz`.

### 2. Extract YOLO-E weights

```bash
uv run python -c "from dimos.utils.data import get_data; print(get_data('models_yoloe'))"
```

The reference prompt model is:

```text
data/models_yoloe/yoloe-11s-seg.pt
```

### 3. Install Ultralytics CLIP

YOLO-E text prompting requires Ultralytics' CLIP fork:

```bash
uv pip install 'git+https://github.com/ultralytics/CLIP.git'
```

The first prompt-mode run may also download `mobileclip_blt.ts` into the Ultralytics user cache.

## One-command demo

```bash
uv run python -m dimos.benchmark.spatiotemporal.demo \
  --source-video assets/simple_demo.mp4 \
  --output-root .artifacts/spatiotemporal-video-qa \
  --duration-s 25 \
  --frame-stride 150
```

Constraints:

- duration must be between 15 and 30 seconds;
- frame stride must be positive;
- the source must be a regular non-symlink file;
- the output path and fixed artifact children must not contain symlinks;
- source and target must not alias by path or hard link; and
- encoded output is reopened and every expected frame is decoded before inference begins.

The reference command uses CPU explicitly for reproducibility.

## Output layout

```text
.artifacts/spatiotemporal-video-qa/
├── office_robot_25s.mp4
├── observations.jsonl
├── summary.json
├── bundle-a/
│   ├── public/
│   └── oracle/
├── bundle-b/
│   ├── public/
│   └── oracle/
```

`bundle-a` and `bundle-b` are independent writes of the same logical teacher data. Their public and oracle manifests must compare equal.
TemporalMemory state is written under an ephemeral directory outside this teacher output and removed after candidate execution.

## What the demo verifies

The command fails unless all of these gates hold:

1. the constrained output video contains exactly the expected number of decodable frames;
2. two fresh YOLO-E detector/tracker instances produce identical observations and statistics;
3. canonical observations can be written and strictly re-read;
4. observations produce accepted spatial facts and intervals;
5. the teacher generates at least one public question;
6. two independent bundle roots have equal public and oracle manifests;
7. replay logical SHA-256 values match across roots;
8. TemporalMemory ingests public frames and reports ready;
9. all public questions receive a valid candidate response;
10. scoring joins every public question to exactly one private oracle answer; and
11. TemporalMemory and detector resources are closed.

## Reference result

Schema v2 makes question IDs episode-scoped, so v1 artifact and summary hashes are intentionally incompatible. Run the one-command demo to generate a fresh local reference; the final acceptance section records counts and digests only after all schema-v2 feature work is complete.

## Verification commands

### Contract and integration tests

```bash
uv run pytest dimos/benchmark/spatiotemporal -q
```

Expected result for this curated import: 105 tests pass. This count grows with the focused TDD changes below.

### Formatting, lint, and typing

```bash
uv run ruff format --check dimos/benchmark/spatiotemporal
uv run ruff check dimos/benchmark/spatiotemporal
uv run mypy
```

### Inspect the machine-readable result

```bash
python3 - <<'PY'
import json
from pathlib import Path
s = json.loads(Path('.artifacts/spatiotemporal-video-qa/summary.json').read_text())
assert s['video']['duration_s'] == 25
assert s['video']['frames'] == 750
assert s['teacher']['detector_repeat_equal'] is True
assert len(s['teacher']['unique_object_ids']) >= 3
assert s['teacher']['relation_facts'] > 0
assert s['teacher']['relation_intervals'] > 0
assert s['teacher']['spatial_questions'] > 0
assert s['teacher']['temporal_questions'] > 0
assert s['candidate']['candidate_used_oracle'] is False
assert s['candidate']['readiness']['ready'] is True
assert s['candidate']['readiness']['ingested_frame_count'] == 25
assert s['candidate']['questions_answered'] == s['teacher']['questions']
assert s['candidate']['status_counts']['missing'] == 0
assert s['candidate']['status_counts']['invalid'] == 0
print('SUMMARY_GATES=PASS')
PY
```

### Verify complete-run repeatability

```bash
cp .artifacts/spatiotemporal-video-qa/summary.json /tmp/stqa-summary-first.json
uv run python -m dimos.benchmark.spatiotemporal.demo \
  --source-video assets/simple_demo.mp4 \
  --output-root .artifacts/spatiotemporal-video-qa \
  --duration-s 25 \
  --frame-stride 150
cmp /tmp/stqa-summary-first.json .artifacts/spatiotemporal-video-qa/summary.json
shasum -a 256 .artifacts/spatiotemporal-video-qa/summary.json
```

`cmp` must produce no output and exit zero.

## Safety and privacy checks

The test suite covers:

- candidate function signature excludes private bundles;
- candidate answers are stable when irrelevant context timestamps change;
- invalid duration and stride fail before output side effects;
- symlinked output roots and artifact children are rejected;
- source/target path aliases are rejected;
- source/target hard-link aliases are rejected without modifying the source;
- silent `VideoWriter.write()` failure is detected by full decode verification;
- duplicate fallback detector labels are rejected;
- observations reject duplicate JSON keys and noncanonical JSONL;
- public manifests cannot reference private artifacts;
- oracle manifests bind to one exact public manifest; and
- questions, answers, and evidence IDs must join exactly.

Do not weaken these checks to make a custom video pass. Improve the input, prompts, detector settings, or explicitly version the benchmark contract.

## Using a different video

A useful custom video should provide:

- a valid finite FPS;
- 15-30 seconds of decodable frames;
- at least three reliably tracked native identities;
- spatially separated object pairs; and
- at least one change that creates non-overlapping relation intervals for temporal questions.

The default prompt set is defined in `demo.py`. Multiple untracked detections with the same fallback label are intentionally rejected because label-only IDs would be ambiguous. Prefer native tracker IDs or a prompt set that yields unique fallback labels.

The demo episode ID is currently fixed for the reference clip. If generalizing the CLI for arbitrary datasets, add an explicit episode argument and preserve the stable-ID contract.

## Development and integration guidance

This benchmark was developed as isolated lanes and should continue to use the same discipline for contract-sensitive changes:

- one branch and worktree per independent lane;
- the main worktree is reserved for controlled integration;
- shared contracts are frozen before parallel implementation;
- lane ownership guards prevent edits outside assigned paths;
- progress is tracked in machine-readable lane ledgers;
- workers commit and push small independently testable units;
- only integration updates the global board, resolves conflicts, or runs private/real gates;
- do not merge a lane until its focused tests, full package tests, Ruff, mypy, and remote-head verification pass;
- keep candidate-facing code physically and structurally separated from oracle data; and
- never use oracle answers to tune or generate candidate predictions during a scored run.

The completed lane and gate record is in:

```text
.hermes/plans/spatiotemporal-video-qa-parallel/board.md
```

## Independent verification agent runbook

A verifier must operate read-only and treat this README as the procedure under test.

### Agent rules

1. Start from `/Users/tian/dimos` on `feat/spatiotemporal-video-qa`.
2. Do not edit, format, commit, reset, clean, or push.
3. Do not read `.env`, credentials, or private user files.
4. Confirm the Git worktree is clean before verification.
5. Follow the prerequisites and one-command demo in this README.
6. Run the package tests, Ruff checks, and mypy.
7. Run the summary assertions and repeatability check.
8. Inspect code/tests for each advertised privacy and safety invariant.
9. Distinguish real YOLO-E inference from the timestamp-scripted TemporalMemory VLM fixture.
10. Return `PASS` only if every applicable gate passes; otherwise return `FAIL` with exact command, exit code, and evidence.

### Required report

```text
VERDICT: PASS|FAIL
BRANCH: <branch>
HEAD: <sha>
WORKTREE_CLEAN: yes|no
LFS_ASSETS: PASS|FAIL
YOLOE_WEIGHTS: PASS|FAIL
PYTEST: <passed/failed>
RUFF_FORMAT: PASS|FAIL
RUFF_CHECK: PASS|FAIL
MYPY: PASS|FAIL
REAL_DEMO: PASS|FAIL
DETECTOR_REPEAT: PASS|FAIL
BUNDLE_REPEAT: PASS|FAIL
FULL_RUN_REPEAT: PASS|FAIL
TEMPORAL_MEMORY_READY: PASS|FAIL
PRIVACY_BOUNDARY: PASS|FAIL
PATH_SAFETY: PASS|FAIL
SUMMARY_SHA256: <sha>
NOTES: <concise evidence and caveats>
```

## Troubleshooting

### `moov atom not found`

The video is probably still an LFS pointer. Check its size and run the Git LFS pull command again.

### `ModuleNotFoundError: No module named 'clip'`

Install the Ultralytics CLIP fork with the command in Prerequisites.

### MobileCLIP download starts on first run

This is expected for YOLO-E text prompting. The file is large and cached after download.

### `duplicate fallback label`

The tracker returned multiple detections of one prompt class without native IDs. The adapter rejects this instead of inventing ambiguous identity. Adjust prompts/confidence or use a video with stronger track continuity.

### `no spatially separated object pairs`

The saved observations did not produce accepted left/right/above/below relations. Inspect detections, normalized boxes, prompts, sampling stride, and relation margin.

### `accepted relations produced no evaluation questions`

The relation set is valid but insufficient for current question generation. Inspect relation IDs and interval ordering rather than bypassing the error.

### OpenCV AVFoundation duplicate-class warning on macOS

The `cv2` and `av` wheels may both bundle FFmpeg AVFoundation classes. The reference run emits this warning but completes successfully. Treat a crash or decode failure as an error; do not treat the warning alone as failure.

### Slow CPU inference

The reference detector explicitly uses CPU for reproducibility. Increase `--frame-stride` to reduce inference calls for exploratory runs, but do not compare resulting hashes or counts to the reference configuration.

### Output path rejected

The demo refuses symlinked roots/children, non-directory roots, source/target aliases, hard-link aliases, and unexpected directory artifacts. Use a fresh regular directory under the repository, such as `.artifacts/spatiotemporal-video-qa`.

## Current verification record

- Integration branch: `feat/spatiotemporal-video-qa`
- Demo implementation commit: `1b9f7dae68e2023867dc20e90b4e7dfd122a6e14`
- Completion record commit: `c34658992e9bf40780e26cb92de7a02876430a94`
- Package tests: 108 passed
- Ruff: passed
- mypy: passed across 874 source files
- Independent code review: passed
- Real demo: passed twice with byte-identical summary

For a shorter operator-oriented version, see `docs/spatiotemporal-video-qa-demo.md`.
