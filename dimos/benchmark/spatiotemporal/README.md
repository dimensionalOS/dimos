# Replayable Spatiotemporal Video-Relation Evals

This package turns a video into a deterministic evaluation bundle for spatial and temporal relationship questions.

The core contribution is the evaluator, not a new vision model:

- a real video is decoded and sparsely sampled;
- YOLO-E produces teacher pseudo-labels and persistent object identities;
- geometry produces canonical spatial facts and sampled-frame intervals;
- deterministic generation produces answer-free public questions and private oracle evidence;
- the same saved observations replay to byte-stable logical bundles under different output roots;
- a candidate receives only the video and public questions;
- an evaluator joins predictions to private answers and emits exact diagnostics; and
- an evaluator-only HTML viewer makes every pseudo-label claim inspectable.

## 1. This PR: reproducible evaluator foundation

### Interviewer fast path

#### Prerequisites

A fresh checkout needs:

- `git` and `git-lfs`;
- access to the DimOS LFS endpoint for `assets/simple_demo.mp4` and the YOLO-E archive;
- `uv`; and
- `gh` only when using the `gh pr checkout` command below.

The reference path runs on macOS or Linux and supports CPU execution. GitHub repository access does not necessarily grant access to the separately configured DimOS LFS endpoint.

#### Reproduce the reference episode

From a PR checkout, one script materializes assets, installs the project and prompt dependency, runs every focused quality gate, executes the real CPU demo, and prints the result locations:

```bash
gh pr checkout 2989
./dimos/benchmark/spatiotemporal/reproduce_reference.sh
```

If Git reports that the branch is already assigned to another worktree, enter the path shown by Git and run the same script there. If setup and tests already passed, rerun only the real demo with:

```bash
./dimos/benchmark/spatiotemporal/reproduce_reference.sh --skip-setup --skip-tests
```

A successful run prints both:

```text
SUMMARY_GATES=PASS
Reproduction complete.
```

Then open:

```bash
open .artifacts/spatiotemporal-video-qa/evidence-viewer/index.html
```

On Linux, open the same `index.html` in a browser.

#### Try another video

After the initial setup, reuse the pipeline with a different video:

```bash
uv run python -m dimos.benchmark.spatiotemporal.demo \
  --source-video /path/to/video.mp4 \
  --output-root .artifacts/my-video-eval \
  --duration-s 25 \
  --frame-stride 150
```

Use a regular 15–30-second video that OpenCV can decode. Useful episodes contain multiple visible, spatially separated objects and at least one relationship change. Arbitrary videos are not guaranteed to yield accepted relations or questions because YOLO-E labels and tracks remain teacher pseudo-labels.

Open the custom result at:

```text
.artifacts/my-video-eval/evidence-viewer/index.html
```

#### Render a dynamic robot-relationship video

After materializing the LFS source video and YOLO-E model described below, render a playable annotated MP4 with:

```bash
mkdir -p .artifacts/dynamic-robot-relations
uv run python -m dimos.benchmark.spatiotemporal.relationship_video \
  assets/simple_demo.mp4 \
  .artifacts/dynamic-robot-relations/robot-relationships.mp4 \
  --robot-label 'quadruped robot' \
  --update-period 1
```

The command detects the highest-confidence configured robot and other prompted objects once per second, derives deterministic `left-of`/`right-of` and `above`/`below` relations from 2D box centers, and retains that overlay between refreshes so the output preserves the source frame rate. The generated video is `.artifacts/dynamic-robot-relations/robot-relationships.mp4`.

These annotations are image-plane YOLO-E pseudo-labels, not 3D or world-coordinate relationships. They do not interpolate motion, and detections can be missing or mislabeled; the overlay reports `Robot not detected` or `No other objects detected` rather than inventing a relationship.

#### Common setup failures

- **LFS authentication or pointer-sized assets:** obtain access to the configured DimOS LFS endpoint, then rerun `git lfs pull --include='assets/simple_demo.mp4,data/.lfs/models_yoloe.tar.gz'`. The reproduction script rejects either asset if it remains an LFS pointer.
- **No relations or questions from a custom video:** choose a clearer clip or adjust its duration and sampling stride. Inspect detection labels, confidence, identity continuity, and object separation rather than weakening privacy or determinism checks.

A reviewer should establish the feature in this order:

1. **Reproduce:** run the script and confirm the summary gates pass.
2. **See:** inspect the annotated evidence viewer rather than trusting aggregate scores.
3. **Verify privacy:** compare `bundle-a/public/` with `bundle-a/oracle/`.
4. **Verify replay:** confirm `bundle-a` and `bundle-b` have the same logical digest.
5. **Understand scope:** treat YOLO-E as a pseudo-label teacher and the bundled candidate as a plumbing smoke test.

### What the PR proposes

The PR establishes a reusable evaluator contract:

```text
video
  -> sampled observations
  -> canonical relation facts
  -> sample-aware intervals
  -> public questions + private answers/evidence
  -> isolated candidate predictions
  -> exact report + inspectable evidence
```

The package is intentionally divided by responsibility:

```text
contracts       models.py, utilities.py
teacher         video_adapter.py, yoloe_adapter.py, relations.py, intervals.py
generation      generation.py, observation_io.py, replay.py
release         bundles.py, ports.py
candidate       temporal_memory_answerer.py, candidate_worker.py
evaluation      runner.py, scoring.py
review          evidence_viewer.py
acceptance      demo.py, reproduce_reference.sh, test_*.py
```

This PR stops at a deterministic and inspectable evaluator. It does not hide teacher defects, automatically promote generated pseudo-labels to ground truth, or claim that the smoke-candidate score measures visual intelligence.

### Concrete examples from the verified reference video

The examples below are copied from the generated schema-v2 artifacts, not invented documentation fixtures.

#### Spatial positive and negative controls

At sampled frame `0`, the teacher derives:

```json
{
  "frame_id": 0,
  "timestamp_s": 0.0,
  "subject_id": "1",
  "predicate": "left-of",
  "object_id": "2",
  "evidence_frame_ids": [0]
}
```

That one accepted fact produces a public positive question and a private answer:

```json
{"text":"Did 1 ever appear left of 2?","predicate":"left-of","object_ids":["1","2"]}
{"expected":true,"evidence_frame_ids":[0]}
```

It also produces the opposite negative control. The public question remains answer-free; the private oracle cites the complete sample schedule to support episode-level absence:

```json
{"text":"Did 1 ever appear right of 2?","predicate":"right-of","object_ids":["1","2"]}
{"expected":false,"evidence_frame_ids":[0,150,300,450,600]}
```

Across the reference run, 3 accepted canonical spatial facts produce 6 spatial questions: 3 positive and 3 negative.

#### Temporal ordering

The private intervals include:

```text
frame 0 / 0.000 s:   "1 left of 2"
frame 300 / 10.001 s: "1 above 3"
frame 600 / 20.002 s: "1 left of 8"
```

From those intervals, generation emits paired temporal checks. For example:

```json
{"text":"Did \"1 left of 2\" happen before \"1 above 3\"?","predicate":"before"}
{"expected":true,"evidence_interval_ids":["interval@frame-0","interval@frame-300"]}

{"text":"Did \"1 left of 2\" happen after \"1 above 3\"?","predicate":"after"}
{"expected":false,"evidence_interval_ids":["interval@frame-0","interval@frame-300"]}
```

The readable aliases above shorten stable SHA-256 interval IDs only for presentation; generated artifacts contain the full IDs. With 3 unanimously ordered relation pairs, the reference run produces 12 balanced temporal questions: before/after positives and their inverse negative controls.

#### Public/private proof

Candidate-visible `public/questions.jsonl` contains only question semantics:

```json
{
  "episode_id": "office_robot_25s",
  "question_kind": "spatial",
  "predicate": "left-of",
  "object_ids": ["1", "2"],
  "reference_ids": [],
  "text": "Did 1 ever appear left of 2?"
}
```

It does not contain `expected`, `evidence_frame_ids`, `evidence_interval_ids`, observations, facts, or intervals. The parent evaluator retains those fields under `oracle/`; it never serializes them into the reference candidate's explicit input or temporary root, and it uses them for scoring after candidate execution.

### Proof matrix

| Claim | Reproducible artifact | Reference evidence |
|---|---|---|
| A video automatically becomes an eval | `observations.jsonl`, `bundle-a/` | 5 sampled frames → 8 observations → 3 facts → 3 intervals → 18 questions |
| Spatial cases are balanced | public questions + private answers | 6 spatial questions: 3 true, 3 false |
| Temporal cases are strict and balanced | interval and answer JSONL | frames 0, 300, 600 → 12 temporal questions: 6 true, 6 false |
| Reference candidate input excludes answers | candidate subprocess and bundle layout | only copied video + public questions are serialized into candidate root |
| Replay is deterministic | `bundle-a`, `bundle-b`, `summary.json` | equal manifests and logical SHA-256 `07b7b998…eaaf0` |
| Results are inspectable | `evidence-viewer/index.html` | 18 oracle rows linked to 5 annotated evidence frames |
| The smoke score is not overclaimed | `summary.json` | `visually_grounded=false`, `score_interpretation=not_model_quality` |
| Teacher defects remain visible | evidence viewer and README | robot ID changes `2`→`3`, missed detections, oven/cabinet mislabeled `refrigerator` |

The reproduction script verifies generated counts, balanced expected values, absence of answer/evidence fields from public question records, summary gates, and representative examples. This proves the trusted reference candidate's explicit-input boundary, evaluator mechanics, and repeatability. It does not provide an OS security sandbox: an untrusted candidate must run in a container or worker with repository and oracle paths unmounted. It also intentionally does not prove that the current pseudo-label teacher or smoke candidate is production-accurate.

## 2. Further directions: from videos to gated self-improvement

The evaluator can become the data engine for continuous improvement:

```text
new videos
  -> generated challenge episodes
  -> quality and disagreement gates
  -> candidate failure clustering
  -> human review/correction
  -> training-data export
  -> candidate N+1
  -> frozen holdout evaluation
```

The next proposed components are:

1. **Episode admission report:** detector confidence, track continuity, label consistency, relation contradictions, evidence coverage, and replay determinism.
2. **Three-set registry:** a frozen reviewed regression set, a reviewed improvement set, and an unreviewed challenge pool.
3. **Failure miner:** cluster spatial, temporal, identity, label, missing-detection, and long-horizon-memory failures.
4. **Review workflow:** accept, correct, or reject generated episodes using the existing evidence viewer as the starting interface.
5. **Training export:** emit only reviewed examples, with provenance back to video, teacher version, schema, and evidence.
6. **Promotion policy:** replace a candidate only when frozen accuracy improves, critical families do not regress, and privacy, latency, and resource gates pass.

Automatically generated data may enter the challenge pool immediately, but it must not enter a trusted benchmark or training set without explicit quality gates.

## 3. Production path

Productionization should happen in explicit stages:

### Stage A — repeatable local and CI evaluation

- Pin model weights, dependencies, prompts, schema, and reference assets.
- Run package tests and a small reviewed fixture in CI.
- Publish manifests, summary, and viewer as build artifacts.
- Preserve candidate subprocess and public/private filesystem isolation.

### Stage B — versioned episode service

- Store immutable source-video hashes and schema-versioned bundles.
- Register teacher version, prompts, thresholds, and sampling schedule.
- Make generation idempotent by episode key.
- Add retention and access policies for private video and oracle evidence.

### Stage C — quality-gated curation

- Compute automatic admission metrics.
- Route uncertain identity, label, or relation cases to review.
- Record reviewer corrections separately from teacher output.
- Promote only reviewed episodes into frozen evaluation sets.

### Stage D — continuous candidate evaluation

- Evaluate every candidate against the same frozen set.
- Run new videos as a separate challenge stream.
- Track per-family accuracy, missing or invalid answers, latency, memory, and regressions.
- Keep oracle data inaccessible to candidate containers and services.

### Stage E — controlled self-improvement

- Select high-value reviewed failures for training.
- Train or tune a new candidate with complete data provenance.
- Compare against the incumbent on untouched frozen holdouts.
- Require policy approval before promotion and support immediate rollback.

Production readiness therefore means more than deploying the demo: it requires trusted data admission, immutable versioning, observability, privacy controls, candidate promotion gates, and rollback.

# Detailed implementation reference

## What this demonstrates

The reference demo exercises real:

- Git-LFS video input;
- OpenCV decode, trim, encode, reopen, and full-frame verification;
- YOLO-E 11s prompt inference;
- tracker identity normalization;
- spatial relation and interval derivation;
- public/private bundle generation and validation;
- root-independent deterministic replay;
- candidate isolation;
- TemporalMemory ingestion/query/cleanup plumbing; and
- exact scoring and evidence rendering.

The demo does **not** claim that YOLO-E outputs are human ground truth. They are teacher pseudo-labels and must be reviewed in `evidence-viewer/index.html`.

The included TemporalMemory candidate is also **not** a model-quality baseline. It uses a timestamp-scripted VLM fixture and a deterministic public-question hash for yes/no responses. Its score proves that candidate plumbing, isolation, readiness, prediction parsing, and scoring run end to end without cloud credentials. Replace it with a real candidate before drawing model-quality conclusions.

## Evaluation boundary

```text
TEACHER / EVALUATOR                         CANDIDATE

source video                                copied source video
    |                                            |
OpenCV sampling                                  v
    |                                      TemporalMemory smoke fixture
YOLO-E pseudo-labels                             |
    |                                            v
canonical spatial facts                    public predictions
    |
sampled-frame intervals
    |
    +--> public questions ----------------------+
    |
    +--> private oracle answers/evidence
                 |
                 v
          exact evaluator report
```

The candidate function receives only:

1. a copied video in a temporary candidate directory;
2. public `Question` records;
3. that temporary directory; and
4. the public duration.

It never receives an `EvaluationBundle`, observations, facts, intervals, evidence IDs, or expected answers. The candidate directory is outside teacher output and is deleted after execution.

## Semantics

### Spatial facts

Image-plane predicates are closed to:

- `left-of` / `right-of`;
- `above` / `below`.

Replay stores one canonical representation per physical relation:

- horizontal facts use `left-of`;
- vertical facts use `above`.

This prevents inverse duplicates such as `A left-of B` and `B right-of A` from receiving double weight.

Spatial questions use episode-level existential wording, for example:

```text
Did object_1 ever appear left of object_2?
```

For a stable physical relation, generation emits:

- one positive question for the observed canonical relation; and
- one negative control for the opposite predicate only when that opposite relation never occurred in the episode.

If a relation flips during the episode, both observed directions become positives and contradictory negatives are omitted. Negative evidence covers the complete sampled-frame schedule.

### Temporal facts

Facts coalesce when they occur at adjacent **sample positions**, not when raw source frame IDs differ by one. For a stride of 150, frames 0 and 150 are adjacent samples. An intervening sampled frame with no relation breaks the interval, including a zero-detection frame.

Temporal questions are generated only from strict, unanimous ordering proofs. Touching, overlapping, containing, or contradictory intervals produce no temporal claim.

Public temporal text is human-readable:

```text
Did "object_1 left of object_2" happen before "object_3 above object_4"?
```

Stable relation IDs remain in structured `reference_ids`; private interval IDs remain in oracle evidence.

### Stable contracts

All records are strict frozen Pydantic v2 models with `extra="forbid"` and no implicit coercion.

Current schema:

```text
spatiotemporal-video-qa/v2
```

Question IDs include episode identity. Relation, interval, question, and bundle IDs are SHA-256 values over canonical semantic preimages. Changing a preimage, enum value, or schema version is an explicit migration and requires regenerated artifacts.

## Package map

| File | Responsibility |
|---|---|
| `models.py` | Strict records and closed enums |
| `utilities.py` | Canonical JSON and stable IDs |
| `relations.py` | Image-plane geometry |
| `intervals.py` | Sample-schedule-aware interval construction |
| `generation.py` | Balanced spatial and strict temporal cases |
| `observation_io.py` | Canonical observation JSONL |
| `bundles.py` | Public/private artifacts and manifests |
| `replay.py` | Observation-to-bundle replay |
| `video_adapter.py` | OpenCV sampling, including zero-detection samples |
| `yoloe_adapter.py` | YOLO-E normalization and identity statistics |
| `evidence_viewer.py` | Private HTML and annotated evidence frames |
| `temporal_memory_answerer.py` | Public-only candidate lifecycle adapter |
| `runner.py` / `scoring.py` | Parsing, scoring, and diagnostics |
| `demo.py` | Real end-to-end acceptance command |

## Prerequisites

Run from the repository root.

### Materialize LFS assets

```bash
git lfs install --local
git lfs pull --include='assets/simple_demo.mp4,data/.lfs/models_yoloe.tar.gz'
python3 - <<'PY'
from pathlib import Path
for name in ('assets/simple_demo.mp4', 'data/.lfs/models_yoloe.tar.gz'):
    size = Path(name).stat().st_size
    print(name, size)
    assert size > 1024, f'{name} is still an LFS pointer'
PY
```

### Extract weights and install the YOLO-E prompt dependency

```bash
uv run python -c "from dimos.utils.data import get_data; print(get_data('models_yoloe'))"
uv pip install 'git+https://github.com/ultralytics/CLIP.git'
```

The first prompt-mode run may download `mobileclip_blt.ts` into the Ultralytics cache.

## Run the real demo

```bash
uv run python -m dimos.benchmark.spatiotemporal.demo \
  --source-video assets/simple_demo.mp4 \
  --output-root .artifacts/spatiotemporal-video-qa \
  --duration-s 25 \
  --frame-stride 150
```

The command uses CPU for reproducibility. Duration must be 15–30 seconds and stride must be positive.

## Output

```text
.artifacts/spatiotemporal-video-qa/
├── office_robot_25s.mp4
├── observations.jsonl
├── summary.json
├── evidence-viewer/
│   ├── index.html
│   └── frames/frame_*.jpg
├── bundle-a/
│   ├── public/
│   └── oracle/
└── bundle-b/
    ├── public/
    └── oracle/
```

`bundle-a` and `bundle-b` are independent writes of the same logical teacher data. Their manifests and logical digest must match.

The viewer is evaluator-only because it contains expected answers and private evidence. It is generated atomically, escapes untrusted text, rejects symlinked path components, preserves an existing viewer on failure, and never writes into the candidate directory.

## Verified reference run

The schema-v2 acceptance run on macOS CPU completed twice with byte-identical `summary.json` files:

- 750 decoded frames over 25 seconds;
- 5 sampled frames per independent detector run;
- 8 YOLO-E observations with 4 native tracker IDs and no fallback IDs;
- 3 canonical relation facts and 3 sampled-frame intervals;
- 18 questions and answers: 6 spatial and 12 temporal;
- 5 annotated evidence frames;
- detector observations equal across two fresh YOLO-E instances;
- bundle logical SHA-256 `07b7b99844b01badbbdecd65227144cdee7d7ef1e6d6671f2f022bc7498eaaf0`;
- full-summary SHA-256 `9a679fe7b1689d5d988f567c87c8be636b7e35d55079c377f58dda79a0e1948c`; and
- 18/18 valid smoke-baseline predictions with zero missing or invalid responses.

Visual inspection confirmed that the boxes are geometrically aligned where detections exist. It also exposed the intended pseudo-label caveats: the same physical robot changes tracker identity from `2` to `3`, the teacher calls a built-in oven/cabinet column a `refrigerator`, and the visible robot is not detected in every sampled frame. These labels are suitable for demonstrating evaluator mechanics, not for claiming a clean human-annotated benchmark.

## Five-minute reviewer walkthrough

1. Open `evidence-viewer/index.html` and confirm the header metrics match the reference run: 5 evidence frames, 4 track IDs, 3 relation intervals, 6 spatial and 12 temporal questions, and a balanced 9/9 oracle split.
2. Follow **One generated proof chain** from frame → derived relation → answer-free public question → private expected answer and evidence. This is the shortest proof of what the evaluator creates.
3. Read **Robot-motion verification** before claiming motion quality. A trustworthy motion claim requires all four checks:
   - relation events are linked to evidence frames;
   - relation intervals have strict temporal order;
   - the robot keeps one identity across those events; and
   - robot detections cover the evidence schedule sufficiently.
4. For this reference clip, confirm the honest verdict: relation ordering is evaluable, but the full robot-motion claim needs review because the robot changes ID `2` → `3` and appears in only 2/5 evidence frames.
5. Use **What changed, and when** to inspect the sampled sequence: `1 left of 2` at 0.000 s, `1 above 3` at 10.001 s, and `1 left of 8` at 20.002 s. The 5.001 s and 15.002 s gaps remain visible and are not interpolated.
6. Inspect all five annotated frames. Confirm box placement, label semantics, confidence, identity continuity, and the relation chip shown for each accepted event. Click any frame for the full-resolution lightbox.
7. Filter **Questions and private oracle** by spatial, temporal, true, or false. Follow evidence chips back to frames and verify both positive cases and inverse negative controls.
8. Read **Reusable motion-evaluation patterns** to connect the same contract to locomotion/patrol, manipulation, long-horizon memory, and release regression gates.
9. Inspect `bundle-a/public/questions.jsonl`: it must contain no expected answers or evidence. Keep `bundle-a/oracle/answers.jsonl` private.
10. Compare `bundle-a` and `bundle-b` manifests and the logical digest in `summary.json` to verify deterministic replay.
11. Confirm `summary.json` labels the candidate as a non-visual plumbing smoke test.

The viewer intentionally distinguishes **relation-order evidence** from a stronger **continuous robot-motion claim**. The current evaluator can prove sampled spatial predicates and strict before/after ordering. Production trajectory claims additionally need stable identity, higher detection coverage, denser sampling, and task-specific motion predicates or pose data.

## Quality gates

```bash
uv run pytest dimos/benchmark/spatiotemporal -q
uv run ruff format --check dimos/benchmark/spatiotemporal
uv run ruff check dimos/benchmark/spatiotemporal
uv run --group lint mypy dimos/benchmark/spatiotemporal
```

Inspect the real-run summary:

```bash
python3 - <<'PY'
import json
from pathlib import Path
s = json.loads(Path('.artifacts/spatiotemporal-video-qa/summary.json').read_text())
assert s['video']['duration_s'] == 25
assert s['video']['frames'] == 750
assert s['teacher']['detector_repeat_equal'] is True
assert s['teacher']['relation_facts'] > 0
assert s['teacher']['relation_intervals'] > 0
assert s['teacher']['spatial_questions'] > 0
assert s['teacher']['temporal_questions'] > 0
assert s['candidate']['candidate_used_oracle'] is False
assert s['candidate']['baseline_kind'] == 'public_only_plumbing_smoke_test'
assert s['candidate']['visually_grounded'] is False
assert s['candidate']['readiness']['ready'] is True
assert s['candidate']['questions_answered'] == s['teacher']['questions']
assert s['candidate']['status_counts']['missing'] == 0
assert s['candidate']['status_counts']['invalid'] == 0
assert s['review']['question_count'] == s['teacher']['questions']
assert Path('.artifacts/spatiotemporal-video-qa', s['review']['index_path']).is_file()
print('SUMMARY_GATES=PASS')
PY
```

Verify complete-run determinism:

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

`cmp` must be silent and exit zero.

## Safety properties covered by tests

- strict and frozen records;
- duplicate-key and noncanonical JSON rejection;
- episode-safe question IDs;
- sample-schedule validation and zero-detection frames;
- inverse-relation canonicalization;
- relation-flip order independence;
- public/private manifest binding;
- exactly one oracle answer per public question;
- root-independent replay;
- candidate signature and filesystem isolation;
- source/target path and hard-link protection;
- symlinked output and ancestor rejection;
- full encoded-video decode verification;
- deterministic, escaped evidence HTML;
- temporal interval-to-frame evidence resolution; and
- failure-clean atomic viewer replacement.

## Extending the evaluator

A production candidate should implement the public `CandidateAnswerer` lifecycle:

1. ingest the public video;
2. report readiness;
3. answer each public `Question`; and
4. release resources.

Do not pass private bundle records into candidate code. Do not use oracle answers to tune predictions during a scored run. Keep teacher generation replayable from saved observations so evaluator changes can be compared independently from detector changes.

## Known limitations

- YOLO-E labels and tracks are pseudo-labels and can be semantically wrong.
- Relations are 2D image-plane geometry, not metric 3D scene relations.
- Sparse sampling can miss short-lived events.
- Episode-level negative questions require the complete sampled-frame schedule.
- The reference candidate score is not a visual-model benchmark.
- The reference episode ID is fixed in the demo; a dataset CLI should expose it explicitly.

If a custom video fails, improve the video, prompts, tracking, confidence threshold, sampling schedule, or versioned benchmark contract. Do not weaken privacy, determinism, or path-safety checks to force a pass.
