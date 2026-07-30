# Reference artifacts: what they are and how to audit them

This directory holds **one sub-directory per recording** — each the committed
output of the offline reference pipeline for that recording, plus the
human-review evidence its questions rest on. Nothing there is hand-measured;
everything is reproducible from the recording with the commands at the bottom.
This file is the protocol those sub-directories are produced and audited under.

## Datasets

| dataset | questions | notes |
|---|---|---|
| [`go2_bigoffice`](go2_bigoffice) | 3 | Dim open-plan office; 1040 frames visited at 4 Hz, 26 labels qualified for review. The recording the keyless full-chain smoke and its frozen model transcript are pinned to. |
| [`go2_short`](go2_short) | 3 | Office/retail interior; 214 frames visited at 4 Hz, 14 labels qualified for review. One of its three questions ships **without** review images, because every candidate frame for it contains an identifiable person — see [`go2_short/README.md`](go2_short/README.md). |

A recording is part of the eval exactly when its directory holds a
`questions.jsonl`: `reference_sets.dataset_names()` globs `*/questions.jsonl`,
and both the live sweep and the artifact tests are parametrized over the result.
Adding a recording is adding a directory here — no harness or test edit.
Directory name, `--dataset` argument and chroma collection name are all the same
string, which is what lets a store, a shard directory and a question set be
matched up without a lookup table.

## What each dataset directory holds

| file | what it is |
|---|---|
| `<dataset>/refs.jsonl` | every label that survived the geometric gates: position, views, spread, per-view robot poses, `location_group` |
| `<dataset>/manifest.json` | every gate value used, the full yield funnel, the location-group conflict table, dataset/weights hashes, timing |
| `<dataset>/review.json` | the human verdict per label — `verified` / `renamed` / `dropped`, each with its reason |
| `<dataset>/questions.jsonl` | the question set: only labels the review kept, after the passability and confusability gates. The only file the live sweep reads |
| `<dataset>/crops/<question_id>.jpg` | identification frame: sharpest LiDAR-supported frame near a contributing view, with the reference position and inlier points drawn |
| `<dataset>/crops/<question_id>_measurement.jpg` | the actual detection frame the measurement came from, with the detection box, in-box inliers and reference drawn |

Crops are committed for the questions, not for every qualified label, and a
question can ship without them if no publishable frame exists — stated in that
dataset's README where it happens. They are presentation: `refs.jsonl` and
`manifest.json` are identical whether `--crops` ran or not, and nothing in the
eval reads a crop.

## How the review was done (and how to redo it)

References are estimates — geometric concentration is not semantic correctness,
and the detector's open-vocabulary labels are frequently wrong on these dim
recordings. The review protocol, in increasing order of effort:

1. **Crop pass**: view every qualified label's crop; judge whether the label (or
   a rename to what is visibly there) fairly names the object. Region labels,
   scene-ambiguous classes and unidentifiable objects are dropped with reasons.
2. **Multi-frame verification**: for kept labels, render the approach context
   (several frames along the robot's path with the reference projected into
   each) and check the marker stays on the object. This is the step that
   catches what single crops cannot: one committed question was dropped when
   its reference proved to sit on a blank wall beside the detected surface
   (bounding-box contamination had pulled the point-cloud median off the
   object) after passing two rounds of crop review.
3. **Full-video spot check**: skim the recording itself. This resolved one
   previously unidentifiable label (a cleaning robot) and confirmed the drop
   reasons for dynamic and multi-instance objects.

Every verdict lands in that dataset's `review.json` with its reason; the funnel
in its `manifest.json` records what the gates dropped before review ever ran.
Review keeps few labels, per dataset: of `go2_bigoffice`'s 26 qualified labels
it kept 3, and of `go2_short`'s 14 it kept 3. Most drops are validity exclusions
(dynamic objects, multiple identical instances, mutually confusable neighbors)
rather than pipeline errors — objects with no unique answer to "where is X"
make no valid question.

## Reproducing

```bash
DATASET=go2_bigoffice  # or any other directory name here
uv run python -m dimos.agents.evals.teacher --dataset $DATASET --out-dir <dir> --crops
uv run python -m dimos.agents.evals.questions --refs <dir>/refs.jsonl \
  --review dimos/agents/evals/reference/$DATASET/review.json --out <dir>/questions.jsonl
```

`refs.jsonl` reproduces byte-identically on one machine; `manifest.json` varies
only in its timestamp and timing fields. `--crops` writes an image pair for
*every* reference; what a dataset directory commits is the pairs for its own
questions, copied out of that run.
