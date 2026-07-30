# Reference artifacts: what they are and how to audit them

This directory is the committed output of the offline reference pipeline for the
`go2_bigoffice` recording, plus the human-review evidence the questions rest on.
Nothing here is hand-measured; everything is reproducible from the recording
with the commands at the bottom.

| file | what it is |
|---|---|
| `refs.jsonl` | every label that survived the geometric gates: position, views, spread, per-view robot poses, `location_group` |
| `manifest.json` | every gate value used, the full yield funnel, the location-group conflict table, dataset/weights hashes, timing |
| `review.json` | the human verdict per label — `verified` / `renamed` / `dropped`, each with its reason |
| `questions.jsonl` | the question set: only labels the review kept, after the passability and confusability gates |
| `crops/<question_id>.jpg` | identification frame: sharpest LiDAR-supported frame near a contributing view, with the reference position and inlier points drawn |
| `crops/<question_id>_measurement.jpg` | the actual detection frame the measurement came from, with the detection box, in-box inliers and reference drawn |

## How the review was done (and how to redo it)

References are estimates — geometric concentration is not semantic correctness,
and the detector's open-vocabulary labels are frequently wrong on this dim
recording. The review protocol, in increasing order of effort:

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

Every verdict lands in `review.json` with its reason; the funnel in
`manifest.json` records what the gates dropped before review ever ran. Of the
26 qualified labels here, review kept 3: most drops are validity exclusions
(dynamic objects, multiple identical instances, mutually confusable neighbors)
rather than pipeline errors — objects with no unique answer to "where is X"
make no valid question.

## Reproducing

```bash
uv run python -m dimos.agents.evals.teacher --dataset go2_bigoffice --out-dir <dir> --crops
uv run python -m dimos.agents.evals.questions --refs <dir>/refs.jsonl \
  --review dimos/agents/evals/reference/review.json --out <dir>/questions.jsonl
```

`refs.jsonl` reproduces byte-identically on one machine; `manifest.json` varies
only in its timestamp and timing fields.
