# Tracklet re-identification: what was measured

`dimos/perception/detection/reid/tracklet_reid.py` merges tracklets a tracker lost. This records what
it was tested against and what the results ruled out.

**The grouping numbers below are withdrawn.** They were measured against a
`connected` and an `OnlineAssociator` that applied the instant-overlap veto only
between the two tracklets of a pair, never to the group a link would create —
and this benchmark's negative class *is* "overlapping in time". So every wrong
merge it counted was a pair the module's own hardest veto would have refused,
had the grouping path asked it. The veto now runs group-wide. The rows are kept,
struck through, because the conclusion drawn from them was published.

## Ground truth without labelling

Re-identification's job is "the tracker lost this; find it again", so the
benchmark makes the tracker lose it on purpose.

| | How | Why it is exact |
|---|---|---|
| **same-object pairs** | cut one tracklet into pieces, discard 30% between them | the tracker held it continuously, so the pieces are one object |
| **different-object pairs** | two tracklets overlapping in time | both visible at one instant in different tracks |

Neither needs a human. The discarded span is a dial, so a method can be scored
on how far it bridges rather than only on whether it merges anything.

Data: one 7.8-minute go2 capture of an SF office (`go2_SF_office_8mins_moshi`),
6,628 frames, 36,189 sightings, 64% placed in 3D. Crops embedded once with CLIP;
every method saw identical input, differing only in the association rule.

**388 pieces from 194 tracks — 194 same-object pairs, 1,246 different-object
pairs.**

## Results

| method | recall | precision | merged same | merged different |
|---|---|---|---|---|
| **pairs only, shipped defaults** | **99.0%** | **100.0%** | 192 | **0** |
| upstream `EmbeddingIDSystem` rule | 99.0% | 100.0% | 192 | 0 |
| pairs only, threshold picked separately | 83.5% | 100.0% | 162 | 0 |
| geometry only, no appearance | 96.9% | 41.2% | 188 | 268 |
| ~~ours, with transitive closure~~ | ~~96.4%~~ | ~~38.5%~~ | ~~187~~ | ~~299~~ |
| ~~ours, online (ids, so groups)~~ | ~~96.9%~~ | ~~23.9%~~ | ~~188~~ | ~~597~~ |
| appearance only @0.63 | 100.0% | 13.5% | 194 | 1,238 |

**The recall column needs one rerun.** These were measured when the reachability
veto compared whole-run centroids; it now compares the sighting either side of
the gap, which is what the speed budget was always modelling. Precision is
unaffected either way — this benchmark's negatives are pairs overlapping in
time, and the overlap veto refuses those before any distance is computed, so no
distance rule can change the zero in the "merged different" column. Recall can
move in either direction and has not been remeasured.

## What this rules out

**Grouping: unmeasured, not refuted.** The pairwise half stands — 192 right,
0 wrong, and its decision logic is unchanged. The grouping half was not what it
claimed to be. `connected` closed links checking only group spread, and
`OnlineAssociator` vetoed a candidate track without asking the entity that track
already belonged to, so a group could hold two tracklets seen at one instant and
an id could absorb a track a caller had explicitly declared a different object.
Both now apply the veto to the whole group, and `co_occurring` binds every
member of an id rather than one track of it.

This benchmark cannot referee that change: its negative class and the new check
are the same predicate, so rerunning it would score the check against itself and
report something near 100% that means nothing.

**So `find_merges` returns pairs and `connected` stays opt-in** — now because
grouping is unmeasured, not because it was measured and failed. What it needs is
a negative class that does not reduce to co-visibility: pairs of one model never
in frame together, labelled by hand, few enough to label in an afternoon. Until
then, whether eight fragments are one chair or three is a judgement, and looking
at eight crops answers it.

**Online.** An id *is* a group, so the online path inherits whatever grouping
turns out to be worth, and cannot revise what it already said. It is in the
module because the shape is right for a robot, not because anything measured
here justifies deploying it.

## What the geometry veto is worth

Upstream vetoes only co-occurrence — things visible in the same frame. Adding
position and speed refused **48 pairs upstream would have merged, all 48
correctly**: things that look alike and were never seen together, like two
chairs of one model at opposite ends of a room.

    similarity 0.914, 5.3 m apart, reachable 5.3 m  -> different objects
    similarity 0.890, 8.0 m apart, reachable 4.1 m  -> different objects

The benchmark cannot score this. Its negatives are defined as *overlapping in
time*, which is exactly what co-occurrence already catches, so the 48 fall
outside the labelled set. The benchmark understates this method's advantage, and
that limitation is the reason to keep the number here rather than in a score.

## Thresholds are the model's, not the code's

Measured on this capture with CLIP: same-object pairs at median 0.967,
different-object pairs at 0.862, separating at 0.925 with 93.7% balanced
accuracy. The repo's person-re-identification default of 0.63 would have merged
nearly every different-object pair — under *that* aggregation.

Under `top_frac`, 0.63 is right and 0.925 costs 15 points of recall. **The
threshold and the aggregation are one setting, not two**, and changing either
without re-measuring the other cost 15 points of recall until both were
measured together. Re-measure by cutting tracklets in half; it takes one GPU
pass.

## Known limits of these numbers

- One capture, one room, one model, one detector.
- Every figure predates the group-wide veto and the boundary reachability veto.
  Pairwise precision survives both (negatives are refused on time overlap before
  either runs); pairwise recall was measured under centroid distance and needs
  one rerun.
- `find_merges` now requires L2-normalised embeddings rather than assuming them,
  so a capture whose vectors were not normalised will raise where it used to
  return inflated cosines.
- Positives are two halves of a tracklet with 30% discarded: temporally adjacent,
  small viewpoint change. **Easier than the real problem**, so every recall
  figure here is optimistic. Widening the cut is the next measurement.
- `torchreid`'s OSNet was never run — its weights need LFS credentials this
  machine lacks — so "is a person-re-identification model better than CLIP for
  office furniture" is open.
- 36% of detections carry no 3D position and are refused outright. On this data
  that was 340,000 of 673,000 refusals. The limit on re-identification here is
  placement, not association.
