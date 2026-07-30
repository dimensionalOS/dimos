# `go2_short`: second question set

The repository's other recording — an office/retail interior, 214 frames visited
at 4 Hz — run through the same pipeline as `go2_bigoffice` with default gates
and no code changes: 14 labels qualified, review kept 3. It exists as evidence
that the pipeline generalises rather than that it was tuned to one room. The
protocol, the file layout and the reproduction commands are in
[`../README.md`](../README.md).

## The three questions

| question id | display name (raw label) | reference x, y, z | threshold | review |
|---|---|---|---|---|
| `go2-short-houseplant` | houseplant (`houseplant`) | 2.05, 0.775, 0.3375 | 3.0537 m | verified |
| `go2-short-newsstand` | toilet paper display (`newsstand`) | 1.5, 14.7875, 0.4 | 2.7241 m | renamed |
| `go2-short-office-desk` | wooden meeting table (`office desk`) | −1.1125, 6.9125, 0.525 | 2.879 m | renamed |

The confusability gate is green. It is also load-bearing here: `vanity` (really
a rolling metal cabinet) is dropped in review because one of its own teacher
viewpoints sits 2.18 m from the paper-goods reference, inside that question's
2.7241 m pass radius, so keeping both would let the wrong retrieval pass.

## Multi-frame verification

Each reference was checked across a strip of the approach — marker projected,
supporting LiDAR points drawn, support counted per frame — the step that caught
`go2_bigoffice`'s `elevator door`. All three passed.

| question | marker sits on | support | surface depth at marker vs reference |
|---|---|---|---|
| houseplant | the dark tapered pot of the large potted plant | 17/22 candidate frames supported, up to 139 pts | −0.06 m (median of 11 measurable frames) |
| toilet paper display | the pyramid of white plastic-wrapped paper multipacks | 35/35 supported, up to 223 pts | −0.03 m (16 frames) |
| wooden meeting table | the dark-wood table, just under the tabletop edge | 29/41 supported, up to 78 pts | **+0.71 m** (16 frames) — see below |

**The meeting table's +0.71 m is cosmetic, not the `elevator door` pattern.**
The marker falls in the see-through gap just under the tabletop, so the 14 px
around it look *past* the table to the far side of the room, which is what the
depth reads. Nothing occludes the marker, the marker is on the table in every
frame of the strip, `z = 0.525 m` is tabletop height, and the 73 support points
within 0.4 m of the reference are the tabletop itself. The visible consequence
is that the marker reads as floating slightly below the tabletop rather than
lying on it. It is recorded here because a reviewer checking the geometry will
find the +0.71 m and otherwise have no explanation for it.

One caveat stays open, from review: a similar potted plant appears in the
background of the `entrance hall` crop. It is most likely the same plant seen
from ~2.3 m away, but if the space holds several floor plants the houseplant
question becomes class-ambiguous. That is a question about the scene, not about
the position, so no geometric check can settle it.

## `go2-short-office-desk` ships without review images

`crops/` holds the pairs for the houseplant and the toilet paper display only.
For the meeting table, every candidate frame for this question contains an
identifiable person; review images are omitted and reproducible locally via
`uv run python -m dimos.agents.evals.teacher --dataset go2_short --out-dir <dir> --crops`.

This is measured, not assumed: two people are seated at the table while the
robot passes it. All 29 LiDAR-confirmed candidates inside the shipped ±1.0 s
window were rendered and viewed, and all 29 show a seated person, most with the
face clearly visible; widening to ±5 s adds 20 more confirmed candidates, all of
which show the same people, while the 35 earlier candidates carry no support at
all. Tightening the framing to `CROP_CONTEXT_SCALE = 1.0` moves the face out of
the crop but keeps the person's torso, arm and legs in it. The question itself
is unaffected — `questions.jsonl` is the only file the sweep reads — and the
verification table above is what stands in for the images.
