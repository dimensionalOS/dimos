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

The figures below are approximate — they are read off a `--crops` run and are
recorded to show the shape of the evidence, not as constants to assert against.
They reproduce with the shipped tools (see [`../README.md`](../README.md)), and
the exact counts move with the gate values in `crops.py`.

| question | marker sits on | support | surface depth at marker vs reference |
|---|---|---|---|
| houseplant | the dark tapered pot of the large potted plant | supported in most candidate frames, ~140 pts at best | ≈ −0.06 m (median over the measurable frames) |
| toilet paper display | the pyramid of white plastic-wrapped paper multipacks | supported in every candidate frame, ~220 pts at best | ≈ −0.03 m |
| wooden meeting table | the dark-wood table, just under the tabletop edge | supported in roughly two thirds of candidate frames, ~80 pts at best | **≈ +0.71 m** — see below |

**The meeting table's +0.71 m is cosmetic, not the `elevator door` pattern.**
The marker falls in the see-through gap just under the tabletop, so the 14 px
around it look *past* the table to the far side of the room, which is what the
depth reads. A surface *behind* the marker is not occlusion and the gate does
not treat it as such (`crops.CROP_OCCLUSION_SLACK_M`); nothing stands in front
of the marker, the marker is on the table in every frame of the strip,
`z = 0.525 m` is tabletop height, and the support points within 0.4 m of the
reference are the tabletop itself. The visible consequence is that the marker
reads as floating slightly below the tabletop rather than lying on it. It is
recorded here because a reviewer checking the geometry will find the +0.71 m and
otherwise have no explanation for it.

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

This is checked, not assumed: two people are seated at the table while the robot
passes it. Every LiDAR-confirmed candidate inside the shipped ±1.0 s window —
about thirty of them — was rendered and viewed, and all of them show a seated
person, most with the face clearly visible. Widening the window to ±5 s adds
roughly twenty more confirmed candidates, all showing the same people, while the
earlier frames carry no LiDAR support at all. Tightening the framing to
`CROP_CONTEXT_SCALE = 1.0` moves the face out of the crop but keeps the person's
torso, arm and legs in it. The counts are approximate for the same reason as the
table above; the finding — that no candidate frame is publishable — is what the
omission rests on. The question itself is unaffected (`questions.jsonl` is the
only file the sweep reads), and the verification table above is what stands in
for the images.
