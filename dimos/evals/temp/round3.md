# Round 3 — representation, not answers

Round 2 ([`eec511636`](../../../dimos/msgs/sensor_msgs/PointCloud2.py)) took the
bench from 0.50 to 0.84 by adding four channels to `agent_encode()`:
`wall_openings`, `enclosure_topology`, `ground_relief`, `unmeasured_pockets`.
Each one is the answer to one of the hand-authored suites, with a `desc`
telling the model which question it answers — *"separate_areas is already the
answer to a room or area count"*, *"Route answers only, at the goal point"*.
The rooms prompt's rubric (*two areas are separate only when the only way
between them is a doorway-width opening*) is implemented verbatim as a 0.6 m
erosion. Six labelled rows per family, thresholds tuned against them, no
holdout.

That is not an encoder. It is a lookup table keyed by eval prompt, and the
agent reads it as one. Two live sessions on 2026-08-20
(`logs/20260820-120309-*` and `logs/20260820-121703-*`, recordings
`data/go2_agentic_memory_20260820.db` and `recording_go2.db`) were reviewed turn by
turn against the lidar. What they showed:

- **Transcription.** Asked to count rooms, the agent printed
  `separate_areas` eight times in a row under a variable it named `rooms`
  and spoke the desc back (*"one connected, undivided floor area with no
  doorway-width separation"*). The channel erodes free floor by 0.6 m and
  drops any core under 2.5 m²: on the session-2 fused map that discarded 86 %
  of mapped floor, and 61 % of the robot's own poses sat on floor that was
  never counted. A room glimpsed through a door cannot clear that bar by
  construction. The count is 1 at every erosion constant from 0.4 to 1.0 m.
- **Confident negatives.** *"Nothing here is a doorway"* was spoken as
  "this building has no doorways" three times. After the user's correction,
  the revised "at least 2" came from a `wall_openings` hit with 1 of 4 votes,
  0.11 m from where the robot had stood for 25 s, whose "walls" spanned
  0.45–0.66 m — the Go2's own footprint in the body band.
- **The clearest floor read as unmeasured.** Asked to face the longest
  clearing, the agent filtered to the encoder's body band (0.15–1.0 m),
  binned by bearing and chose 217°. The three sectors from 270° to 360° had
  zero body-height returns and 5,000+ floor returns each, out to the edge of
  the frame. It called them *"unmeasured rather than confirmed clear"* — the
  `unmeasured_pockets` vocabulary — then drove 3.3 m straight through one
  with no obstacle. The encoding has a channel for where obstacles are and a
  channel for where nothing was measured; *measured floor with nothing on
  it* is the complement of both and is never emitted.
- **The window is invisible.** Every per-frame cloud is a ±3.17 m
  axis-aligned box around the robot. The 57 points that won the clearance
  pick were all within 0.25 m of the box's diagonal corner: a clipped object,
  not a far face. Nothing in the output distinguishes the sensor's reach from
  the world's edge.
- **The main readout never fit.** `MemoryQuerySkill.max_output_chars` is
  4000; `ENCODE_SOFT_CAP` is 5940. Every full `print(cloud.agent_encode())`
  was cut at `ground_relief.desc`; `wall_openings` and `unmeasured_pockets`
  were never delivered in one. Per frame, ~3.6 kB of the 5.5 kB is prose and
  the agent used none of it — it wrote its own radial code every time.
- **Keys vanish.** `enclosure_topology` returns `None` when no core reaches
  2.5 m², and the key is dropped. The agent hit `KeyError` in the one frame
  that said "you are in a 1.1 m passage", treated it as transient and
  retried.
- **Phantoms.** `ground_relief` carried a −0.2 m "sunken area" for eight
  minutes that vanished when the robot stood on it (odom z flat at 0.30 m),
  and a 0.45–0.89 m "run of steps" that was gone eight minutes later. The
  agent never read `areas` — the cap cut it — so no harm this time.

Both failures are the same design. The loop had two ways to raise the score:
emit a better picture of the geometry, or emit the answer plus instructions to
copy it. The second is cheaper and the harness could not tell them apart.
Round 3 makes the second impossible and the first measurable.

## Principle

`agent_encode()` emits what the sensor measured, laid out so a model can
reason over it. It does not name doors, rooms, steps or routes; it does not
classify cells as free or navigable; it does not tell the model what to do
with any field. One frozen legend explains the format. The loop changes what
geometry is emitted and how it is laid out — nothing else.

Optimize on low-level geometry with computed truth. Test on high-level
semantics with human labels. A representation that generalizes moves the
semantic holdout without ever having seen it; an answer key moves nothing.

## 1. Seed

Revert `eec511636`. Start from the run-1 encoder (`1a780ff3b`) with its
prescriptive prose stripped: `exact_stats.note`, `footprint_note`, the
`compass` paragraph, and the clearance formula in `body_height_occupancy.desc`
all go. What survives is data — centroid, ranges, footprint, obstacle boxes.

Add one channel: a **min/max height raster**. Per x-y cell, the lowest and
highest return z, quantized to 0.1 m, as two characters; a cell with no
returns is `..`. No z bins: the only edges are the quantization step and the
clamp range (−0.5 to 2.5 m, 31 levels, alphabet `0-9A-U`). Cell is the
smallest of 0.25 / 0.5 / 1.0 m that keeps both axes ≤ 48 cells, so a single
±3 m frame is ~26 × 26 and a 15 m fused map is ~30 × 30 at 0.5 m.

One frame, abbreviated:

```json
{
  "frame_id": "world",
  "ts": 1787252675.9,
  "num_points": 52007,
  "window_m": {"x": [-1.73, 4.62], "y": [4.57, 10.82], "z": [-0.47, 1.17]},
  "centroid_xy_m": [1.42, 7.30],
  "floor_footprint_m2": 38.6,
  "raster": {
    "cell_m": 0.25,
    "origin_xy_m": [-1.75, 4.50],
    "z_step_m": 0.1,
    "z_min_m": -0.5,
    "rows": [
      "10.75 ..........5G5G5G5G..................",
      "10.50 ..........5G5G5H5G..................",
      " 7.75 5555555555555555555555555555555555555",
      " 7.50 5555555555555555555555555555555C5555",
      " 4.75 ................55555555555555555555"
    ]
  },
  "boxes": {
    "z_m": [0.15, 1.0],
    "xmin:xmax@ymin:ymax": "-0.73@10.82,0.43:0.88@10.73:10.82,..."
  }
}
```

Rows run north to south with their y; columns west to east from
`origin_xy_m`. `55` is a cell whose lowest and highest return are both at
z ≈ 0.0: bare floor. `5G` is floor at 0.0 with something reaching 1.1 m
above it. `..` is nothing measured. Runs of one pair may be RLE'd if the
loop finds it worth the legibility cost.

This is a rendering of what the lidar measured, not a classification.
Nothing in it is called *free*, *obstacle*, *floor* or *unmeasured*. The
reader derives those: a run of `55` is floor that was swept and had nothing
on it; `..` is never-measured; a break in a run of `5G` with `55` continuing
through it is a gap in something, and the same break with `..` behind it is
a shadow (glass, or the sensor's range); the min character stepping `5 6 7 8`
along a row is floor rising — a ramp or steps — with no floor-height channel
needed to say so. A G1 reads the same two characters as a Go2 and applies
its own height. There is no *free* state for the movement controller to
disagree with — navigability is the controller's call, and the agent asks
it.

What it loses: what lies between min and max. A table and a crate look the
same. A third character for return count, or per-slab presence bits, would
restore some of that; whether it earns its bytes is for the loop. It still
assumes a ground plane. That is a declared default for ground robots; a
drone blueprint swaps in a voxel encoder.

Everything else is run-1 data with the prose removed. `window_m` is the old
`x/y/z_range` renamed so a reader knows it is the sensing box, not the
world. `boxes` keeps the exact extents for numeric clearance, labelled by
its z band so it is visibly one view of the cloud rather than *the* obstacle
definition.

The legend is one frozen class constant, not in the per-frame output:

> `raster.rows`: one row per `cell_m` of y, north to south, prefixed with
> its y; two characters per cell, west to east from `origin_xy_m`. First
> character is the lowest return in the cell, second the highest, as
> `round((z - z_min_m) / z_step_m)` in the alphabet `0-9A-U`, clamped.
> `..` is a cell with no returns. `window_m` is the extent the sensor
> covered this frame. `boxes`: exact x–y extents of returns within `z_m`,
> world meters. `floor_footprint_m2`: cells with any return × cell area.

That is the whole vocabulary, ~350 bytes, and the `prose` gate hashes it.

What the seed does **not** carry, deliberately:

- Derived floor features. The min character *is* the floor height where
  floor was seen; the encoder does not go on to find patches, levels or
  steps in it. Note the noise: round 2's 10th-percentile floor produced a
  −0.2 m phantom at 3 m range. Measure the noise floor of min z per cell
  before trusting the 0.1 m step at range, and say in the legend if it is
  coarser than the quantization suggests.
- A pose. `agent_encode()` is on the cloud alone. Anything pose-relative —
  bearings, nearest obstacle, longest clear run — is the agent's to compute
  from the raw points, not the encoder's.

The frame's spatial window (a ±R box about the sensor) is in the output as
`window_m` and in the legend as a sensing fact, so a reader knows where the
raster ends because the lidar did.

The seed is a human commit, reviewed, and baselined before the first
experiment.

## 2. New pre-gate: `prose`

Mechanical enforcement of the principle. Runs from the worktree, costs
nothing.

| Check | Rule |
|---|---|
| Output strings | every string value in the encoding is encoded data (`^[-0-9.:@,;\| ]*$`) or ≤ 40 chars |
| Prose bytes | non-data bytes per frame ≤ 400 over `BUDGET_SAMPLE` |
| Vocabulary | no banned word in the output *or* in any string literal reachable from `agent_encode` (AST walk of `PointCloud2.py`) |
| Legend | `AGENT_ENCODE_LEGEND` is one class constant; `static` hashes it |
| Key stability | every legend key is present on every frame, empty allowed; a frame that drops a key fails |
| Cap | `ENCODE_SOFT_CAP` and `MemoryQuerySkill.max_output_chars` derive from one constant, encoder ≤ skill |

Banned vocabulary, first cut — semantic nouns, classifications, scorer words,
imperatives:

```
door doorway room stair step platform landing ramp
free clear navigable reachable route unknown
answer question count
never always use take instead already "do not" "not a" only
```

Crude on purpose. An optimizer that learns to dodge the word list shows up in
the per-experiment diff, which is where the review happens anyway.

The legend rule means a new top-level key needs a legend entry, and the loop
cannot write one. Adding a named channel is a design decision; the loop
proposes it by shipping the data under a placeholder key and a human names it
between rounds, or not.

## 3. Slices

### Train — geometry, computed truth

What the loop scores on. Keep the existing `clearance` and `crossing` train
slices and the frozen geometry suite. Reword `route`: its third answer is
literally *reachable / unknown*, which asks the encoder to be a planner and is
where round 2's *"not reachable"* prose came from. Sensing terms only — *does
the straight line to the goal cross only cells that were measured and whose
highest return is below 0.15 m?* The 0.15 m is the question's definition of
"nothing on the floor", on the eval side; the encoder carries no such edge.

Add generated families in the `go2_pointcloud_clearance` pattern — truth
from the full-resolution cloud plus odom, rows emitted by a `rows()`
function, sliced by `split.py`:

| Family | Question | Truth |
|---|---|---|
| `free_range` | From (x, y), which 8-way bearing has the longest run of measured cells with no return above 0.15 m before a cell that has one, or the frame edge, and how far? Capped at the inscribed radius so the window's diagonal cannot win. | ray-march at full resolution |
| `floor_height` | Lowest return at (x, y) relative to the robot's cell | min z, with a measured noise floor |
| `free_disk` | Largest circle of measured cells with no return above 0.15 m near the pose: centre and radius | distance transform |
| `gap_width` | Width of the narrowest gap between returns above 0.15 m within 2 m of (x, y) | direct |
| `coverage` | Fraction of a given 3 m square with any return at all | cell mask |

The 0.15 m in these questions is the eval's definition, stated in the
prompt. The encoder never sees it.

`free_range` is built adversarially: sample poses where the nearest-obstacle
bearing and the longest-floor bearing differ. That gap is the clearance
failure. An encoder that carries only obstacle distance scores at chance on
it.

None of these name a door, a room, a step or a route. They pull toward a
representation that carries elevation, measured emptiness and structure —
the ingredients — without ever asking for the conclusion.

### Holdout — semantics, human labels

Gate only. The loop never sees a score from these.

- The 24 hand-authored `doorway` / `rooms` / `floorlevel` / `stairs` rows,
  retagged `holdout`. These are the rows round 2 compiled in; now they are
  the test of the thesis — that a good picture lets the model find the door
  on its own. Report the positive-half mean, as `tool_evo_bench` already
  does.
- The existing `clearance` / `route` / `crossing` holdout slices, unchanged.
- The 19:04:35Z frame from `go2_agentic_memory_20260820`, pose (1.44, 7.68),
  labelled: the longest clearing is 300–330°, not 217°.

### Threshold probes — holdout

Synthetic edits of real frames that break round 2's constants. Made by point
deletion and translation on a labelled frame; ~6 rows per probe.

| Probe | Edit | Expected |
|---|---|---|
| wide door | widen a doorway gap to 1.3 m | still a doorway |
| curb | raise a 1 m² patch of floor by 0.25 m (above the measured noise floor) | still a level change |
| arch | join two rooms through a 1.5 m opening | still two rooms |
| cut floor | delete floor returns past 3 m along a clear bearing | clear to 3 m, nothing measured beyond |
| corner | place the only far returns at the frame's diagonal corner | not the longest clearing |

`cut floor` is the clearance failure run both ways: the encoder has to carry
the difference between *floor measured, no obstacle* and *nothing measured*,
or it fails in one direction or the other.

## 4. Score and acceptance

An experiment commits on train gain plus the existing `floors` and holdout
non-regression gates. Nothing new there.

The **round** ships only if the semantic holdout positive-half mean and the
threshold-probe mean beat the seed's. A run that raises train and leaves the
holdout flat is a result — it says the representation is not there yet, not
that a door detector is needed.

This is the bet the round makes, and it should be said plainly: the loop is
never trained on doors, rooms or stairs. It is trained on rendering geometry
legibly, and the holdout tests whether a model can find the door in that
rendering on its own. The door rows stay out of train because the gates
check text, not function — a channel that lists 0.7–1.0 m breaks between
wall runs passes `prose` and is a door detector under another name. The only
thing that stops the loop building one is that door rows never pay.

Run the blind ablation once, up front, on the new families. New questions,
so it is due regardless.

## 5. Budget

6 kB / 80 ms per frame, unchanged, and now equal to the skill's output cap.
The prose cap inside it frees ~3.5 kB for data, which is what the raster
needs. Measure the seed first; the run-1 encoder was 2.6–3.3 kB.

## 6. Setup, in order

1. Revert `eec511636`. Strip the notes. Add the min/max height raster under
   the frozen legend. Tie the two caps. Human commit.
2. Write the five generated families into `suites/` in the clearance
   pattern; reword `route`; `split.assign` each.
3. Retag the 24 hand-authored rows `holdout`. Label the 12:04:35 frame.
   Author the threshold probes.
4. Add `prose` to `tool_evo_gate`. In `tool_evo_bench`, drop `doorway`,
   `rooms`, `floorlevel`, `stairs` from `SCORED_FAMILIES`; add the generated
   families.
5. Blind ablation on the new suites.
6. `--write-floors`, holdout baseline, `freeze`, commit.
7. `evo init` with `static && budget && prose` as the pre chain, the existing
   `floors` and holdout post gates. `/evo:discover` seeded with one question:
   *what does the model need to see to find the door on its own?*

Steps 1–6 are a day or two of human work before the first paid call.

## 7. Cost

Per experiment about 160 train calls and 110 gate calls, against 122 + 25
today — roughly 1.8× per experiment, ~45 minutes wall clock. The generated
families are the bulk of it; `coverage` and `gap_width` can go if cost
matters more than breadth.

## What this is not

Not a prompt fix. Telling the subagents not to overfit leaves every avenue
open that the gates leave open, and the loop will find it.

Not keeping the round-2 channels behind a flag. They are the comparison
baseline for this round and nothing else. If the thesis holds, a model
reading the raster finds the door; if it does not, that is worth knowing
before anyone writes a better door detector.

Not a tri-state grid, and not height slabs. Earlier drafts proposed both.
*Free* is a classification, it embeds the Go2's height band, and it is a
claim the movement controller can contradict. Slab edges at 0.15 and 1.0 m
are the same band with the label removed — the edges decide what the agent
can see, and a 10 cm curb vanishes inside a 0.3 m slab. Min and max z per
cell have no edges but the quantization step, and leave the rest to the
reader. Uniform slabs are the loop's first alternative to compare against.
