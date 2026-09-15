# Query plan

Where the query is going, and why. Written 2026-09-12 with Jeff, working through the
current design step by step. Nothing here is built yet unless it says so.

**Target: a 100 ms query.** Today "soda" over 3,462 embedding frames takes 92.5 s.

## What the query does now

1. Turn the text into a vector, one per model.
2. Load every embedding frame's patch grids from the db.
3. Score every patch; keep the ones that beat their background.
4. For each hot patch, look up where the camera was and how far away that patch was,
   and draw a thin shell at that distance along its ray.
5. Add the shells into a voxel grid.
6. Clean up: drop voxels with no depth behind them, drop lonely ones, carve to object
   size, group what is left into blobs.
7. Return the blobs, ranked.

## Measured, on grocery.db, 3,462 embedding frames

| | |
|---|---|
| load the embedding frames | 1.2 s |
| answer without refine | 9.9 s |
| answer with refine | 92.5 s |
| build the tf buffer | 4.1 s, once per process |
| one tf lookup | 141 us, so 0.85 s for 6,000 hot patches |
| resident memory | 6.3 GB |

Two things dominate, and neither is the search:

**Refine is 89% of the query, and its cost is the wrong shape.** `refine.Grid` allocates
a dense box over the hot voxels' bounding box. For "soda" that box is 557x385x151 =
32.4 million cells holding 7,807 hot voxels: 0.024% occupancy. It is
O(bounding-box volume), not O(hot voxels), so spreading the same heat across a larger
store grows it cubically while the useful data does not move.

**Everything is loaded because of how it is stored.** The numbers the query needs live
inside one pickled blob per embedding frame, so reading one patch means unpickling the
whole row -- 6.3 GB of them.

## Building the index costs too, and it was mostly SQLite

Measured 2026-09-12 while rebuilding grocery.db with four models.

| | |
|---|---|
| the ingest, before the write-ahead log was fixed | 16.4 images/s decaying to 0.67 |
| the ingest, after | ~3.3 images/s loaded, 4.4x realtime, and steady |
| one patch row through `Stream.append` | 815 rows/s |
| one patch row in bulk | 3,677 rows/s |
| so400m-naflex@1024 on an RTX 5070 Laptop | 0.97 frames/s |

**The write-ahead log was 92% of the wall time.** Sampling a live pass put almost all
of it inside `sqlite3_wal_checkpoint_v2`: SQLite checkpoints on every commit once the
log passes a thousand pages, and a passive checkpoint never *shrinks* the file, so the
log grew all run and every commit rescanned all of it. The fix turns the automatic
checkpoint off and folds the log deliberately, from inside the writing process, with the
read iterator closed -- a fold from anywhere else, or with a read open, waits on a lock
someone already holds and stops the pass dead. Any second writer or long reader on the
database while a pass runs brings that back, so a query and an ingest cannot share it.

**Writing is one commit per patch.** `Backend.append` commits after every observation
and hands sqlite-vec its vector as `json.dumps` of a list of floats -- fifteen kilobytes
of text per NaFlex patch to parse back into the floats we already had. One transaction
per frame, `executemany`, and the vector as a raw float32 blob is 4.5x faster, and the
rows are indistinguishable from the store's own: same payload, same tags, same width,
same similarity to 6e-08.

**A second machine needs no copy of the recording.** The database is far too big to ship
and SQLite locks are not safe over a network mount, but the GPU only ever needs the
image. Serving the stored WebP bytes untouched over HTTP and taking grids back keeps the
recording on one machine, and asking the database every minute which frames the local
pass has kept lets the remote model run *beside* that pass rather than after it. Two
models shared one laptop GPU at 0.78 and 0.96 frames/s without slowing each other.

## The layout

Three kinds of thing, three streams.

**depth_thumbnails** -- one row per embedding frame: a small point cloud in the camera's own
frame, plus the camera frame and timestamp needed to place it. Nothing else: once each
patch carries its own ray and depth, the occupancy check is the only thing that needs a
per-frame row at all, so the stream is named for the one job it does.

**patches, one stream per model** -- `hyperspace_patches__m_<model>`, one row per patch:
the vec0 vector, plus camera frame, timestamp, cell number, ray direction and depth.
Everything needed to place a hit, so a search result needs no second read. The per-model
streams are shipped (commits 7fa21a969, 25f617f46); the self-contained row is not.

What leaves today's embedding-frame row, and why: `grid`/`grids` become the per-model patch
streams; `intrinsics` is unnecessary once every patch and every thumbnail point carries
its own direction; `rows`/`cols`/`grid_shapes` collapse into the one fixed cell grid;
`thumbnail_mm`/`thumbnail_stride` become the point cloud; and
`members`/`member_specs`/`model` go because the stream name already says which model
wrote the row -- which also disposes of the misnamed `model` field rather than renaming
it.

**tf** -- the recording's own stream, untouched.

### A fixed cell grid, chosen independently of the models

Say 48x48 for every embedding frame, with each model's patches resampled onto it at ingest. Then
cell 231 means the same place in every model, so any subset of models can be combined at
runtime and a fourth model can be added next month without touching the first three.

Today the shared grid is derived from whichever models are in the ensemble, which is
what forces them to be ingested as a group.

### The thumbnail is a point cloud, not a depth picture

The occupancy step is the only thing that reads it: every embedding frame's depth is placed
into the world to give a rough "we saw a surface here" cloud, and answer voxels that are
not on it get dropped. That is what removes hits floating in mid-air. It deliberately
uses the whole frame rather than the hot patches, so it is a check rather than a
restatement of what the patch already claimed.

Storing it as 3D points in the CAMERA's frame instead of a depth raster means nothing
has to be unprojected at query time: no intrinsics, no ray table. It costs three numbers
per point instead of one, about 3x as int16 millimetres.

The camera's world pose must NOT be baked in. That would remove the tf lookup too, and a
loop closure would then silently leave every thumbnail wrong -- the same reason the tf
design below stores ids rather than values.

## The query

1. Encode the text once per model.
2. Ask each model's vec stream for its nearest patches.
3. Keep the patches that came back from every model -- which is very nearly what
   min-pooling means, a cell all the models like.
4. Walk that list of ids. Each one carries its own frame, timestamp, depth and ray, so
   it becomes a point in the world without reading anything else.
5. Add its shell into the voxels.
6. Clean up and group.

Steps 2-5 stream: take an id, place it, move on. Step 6 cannot -- finding connected
components needs the finished map, and the score normalisation needs to know the top
score. So: stream 2-5, batch 6.

### sqlite-vec is brute force

There is no index. `vec0` compares the query against every stored vector on every
search: O(N*d) plus a top-k heap. Asking for the top 10 costs the same as the top 10,000.

Two things follow. Growing k to find the threshold is free, but it also buys nothing.
And the win from searching in the db is not that the search is clever -- it is that
reading 2M vectors sequentially inside sqlite is far cheaper than unpickling 3,462 blobs
into our own memory.

## tf: FlexTf

Built and measured (`flextf.py`, commits fe0639e49 and 98fcc13a4). Each edge is one
growing `(N, 8)` array -- timestamp, translation, quaternion -- so a batch of moments is
answered by one `searchsorted` and one vectorised interpolation per edge, whatever the
batch size.

The costs that matter, per event rather than per run:

| | FlexTf | MultiTBuffer |
|---|---|---|
| per tf update (one message, 16 edges) | 19 us | 17 us |
| per lookup | 2.0 us | 112 us |

`batch_get` is ~0.4 ms fixed per call plus 1.95 us per lookup, so the fixed part stops
mattering above a few hundred. Receiving costs 2 us more per message; each lookup costs
110 us less.

Two things it does that `MultiTBuffer` does not. `batch_get` fails PER ENTRY rather than
all-or-nothing, because a stamp outside one edge's range must not cost its neighbours
their answers, and it takes a list of sources for a rig with more than one camera.
`reform_edge` overwrites the transforms at given timestamps rather than appending
corrections, so a loop closure changes answers already being given -- which is why the
values live in a mutable array at all. It fixes memory only: the correction still has to
reach the recording separately, or a restart reads the old value back.

One deliberate behaviour change. `TBuffer.get` calls `find_closest`, so today's answers
SNAP to the nearest recorded transform -- at 20 Hz that is up to 25 ms of stale pose for
a camera that did not fire on a tf tick. FlexTf interpolates, lerp and slerp, which is
the choice made over zero-order hold in the June design.

### What made a first attempt slower than what it replaces

Worth keeping, because the wrong explanation survived two rounds. Receiving all 16,048
messages took 1,243 ms, five times the buffer it replaces, and the reason was not the
write: a row is 64 bytes and the write costs 0.35 us. Per edge per message the code did
FIVE numpy operations -- `asarray` 0.38 us, the slice write 0.73 us, two scalar reads,
and `np.diff(...) >= 0` to check the block arrived in order at 3.42 us. A message fans
out to all 16 edges, so that ran 257,000 times: 1.1 s of the 1.24 s, in the order check
alone. Comparing two floats in Python answers the same question in 0.04 us.

The deferred-flush design that followed was a workaround for the wrong diagnosis, and it
is gone.

## tf, the earlier plan

Building the buffer costs 4.1 s and holding it is the load-everything approach that
Jeff's June work (`cfe1d7115`, long task BottomReptile) replaced -- and it cannot stay,
because tf has to handle live loop closures.

That design: each tf message stores a tree snapshot whose **edges are keys into the tf
table, not copied values**. A lookup walks the tree and fetches the few rows it
references -- measured then at 4 rows per query instead of loading all 38,370, 13.3x
faster. Because the edges are keys, a correction that EDITS a row propagates to every
snapshot referencing it, so there is nothing to invalidate. That is also why a
`DeformationNode` must edit existing messages rather than append corrective ones.

For hyperspace: we already walk the whole tf stream at ingest, so build it there. Per
embedding frame, store the row ids of the two tf entries bracketing its timestamp for each edge of
the camera->odom chain, about ten ids. At query, fetch those rows, interpolate, compose.
No buffer.

Ids not poses is the whole trick.

Two caveats. It only covers the chains precomputed at ingest, not an arbitrary
`tf.get(target, source, ts)`. And **mem2's `Stream` has no fetch-by-id** -- it has
`at(t)`, `tags()`, `order_by`, seek and time selectors, but nothing to fetch a set of
row ids. Jeff's June work added `fetch_by_ids` for exactly this, and it is not in dimos
today. Resolving by `at(ts)` instead would work but resolves by time rather than by row,
so corrections stop propagating, which defeats the point.

Cheap win available before any of that: patches from one embedding frame share a
timestamp, so look up once per frame rather than once per patch. Better still, every
timestamp is known up front, so sort them and interpolate the whole batch in numpy in
one pass -- batching beats threading here, where each lookup is a few tiny matrix ops
and the cost is Python call overhead the GIL holds anyway.

Note that `placer()` calls `self.tf.get` once per frame against a `MultiTBuffer` today,
and core has no batched `get`. Both the batch and the tree mean doing the interpolation
in hyperspace instead, so `self.tf.get` goes away either way.

## What each piece needs

| | db | core |
|---|---|---|
| per-model vector streams | yes (shipped) | no |
| batched tf interpolation | no | no |
| patch carries frame, ts, depth, ray | yes, re-ingest | no |
| photos stream, point-cloud thumbnail | yes, re-ingest | no |
| fixed model-independent cell grid | yes, re-ingest | no |
| FlexTf batched lookups | no | no |
| tf tree of row ids | yes | **yes** -- `fetch_by_ids` |
| sparse refine | no | no |

## Grouping the patches

Clustering runs on the hot patches, not on a dense grid. Two shapes, and the second is
the one to build.

**Points, radius, KD-tree.** `cKDTree.query_pairs(r)` then connected components over the
pairs -- DBSCAN with `min_samples=1`, no grid, `r` a real distance. Measured at grocery's
size, 7,807 points with r=0.2: **11 ms**, against 82.6 s for the dense chain. It scales
poorly though -- 6,000 points 6 ms, 20,000 56 ms, 50,000 336 ms -- and is sharply
sensitive to the radius: at 20,000 points r=0.5 costs 236 ms against r=0.2's 56 ms,
because the pair count grows with the cube of the radius inside a dense blob. Use
`output_type="ndarray"`; the default builds a python set of tuples and dwarfs the search.

**Boxes, sweep and prune.** Each patch is a pyramid, so take its axis-aligned bounding
box and group boxes that overlap: sort by x-min, sweep, keep an active set, and test y
and z only on the pairs that overlap in x. Union-find the result. O(n log n) plus the
number of real overlaps, no grid, and no radius to pick.

The box is the better primitive because it carries the distance for free. A far patch's
box is large -- its depth is uncertain and its pixel footprint covers more of the world
-- while a near patch's is small. So two far patches merge readily and two near ones
have to genuinely touch, which is the behaviour a fixed radius has to be tuned into.

Not yet designed: how a group is SCORED once it exists. It should use the match strength
and the agreement between models, and a big box should count for less than a small one
at the same score.

## Measured on the rebuilt grocery.db, four models, 8.5M patch rows

3,462 embedding frames, every model over the same frames: base-224 (196 patches),
base-256 (256), base-naflex@1024 (1008), so400m-naflex@1024 (1008).

| | |
|---|---|
| load the four text towers | 1.8 s, once per process |
| encode the text | 16.0 s |
| search all four vec streams | 24.6 s |
| read the winning rows' vectors back | 7.5 s |
| **a whole query** | **48 s** |

Nothing is loaded up front any more: the old path unpickled 6.3 GB of embedding frames
before it could score anything, and this reads only the rows that won.

Two of the three costs are avoidable. **Encoding is sixteen seconds for six prompts,
and five of them are the fixed background list** -- identical for every query, so they
belong in a cache; that alone is most of it, and running the tower on the GPU rather
than the CPU is the rest. **Reading 16,000 winning vectors back takes 7.5 s** through
one `rowid IN (...)`, which wants chunking.

Search is the floor, and it scales with rows exactly as brute force should: 678k rows
2.4 s, 3.5M rows 8.1 s, and so400m's wider vectors 11.2 s for the same row count.

Which points at the real lever: **the patches only have to rank frames.** One cheap
model does that -- base-224 alone is 2.4 s of search against 24.6 s for all four, and
on "a basket" the top ten episodes were the same either way, agreed by all four models.
The expensive models are then something to switch on when comparing, not a toll every
query pays.

## Frames first, geometry second

Jeff's 2026-09-12 proposal, and it removes most of the grouping problem. Rather than
placing every hot patch in the world and hoping the pyramids overlap: rank *frames* by
the text, split them into episodes on a gap in time, take the best frame of each
episode, run an open-vocabulary detector (OWLv2) on that one image with the same words,
and turn its 2D box plus that frame's depth into a 3D box. Results come out per episode,
so they can be emitted as they are found.

What it costs: OWLv2 measured on real grocery frames on an RTX 5070 Laptop, 1280x720,
five prompts -- base-patch16 **725 ms a frame** (155M params, resized to 960x960; 7 ms
of that is the text side and cacheable, 340 ms is CPU preprocessing, 434 ms the
forward), large-patch14 **2,729 ms** (438M, 1008x1008, and it returned fewer boxes at
the same threshold, so the two cannot be compared at a fixed one).

What it buys: no overlap margin to tune, no group score to invent, and no need for
per-patch depth to be accurate -- the box comes from the detector and the depth image,
not from a pyramid. Cross-model agreement still works, as *which models voted for this
episode*, which needs nothing to coincide in space.

Both paths share the first step, so they can be compared rather than chosen between.

## Still open

- Refine on the sparse set instead of a dense box, or capped the way memory_world caps
  it (`REFINE_MAX_VOXELS`, `REFINE_MAX_EXTENT_M`).
- `hot_patches` is pinned at its 6,000 cap against 1.56M patches searched.
- Recall: with three models, the 2nd-lowest beats the lowest -- P 0.73 R 0.88 against
  min's P 0.74 R 0.71. Untested: whether min is simply degenerating to whichever model
  scores lowest overall, which per-model z-scoring would fix.
- OWLv2's threshold is one global number (0.5) and its scores are not comparable across
  words: a green plastic basket scores 0.66 and a real loaf of bread 0.42, so "bread"
  answers nothing on grocery while its correct answers sit just under the line. Accepted
  as a blind spot 2026-09-13; per-word calibration is the fix if it starts to matter.
- The tf that an in-place ingest used to copy into the recording (fixed 2026-09-12: it
  had duplicated grocery.db's tf 5.3x). The dedupe is separate from the layout work.
