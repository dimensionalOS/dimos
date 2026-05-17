# Autoresearch: relocalization

Have the LLM iteratively improve a point-cloud relocalization algorithm.

## Task

Given a global indoor 3D map and a local body-frame submap (pose cleared),
estimate the rigid transform that places the local submap into the global
map's world frame. This is the kidnapped-robot global relocalization problem.

The dataset is 20 pre-built test centers from a Unitree Go2 lidar log
(`go2_hongkong_office.db`) with PGO-corrected groundtruth poses.

## Setup

1. **Agree on a run tag** with the user (e.g. `mar5`). The branch
   `autoresearch/<tag>` must not already exist.
2. **Create the branch**: `git checkout -b autoresearch/<tag>` from current
   master.
3. **Read the in-scope files** (full paths, repo-relative):
   - `dimos/mapping/relocalization/program.md` — this file.
   - `dimos/mapping/relocalization/relocalize.py` — **the only file you modify**. Defines
     `relocalize(global_map, local_map) -> 4x4 numpy.ndarray` and nothing else.
   - `dimos/mapping/relocalization/run.py` — **READ ONLY** entry point. Imports your
     `relocalize` and evaluates it. Owns data loading, success thresholds, and time
     budget enforcement. You run this; you never invoke `relocalize.py` directly.
4. **Verify data exists**: `dimos/mapping/relocalization/data/global_map.npy` and
   `dimos/mapping/relocalization/data/test_frames.pkl` must be present. If
   missing, ask the human — the data was generated from upstream PGO assets
   that aren't included in this repo.
5. **Initialize `dimos/mapping/relocalization/results.tsv`** with just the header row.
   The baseline numbers will be recorded after the first run. Do **not**
   commit this file — keep it untracked.
6. **Confirm setup looks good** with the user, then start the loop.

## File layout

```
dimos/mapping/relocalization/
├── program.md                  this file
├── relocalize.py               MODIFIABLE — agent edits here
├── run.py                      read-only harness
├── results.tsv                 your experiment log (gitignored)
└── data/
    ├── global_map.npy          (N, 3) float32, PGO two-pass global map
    └── test_frames.pkl         list[dict] — 20 test centers with body
                                submap + SLERP-PGO groundtruth pose
```

## Experimentation

Each run has a **fixed 5-minute wall-clock budget** (enforced inside
`run.py`). Launch it as:

```
uv run dimos/mapping/relocalization/run.py > dimos/mapping/relocalization/run.log 2>&1
```

Do **not** use `tee`. Redirect stdout+stderr to `run.log`. Don't let the
script output flood your context.

### What you CAN do

- Modify `dimos/mapping/relocalization/relocalize.py`. Anything in there is fair game:
  voxel sizes, descriptor choice, RANSAC params, multi-restart, multi-scale,
  ICP variants, candidate pruning, alternative algorithms (FGR, etc.). The
  function signature `relocalize(global_map, local_map) -> 4x4 ndarray`
  must stay.

### What you CANNOT do

- Modify `run.py` or anything in `data/`. The evaluation
  function, success thresholds (1m / 15°), and time budget are fixed.
- Add new package dependencies. Use only what's already in
  `pyproject.toml` (notably `open3d`, `numpy`, `scipy`).
- Pre-compute results outside `relocalize()` (e.g. caching the global
  map's FPFH features across calls is fine *within one run*; the
  evaluator instantiates fresh state per process).

### Goal

Maximize **`success_rate`** (fraction of frames within 1m translation
AND 15° rotation of groundtruth) across the 20 test frames, within the
5-minute total budget. Higher is better.

Metric hierarchy (apply in order — only consult the next when the
previous is tied):

1. **`success_rate`** — higher is better. Primary.
2. **`median_distance`** — lower is better. Translation tightness once
   you're succeeding.
3. **`total_seconds`** — lower is better. Speed only matters as a
   tiebreaker; never sacrifice accuracy for it.

`average_distance` is reported for debugging only — do **not** optimize
for it directly. One catastrophic 50m outlier dominates the mean even
when 19/20 frames are perfect, so it punishes the wrong things.

Eval is fully deterministic: `run.py` seeds Open3D's RNG per-frame, so
RANSAC variance is zero between runs of the same `relocalize.py`. Any
metric change you see is a real algorithmic change.

### Simplicity criterion

All else being equal, simpler is better. A 0.01m improvement that adds
30 lines of duct tape isn't worth it. Removing complexity and getting
equal-or-better results is a win on its own.

### First run

Always begin by running `run.py` against an unmodified `relocalize.py` to
establish a baseline. The shipped baseline (multi-scale FPFH+RANSAC + ICP
+ gravity prior) gets roughly `success_rate ≈ 0.45` (9/20 frames),
`median_distance ≈ 1.07m`, `average_distance ≈ 4.36m` on this dataset.

## `relocalize()` signature contract

```python
def relocalize(
    global_map: open3d.geometry.PointCloud,   # target, in world frame
    local_map:  open3d.geometry.PointCloud,   # source, in body frame
) -> numpy.ndarray:                            # shape (4, 4), homogeneous
    ...
```

The returned 4x4 matrix `T` must transform body-frame points to world-frame:
`world = T @ [body; 1]`. If your code returns a wrong shape or raises, the
evaluator logs `crash` and skips that frame.

The function may take seconds per call — that's expected. The 5-minute
budget covers *all* 20 calls; if it expires mid-run, the evaluator stops
and reports metrics over whatever frames completed. So you can also
spend more time per frame at the cost of fewer frames evaluated; the
average is over those completed.

## Output format

After the run, `run.py` prints a summary block:

```
---
average_distance:    8.142347
median_distance:     6.886670
average_rotation:    78.51
success_rate:        0.3000
total_seconds:       20.0
avg_call_seconds:    1.00
num_frames_done:     20
num_frames_total:    20
num_crashed:         0
all_distances:       [15.477, 0.255, ...]
all_rotations:       [95.8, 4.2, ...]
```

Extract the headline metric:

```
grep "^average_distance:" dimos/mapping/relocalization/run.log
```

## Logging results

After each run, append one TSV row to `dimos/mapping/relocalization/results.tsv`
(tab-separated; commas occur inside descriptions and will break CSV):

```
commit	success_rate	median_distance	average_distance	total_seconds	status	description
```

1. git commit hash (7 chars)
2. `success_rate` (e.g. `0.45`) — `0.0` for crashes — **primary metric**
3. `median_distance` (e.g. `1.073`) — `0.0` for crashes — tiebreaker
4. `average_distance` (e.g. `4.359`) — `0.0` for crashes — debug only
5. `total_seconds` (e.g. `289.4`) — `0.0` for crashes
6. status: `keep`, `discard`, or `crash`
7. short text description of what was tried

Example:

```
commit	success_rate	median_distance	average_distance	total_seconds	status	description
a1b2c3d	0.4500	1.073	4.359	64.4	keep	baseline multi-scale ransac
b2c3d4e	0.6000	0.812	3.110	240.3	keep	added FGR + filter ICP at 0.2
c3d4e5f	0.4000	1.450	5.220	298.0	discard	dropped FGR (made it worse)
d4e5f6g	0.0	0.0	0.0	0.0	crash	teaser++ import (not installed)
```

## The experiment loop

LOOP FOREVER:

1. Inspect git state — which branch + commit you're on.
2. Edit `relocalize.py` with one experimental idea. Keep changes focused.
3. `git add dimos/mapping/relocalization/relocalize.py && git commit -m "<idea>"`
4. Run: `uv run dimos/mapping/relocalization/run.py > dimos/mapping/relocalization/run.log 2>&1`
5. Pull metrics: `grep -E "^(success_rate|median_distance|average_distance|total_seconds):" dimos/mapping/relocalization/run.log`
6. If the grep is empty, the run crashed. `tail -n 50 dimos/mapping/relocalization/run.log`
   to read the traceback and try to fix.
7. Append a row to `results.tsv`.
8. Compare against the last `keep` row using the metric hierarchy:
   `success_rate` (higher wins) → `median_distance` (lower wins) →
   `total_seconds` (lower wins). If the new commit wins, **advance** —
   keep it. If it loses or strictly ties on all three, `git reset --hard
   HEAD~1` and try a different idea.

**Timeout:** Each run should finish in ~5 minutes (enforced inside run.py).
If a run exceeds 10 minutes wall-clock (e.g. you accidentally disabled the
budget), kill it and treat as failure.

**Crashes:** If the crash is dumb (typo, missing import you can fix
without adding a dependency), fix and re-run. If the idea is fundamentally
broken (e.g. needs a non-installed package), log `crash` and move on.

**NEVER STOP**: once the loop has begun, do **not** pause to ask the human
whether to continue. The user may be asleep. You are autonomous. If you
run out of ideas, re-read this file, re-read `relocalize.py`, re-read the
Open3D docs linked below, look for combinations of previous near-misses,
or try a more radical change (different solver, different descriptor,
multi-restart, candidate re-ranking). The loop runs until manually
interrupted.

## References

Open3D point-cloud registration:

- **RANSAC with feature matching** (the baseline's core call):
  https://www.open3d.org/docs/latest/python_api/open3d.registration.registration_ransac_based_on_feature_matching.html
- **Global registration tutorial** (FPFH preprocessing, RANSAC, FGR, ICP refinement):
  https://www.open3d.org/docs/release/tutorial/pipelines/global_registration.html
- **ICP variants** (point-to-point, point-to-plane, generalized ICP / GICP):
  https://www.open3d.org/docs/latest/python_api/open3d.pipelines.registration.registration_icp.html
- **FPFH feature**:
  https://www.open3d.org/docs/latest/python_api/open3d.pipelines.registration.compute_fpfh_feature.html
- **FGR (Fast Global Registration)**:
  https://www.open3d.org/docs/release/python_api/open3d.pipelines.registration.registration_fgr_based_on_feature_matching.html

Failure modes you'll likely see on this dataset:

- **Z-only matches dominate** — the map is very flat (walls only ~1m
  tall), so RANSAC will gleefully pick correspondences that agree only
  on z (floor↔floor, ceiling↔ceiling). The resulting transform has
  near-arbitrary (x, y, yaw) but reports high fitness. Mitigations:
  reject candidates whose inlier set is z-degenerate (low spread in xy),
  use `CorrespondenceCheckerBasedOnEdgeLength`, or remove floor/ceiling
  points before feature extraction.
- **180° yaw flips** — the office has corridor symmetry. RANSAC happily
  matches walls running in either direction.
- **Wrong-room matches** — repetitive layout means FPFH descriptors
  aren't globally unique.
- **High RANSAC fitness paired with low ICP fitness at fine scale** —
  coarse-scale "perfect match" collapses to <5% inliers at 2cm. Use
  fine-ICP fitness (or stricter inlier ratio) for candidate ranking
  rather than RANSAC's own fitness.

Ideas worth trying (not exhaustive):

- Multi-restart RANSAC with different seeds; rank by ICP fitness at a tight
  threshold rather than by RANSAC's report.
- FGR alone or in parallel with RANSAC; pick best by ICP fitness.
- Tighter / looser `CorrespondenceCheckerBasedOnEdgeLength`.
- Reject obviously-wrong yaws via a gravity / ground-plane prior
  (currently a simple z-tilt filter — could be tightened).
- Pre-extract a single set of FPFH features on `global_map` and cache it
  in a module-level variable so repeat calls are cheaper. (Within one
  process; new runs always re-import.)
- Different voxel sizes for the FPFH/RANSAC stage vs ICP refinement.
- Increase RANSAC iterations (currently 500k).
- Different `ransac_n` (3 vs 4).
- Different ICP variants (point-to-plane, generalized ICP, colored ICP).
