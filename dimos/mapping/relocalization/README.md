# Relocalization: mode-switched tracking

Adds a cheap "tracking" mode to `RelocalizationModule`, alongside the existing
full FPFH+RANSAC+ICP global search, plus a ground-truth benchmark harness to
validate the trade-off instead of only trusting self-reported ICP fitness.

## How to run it

Everything below assumes `uv` is installed and you're in the repo root
(`uv run` auto-syncs dependencies on first use — no manual `pip install`
needed). First run downloads `go2_hongkong_office.db` and its premap from
Git LFS automatically.

**One-shot comparison** (recommended — runs both modes, prints a table, saves plots):

```bash
./dimos/mapping/relocalization/compare.sh go2_hongkong_office go2_hongkong_office_twopass_map 15
```

Writes logs and 4 PNGs (per-attempt latency, per-attempt translation error,
total-wall-time bar chart, fitness-vs-real-error scatter) to a fresh
`/tmp/reloc_comparison_<timestamp>/`.

**Either mode individually:**

```bash
PYTHONUNBUFFERED=1 uv run python -m dimos.mapping.relocalization.eval go2_hongkong_office \
  --map-file go2_hongkong_office_twopass_map --mode global --max-attempts 15   # baseline

PYTHONUNBUFFERED=1 uv run python -m dimos.mapping.relocalization.eval go2_hongkong_office \
  --map-file go2_hongkong_office_twopass_map --mode tracking --max-attempts 15  # our method
```

(`PYTHONUNBUFFERED=1` matters if you're piping/teeing output — Python fully
buffers stdout when it isn't a TTY, so without it you won't see anything
until the process exits.)

`--max-attempts` caps the run — each full search costs 15-70s depending on
machine load, so a 15-attempt run is a reasonable smoke test; drop it to
benchmark the whole recording.

To re-plot from existing logs without re-running the (slow) benchmark:

```bash
uv run python -m dimos.mapping.relocalization.plot_comparison \
  --global-log <path> --tracking-log <path> --out-dir <dir>
```

## The gap this fills

Went looking for gaps in the navigation/relocalization stack before landing
here (see `docs/capabilities/navigation/relocalization.md` for how the
existing system works). Several candidates came up: no ground-truth eval for
relocalization accuracy (only self-reported ICP fitness), no pose-jump/drift
detection on the published TF, no tracking/warm-start mode (every cycle pays
the full global-search cost), no multi-premap support, no last-known-pose
persisted to non-volatile storage across power cycles to seed the next
session's relocalization search instead of starting cold, no richer
localization-quality signal exposed downstream. Picked the tracking/warm-start
gap specifically, for reasons below.

## Why this feature, and why not the alternatives

**Scoped for a personal laptop, no GPU dependency, and a tight time budget.**
`relocalize()`'s actual registration work (FPFH, RANSAC, ICP) runs through
Open3D's legacy **CPU** pipeline, not the CUDA tensor API — only the
`VoxelGrid` map accumulation uses GPU, and that's not the bottleneck. So this
feature needed no GPU passthrough, no simulation environment, no live robot —
just a recorded dataset already sitting in Git LFS, replayed offline. That
made it tractable to build and validate end-to-end in the time available.

**Purposefully not pursuing a live factor-graph approach.** The existing
system already uses factor graphs (GTSAM) for premap construction — see
`dimos/mapping/loop_closure/pgo.py` — but strictly offline: it needs to see
the *whole* recording, including future loop closures, to correct past poses.
Building an online, live factor-graph-based correction system (continuously
re-optimizing a pose graph against streaming relocalization observations)
would be a substantially larger undertaking — new state management, new
failure modes, much more surface area to validate — and didn't fit a
few-hour scope. The mode-switch approach reuses the existing, well-tuned
global search unchanged and adds a much smaller, bounded piece of new logic
on top.

**Purposefully not pursuing a LIO-SAM-style integration.** This codebase
already has FAST-LIO2 and Point-LIO (`dimos/hardware/sensors/lidar/`) for
platforms with a real 3D LiDAR (mid360 rigs, the G1 humanoid) — LIO-SAM would
mean bolting loop closure onto one of those existing LIO front-ends. That's
real work, but it's *integrating an existing, well-documented open-source
system* rather than a feature we could reason about and validate ourselves in
this scope — and it doesn't even apply to the dataset used here (this
recording comes from the standard WebRTC-based Go2 pipeline, not a
mid360/FAST-LIO one), so it wouldn't have been testable with the data at
hand.

## What I hoped to achieve

The measured baseline showed `relocalize()` costing 13-70s wall-clock per
call (machine-load dependent) against a nominal 2-second re-localization
budget (`RELOC_INTERVAL`) — meaning the live system cannot keep
up with its own throttle interval. Goal: cut that cost dramatically for the
common case (already have a recent, good lock) while keeping accuracy in the
same ballpark, by reusing the last good alignment as a seed for a cheap local
ICP instead of re-running the full global search every cycle — falling back
to the global search whenever that seed can no longer be trusted.

## Architecture

- **`relocalize.py`**: added `track(global_map, local_map, seed_T, yaw_fan_deg)`
  — no FPFH, no RANSAC, a small fan of seeded ICP attempts around `seed_T`,
  scored by the same wall-only fitness trick the existing global search
  already uses (to avoid floor/ceiling rotational-symmetry masking a
  wrong-yaw candidate). `_wall_subset` was moved to module level so both
  paths share it.
- **`module.py`**: `RelocalizationModule` now tracks `last_good = (T, fitness,
  ts)`, a missed counter, and the position/time of the last full search.
  `should_force_full_search()` decides SEARCHING vs. TRACKING each cycle
  (no lock yet / stale lock / too many misses / too long or too far since an
  absolute reacquisition). TRACKING seeds `track()` from `last_good.T`
  directly. SEARCHING calls the original `relocalize()`, unchanged.
- **`eval.py`**: offline ground-truth benchmark. Replays a recording's own
  `lidar` stream through the same accumulation logic `VoxelGridMapper` uses
  live, calls `relocalize()`/`track()` at the same cadence, and compares the
  output against the *recording's own* PGO-corrected trajectory —
  `PoseGraph.correction_at(ts)` which is the transform relocalization is
  trying to estimate, so replaying a recording against a premap built from
  itself gives a ground truth substitue with no extra data collection.
- **`compare.sh` / `plot_comparison.py`**: run both modes back-to-back and
  turn the logs into a comparison table + plots.

## Observations

- **~48-67% reduction in total wall-clock time** across a 15-attempt run
  (exact number depends on how often the reset get triggered — see tuning
  below), with per-tracking-attempt latency dropping from 13-20s to ~0.05-0.1s.
- **Tracking-mode accuracy lands within a modest, bounded margin of the
  baseline** — typically 20-80mm worse median translation error,
  under a degree worse median yaw error.
- **Two regressions found and fixed during development, both worth knowing
  about**: (1) briefly tried seeding the tracking-mode search on the robot's
  own odometry-measured heading change, which caused yaw errors up to ~100°
  — a real robot turn is already correctly represented in the accumulated
  map via per-frame odometry placement, so re-centering the search on "how
  much did the robot turn" double-counts real motion as if it were error.
  (2) A subsequent fix (widening the search fan based on distance/time since
  the last successful attempt) turned out to silently reset every ~2 seconds
  because `last_good` updates on every tracking success too — so the fan
  almost never actually widened across a streak. Fixed by keying the
  fan-width calculation off the last *full search* instead, which only
  resets on an absolute reacquisition. 
- **The residual accuracy gap looks structural, not something a
  reset-cadence knob can fix.** Even the untouched global search's own
  accuracy trended worse (~0.34m → ~0.42m over a 30-second
  replay window, no tracking mode involved at all) — pointing at the
  accumulated map's own shape evolving as column-carving keeps replacing
  older, less-drifted regions with newer, more odometry-drift-affected ones.
- **Self-reported ICP fitness does not reliably reveal this kind of drift.**
  Within a tracking streak, fitness *increased* (0.92→0.96) while real
  ground-truth error also increased (0.37m→0.45m) — ICP reports how well it
  converged to something nearby, not whether that something is still
  correct. `fitness_vs_trans_err.png` from `compare.sh` shows this visually.

## Known limitations / untested edges

- `MAX_TRACKING_MISSES` (3) has never actually fired in any eval run — every
  tracking attempt across every run so far has been accepted, so this
  boundary condition is unvalidated.
- `SANITY_CHECK_INTERVAL` (30s) exists for the "robot barely moving, or
  picked up while stationary" case — the test recording keeps moving
  continuously, so the distance trigger always fires first and this constant
  has never actually been exercised. Kept close to `MAX_TRACKING_AGE` for
  internal consistency, not validated against data.
- The yaw fan is yaw-only — there's no equivalent perturbation for translation.
- Ground truth requires the premap to be built from the *same* recording
  being replayed (for the `correction_at(ts)` trick to hold) — this eval
  harness can't currently validate against an independently-collected premap.

## Scope of improvement

- **A translation-perturbation fan**, mirroring the yaw fan, sized similarly
  by distance/time since the last full search.
- **A heatmap of where fitness/error consistently degrades**, using the
  per-attempt logs already being produced, as a tool to flag premap regions
  needing a re-scan.
- **Classify LiDAR points as static vs. dynamic** before the ICP correspondence 
  search, so transient obstacles that never appear in the static premap don't
  get treated as failed or false correspondences in scenes with heavier
  traffic than the built map.
- **Tune the drift-rate assumption per robot type, instead of one fixed 
  number for every platform** The current constant (0.3°/m) was only
  tested on the Go2 — a quadruped, a humanoid, and a drone all drift
  differently, so this number likely won't transfer as-is. Not a quick
  fix, though: it needs real uncertainty data from IMU/odometry, and most
  robot drivers in this codebase don't currently produce that. The actual
  blocker is missing sensor infrastructure elsewhere, not something
  fixable inside this feature alone.
