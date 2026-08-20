# Why the robot kept changing its mind — `20260805-033007.zenoh.mcap`

86 s of `go2-zenoh-motion` in a cluttered room: 84 local maps, 428 published local
plans (57 of them single-pose holds), 77 global paths, 3 goals reached. Symptom:
the robot weaves and stalls in front of an obstacle instead of committing.

Every number below is over the **410 ticks whose inputs are all in the file** —
the first 18 plans predate the recording's first `planner_path`, so their carrot
cannot be reconstructed. That set holds 39 of the 57 holds.

Reproduce everything below with

```bash
python -m dimos.navigation.motion.adapter.diagnose ml-trajectory-research/20260805-033007.zenoh.mcap
python -m dimos.navigation.motion.adapter.diagnose ml-trajectory-research/20260805-033007.zenoh.mcap \
    --only replay --z-offset 0.29        # the counterfactual in §4
```

## Verdict

The planner is not the unstable part — **its input is**. In order of contribution:

1. The plan only changes when a **new local map** arrives. Between maps it is
   steady to 0.15 m; across a map boundary it moves 0.85 m on average and flips
   outright 10× as often.
2. A one-input-at-a-time ablation on the replayed planner puts **97 % of the flip
   magnitude on the cloud** (2.99 m of 3.13 m); pose contributes 0.07 m, the
   carrot 0.14 m.
3. The cloud moves that much because the raycaster's emitted window **breathes**
   (radius 2.53 → 5.88 m, up to 2.36 m in one frame — 68 % of all voxel churn)
   and because 10 % of the map genuinely turns over every frame.
4. On top of that the planner reads **the wrong 0.4 m slice** of it: its body band
   sits 0.33–0.73 m above the floor instead of 0.05–0.45 m. It is blind to the
   bottom third of every obstacle and steers off table-height clutter.

Latency, pose jumps, carrot jumps and planner nondeterminism are all ruled out
below.

## 1. The stack replays exactly, so the offline answer is the on-robot answer

Re-planning every recorded tick from its own recorded inputs (latest local_map at
the tick, odometry resolved into `base_link` off the recorded tf, carrot 5 m of
arc along the latest `planner_path`) reproduces what the robot published:

| | |
|---|---|
| replay vs recorded plan | mean **0.13 m**, median 0.04 m, p90 0.12 m |
| holds | 39 recorded / 39 replayed, 37 the same ticks |
| same inputs twice | **bit-identical** — the rust `target` episode is stateless and deterministic |
| plan cost | 30 ms/tick (5 Hz budget is 200 ms) |

This also pins the deployed config: the replay matches at `cloud_z_offset = 0.0`,
the blueprint default (§4 is about that being wrong, not about it being unknown).

**Nondeterminism is not the cause.** Identical inputs give identical plans.

## 2. Attribution: it is the cloud

370 consecutive tick pairs, re-planned four times each — all inputs moved, then
only the cloud, only the pose, only the carrot:

| divergence from the previous tick's plan | all pairs | the 25 flips > 0.5 m |
|---|---|---|
| all three inputs move (the real change) | 0.272 m | 3.126 m |
| **cloud only** | **0.212 m** | **2.989 m** |
| pose only | 0.058 m | 0.073 m |
| carrot only | 0.018 m | 0.137 m |

24 of the 25 flips land on a tick where a new local map had arrived. Across a
flip the robot had moved **0.072 m** and the carrot **0.14 m** — neither is a
jump, and the MLS global path holds still for seconds at a time.

Recorded plans, split by whether the local map changed under them:

| | pairs | mean divergence | p90 | flips > 0.5 m |
|---|---|---|---|---|
| same local map | 296 | 0.15 m | 0.16 m | 8 (2.7 %) |
| new local map | 66 | **0.85 m** | 3.83 m | **18 (27 %)** |

The planner runs at 5 Hz over a 1 Hz map, so four ticks in five re-solve an
unchanged world and the fifth can jump. That is the ~1 Hz twitch the operator
sees.

The plans that result are long: mean arc **7.0 m for a carrot 2.8 m away**
(detour ratio mean 2.34×, p90 4.78×, max 7.21×), and the published plan's length
steps by 0.87 m on average and by up to **22.2 m** — the planner is swapping
between "squeeze past" and "go all the way around", not nudging a line.

## 3. What the local map does per frame (0.08 m voxels)

| | |
|---|---|
| map size | 28 078 voxels |
| appear / disappear per frame | 2 872 (10.2 %) / 2 738 (9.8 %) |
| of that, **interior** (well inside both frames' windows) | 958 / 827 = **32 %** |
| **crop boundary** | **68 %** |
| emitted window radius | 2.53 – 5.88 m, per-frame step mean 0.51 m, **max 2.36 m** |
| planner band (z 0.05–0.45) | 2 405 voxels, 11.0 % appear / 10.3 % disappear per frame |
| interior vanishings with < 4 occupied neighbours | 15 %, median 2.07 m from the sensor |

The boundary half is `RayTracingVoxelMapConfig.region_percentile = 95`: the
emitted cylinder is sized to the 95th percentile of the *last ten sweeps'* point
distances, so an open doorway or a long corridor glimpse inflates it by 2 m and
the next batch collapses it again. Every collapse deletes thousands of voxels the
planner was routing around, and every expansion invents them back.

The interior third is genuine map maintenance (raycast clearing and re-seeding)
plus a thin-obstacle residue: 15 % of interior vanishings are voxels with fewer
than `support_min = 4` neighbours — exactly the class that `emit_points` only
publishes on the frame their sweep hits them (the "chair leg / box edge" case
the thin-obstacle fix in `voxel_ray_tracer.rs` addresses). The fix is present in
this recording's behaviour — isolated voxels *do* appear — but it only survives
for the one emitted frame that saw them, so they still blink at ~1 Hz.

Rerun (`recordings/20260805-033007.zenoh-diagnose.rrd`) shows this per frame:
`world/appeared` green, `world/disappeared` red over the grey map, with the
recorded plan (blue) and the replayed plan overlaid.

## 4. The planner's body band is 0.29 m too high

`planners/target.py` slices the cloud at `Z_BAND = (0.05, 0.45)` — absolute z,
assuming the plan poses sit on the ground. In this recording they do not:

> Since superseded twice over: the band moved into `motion/obstacles.py` as the
> `raw_band` model, and the search then stopped slicing at all — it takes
> obstacle xy and nothing else. `RAW_BAND` is the only place this span is
> written now.

- floor in the map, 2nd percentile within 1.5 m of the robot: **z = −0.28 m**
- `base_link` resolved off tf: z = +0.05 m (`ROBOT_HEIGHT 0.45 − mount 0.16 = 0.29 m`
  above ground — the odom origin is at base height, not at the floor)
- so the planner's obstacle slice is **0.33 – 0.73 m above the floor**

The Go2 collision box (`GO2_BODY`, 0.85 × 0.50 × 0.40, lifted to sit on the
ground) occupies 0.00 – 0.40 m above the floor. The planner is therefore blind to
everything below 0.33 m — steps, box edges, chair and table legs, the bottom of
every wall — and treats table tops and chair backs at 0.4–0.7 m as the world.
Anchoring the band to the floor instead would put **6 996 voxels in view instead
of 2 405 (2.9×)**.

The knob exists: `MotionPlannerConfig.cloud_z_offset`, left at 0.0 by
`go2_zenoh_motion`. But it is a knife edge, not a dial — replaying the whole
recording at each offset:

| `cloud_z_offset` | holds (of 410 ticks) | mean plan arc |
|---|---|---|
| 0.00 (deployed) | 39 | 6.81 m |
| +0.20 | 243 | 2.44 m |
| +0.29 (floor-anchored) | **410 — every tick refuses** | 0 m |

At +0.29 the floor's own voxel slab (centres at −0.28, ±0.04 of quantisation)
lands inside the band and the robot is walled in. So: the band must be anchored to
an *estimated* floor with the ground slab excluded — which
`adapter/replay.py` already does (5th percentile of the neighbourhood within
2.5 m) — not to a hand-set constant.

Note what +0.20 says about the room: with a merely *less wrong* band the planner
refuses 59 % of ticks. The corridor it was asked to drive is genuinely tight at
body height; the deployed band hid that by looking over the obstacles and let it
weave through gaps that are not there.

## 5. Latency and cadence are healthy — not the cause

| stream | n | median Δt | p95 | max |
|---|---|---|---|---|
| local_map | 84 | 1030 ms | 1147 ms | 1248 ms |
| odometry | 2594 | 33 ms | 35 ms | 146 ms |
| planner_path | 77 | 1023 ms | 1229 ms | 4933 ms |
| path (out) | 428 | 200 ms | 280 ms | 479 ms |

Age of the input each tick actually planned on: local_map mean 529 ms / max
1248 ms, odometry mean 18 ms / max 49 ms, planner_path mean 690 ms / max 4803 ms
(one MLS re-route gap). Nothing approaches `max_map_age_s = 5.0`, so none of the
holds are stale-map holds — every hold in this recording is the planner saying
*no route*. No dropped-link stalls, no publish gaps.

## Fixes, ranked by expected effect

1. **Anchor the obstacle band to the floor**, per tick, with the ground slab
   excluded — not `cloud_z_offset` as a constant (+0.29 refuses everything).
   Today the planner steers off a slab 0.33 m over the robot's head-room and is
   blind to the bottom 0.33 m of every obstacle.
2. **Stop the emitted window from breathing.** A fixed local-map radius (or
   hysteresis on `region_percentile`) removes 68 % of the churn the planner sees,
   for free.
3. **Gate replanning on map arrival** (or raise the map rate: `emit_every = 10`
   at 10 Hz lidar is a 1 Hz map under a 5 Hz planner). Between maps the plan is
   already stable to 0.15 m, so four ticks in five are wasted work whose only
   output is jitter.
4. **Give the search commitment.** The `target` episode is stateless: identical
   inputs → identical plan, but a 10 % map turnover buys a homotopy switch at zero
   cost. `AvoidanceConfig` already carries `side_gain` / `turn_dir_hysteresis`
   for the gradient planner; the rust SE(2) search uses neither.
5. **Carry live voxels for a few frames** in `emit_points` — a thin obstacle that
   is only real on the sweep that hit it still blinks at the emit rate (15 % of
   interior vanishings).

## Fixes landed

Fixes 1–3 above are in. Replaying the same 410 ticks, one search per tick, all
four numbers from `adapter/diagnose.py` on the same code path:

| | plans published | holds | flips > 0.5 m | mean divergence | mean plan arc |
|---|---|---|---|---|---|
| **baseline** (raw band, 5 Hz) | 410 | 39 | 25 | 0.275 m | 7.53 m |
| **fix 1** floor-anchored, 5 Hz | 410 | **175** | **18** | 0.299 m | **6.61 m** |
| fix 3 raw band, gated | 107 | 10 | 27 | 1.017 m | 8.27 m |
| **fix 1 + 3** anchored, gated | **107** | 42 | 17 | 0.908 m | 7.63 m |

(arc is over the ticks that produced a plan; the carrot averages 2.82 m.)

Read it in flips per MINUTE, not per published plan — gating shrinks the
denominator to only the pairs that could ever differ, so a per-pair divergence
is not comparable across the two cadences. Over the recording's 82.6 s:
**18.2 flips/min baseline → 13.1 with the band anchored → 12.3 anchored and
gated**, on 74 % fewer published plans and 74 % less search.

**The holds go UP, and that is the fix working.** §4 predicted it: at a merely
less-wrong band (+0.20) the planner already refused 59 % of ticks, because the
corridor it was driven through is genuinely tight at body height and the
deployed band hid that by looking over the obstacles. Anchored, the band sees
clutter 0.2–0.5 m from the robot on essentially every tick — real returns,
clustered by direction, not a ground carpet. A planner that refuses where there
is no room is correct; making it move anyway is the recovery layer, and that is
a separate piece of work.

Two things the numbers here cannot say:

- **Fix 2 is unscoreable on this recording.** The breathing window is baked into
  the local maps it contains; a fixed emit radius changes what the mapper
  publishes, not how the planner reads what was published. It is covered by
  `voxel_ray_tracer.rs::region_radius_tests` instead — the property pinned there
  is that the same map cropped by a near batch's window and by a far batch's
  window is the same set of voxels, which is exactly the 68 % of churn §3
  attributes to the boundary.
- **The ground margin is 0.16 m, two voxel layers, not one.** A floor whose true
  height sits near a voxel boundary quantises into both layers either side of
  it. At one voxel the robot is inside its own band on 100 % of ticks here and
  every tick refuses — the +0.29 failure again, one layer lower. At two it is
  7 %.

**The follower is now on the same band.** Its room hint
(`adapter/follower.py::_clearance_for`) anchors the local map through the same
`FloorAnchor`, off the same tf prior, before `path_clearance` slices `Z_BAND`
out of it (that second slice is gone now: `path_clearance` measures the model's
hard set as given) — so its speed governor and the planner's stamped profile measure one
world again. Unanchored (no `lidar_height`, no mount leg, or
`floor_anchor=False`) it degrades to the raw band exactly as the planner does,
and on a floor already at zero it is a no-op, so the referee's sim scores cannot
move. This recording cannot score it: the replay is a planner harness and the
follower's twist is not in the file — every number above is the planner's, and
re-running `--only replay` after the change reproduces them unchanged.

## Superseded: fix 1 is now the body, not an estimate

Everything above about `FloorAnchor` / `estimate_floor` is history. Two robot
recordings the next morning (`20260805-173105`, `-173133`) settled it: the
deployed anchor **never engaged**. `diagnose.py`'s config sniff, replaying each
tick under each band and keeping the one whose holds agree with what was
published, says raw 18/18 and 21/21 against anchored 9/18 and 10/21 — and 40/41
against 25/41 on this recording. The knob was on and the band was raw anyway,
because anchoring needs a tf prior it silently degrades without, and an
estimator that fails quietly into exactly the behaviour it was written to
replace is not a reference.

The replacement is `motion/obstacles.py`: the base rides `emb.base_height`
above the surface its feet stand on, so `ground_z = base_z - base_height` and
the cloud is re-referenced to it. No scene estimate, no prior, no tolerance, no
degradation mode. The 0.16 m ground exclusion survives — that number was about
voxel quantisation, not about anchoring, and the second bullet above still
holds. `--model raw_band` replays the band this recording actually ran.

Replaying all 410 ticks under it, against the "Fixes landed" table above:

| | holds | mean plan arc | mean divergence |
|---|---|---|---|
| raw band (what the robot ran) | 39 | 6.96 m | — |
| floor-anchored (deleted) | 175 | 6.61 m | 0.299 m |
| **body_band** | **133** | **5.25 m** | 0.29 m |

Same regime as the estimator, slightly less blocking and a shorter plan: the
per-tick quantile wandered, a body reference does not.

## Artifacts

- `recordings/20260805-033007.zenoh-diagnose.rrd` — per-frame appeared/disappeared
  voxels, recorded vs replayed plan, robot, carrot
- `recordings/20260805-033007.zenoh-diagnose/{churn,plans,latency}.svg`
- tool: `dimos/navigation/motion/adapter/diagnose.py` (`--only`, `--model`,
  `--z-offset`, `--spawn`, `--no-ablate`)
