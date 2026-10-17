# eval_loop: gsc_pgo vs Ivan's go2 PGO

## Claims

Who built what, and when: gsc is the author's, first working on 2026-03-14 and ported to Rust in July. Ivan's
`pgo.py` dates from 2026-05-09 and his live module (`PGOMap`) merged on 2026-10-01 (#4331
leshy/feat/go2-realtime-pgo, "plain go2 real time loop closure"). This benchmark started on 2026-10-13, 12
days after that merge. About 20 gsc variants were tried on these recordings during it (seed, yank, strong
accept, GNC thresholds, an Ivan-like config) and one default was changed (below); Ivan's PGO runs at his two
fixed configs.

Results files are in `$EVAL_LOOP_OUT`: `<dataset>.summary.json` (single runs), `<dataset>.repeat.json`
(Ivan's run-to-run range: 10 runs, 5 on the huge loops), `huge_loop_pointlio.timing_compare.{json,html}`,
`huge_loop_pointlio.mls_summary.json`.

0. **The design of Ivan's live module is wrong for a planner: it outputs the whole global map constantly.**
   It publishes the whole corrected map every 5th frame (twice a second) and on every loop-closure rebuild,
   and offers nothing smaller: no correction to apply, only the map and its keyframe poses. A planner that
   keeps its own map can only take that as a full rebuild. On main, `unitree-go2-pgo` feeds it to the 2D
   costmap and A*, not to MLS; this measures what it would cost the MLS planner (main's go2 MLS settings)
   (`mls_load.py`, `mls_report.py`, huge_loop_pointlio, 60 min):
   - Taking every publish of his map: MLS `update_global_map` needs a median 8.2 s per publish (up to 15 s,
     about 1.4 s per million points, maps up to 10M points), so it is behind from minute 2. Downsampling
     his 5 cm map to the planner's 8 cm first barely helps (7.7 s).
   - Taking only his 120 rebuilds (one per 10 s cooldown at most): 949 s of MLS per hour (894 s
     downsampled), each a median 8.2 s full rebuild.
   - Whole-map rebuilds cost about the same whatever PGO is behind them: gsc's closures taken that way (a
     full rebuild of the ray tracer's map after each, same 10 s cooldown, 284 rebuilds from 353 closures)
     cost 1156 s per hour.
   - Patching in place instead (only the 4 m cells holding a keyframe that moved more than a voxel, plus
     their neighbors, go to `update_region`, same cooldown): gsc's 197 corrections cost MLS 19 s per hour
     (median 0 cells, p90 0.28 s, worst 2.5 s). Ivan's 119 rebuilds patched the same way cost 43 s per hour
     (median 914 cells, 0.24 s; worst 3.3 s): his map is re-anchored to the robot, so more of it moves per
     correction. Read from his start-anchored poses instead, his corrections move about as little as gsc's
     (a reviewer's re-alignment: median keyframe motion per rebuild 0.09 m -> 0.00 m), so the 2x is mostly his
     choice of map anchor, not the PGO.
   - Not measured: correcting the stored points themselves (the patched cells are re-read from the ray
     tracer's map as they are), only what the planner pays. gsc publishes its correction for something
     like [fast-reform](https://github.com/jeff-hykin/fast-reform) to apply; that isn't on main yet. Ivan's
     module publishes no correction to apply, only the corrected map and its keyframe poses.
   - For scale, the per-frame `update_region` with the ray tracer's local cylinder, which main's go2 nav
     blueprints already run and which both designs need, costs a median 18-29 ms per frame depending on
     machine load (p99 58-107 ms), 760-1220 s per hour.
   - Goals: MLS holds a goal as a fixed point in the map frame. Ivan's map is anchored at the robot and
     re-corrected on every rebuild, so a spot in it moves a median 0.7 m a minute later, 3.6 m after five
     minutes and 4.7 m after fifteen (90th percentile 3.3, 6.7 and 9.1 m, his keyframes across 120
     rebuilds), unless the goal is updated from his `pgo_keyframes`, which nothing on main does. gsc's map
     frame is fixed at the start, so a goal held in it and converted through `map -> odom` moves only by the
     corrections themselves: a median 0.07, 0.35 and 1.2 m (90th percentile 0.8, 2.2 and 6.8 m, its
     keyframes across 373 closures).

   ![MLS planner seconds per hour of driving for each way of taking loop closures](figures/claim0_planner_load.png)
1. **Live, gsc keeps up and Ivan's module doesn't.** Both live modules replayed frame by frame on
   huge_loop_pointlio (60 min, ~3 km) on an Apple M5 Max (`live_timing.py`, `pgo-eval`'s `frame_timing`,
   `compare_timing.py`). Latency = time from a scan's arrival until the output that includes it is
   published, through each module's own queue: Ivan's keeps only the newest waiting scan (`backpressure`,
   `module.py:83`) and outputs the global map; gsc's is a 100-deep FIFO (`max_scan_queue`) and outputs its
   corrected odometry every scan, from its pipeline thread (its GNC solve and the iSAM2 rebuild after it run
   on its worker; adopting a solve is a swap, under 1 ms).
   - Ivan's live module: median 98 ms, p99 1.7 s, worst 3.5 s; 48% of outputs take over 100 ms and 107
     over a second, and 6.5% of scans are dropped. Per frame it costs 18 ms over the first 10 min and 73 ms
     over the last (worst frame 7.4 s), and the process grows to 13.4 GB. Nearly all of that is its map:
     every frame goes into a 5 cm voxel grid that keeps each frame's points, and every loop re-projects all
     of it.
   - gsc's live module: median under 1 ms, p99 0.18 s, worst 0.53 s; 4.2% of outputs over 100 ms, none
     over a second, no scans dropped. 9 ms per frame over the first 10 min, 15 ms over the last; 560 MB.
     gsc was measured at load average about 32 with other jobs running and Ivan in an earlier, separate run;
     each is one replay. They leave out the map and corrector gsc's design relies on downstream (claim 0).

   ![Output latency over the drive, Ivan's live module vs gsc's](figures/claim1_latency.png)
   Interactive per-frame version: `huge_loop_pointlio.timing_compare.html` (`compare_timing.py`).
2. **gsc closes big loops, even from the go2's own lidar and leg odometry over a walk measured in miles.** On
   a 53 min, ~3 km (1.9 mi) walk logged by the go2's built-in L1 lidar and leg odometry, gsc pulls the
   AprilTag spread from 429 m to 8.5 m. Ivan's PGO only looks for a revisit within 2 m of where its odometry
   thinks it is (`loop_search_radius`, `pgo.py:83`), so at its settings it can't close any loop once drift
   passes 2 m: it leaves 410 m there, and on small_loop_go2 (17 m of drift) it leaves 14 m where gsc closes it
   to 0.18 m. The walk was recorded twice: on the go2 (`huge_loop_go2*`: its own lidar and leg odometry, and
   its Mid-360 through Point-LIO and FAST-LIO) and on a realsense rig carrying another Mid-360
   (`huge_loop_pointlio`, `huge_loop_fastlio`). It revisits places mid-route but doesn't end where it started
   (end gaps 84-233 m depending on odometry), so these test mid-route revisits. AprilTag spread, m
   (raw / Ivan's `PGO()` over 5 runs / gsc's defaults):
   - huge_loop_go2 (leg odometry): 429 / 360-408 (410 in the table's run; `LIVE_PGO` 384-425, 355 in the
     table's run) / 8.5. Voxels: Ivan +0.4% to +1.5%,
     gsc -23.9%.
   - huge_loop_go2_fastlio: 596 / 570-574 / 11.3. Voxels: -0.15% to -0.12% vs -11.0%.
   - huge_loop_go2_pointlio: 226 / 216-293 (worse than raw on 4 of 5 runs) / 167. Voxels: -0.35% to -0.05% vs -13.5%.
   - huge_loop_fastlio (realsense), where FAST-LIO diverges: 130 / 125-202 (worse than raw on 4 of 5 runs;
     220 in the table's run) / 24. `LIVE_PGO` 165-216 m. Voxels: Ivan +0.05% to +0.7%, gsc -9.1%.
   - huge_loop_pointlio (realsense): 92 / 13.5-13.9 / 2.4. Voxels: -9.7% vs -11.0%.
   - Ivan's `PGO()` raised an uncaught Open3D error on 3 of its 11 realsense huge-loop runs (1 of 5 repeats
     on each, and the table's pointlio run) and on none of the 18 go2 huge-loop runs; `LIVE_PGO` on none.
     See "Crashes" below.
   - small_loop_go2 (a 13 min loop that ends where it starts): gsc closes a 17 m drift gap to 0.18 m (-11.1%
     voxels). Ivan leaves 14.0-14.2 m (10 runs) and `LIVE_PGO` 14.4 m, both with a worse map than raw (+2.5% /
     +2.0%).

   Before (raw odometry) and after gsc's loop closure, from the PR description (rendered with a July build of gsc):

   ![huge_loop_go2](https://github.com/user-attachments/assets/11e52e6a-047f-455f-96f2-d2536b7e8f74)
   ![huge_loop_fastlio](https://github.com/user-attachments/assets/11b8e2be-bf70-4876-bfac-385d1ea8400a)
   ![huge_loop_pointlio](https://github.com/user-attachments/assets/6f2ba8ab-8421-4a61-a806-b2844baabf77)
   ![small_loop_go2](https://github.com/user-attachments/assets/f5ab02ae-1845-4df8-ace8-3c4dc597ee39)
3. **On short, low-drift recordings (go2_hongkong_office, hk_village1-6) there's little to correct and the two
   are within noise.** Voxel changes there are under the metric's noise floor (a rigid 10 cm vertical shift of
   the raw map moves the count by up to 7% on the villages and 17% on the office), and the village tag totals
   are about 1 m apart (gsc 50.0 m, Ivan's `PGO()` mean 49.1 m over 10 runs; raw 53.3). The one visible
   difference: Ivan's `PGO()` smears hk_village3 on every run (+6.3% to +7.0% voxels, visible in the top-down).

   Top-down (raw, Ivan's `PGO()`, Ivan's `LIVE_PGO`, gsc): hk_village3, where Ivan's `PGO()` doubles the walls,
   and two where the corrected maps are nearly the same:

   ![hk_village3](figures/claim3_hk_village3.png)
   ![hk_village4](figures/claim3_hk_village4.png)
   ![go2_hongkong_office](figures/claim3_go2_hongkong_office.png)
4. **gsc is far lighter than Ivan's module even before any planner.** Same 60 min huge_loop_pointlio replay
   (`compare_timing.py`): Ivan's live module grows to 13.4 GB (4.4 GB at 10 min, 8.1 GB at 30) and spends
   2374 s of compute on the hour. gsc's grows to 0.56 GB (0.16 GB at 10 min) and spends 1132 s on all
   threads, 340 s of it on the pipeline thread (the rest is the GNC re-solve and iSAM2 rebuild after each of
   its 373 closures, on its worker). gsc was measured with other jobs loading the machine, so its compute is
   if anything high; neither number includes a planner.

   ![Memory and cumulative compute of each live module over the drive](figures/claim4_efficiency.png)

gsc's defaults damp loops whose residuals are large (see "Why"); no test here injects false loop
matches, so the robustness that is meant to buy is not measured.

Defaults changed by this benchmark: `loop_gnc_inlier_probability` 0.99 → 0.95. Without it, gsc's GNC
kept village loops that demanded rotations of up to 111 deg against odometry, and hk_village3 came out
+1.06%. The huge loops and small_loop are unchanged by it: pointlio keeps 353 loops (354 before) with the
same voxels, fastlio is -9.12% (-9.08% before), and small_loop's gap is 0.23 m either way.

## The two PGOs

| step | Ivan's PGO (`dimos/navigation/go2/loop_closure/pgo.py`, from main) | gsc_pgo (`gsc_pgo/`, this branch) |
|---|---|---|
| find a revisit | keyframes within 2 m of the current corrected pose (`pgo.py:584`) | Scan Context: a polar max-height descriptor per keyframe (20 rings x 60 sectors); ring-key lookup, then the top 10 by column-shifted cosine, which also gives a yaw estimate. Falls back to a 3 m position search |
| verify | point-to-plane ICP from the current pose (Open3D), 1 m correspondence, inlier fitness <= 0.3 | point-to-point ICP (SVD) seeded at the matched keyframe with the Scan Context yaw, 10 m correspondence, fitness over all points <= 0.15, plus occupancy and degeneracy gates (all configurable) |
| weigh | plain factor; loops never removed | Huber kernel, then a GNC outlier pass that can drop loops |
| loop gap | > 20 s between keyframes (`pgo.py:598`) | > 5 s |

Ivan's PGO runs at two configs: `PGO()` (his `eval.py:78`) and `LIVE_PGO` (main's `unitree-go2-pgo`
blueprint, `unitree_go2.py:111`, via `PGOVoxelMapper`, `module.py:43`, `pgo_map.py:46`; it differs only
in `key_pose_delta_deg`, 45 vs 10). gsc runs at its defaults, which `bench.py: check_gsc_defaults` asserts
equal the live module's on every run (the module's extra fields are live- or publish-only; after a default
changes, `variant.py gsc '{}'` re-runs gsc in place), and at `ivan_equivalent.json` via `variant.py`. Both get the same scans and poses, and the same code
(`bench.py: correction_table` / `correction_at`, identical to Ivan's own `PoseGraph._corrections`,
`pgo.py:225`) turns each run's keyframes into a per-scan correction.

## Results

Voxels = occupied 0.2 m voxels of the corrected map vs raw (lower = duplicate walls merged), over a fixed
random subsample of each scan (as few as 200 points per scan on the huge loops). Loops = closures in the
final graph (gsc: GNC weight above 0); in brackets, how many are between keyframes more than 20 s apart,
Ivan's minimum. Output of `run.py --tables`:

| dataset | scans | Ivan PGO() loops / voxels | Ivan LIVE_PGO loops / voxels | gsc, Ivan-equivalent config loops / voxels | gsc, defaults loops / voxels |
|---|---|---|---|---|---|
| small_loop_go2 | 5729 | 32 / +2.5% | 24 / +2.0% | 28 (28 >20 s) / +0.7% | 40 (17 >20 s) / -11.5% |
| huge_loop_go2 | 24557 | 69 / +1.5% | 63 / +1.8% | 70 (70 >20 s) / +0.4% | 44 (27 >20 s) / -23.9% |
| huge_loop_go2_pointlio | 24217 | 69 / -0.2% | 55 / -0.1% | 70 (70 >20 s) / -0.4% | 261 (218 >20 s) / -13.5% |
| huge_loop_go2_fastlio | 30317 | 63 / -0.1% | 50 / -0.2% | 63 (63 >20 s) / -0.1% | 236 (188 >20 s) / -11.0% |
| huge_loop_pointlio | 36023 | 235 / -9.7% (CRASH unguarded) | 174 / -9.8% | 234 (234 >20 s) / -9.6% | 353 (218 >20 s) / -11.0% |
| huge_loop_fastlio | 34942 | 121 / +0.7% | 64 / +0.4% | 220 (220 >20 s) / -9.3% | 338 (206 >20 s) / -9.1% |
| mid360_athens_stairs | 3093 | 22 / +0.1% | 22 / -0.0% | 22 (22 >20 s) / +0.4% | 34 (14 >20 s) / +0.2% |
| go2_hongkong_office | 4235 | 40 / -1.5% | 37 / -3.6% | 40 (40 >20 s) / +1.7% | 35 (17 >20 s) / -5.0% |
| hk_village1 | 956 | 15 / +2.1% | 15 / +3.2% | 15 (15 >20 s) / +0.0% | 16 (6 >20 s) / +0.0% |
| hk_village2 | 790 | 10 / +1.1% | 10 / +1.2% | 10 (10 >20 s) / -0.3% | 4 (3 >20 s) / +0.7% |
| hk_village3 | 702 | 12 / +6.4% | 10 / +0.9% | 13 (13 >20 s) / +0.8% | 11 (5 >20 s) / +1.0% |
| hk_village4 | 1395 | 16 / +2.0% | 15 / +0.5% | 16 (16 >20 s) / +0.8% | 4 (3 >20 s) / -0.2% |
| hk_village5 | 982 | 14 / +1.6% | 13 / +0.3% | 15 (15 >20 s) / +0.6% | 8 (4 >20 s) / +0.2% |
| hk_village6 | 1420 | 21 / -0.7% | 20 / -1.1% | 20 (20 >20 s) / +0.9% | 10 (6 >20 s) / +0.3% |

### gsc configured like Ivan's PGO

To check that gsc's gains don't come from tuning, gsc is also run with Ivan's settings. It then finds
about the same loops as his PGO but still builds different maps, so the gains come from the implementation.
`ivan_equivalent.json` sets gsc to Ivan's settings: position-only loop search within 2 m, a 20 s minimum
loop gap, his submap and ICP limits (1 m correspondence, inlier-only fitness, 10 points minimum, seeded at
the current pose), his loop and odometry noise and refinement passes, and no robust kernel or outlier
rejection.
- Loops are within 10% of Ivan's `PGO()` on 12 of 14 recordings. The exceptions: small_loop_go2 (28 vs
  32) and huge_loop_fastlio (220 vs 121).
- The village AprilTag total is 49.36 m, inside Ivan's 10-run range.
- On huge_loop_fastlio it keeps 220 loops to his 121 and gets nearly all of gsc's default voxel gain
  (-9.3% vs -9.1%; Ivan +0.7%), so that gain comes from the implementation, not from Scan Context or GNC.
- Against the min-max of Ivan's `PGO()` runs (10, or 5 on the huge loops), gsc at his config's voxels land
  inside on 1 recording (huge_loop_go2), below on 8 and above on 5. Only three differences clear the
  metric's noise floor (see claim 3): huge_loop_fastlio (-9.3% vs +0.05 to +0.50%), hk_village3 (+0.84% vs
  +6.32 to +6.97%) and hk_village4 (+0.83% vs +1.72 to +5.12%); the rest don't rank the two.
- Config can't mirror the ICP algorithm (gsc point-to-point, Ivan point-to-plane in Open3D) or its
  stopping rules, gsc rejecting an ICP with under 3 correspondences, Ivan downsampling each keyframe
  cloud as well as the submap, or gsc's one extra iSAM2 pass on updates without a loop.

Ivan's AprilTag spread (his `eval.py` pipeline at its CLI defaults, `bench.py: detect_markers`; per marker,
summed pairwise distance between its corrected sightings; m, lower = tighter). It runs where a recording's
images are posed in its scans' world (median offset under 0.5 m, a threshold picked after seeing the
offsets). huge_loop_pointlio's images are posed against fastlio, and its pointlio streams were re-run on
2026-07-11 and stamped with that day's clock, so its markers are shifted onto pointlio's clock and
re-expressed from fastlio's world into pointlio's at each sighting (`recordings.py: tag_odom,
clock_offset_s`; `bench.py: rebase_markers`). The huge loops use the recording's own `camera_info`; the
others Ivan's go2 intrinsics. On the go2 huge loop the images' stored poses come from an old FAST-LIO build
with velocity spikes (tags jumped 50 m within seconds), so there each image's camera pose is the scan pose
at its time times the recording's static tf to `camera_optical` (`recordings.py: tag_camera_frame`;
`bench.py: camera_pose_from_tf`), and the stored image poses aren't read. On the realsense loops marker id 17 is left out (`recordings.py:
tag_ids_moving`): it is on the back of a robot dog that moves around the course, so its summed distances
measured the dog, not the correction (3063 of fastlio's 3193 m raw).

| dataset | tracks | raw | Ivan PGO() | Ivan LIVE_PGO | gsc, Ivan-equivalent config | gsc, defaults |
|---|---|---|---|---|---|---|
| small_loop_go2 | 2 | 0.98 | 0.85 | 0.69 | 0.36 | 0.03 |
| huge_loop_go2 | 10 | 429.34 | 410.47 | 355.17 | 426.54 | 8.48 |
| huge_loop_go2_pointlio | 14 | 225.74 | 219.49 | 247.24 | 232.22 | 167.17 |
| huge_loop_go2_fastlio | 10 | 595.53 | 574.30 | 577.44 | 576.08 | 11.26 |
| huge_loop_pointlio | 15 | 91.62 | 13.69 | 14.08 | 13.77 | 2.36 |
| huge_loop_fastlio | 15 | 130.17 | 220.00 | 166.64 | 113.62 | 24.17 |
| hk_village1 | 5 | 6.68 | 5.99 | 8.57 | 5.75 | 6.04 |
| hk_village2 | 4 | 21.73 | 19.34 | 20.15 | 19.74 | 21.77 |
| hk_village3 | 4 | 4.79 | 4.96 | 4.87 | 4.55 | 4.42 |
| hk_village4 | 5 | 14.13 | 12.89 | 11.64 | 13.78 | 12.77 |
| hk_village5 | 4 | 2.29 | 2.90 | 2.42 | 2.86 | 2.16 |
| hk_village6 | 4 | 3.66 | 2.09 | 1.68 | 2.67 | 2.86 |
| TOTAL hk_village1-6 | | 53.28 | 48.16 | 49.33 | 49.36 | 50.01 |

## Why

Part of the gap between default gsc and Ivan is strategy, which config switches between:

- Ivan only looks within 2 m and trusts every loop fully, so he can't
  reach a revisit once drift passes about 2 m: small_loop_go2's revisit is about 17 m apart in drifted
  odometry, and neither Ivan config nor gsc at the Ivan-equivalent config closes it.
- gsc finds revisits by shape at any drift, which also proposes many wrong ones, so it damps loops with
  large residuals (Huber) and drops inconsistent ones (GNC). That is why it closes small_loop and makes the
  sharper map on drifty recordings. Giving its Scan Context candidates full weight (`strong_accept_range`, default off) made the go2 maps worse.

Implementation matters as much as strategy on some recordings: at the same config gsc closes 220 loops on
huge_loop_fastlio to Ivan's 121 and gets nearly all of its default voxel gain there, and it makes a less
sharp office map.

## Reproduce

```bash
# 1. gsc_pgo's offline runner (needs nix: the crate's flake brings gtsam)
(cd dimos/navigation/jnav/components/loop_closure/gsc_pgo/rust && nix develop path:. --command cargo build --release --features native --bin pgo-eval)

# 2. python env + a console.dimensional.org login (for the cloud recordings)
uv sync && source .venv/bin/activate
dimos login

cd dimos/navigation/jnav/components/loop_closure/eval_loop
python run.py                                                   # all recordings: prep, both Ivan configs, gsc defaults, maps
python variant.py gsc_ivancfg "$(cat ivan_equivalent.json)" <recording> ...   # add the Ivan-equivalent gsc column
python run.py --tables                                          # reprint the tables from the summaries
python spread.py eval_loop_results <earlier run dirs>           # Ivan-equivalent gsc vs Ivan's run-to-run range
```

Data: cloud recordings go to `~/.cache/dimos/eval_loop/` (`EVAL_LOOP_DATA`), LFS ones to `data/`, results
to `./eval_loop_results/` (`EVAL_LOOP_OUT`); the gsc binary is `gsc_pgo/rust/target/release/pgo-eval`
(`PGO_EVAL`). Recordings (`recordings.py`):

| dataset | source | what it is |
|---|---|---|
| `small_loop_go2` | cloud `outdoor_small_loop_2026-05-28.db.lz4` | go2 onboard lidar, ~13 min outdoor loop that ends where it starts |
| `huge_loop_go2`, `huge_loop_go2_pointlio`, `huge_loop_go2_fastlio` | cloud `mem2.db.lz4` `dbf5283f…` | the go2 on the ~3 km walk: its own lidar + leg odometry, and its Mid-360 through two LIOs (53 min) |
| `huge_loop_pointlio`, `huge_loop_fastlio` | cloud `huge_loop_realsense_2026-06-04.db.lz4` | the same walk on a realsense rig: Mid-360, ~60 min, two odometries |
| `mid360_athens_stairs` | LFS | Ivan's 5 min Mid-360 stairwell loop; no sign in git that gsc was run on it |
| `go2_hongkong_office`, `hk_village1`..`6` | LFS | Ivan's go2 recordings; his `eval.py` tunes on the villages |

`prep.py` turns the cloud and Mid-360 recordings into the world-frame, pose-stamped `lidar` stream Ivan's
PGO takes. In the huge-loop db, `fastlio_odometry` was labeled `base_link` though it is the Mid-360's pose,
and `tf`'s `odom→base_link` was a copy of it, so the 36 deg mount was applied twice. Both were fixed in the
db (2026-10-15): the odometry's child is now `mid360_link`, and `odom→base_link` is that pose with the mount
removed. The recording's stored camera poses agree with the fixed tf chain (median 0.00 deg off, vs 36 deg
before). In the go2 huge-loop db the LIO odometries are also labeled `base_link` but report the Mid-360's
pose (the lidar's own IMU puts it 50-52 deg from gravity; the odometry read as the lidar's pose agrees,
read as `base_link` plus the 44 deg mount it would be 94-102 deg), so `prep.py` takes them as `mid360_link`'s
pose (`recordings.py: odom_pose_of`); the db itself is unchanged.

## Limits

- **Home turf.** Treat every row as in-sample for gsc: its author developed and evaluated it on these
  recordings (e.g. `595e3d919e`, `cd1affa300`, `ba5103dece` and `9ef120dbe9` on the huge-loop drive;
  `9acf13a644` on the small loop, values since reverted; `b2c5e41110` on hk_village5; `e237773100` and three
  AprilTag commits on the earlier `jeff/feat/jnav` line). Ivan's `eval.py` tunes on hk_village1-6; no commit
  shows him tuning on the small or huge loop. mid360_athens_stairs is the only recording with no sign of gsc,
  and it barely drifts (raw end gap 0.49 m), so it doesn't separate them.
- **Edited inputs.** In the huge-loop cloud db the FAST-LIO streams, `camera_info` and `tf` were rewritten
  after recording, and the Point-LIO streams were generated offline about 37 days after the drive; both
  PGOs get the same edited inputs.
- **Variance.** Ivan's results move between runs (e.g. hk_village3 `PGO()` voxels +7.1 / +4.9 / +6.4% across
  three full runs); gsc's were identical. Most numbers here are single runs.
- **Metrics.** The voxel count can drop while a map gets worse, so check the top-down PNGs, and it has a
  noise floor of several percent on go2 maps (a rigid 5-15 cm vertical shift moves it by up to 17%; under
  0.5% sideways, about 0% on the Mid-360 stairwell map), so only large differences rank methods; the AprilTag
  spread rests on 2-21 tracks per recording; end gap is only a closure measure on small_loop_go2.
- **Offline replays.** `pgo-eval --gnc each-loop` (the default) runs the batch GNC re-solve and iSAM2
  rebuild inline after every closure and adopts them at once; its scores are the trajectory the live module
  would hold if each background solve landed before the next scan (live, they land up to a few seconds later).
  Its per-frame times record the worker's share separately; the live latency counts only the rest. `--gnc
  final` (one re-solve after the whole drive, which the live module never does) is what earlier versions of
  this benchmark scored. Ivan's `LIVE_PGO` is replayed without his live odometry pairing.
- **Crashes.** Ivan's `PGO()` raised Open3D's singular-system error (`pgo.py:744`, uncaught) on 3 of its 11
  huge-loop runs (`<dataset>.repeat.json: crashed_unguarded`); such runs are repeated with
  `bench.py: guarded_icp`, which turns that error into his own ICP rejection (`pgo.py:724`), patched in the
  benchmark process only. Nothing in his `pgo.py`, `pgo_map.py` or `module.py` catches it; what it does to
  the live module depends on how the stream layer handles a raising callback, which isn't tested here.
- **Not covered.** `pgo_auto.py` (imported by nothing), recordings that weren't run (grassy_field,
  gir_stairs1, gir_park1_2, china_office1, LFS `hk_building_*` and others), and an earlier head-to-head in
  `af49203159`'s BENCHMARK.md where an older gsc config was marked HARM on 7 of 8 go2 runs.

## Live cost on a long recording

```bash
python live_timing.py huge_loop_pointlio & ./watchdog.sh $! 32   # Ivan's live module, per frame; kill at 32 GB RSS or <12 GB free
mkdir -p "$EVAL_LOOP_OUT/timing"
"$PGO_EVAL" --db "$EVAL_LOOP_OUT/huge_loop_pointlio.gsc.db" --odom-stream bench_odom --lidar-stream bench_lidar \
  --loop-snapshots --output "$EVAL_LOOP_OUT/timing/huge_loop_pointlio.gsc_default.json"   # gsc per frame
cp "$EVAL_LOOP_OUT/timing/huge_loop_pointlio.gsc_default.json" "$EVAL_LOOP_OUT/timing/huge_loop_pointlio.gsc_snapshots.json"
python compare_timing.py huge_loop_pointlio                       # -> huge_loop_pointlio.timing_compare.{json,html}
# MLS planner downstream of each map design (needs the MLS and ray tracer python bindings: maturin develop
# --release in mls_planner/rust/py and ray_tracing/rust/py; on macOS with -undefined dynamic_lookup)
python mls_load.py huge_loop_pointlio region                      # ray tracer cylinder -> update_region, every frame
python mls_load.py huge_loop_pointlio whole --whole-every 40      # Ivan's published map -> update_global_map
python mls_load.py huge_loop_pointlio region --patch-from "$EVAL_LOOP_OUT/timing/huge_loop_pointlio.gsc_snapshots.json"   # gsc patched in place
python mls_load.py huge_loop_pointlio region --patch-from "$EVAL_LOOP_OUT/huge_loop_pointlio.mls_whole.json" --label patch_ivan
python mls_report.py huge_loop_pointlio                           # -> huge_loop_pointlio.mls_summary.json
```

`live_timing.py` replays the prepped recording through Ivan's `PGOMap` at `PGOVoxelMapper`'s defaults
(`LIVE_PGO`) with `emit_every=5`, as `unitree_go2.py:114` sets it. gsc's live module runs the same `GscPgo`
steps `pgo-eval` times (`add_key_pose`, loop search, `smooth_and_update`, and on a closure the GNC adoption)
and publishes a correction rather than a map. `compare_timing.py` turns each module's per-frame cost into
output latency through that module's own queue. The timings are single replays on a shared machine, and
other jobs change them.
