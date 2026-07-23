---
title: "Relocalization benchmark"
description: "Score relocalization honestly on a held-out same-scene pair with the --eval flag, per source, in replay or live."
---

The benchmark measures how well relocalization actually locates the robot. It scores the `world → map` transform the real pipeline publishes, per source (`ransac`, `fiducial`, `last_pose`), against loop-closure truth. It is the same blueprint you deploy, [`unitree-go2-relocalization-lidar-fiducial`](/docs/capabilities/navigation/relocalization.md), plus a `--eval` collector, so replay and live run identical modules.

Read [Premap & Relocalization](/docs/capabilities/navigation/relocalization.md) first. This doc adds the held-out rule, the `--eval` output, and how to read it.

## Held-out is the rule

To measure relocalization, the premap and the replay must be **two different runs of the same space**:

- **Premap** comes from run **A** (`dimos map global A --export`).
- **Marker map** also comes from run **A**, in A's PGO-corrected frame. There is no survey command yet; see [The marker map](/docs/capabilities/navigation/relocalization.md#the-marker-map) for the code path that produces one today.
- **Replay** is run **B**, a separate drive of the same room.

The robot then has to relocalize on geometry it recorded on a different pass. That is the whole test: **the replay of B succeeding against A's map is the relocalization working.** Replaying the recording the premap was built from measures memorization, not relocalization, and always looks perfect.

Truth is the recording's own loop-closure PGO (`PoseGraph.correction_at(t)`), external to the system under test. A fix counts as `success` when its translation lands within **1 m** of truth. Truth is PGO silver, not survey-grade ground truth, so treat absolute error as a floor.

## Run the eval

The `--eval` flag attaches the collector to a normal run. It subscribes to the published `world → map` TF, odom, and the `relocalize accepted` log line, computes per-source stats at shutdown, prints the table, and writes the JSON plus the trajectory plot.

**Replay** (no robot; B against A's map):

```bash
uv run --project . \
  dimos --replay --replay-db=<B> \
  run unitree-go2-relocalization-lidar-fiducial --eval \
  -o relocalizationmodule.map_file=<A>
```

**Live** (on hardware; drive B while `<A>` is the loaded map). Drop `--replay`; set `ROBOT_IP` and `UNITREE_AES_128_KEY`:

```bash
uv run --project . \
  dimos --robot-ip <ROBOT_IP> \
  run unitree-go2-relocalization-lidar-fiducial --eval \
  -o relocalizationmodule.map_file=<A>
```

> The `*-lidar-fiducial` preset enables the fiducial prior, so the old
> `-o …use_fiducial_prior=true` is gone. The marker map lives on the `fiducial`
> entry inside `relocalizationmodule.priors`, and a dotted `-o` cannot index a
> list, so `Config` carries a sibling `marker_map_file` that wins over the entry's
> own: `-o relocalizationmodule.marker_map_file=<path>` (see `_start_fiducial_prior`
> in [`module.py`](/dimos/mapping/relocalization/module.py)). A blueprint can also
> bake it in with `FiducialPriorConfig(marker_map_file=…)`.
>
> Producing that file is the gap: no shipped command writes a marker map. See
> [The marker map](/docs/capabilities/navigation/relocalization.md#the-marker-map).

The LCM bus is exclusive: run one replay at a time. Check `pgrep -af 'dimos --replay'` before launching.

## Reading the output

At shutdown `--eval` prints one table, then the prior-activity census, the overall
line, the correction block, and the artifact paths (numbers illustrative):

```
=== RelocEval: survey2 vs survey1 [held_out] ===
source    prop  acc  rej  false  %traj   med_err  med_fit
--------  ----  ---  ---  -----  ------  -------  -------
ransac    28    28   3    2      100.0%  0.410m   0.660
fiducial  24    0    0    0      0.0%    -        -
--------  ----  ---  ---  -----  ------  -------  -------
TOTAL     52    28   3    2
prior activity: fiducial proposed 24/28 cycles, won 0
overall: accepts=28 rejects=3 coverage=97.4% first-fix=12.0s
correction (at the robot, n=27): med=0.180m p90=0.640m med_yaw=1.2deg med_rate=0.0900m/m; total 4.860m over 54.00m driven (0.0900m/m)
  first fix: 3.210m 14.0deg (ransac, acquisition -- excluded above)
  by source: ransac 0.180m 0.0900m/m (n=27)
held-out: premap=survey1 replay=survey2
[releval] wrote out/eval/releval.eval.json
[releval] wrote out/eval/releval.trajectory.png
[releval] wrote out/eval/releval.accepted_fixes.csv
```

Every relocalization cycle, each prior may propose candidates and the shared fitness judge picks one winner. The columns:

| Column | Meaning |
|--------|---------|
| `prop` | Cycles where this source put at least one candidate forward (run-log census) |
| `acc` | Accepts this source won and published (`relocalize accepted` lines) |
| `rej` | Rejects this source's winning candidate took; `-` when the log named no source |
| `false` | Accepts whose held-out error exceeded 1 m (a wrong accept) |
| `%traj` | Share of the covered trajectory localized by this source |
| `med_err` | Median distance from PGO truth over this source's accepts (meters) |
| `med_fit` | Median ICP fitness of this source's accepts |

`won` and `success` stay in the JSON (`stats_to_dict`), not the printed table.

**Live** drops `false` and `med_err` — there is no PGO truth in-process. Everything else, including the correction block, prints in both modes.

### The trajectory plot

`--eval` writes `out/eval/releval.{eval.json,trajectory.png,accepted_fixes.csv}` (`-o reloceval.tag=<name>` renames the stem). The PNG is the robot path in the `map` frame, each segment colored by the source that won that stretch. Tags are drawn only with `-o reloceval.marker_map_file=<survey>`; the flag does not infer it from the reloc module. It shows at a glance where each source carried localization and whether the fiducial prior ever won near a tag.

## Metrics, and why they are split

- **`proposed` vs `won` is the key split.** A prior can propose in most cycles and still win none. That is the honest picture: the marker was seen and aggregated, but its pose did not fit the walls as well as lidar. Proposed-but-never-won points at marker-pose accuracy; never-proposed points at detection or the marker map.
- **`med_fit` is not `med_err`.** Fitness is the judge's confidence in the alignment; error is distance from truth. A confident wrong pose has high fitness and high error. Report both.
- **`%traj` per source** says who is actually doing the work over the drive, not just who won a single cycle.
- **Coverage** — the share of odom samples that had a published fix active — says how much of the drive was localized at all.

## Tunable parameters

Everything the benchmark sweeps is module or prior config, so a search over embodiments (DIM-944) reads and writes these directly (`priors[…]` entries are set by preset, not by a dotted `-o`):

| Parameter | Default | Sweeps |
|-----------|---------|--------|
| `priors[ransac].fitness_threshold` | `0.6` | Accept bar for a RANSAC fix (wall overlap is its whole evidence) |
| `priors[fiducial].fitness_threshold` | `0.6` | Accept bar for a tag fix (same wall-fitness bar as any source) |
| `priors[ransac].min_local_points` | `50000` | How much live map the FPFH search needs first |
| `priors[ransac].interval_s` | `2.0` | RANSAC sweep cadence (was the module's `reloc_interval_s`) |
| `gravity_tilt_max_deg` | `10.0` | Up-axis gate against gravity-inconsistent flips |
| `marker_length_m` | `0.10` | Marker size for the pose solve |
| `ambiguity_ratio_min` | `2.0` | IPPE mirror-flip gate strength |

The per-source counts come from the run log, so a sweep is reproducible: same recording, same config, same counts. `coverage` and `first-fix` are wall-clock derived and jitter run to run.

## Fiducial caveats

A marker helps only if its pose in the marker map is accurate. Four things break that:

- **The marker map is aggregated by a simple windowed mean.** The path that produces it (`corrected_marker_transforms` → `DetectMarkers(smoothing_window=…)`) averages translation and takes a sign-aligned linear quaternion mean over the window. It has no outlier rejection, so one bad glimpse pulls the stored pose. The robust batch aggregator — visit clustering, Huber IRLS, Markley quaternion eigen-mean — exists as `aggregate_visits` in [`apriltag_aggregation.py`](/dimos/perception/fiducial/apriltag_aggregation.py) and is tested, but nothing calls it on the survey path. Only its streaming sibling `TagAggregator` runs, inside the live prior. Wiring the batch aggregator into marker-map production is a follow-up.

- **Mirror flip (IPPE planar ambiguity).** A planar tag has two pose solutions that both reproject well; the wrong one is a near-mirror of the right one and can be tens of degrees off. See Collins & Bartoli (2014) and Schweighofer & Pinz. The `ambiguity_ratio_min` gate rejects a glimpse whose flipped solution reprojects nearly as well as the best one.
- **The gate runs at the detector.** `MarkerDetectionStreamModule` applies `ambiguity_ratio_min` (default 2.0) before anything reaches the wire, so an ambiguous view never publishes. The live `Detection3DArray` wire drops `corners_px`, so the prior's own second copy of the gate is inert; there the medoid seed plus Huber aggregation are the backstop.
- **Per-unit calibration.** One static fisheye intrinsic serves every Go2, with no per-unit calibration, so a marker pose inherits that camera's error. DIM-1308.

## Worked example: sf_office survey1 → survey2

Premap and marker map from `sf_office_go2_20260718_survey1`; replay of `sf_office_go2_20260720_survey2`, a separate drive of the same office.

```bash
uv run --project . \
  dimos --replay --replay-db=sf_office_go2_20260720_survey2 \
  run unitree-go2-relocalization-lidar-fiducial --eval \
  -o relocalizationmodule.map_file=sf_office_go2_20260718_survey1.pc2.lcm \
  -o relocalizationmodule.marker_map_file=sf_office_go2_20260718_survey1.json \
  -o reloceval.marker_map_file=sf_office_go2_20260718_survey1.json
```

The marker map was derived from survey1 with
`corrected_marker_transforms`, the same path the replay-gate fixture uses, not by an
operator survey command; none exists. Result:
**survey2 relocalized against survey1's map.** Over 28 relocalization cycles the fiducial prior proposed in 24, and won 0. Lidar RANSAC won every cycle. The plumbing works end to end; the prior loses the wall-fitness judge because the marker-map poses still carry mirror-flip and calibration error, so `med_err` for a real marker benefit is pending the marker-pose fix (DIM-1308) and the held-out accuracy alignment.

![survey2 relocalized against survey1's map, trajectory colored by winning source with marker-map tags](assets/survey2_heldout_trajectory.png)

The plot is entirely one color: every segment won by `ransac`, no fiducial-won stretch near any tag. `--eval` wrote it to `out/eval/releval.trajectory.png`.

This is the held-out benchmark doing its job. It shows a real, honest gap (marker poses are not yet good enough to beat lidar) rather than a memorized pass that would have hidden it.

## Related docs

- [Premap & Relocalization](/docs/capabilities/navigation/relocalization.md) — build the premap, enable the fiducial prior, config reference.
- [Go2 platform guide](/docs/platforms/quadruped/go2/index.md) — hardware, simulation, blueprints.
