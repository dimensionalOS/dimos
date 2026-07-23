---
title: "Premap & Relocalization"
description: "Record a Go2 run, export a loop-closed premap with dimos map, and relocalize on replay or live hardware."
---

Relocalization lets a Go2 navigate on a previously built map instead of only on what it sees right now. At runtime, `RelocalizationModule` aligns live LiDAR to a saved premap and publishes a `world → map` transform, so the costmap and planner operate on the live scan and premap together.

![relocalize on the live go2 and nav_to a point in the premap](https://raw.githubusercontent.com/dimensionalOS/dimos-docs-assets/main/capabilities/navigation/assets/reloc_and_nav_to.webp)

> **Note:** Requires DimOS v0.0.13 or newer for PGO loop closure and `dimos map` export.

This guide takes four steps:

1. Record a walk-through with `unitree-go2-memory`
2. Build the premap with `dimos map global {DB_NAME} --export`
3. Test relocalization in replay, no robot needed
4. Deploy on the live Go2

Throughout this guide, `{DB_NAME}` is the stem of your recording, for example `recording_go2` for `recording_go2.db`. For `map_file`, pass the same stem and DimOS appends `.pc2.lcm` automatically.

## 1. Record a run

Drive the Go2 through the space you want as your premap. Close loops when you can because PGO uses revisits to correct drift.

```bash
dimos --robot-ip {YOUR_ROBOT_IP} run unitree-go2-memory
```

If `ROBOT_IP` is set in the environment or `.env`, you can omit `--robot-ip`:

```bash
dimos run unitree-go2-memory
```

This writes `recording_go2.db` to the repo root (`DIMOS_PROJECT_ROOT`) and records `lidar`, `odom`, and `color_image` plus the live TF tree. The recorder stamps lidar frames with the latest odom pose so `dimos map global` can reconstruct poses later- see [`Go2Memory`](/dimos/robot/unitree/go2/blueprints/smart/unitree_go2.py).

### Quick validation (optional)

Before building a premap, inspect the recording:

```bash
dimos mem summary recording_go2
dimos map replay recording_go2 --duration 60
```

`summary` prints stream names and time ranges. `replay` opens Rerun so you can confirm lidar and odometry look sane.

## 2. Build the premap

Export a loop-closed global map as `.pc2.lcm`:

```bash
dimos map global recording_go2 --export
```

| Flag | Effect |
|------|--------|
| `--export` | Run PGO and write `./{DB_NAME}.pc2.lcm` to the current working directory (implies `--pgo`) |
| `--no-gui` | Skip launching Rerun for headless servers or CI |
| `--pgo-tol 0.3` | Spatial dedup tolerance for keyframes in meters. Use `0` to keep all posed frames |
| `--voxel 0.05` | Voxel size in meters (default matches live mapper) |
| `--markers-out {DB_NAME}.json` | Also write one `map_T_tag` per marker id into the premap frame, for the fiducial prior (implies `--markers`). See [The marker map](#the-marker-map) |

`{DB_NAME}` accepts a bare stem, `./path/to/file.db`, or an absolute path. Bare names resolve in this order:

1. Current working directory
2. `DIMOS_PROJECT_ROOT`
3. `data/` via LFS (`get_data`)

Examples:

```bash
dimos map global recording_go2 --export --no-gui
dimos map global ./recordings/office_walk.db --export
dimos map global data/go2_hongkong_office.db --export
```

Sample log:

```
running PGO twopass map...
  Pass 1: 908 frames, 1 keyframes
exporting PGO twopass map to .../recording_go2.pc2.lcm...
wrote .../recording_go2.pc2.lcm
```

Open the companion `{DB_NAME}.rrd` in Rerun to verify loop closure before deploying to hardware.

## 3. Relocalize in replay

Test alignment without the robot. `unitree-go2-relocalization-lidar` is `unitree-go2` plus `RelocalizationModule` (RANSAC prior only):

```bash
dimos --replay --replay-db recording_go2 run unitree-go2-relocalization-lidar \
  -o relocalizationmodule.map_file=recording_go2
```

`map_file` resolves `{DB_NAME}.pc2.lcm` with the same search order as above (cwd, then project root, then `data/`).

### Reading the logs

```
relocalization module started map_file='recording_go2' loaded_map_frame_id='map'
ransac reloc skipped n_pts=37770 min_local_points=50000
relocalize rejected source=ransac fitness=0.433 threshold=0.6
relocalize accepted fitness=0.657 time_cost_s=3.0
```

`ransac reloc skipped` means the live submap is still warming up — fewer than the `ransac` entry's `min_local_points` accumulated. Only the RANSAC search is gated on it; a fiducial burst fires on a sparse submap too. `relocalize rejected` means an alignment was found but its wall fitness was under the bar of the prior that proposed it: `source=` names that prior and `threshold=` is its own bar, on every reject. Once `relocalize accepted` lines appear the `world → map` TF is live; `source=` there names the proposer that won the judge (`ransac`, `fiducial`, `last_pose`), omitted only on the lidar-only preset, where the plain solve runs and nothing was won, and `margin=` is its wall fitness minus the best rival source's.

> **To measure it, replay a different run than the premap.** Replaying the recording the premap was built from measures memorization, not relocalization. `--eval` scores the run against loop-closure truth — see [Eval](#eval).

### Rerun visualization

Watch alignment in Rerun, which is enabled by default on Go2 blueprints:

- **Merged map** shows the premap transformed into `world` plus the live scan, column-carved together.
- Toggle the merged map entity off to compare the live scan alone against the merged costmap.

## 4. Relocalize on a live robot

Run the replay test first. On hardware, use the same blueprint and `map_file`:

```bash
dimos --robot-ip {YOUR_ROBOT_IP} run unitree-go2-relocalization-lidar \
  -o relocalizationmodule.map_file=recording_go2
```

Before sending navigation goals, walk through this checklist:

1. Place the Go2 in a region that overlaps the premap on the same floor with recognizable geometry.
2. Wait for `relocalize accepted` info lines. Skipped and rejected lines are normal for the first 30 to 60 seconds.
3. Confirm stable `world → map` TF in Rerun before sending navigation goals.
4. Click to navigate or use agent skills such as `navigate_with_text` on the aligned costmap.

## How it works

The `unitree-go2-relocalization-lidar` blueprint is the standard [Go2 navigation stack](/docs/capabilities/navigation/deep_dive.md) plus `RelocalizationModule`:

<details>
<summary>diagram source</summary>

```python skip fold output=assets/go2_reloc_blueprint.svg
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.introspection.svg import to_svg
from dimos.mapping.relocalization.module import RelocalizationModule
from dimos.mapping.relocalization.priors import RansacPriorConfig
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2

unitree_go2_relocalization_lidar = autoconnect(
    unitree_go2,
    RelocalizationModule.blueprint(priors=[RansacPriorConfig()]),
).global_config(n_workers=11)

to_svg(unitree_go2_relocalization_lidar, "assets/go2_reloc_blueprint.svg")
```

</details>

![unitree-go2-relocalization-lidar blueprint module graph](assets/go2_reloc_blueprint.svg)

Note that [`CostMapper`](/dimos/mapping/costmapper.py) builds the costmap from the merged map only while [`RelocalizationModule`](/dimos/mapping/relocalization/module.py) has a good alignment; until then it falls back to the live map alone.

### File formats

| File | Format | Produced by | Consumed by |
|------|--------|-------------|-------------|
| `{name}.db` | memory2 SQLite (`lidar`, `odom`, `color_image`, …) | `unitree-go2-memory` | `dimos map *`, `--replay-db` |
| `{name}.pc2.lcm` | LCM-encoded `PointCloud2` premap | `dimos map global --export` | `RelocalizationModule` (`map_file`) |
| `{name}.rrd` | Rerun recording (visual QA) | `dimos map global` | Rerun viewer |
| `{name}.json` | `marker_id → map_T_tag` marker map | `dimos map global --markers-out` | `RelocalizationModule` fiducial prior (`marker_map_file`) |

## Configuration reference

CLI overrides use blueprint module config (`-o relocalizationmodule.<field>=…`):

| Field | Default | Description |
|-------|---------|-------------|
| `map_file` | `None` (module disabled) | Premap stem or path. DimOS appends `.pc2.lcm` automatically |
| `gravity_tilt_max_deg` | `10.0` | Reject a candidate whose up axis tilts more than this from world-z (degrees) |
| `use_carving` | `true` | Column-carve when merging premap and live scan |
| `publish_loaded_map` | `false` | Republish raw premap on `loaded_map` every 2 s |

### Priors

`priors` is a list of candidate proposers, each an equal, toggleable entry keyed by `type`. Every entry goes through the same wall-fitness judge; none bypasses it. The list is set by preset (blueprint), not by a flat `-o` override (the dotted override parser cannot index a list).

Each entry also owns its own **trigger**, and each fires an independent relocalization judged on its own candidates. `ransac` sweeps every `interval_s` (2.0 s; a global search waits on no event). `fiducial` fires the moment a tag burst completes, on the detections callback itself, and is judged against the last `global_map` received — waiting for the next cloud would be dead time, since the tag candidate is composed from the marker alone. A pool of one is normal: the judge is a validator (wall fitness, gravity tilt, wall evidence), not a tournament. `reloc_interval_s` on the module config is gone — it is now `interval_s` on the `ransac` entry, and a stale key raises.

The **accept bar is per entry** too (`fitness_threshold`, minimum wall fitness 0 to 1). `ransac` and `fiducial` each state `0.6`: a fix is judged on the walls whatever proposed it, and a decoded tag id names the tag without showing the composed pose fits. `last_pose` states no bar and inherits the base `0.45`. `fitness_threshold` and `min_local_points` on the module config are gone the same way — set them on the entry, and a stale key raises.

| `type` | `fitness_threshold` | Own params |
|--------|---------------------|------------|
| `ransac` | `0.6` | `interval_s` (`2.0`, seconds between sweeps), `min_local_points` (`50000`, live map points the FPFH search needs before it fires) — the search knobs live in `relocalize.py` |
| `last_pose` | `0.45` (base default) | none — the seed is captured at runtime; no trigger of its own, it rides along with whichever prior fired |
| `fiducial` | `0.6` | `marker_map_file`, `marker_length_m`, `aruco_dictionary`, `ambiguity_ratio_min`, `camera_info`, `aggregation` |

Every entry also carries `enabled`. The three Go2 presets:

| Blueprint | Priors |
|-----------|--------|
| `unitree-go2-relocalization-lidar` | `ransac` (2.0 s sweep) |
| `unitree-go2-relocalization-lidar-fiducial` | `ransac` (2.0 s sweep) + `fiducial` (burst) |
| `unitree-go2-relocalization-fiducial` | `fiducial` only — burst-triggered, no periodic timer |

The `fiducial` type needs a `marker_map_file` (`map_T_tag` per id, JSON or YAML — see [The marker map](#the-marker-map)) and consumes an `aggregated_detections` `Detection3DArray` In stream; `MarkerDetectionStreamModule` publishes the matching Out, so the two autoconnect by name/type in the `*-fiducial` blueprints (which share the fiducial family via `aruco_dictionary`). Each tag's sightings aggregate into one `world→map` candidate (medoid seed + Huber IRLS) that competes on wall fitness once and is then dropped — re-offering it later would be the same measurement against a world that has drifted further. It never publishes a pose on its own.

One constant is not overridable via CLI:

| Constant | Value | Role |
|----------|-------|------|
| `PUBLISH_INTERVAL_S` | `2.0` s | TF and `loaded_map` publish rate |

To accept all candidates for visualization only (not for production nav), set the bar to `0.0` on the entry — `-o relocalizationmodule.fitness_threshold=0.0` raises now, since the bar lives on the prior and a dotted `-o` cannot index a list:

```python
RelocalizationModule.blueprint(priors=[RansacPriorConfig(fitness_threshold=0.0)])
```

### The marker map

To use the fiducial prior, the premap needs the marker locations on it: `marker_id → map_T_tag`, JSON or YAML, read by `load_marker_map` in [`fiducial_relocalization.py`](/dimos/perception/fiducial/fiducial_relocalization.py):

```yaml
markers:
  3:
    translation: [1.243, -0.518, 0.902]  # meters, map frame
    rotation: [0.0, 0.0, 0.707, 0.707]   # qx qy qz qw, map_T_tag
```

`dimos map global {DB_NAME} --markers-out {DB_NAME}.json` writes it — one `map_T_tag` per marker id, every detection of that id reduced to a single location by the robust Huber-IRLS + Markley estimator ([`robust_cluster_pose`](/dimos/perception/fiducial/apriltag_aggregation.py)), PGO-corrected into the same frame `--export` builds the premap in. Add it to the export so one command produces the premap and its marker locations together:

```bash
dimos map global {DB_NAME} --export --markers-out {DB_NAME}.json
```

One static fisheye intrinsic serves every Go2 with no per-unit calibration, so each stored location inherits that camera's error (DIM-1308) — good, not survey-grade.

## Eval

`--eval` attaches the `RelocEval` collector to a normal run — the same blueprint you deploy — and scores the published `world → map` TF per source (`ransac`, `fiducial`, `last_pose`) against the recording's own loop-closure PGO (`PoseGraph.correction_at(t)`). At shutdown it prints a per-source table and writes JSON, a trajectory PNG, and a CSV.

To measure relocalization, replay a **different run** of the same space than the premap was built from — replaying the premap's own recording measures memorization. A fix counts as `success` within 1 m of PGO truth; PGO is silver, not survey-grade, so treat that as a floor.

**Replay** (no robot; replay `<B>` against `<A>`'s premap and marker map):

```bash
uv run --project . \
  dimos --replay --replay-db=<B> \
  run unitree-go2-relocalization-lidar-fiducial --eval \
  -o relocalizationmodule.map_file=<A> \
  -o relocalizationmodule.marker_map_file=<A>.json \
  -o reloceval.marker_map_file=<A>.json
```

**Live** drops `--replay`, sets `ROBOT_IP` + `UNITREE_AES_128_KEY`, and drops `false`/`med_err` (no PGO truth in-process); everything else prints in both modes:

```bash
uv run --project . \
  dimos --robot-ip <ROBOT_IP> \
  run unitree-go2-relocalization-lidar-fiducial --eval \
  -o relocalizationmodule.map_file=<A> \
  -o relocalizationmodule.marker_map_file=<A>.json \
  -o reloceval.marker_map_file=<A>.json
```

Run one replay at a time — the LCM bus is exclusive (`pgrep -af 'dimos --replay'`).

`--eval` writes `out/eval/releval.{eval.json,trajectory.png,accepted_fixes.csv}` (`-o reloceval.tag=<name>` renames the stem). The PNG is the robot path in the `map` frame, each stretch colored by the source that won it; tags are drawn with `-o reloceval.marker_map_file=<A>.json`, showing at a glance which source carried localization and whether the fiducial prior won near a tag. The per-source table:

| Column | Meaning |
|--------|---------|
| `prop` | Cycles where this source put a candidate forward |
| `acc` / `rej` | Accepts this source won and published / rejects its winner took |
| `false` | Accepts whose held-out error exceeded 1 m (replay only) |
| `%traj` | Share of the covered trajectory this source localized |
| `med_err` | Median distance from PGO truth over its accepts, meters (replay only) |
| `med_fit` | Median ICP fitness of its accepts |

`proposed` vs `won` is the honest split: a prior can propose every cycle and win none — the tag was seen but its stored location did not fit the walls as well as lidar. `med_fit` (the judge's confidence) is not `med_err` (distance from truth); a confident wrong pose scores high on both, so the table reports each. Everything the eval sweeps is the module and prior config in the [Configuration reference](#configuration-reference), so an embodiment search (DIM-944) reads and writes those fields directly, and the per-source counts come from the run log, so a sweep is reproducible.

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| `Relocalization module disabled (no map_file configured)` | Missing `-o relocalizationmodule.map_file=…` | Set `map_file` to your premap stem |
| File not found for `.pc2.lcm` | Export not run or wrong cwd | Run `dimos map global … --export` and check cwd or `data/` |
| Long stretch of `ransac reloc skipped` | Map still accumulating points | Wait or drive slowly through mapped geometry |
| Repeated `relocalize rejected` | Poor overlap with premap or wrong space | Start in a known area and check premap in `.rrd`; the line's `source=` names which prior is failing |
| Nav works but map looks misaligned | Low fitness accepted in debug mode | Put that prior's `fitness_threshold` back to its default (`ransac` and `fiducial` `0.6`) |
| PGO map looks wrong | Bad odometry in recording | Run `dimos map replay` or `summary` and re-record with smoother motion |

## Related docs

For hardware setup, simulation, and the full blueprint list, see the [Go2 platform guide](/docs/platforms/quadruped/go2/index.md). The [v0.0.13 release notes](https://github.com/dimensionalOS/dimos/releases/tag/v0.0.13) summarize the PGO, `dimos map`, and relocalization work this guide builds on.
