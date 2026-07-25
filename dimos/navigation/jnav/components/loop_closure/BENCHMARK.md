# gsc_pgo loop-closure benchmark

Self-consistency of PGO-corrected trajectory vs raw odometry, across every usable
recording in `~/datasets/mid360_recordings/` and `~/datasets/go2_recordings/`.

- **sensor** — `go2` = onboard L1 short-range lidar (go2 config); `mid360` = mounted
  Livox (module defaults). One config per sensor, applied to every map of that sensor.
- **odom** — the odometry variant replayed (pointlio preferred, fastlio fallback; go2
  uses the recording's native `odom`/`go2_odom`).
- **raw_spread / corr_spread** — start↔end position gap (m) before/after correction. A
  loop that truly closes drops corr_spread toward 0; `corr > raw` means the map was
  deformed (worse).
- **tag_impr** — AprilTag re-sighting spread ratio (higher better). **vox_impr** — lidar
  voxel-agreement improvement (higher better).
- **verdict** — `improve` (loop visibly closes), `no-harm` (already-tight map preserved),
  `HARM` (correction deforms the map).

Metrics are self-consistency, not vs a ground-truth SLAM (FAST-LIO/Point-LIO are the
input odometry, not a baseline to beat). Two PGO implementations are compared: the Rust
`gsc_pgo` (scan-context + gtsam ISAM2, module default config) and Ivan's pure-module
`IvanPGO` (KDTree-radius + ICP, `dimos/mapping/loop_closure/pgo.py` core).

## gsc_pgo (primary)

| recording | sensor | odom | closures | keyframes | raw_spread | corr_spread | tag_impr | vox_impr | note | verdict |
|---|---|---|--:|--:|--:|--:|--:|--:|---|---|
| outdoor_small_loop | go2 | odom | 105 | 945 | 0.122 | 0.868 | -6.126 | -0.0544 |  | HARM |
| outdoor_small_loop | mid360 | fastlio_odometry | 93 | 1154 | 0.122 | 0.123 | -0.008 | -0.0002 |  | no-harm |
| grassy_field | go2 | odom | 115 | 1057 | 61.010 | 57.915 | 0.051 | 0.0518 | no-harm target | improve |
| grassy_field | mid360 | fastlio_odometry | 92 | 1308 | 61.010 | 11.569 | 0.810 | 0.0016 | no-harm target | improve |
| gir_stairs1 | go2 | odom | 184 | 1057 | 0.035 | 1.672 | -47.201 | 0.0634 |  | HARM |
| gir_stairs1 | mid360 | pointlio_odometry | 124 | 1283 | 0.035 | 0.050 | -0.450 | 0.0009 |  | no-harm |
| gir_stairs2 | go2 | odom | 400 | 1677 | 0.969 | 6.166 | -5.363 | -0.1087 |  | HARM |
| gir_stairs2 | mid360 | pointlio_odometry | 317 | 1936 | 0.969 | 0.067 | 0.931 | 0.0256 |  | improve |
| gir_park1 | go2 | odom | 174 | 1203 | 0.048 | 8.266 | -172.581 | 0.0231 |  | HARM |
| gir_park1 | mid360 | pointlio_odometry | 162 | 1699 | 0.048 | 0.091 | -0.911 | -0.0006 |  | HARM |
| gir_park1_2 | go2 | go2_odom | 93 | 535 | 0.081 | 16.622 | -204.265 | -0.0174 |  | HARM |
| gir_park1_2 | mid360 | fastlio_odometry | 35 | 606 | 0.081 | 0.185 | -1.290 | 0.0001 |  | HARM |
| huge_loop_go2 | go2 | odom | 228 | 2467 | 10.809 | 300.800 | -26.828 | -0.0700 |  | HARM |
| huge_loop_go2 | mid360 | fastlio_odometry | 247 | 2649 | 59.557 | 2.275 | 0.962 | 0.0089 | loop closes | improve |
| huge_loop_go2 | mid360 | pointlio_odometry | 305 | 2479 | 10.809 | 0.093 | 0.991 | 0.0125 | loop closes | improve |
| huge_loop_realsense | mid360 | pointlio_odometry | 347 | 2716 | — | — | — | 0.0120 | voxel-only | improve |
| china_office1 | go2 | go2_odom | 329 | 1194 | 0.242 | 1.974 | -7.171 | 0.0142 |  | HARM |
| china_office1 | mid360 | pointlio_odometry | 226 | 1579 | 0.242 | 0.239 | 0.009 | 0.0008 |  | no-harm |
| sf_office_survey1 | go2 | go2_odom | 18 | 930 | 1.695 | 1.669 | 0.016 | 0.0043 |  | no-harm |
| sf_office_survey2 | go2 | go2_odom | 20 | 879 | 4.044 | 4.059 | -0.004 | -0.0053 |  | HARM (marginal) |

## IvanPGO (comparison — same recordings, same odom)

| recording | sensor | odom | closures | keyframes | raw_spread | corr_spread | tag_impr | vox_impr |
|---|---|---|--:|--:|--:|--:|--:|--:|
| outdoor_small_loop | go2 | odom | 24 | 941 | 0.122 | 0.662 | -4.435 | -0.0819 |
| outdoor_small_loop | mid360 | fastlio_odometry | 43 | 1125 | 0.122 | 0.118 | 0.029 | -0.0079 |
| grassy_field | go2 | odom | 38 | 1057 | 61.010 | 62.199 | -0.019 | 0.0028 |
| grassy_field | mid360 | fastlio_odometry | 20 | 1267 | 61.010 | 60.582 | 0.007 | -0.0073 |
| gir_stairs1 | go2 | odom | 54 | 1048 | 0.035 | 0.662 | -18.088 | 0.0065 |
| gir_stairs1 | mid360 | pointlio_odometry | 61 | 1258 | 0.035 | 0.041 | -0.179 | -0.0019 |
| gir_stairs2 | go2 | odom | 196 | 1671 | 0.969 | 2.847 | -1.938 | 0.0812 |
| gir_stairs2 | mid360 | pointlio_odometry | 197 | 1789 | 0.969 | 0.096 | 0.901 | 0.0119 |
| gir_park1 | go2 | odom | 62 | 1201 | 0.048 | 1.381 | -28.011 | -0.0457 |
| gir_park1 | mid360 | pointlio_odometry | 127 | 1562 | 0.048 | 0.087 | -0.822 | -0.0049 |
| gir_park1_2 | go2 | go2_odom | 15 | 534 | 0.081 | 1.100 | -12.589 | -0.0118 |
| gir_park1_2 | mid360 | fastlio_odometry | 14 | 601 | 0.081 | 0.962 | -10.875 | -0.0065 |
| huge_loop_go2 | go2 | odom | 68 | 2463 | 10.809 | 22.767 | -1.106 | -0.0674 |
| huge_loop_go2 | mid360 | pointlio_odometry | 64 | 2175 | 10.809 | 10.015 | 0.073 | -0.0067 |
| huge_loop_go2 | mid360 | fastlio_odometry | 57 | 2317 | 59.557 | 57.957 | 0.027 | -0.0144 |
| huge_loop_realsense | mid360 | pointlio_odometry | 233 | 2440 | — | — | — | 0.0055 |
| china_office1 | go2 | go2_odom | 138 | 1185 | 0.242 | 2.096 | -7.680 | 0.0451 |
| china_office1 | mid360 | pointlio_odometry | 131 | 1551 | 0.242 | 0.241 | 0.003 | -0.0022 |

On the huge loop, gsc_pgo (pointlio) drops corr_spread to **0.093 m** (loop closes);
IvanPGO leaves it at **10.015 m** (barely corrected). gsc_pgo closes it; IvanPGO does not.
With FAST-LIO odom (larger revisit drift), gsc_pgo now also closes the huge loop —
**59.557 m → 2.275 m** — after raising `loop_candidate_max_distance_m` from 80 to 200 m so
the far-drifted revisit still reaches ICP (the old 80 m gate skipped it). IvanPGO on the
same fastlio run stays at **57.957 m** (does not close).

## Skipped / not run

- `misc/`, `_deleted_gt_streams_backup` — not recordings.
- `sf_office_survey1`, `sf_office_survey2` now **run and benched** (rows above). The
  co-resident `dim-lcm-constellation` peers still spam
  `zenoh … Received a close message … Unable to connect to any locator of scouted peer`
  warnings on the shared multicast bus, but the eval session establishes and the full
  ~360 s lockstep replay completes anyway — the stray peers slow scouting, they do not
  fatally wedge it. Both are bare-Go2 L1 maps, already tight (1.7 m / 4.0 m spread):
  survey1 preserves the map (no-harm), survey2 nudges spread by 1.5 cm (marginal HARM,
  the known go2 tight-map pattern). Re-run:
  `python temp_dont_commit.ignore/hk_scratch/demo_bench.py --only sf_office_survey1`
  (and `--only sf_office_survey2`).

## Reproduce

Runner: `temp_dont_commit.ignore/hk_scratch/demo_bench.py` (slim-copies each recording to
scratch, never touches originals). Per-recording before/after top-down + isometric PNGs
land in `eval_results/<recording>__<label>__GscPGO/` (gitignored). go2 config =
`dimos/robot/unitree/go2/blueprints/smart/unitree_go2_pgo.py`; mid360 config = gsc_pgo
module defaults.
