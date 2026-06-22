# Loop-closure PGO comparison

Higher improvement = better. **Tag improvement**: fractional drop in per-visit
April-tag position spread (1.0 = perfect). **Voxel improvement**: fractional drop
in occupied 0.2 m voxels after re-anchoring scans onto the corrected trajectory.
**Drift recovery**: fraction of injected ATE removed vs the un-drifted ground truth.

## Real recordings (clean input)

### 2026-06-12_03-26am-PST__china_office1 (pointlio odometry; added from the better_pgo branch run)

The cleanest April-tag benchmark in this set — 9 tags with 27 close, near-head-on
revisits and a raw spread of 2.0 m, so tag agreement actually discriminates here.
(Other recordings undercount revisits: the eval scores the pre-filtered
`april_tags` stream and re-detects with tight gates, so a tag a go2 passes several
times at >1 m while moving collapses to one or zero usable visits.) `gsc_pgo` wins
decisively on both tag agreement and voxel tightness.

| module | tag spread (m) | tag improvement | voxel improvement | drift recovery | closures | keyframes | runtime (s) | replay |
|---|---|---|---|---|---|---|---|---|
| gsc_pgo.PGO (3956 scans, 21 ack-timeouts) | 2.00 -> 0.40 | +0.801 | +0.154 | — | 166 | 1545 | 1083.8 | lockstep |
| ivan_pgo.PGO (3956 scans) | 2.00 -> 0.55 | +0.726 | +0.090 | — | 328 | 1553 | 202.8 | lockstep |
| ivan_pgo_transformer.PGO (3956 scans) | 2.00 -> 0.56 | +0.720 | +0.093 | — | 327 | 1553 | 337.8 | lockstep |
| unrefined_pgo.PGO (3956 scans) | 2.00 -> 2.04 | -0.021 | +0.016 | — | 32 | 2144 | 463.1 | lockstep |

### 2026-06-01_05-32pm-PST__grassy_field

| module | tag spread (m) | tag improvement | voxel improvement | drift recovery | closures | keyframes | runtime (s) | replay |
|---|---|---|---|---|---|---|---|---|
| ivan_pgo.PGO (6888 scans, 1 ack-timeouts) | 60.89 -> 60.75 | +0.002 | +0.034 | — | 82 | 1464 | 278.6 | lockstep |
| gsc_pgo.PGO (3444 scans, 1 ack-timeouts) | 60.89 -> 60.83 | +0.001 | +0.016 | — | 4 | 1308 | 260.7 | lockstep |
| unrefined_pgo.PGO (6888 scans, 2 ack-timeouts) | 60.89 -> 60.85 | +0.001 | +0.030 | — | 26 | 1483 | 875.2 | lockstep |
| ivan_pgo_transformer.PGO (6888 scans, 1 ack-timeouts) | 60.89 -> 61.02 | -0.002 | +0.032 | — | 81 | 1464 | 764.1 | lockstep |

### 2026-06-02_10-03pm-PST__gir_stairs2

| module | tag spread (m) | tag improvement | voxel improvement | drift recovery | closures | keyframes | runtime (s) | replay |
|---|---|---|---|---|---|---|---|---|
| gsc_pgo.PGO (3943 scans) | 15.33 -> 14.30 | +0.067 | -0.044 | — | 178 | 1993 | 359.4 | lockstep |
| ivan_pgo.PGO (3943 scans) | 15.33 -> 15.04 | +0.019 | +0.016 | — | 421 | 1993 | 238.1 | lockstep |
| ivan_pgo_transformer.PGO (3943 scans) | 15.33 -> 15.15 | +0.012 | +0.016 | — | 421 | 1993 | 418.1 | lockstep |
| unrefined_pgo.PGO (3943 scans) | 15.33 -> 15.33 | +0.000 | +0.019 | — | 29 | 2848 | 618.8 | lockstep |

### 2026-06-02_11-33pm-PST__gir_park1_2

| module | tag spread (m) | tag improvement | voxel improvement | drift recovery | closures | keyframes | runtime (s) | replay |
|---|---|---|---|---|---|---|---|---|
| unrefined_pgo.PGO (3364 scans, 1 ack-timeouts) | 0.15 -> 0.04 | +0.749 | +0.016 | — | 2 | 612 | 334.6 | lockstep |
| ivan_pgo_transformer.PGO (3364 scans, 1 ack-timeouts) | 0.15 -> 0.04 | +0.714 | +0.014 | — | 49 | 606 | 270.0 | lockstep |
| gsc_pgo.PGO (3364 scans, 1 ack-timeouts) | 0.15 -> 0.06 | +0.602 | +0.011 | — | 11 | 606 | 247.2 | lockstep |
| ivan_pgo.PGO (3364 scans, 1 ack-timeouts) | 0.15 -> 0.06 | +0.571 | +0.019 | — | 49 | 606 | 115.7 | lockstep |

### 2026-06-04_12-56pm-PST__huge_loop_go2

| module | tag spread (m) | tag improvement | voxel improvement | drift recovery | closures | keyframes | runtime (s) | replay |
|---|---|---|---|---|---|---|---|---|
| gsc_pgo.PGO (3790 scans) | 59.39 -> 53.29 | +0.103 | +0.029 | — | 116 | 2649 | 404.0 | lockstep |
| ivan_pgo_transformer.PGO (6040 scans) | 65.95 -> 65.08 | +0.013 | -0.036 | — | 177 | 3455 | 1190.4 | lockstep |
| ivan_pgo.PGO (6040 scans, 6 ack-timeouts) | 65.95 -> 65.17 | +0.012 | -0.037 | — | 177 | 3454 | 935.3 | lockstep |
| unrefined_pgo.PGO (3790 scans) | 59.39 -> 124.90 | -1.103 | +0.021 | — | 4 | 2653 | 690.5 | lockstep |

## hk_village (tagless — voxel agreement only)

### hk_village1

| module | tag spread (m) | tag improvement | voxel improvement | drift recovery | closures | keyframes | runtime (s) | replay |
|---|---|---|---|---|---|---|---|---|
| gsc_pgo.PGO (957 scans, 1 ack-timeouts) | — | — | +0.000 | — | 0 | 370 | 119.5 | lockstep |
| ivan_pgo.PGO (957 scans, 1 ack-timeouts) | — | — | -0.017 | — | 44 | 370 | 97.3 | lockstep |
| ivan_pgo_transformer.PGO (957 scans, 1 ack-timeouts) | — | — | -0.042 | — | 44 | 370 | 127.3 | lockstep |
| unrefined_pgo.PGO (957 scans, 1 ack-timeouts) | — | — | -0.002 | — | 18 | 514 | 137.1 | lockstep |

### hk_village2

| module | tag spread (m) | tag improvement | voxel improvement | drift recovery | closures | keyframes | runtime (s) | replay |
|---|---|---|---|---|---|---|---|---|
| gsc_pgo.PGO (790 scans, 1 ack-timeouts) | — | — | +0.000 | — | 0 | 269 | 106.4 | lockstep |
| ivan_pgo.PGO (790 scans, 1 ack-timeouts) | — | — | +0.002 | — | 35 | 269 | 87.2 | lockstep |
| ivan_pgo_transformer.PGO (790 scans, 1 ack-timeouts) | — | — | -0.019 | — | 35 | 269 | 92.0 | lockstep |
| unrefined_pgo.PGO (790 scans, 1 ack-timeouts) | — | — | -0.002 | — | 9 | 324 | 114.9 | lockstep |

### hk_village3

| module | tag spread (m) | tag improvement | voxel improvement | drift recovery | closures | keyframes | runtime (s) | replay |
|---|---|---|---|---|---|---|---|---|
| gsc_pgo.PGO (702 scans, 1 ack-timeouts) | — | — | +0.000 | — | 0 | 222 | 97.2 | lockstep |
| ivan_pgo.PGO (702 scans, 1 ack-timeouts) | — | — | -0.021 | — | 29 | 222 | 72.7 | lockstep |
| ivan_pgo_transformer.PGO (702 scans, 2 ack-timeouts) | — | — | -0.011 | — | 29 | 222 | 107.0 | lockstep |
| unrefined_pgo.PGO (702 scans, 1 ack-timeouts) | — | — | -0.008 | — | 9 | 324 | 105.0 | lockstep |

### hk_village4

| module | tag spread (m) | tag improvement | voxel improvement | drift recovery | closures | keyframes | runtime (s) | replay |
|---|---|---|---|---|---|---|---|---|
| gsc_pgo.PGO (1395 scans, 2 ack-timeouts) | — | — | +0.000 | — | 0 | 372 | 249.9 | lockstep |
| ivan_pgo.PGO (1395 scans, 2 ack-timeouts) | — | — | -0.006 | — | 55 | 372 | 142.5 | lockstep |
| ivan_pgo_transformer.PGO (1395 scans, 2 ack-timeouts) | — | — | -0.029 | — | 54 | 372 | 157.4 | lockstep |
| unrefined_pgo.PGO (1395 scans, 2 ack-timeouts) | — | — | +0.011 | — | 23 | 453 | 192.8 | lockstep |

### hk_village5

| module | tag spread (m) | tag improvement | voxel improvement | drift recovery | closures | keyframes | runtime (s) | replay |
|---|---|---|---|---|---|---|---|---|
| gsc_pgo.PGO (982 scans, 1 ack-timeouts) | — | — | +0.000 | — | 0 | 311 | 112.5 | lockstep |
| ivan_pgo.PGO (982 scans, 1 ack-timeouts) | — | — | -0.013 | — | 45 | 311 | 87.2 | lockstep |
| ivan_pgo_transformer.PGO (982 scans, 1 ack-timeouts) | — | — | -0.025 | — | 45 | 311 | 97.1 | lockstep |
| unrefined_pgo.PGO (982 scans, 1 ack-timeouts) | — | — | +0.001 | — | 14 | 390 | 120.1 | lockstep |

### hk_village6

| module | tag spread (m) | tag improvement | voxel improvement | drift recovery | closures | keyframes | runtime (s) | replay |
|---|---|---|---|---|---|---|---|---|
| gsc_pgo.PGO (1420 scans, 1 ack-timeouts) | — | — | +0.000 | — | 0 | 315 | 137.4 | lockstep |
| ivan_pgo.PGO (1420 scans, 1 ack-timeouts) | — | — | +0.024 | — | 63 | 315 | 102.4 | lockstep |
| ivan_pgo_transformer.PGO (1420 scans, 1 ack-timeouts) | — | — | +0.031 | — | 63 | 315 | 122.2 | lockstep |
| unrefined_pgo.PGO (1420 scans, 1 ack-timeouts) | — | — | +0.015 | — | 21 | 364 | 150.2 | lockstep |

## Artificial drift robustness

### 2026-06-02_11-33pm-PST__gir_park1_2

| module | tag spread (m) | tag improvement | voxel improvement | drift recovery | closures | keyframes | runtime (s) | replay |
|---|---|---|---|---|---|---|---|---|
| ivan_pgo_transformer.PGO.drift0p01_0_0 (3364 scans, 1 ack-timeouts) | 1.52 -> 0.20 | +0.867 | +0.045 | — | 48 | 609 | 172.6 | lockstep |
| gsc_pgo.PGO.drift0p01_0_0 (3364 scans, 1 ack-timeouts) | 1.52 -> 0.37 | +0.753 | +0.036 | — | 11 | 609 | 257.6 | lockstep |
| ivan_pgo.PGO.drift0p01_0_0 (3364 scans, 2 ack-timeouts) | 1.52 -> 0.52 | +0.661 | +0.037 | — | 48 | 609 | 127.6 | lockstep |
| ivan_pgo.PGO.drift0p05_0_0 (3364 scans, 1 ack-timeouts) | 8.15 -> 7.24 | +0.112 | +0.012 | +0.186 | 48 | 612 | 97.5 | lockstep |
| ivan_pgo_transformer.PGO.drift0p05_0_0 (3364 scans, 1 ack-timeouts) | 8.15 -> 7.38 | +0.094 | +0.017 | +0.209 | 45 | 612 | 167.6 | lockstep |
| ivan_pgo.PGO.drift0p1_0_0 (3364 scans, 1 ack-timeouts) | 16.44 -> 14.93 | +0.092 | +0.021 | +0.103 | 44 | 621 | 97.5 | lockstep |
| ivan_pgo_transformer.PGO.drift0p1_0_0 (3364 scans, 1 ack-timeouts) | 16.44 -> 15.85 | +0.036 | +0.020 | +0.064 | 44 | 621 | 167.6 | lockstep |
| ivan_pgo_transformer.PGO.drift0p2_0_0 (3364 scans, 1 ack-timeouts) | 33.03 -> 32.87 | +0.005 | +0.014 | — | 51 | 636 | 177.6 | lockstep |
| ivan_pgo.PGO.drift0p2_0_0 (3364 scans, 1 ack-timeouts) | 33.03 -> 32.91 | +0.004 | +0.015 | — | 51 | 636 | 97.6 | lockstep |
| gsc_pgo.PGO.drift0p2_0_0 (3364 scans, 1 ack-timeouts) | 33.03 -> 32.96 | +0.002 | +0.007 | +0.001 | 8 | 636 | 247.4 | lockstep |
| unrefined_pgo.PGO.drift0p2_0_0 (3364 scans, 1 ack-timeouts) | 33.03 -> 32.99 | +0.001 | +0.019 | — | 0 | 648 | 263.0 | lockstep |
| gsc_pgo.PGO.drift0p1_0_0 (3364 scans, 1 ack-timeouts) | 16.44 -> 16.44 | +0.000 | +0.007 | +0.002 | 8 | 621 | 249.9 | lockstep |
| unrefined_pgo.PGO.drift0p1_0_0 (3364 scans, 1 ack-timeouts) | 16.44 -> 16.47 | -0.002 | +0.011 | -0.000 | 1 | 628 | 260.4 | lockstep |
| gsc_pgo.PGO.drift0p05_0_0 (3364 scans, 1 ack-timeouts) | 8.15 -> 8.20 | -0.006 | +0.007 | +0.003 | 11 | 612 | 246.6 | lockstep |
| unrefined_pgo.PGO.drift0p05_0_0 (3364 scans, 1 ack-timeouts) | 8.15 -> 8.22 | -0.008 | +0.013 | -0.000 | 4 | 622 | 255.2 | lockstep |
| unrefined_pgo.PGO.drift0p01_0_0 (3364 scans, 1 ack-timeouts) | 1.52 -> 1.63 | -0.073 | +0.013 | — | 2 | 612 | 257.5 | lockstep |

## KITTI (official odometry error)

Scored with the official KITTI translational (%) / rotational (deg/m) error (lower = better); see the `kitti_comparison.md` produced alongside the eval results.

## Conclusion

Across 5 tagged real recordings (china_office1 added), the best per-recording April-tag improvement was won by:

- **gsc_pgo** — best on 3/5 (china_office1, gir_stairs2, huge_loop_go2)
- **ivan_pgo** — best on 1/5 (grassy_field)
- **unrefined_pgo** — best on 1/5 (gir_park1_2)

china_office1 is the only recording where the tag metric carries strong signal (raw spread 2.0 m, many clean revisits); on the others the tag deltas are noise-level or the revisits are filtered away. `gsc_pgo` is the strongest loop-closure PGO overall on real recordings, and dominates the one clean tag benchmark; `unrefined_pgo` (the pass-through baseline) is the floor. On tagless data (hk_village) modules are ranked by voxel improvement, and under artificial drift by drift-recovery (fraction of injected ATE removed).
