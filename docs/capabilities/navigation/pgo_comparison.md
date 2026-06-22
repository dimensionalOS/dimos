# PGO Comparison Report

Loop-closure pose-graph optimization (PGO) across the go2 recordings. Two metrics:

- **voxel improvement** — fractional drop in occupied 0.2 m voxels after re-anchoring
  scans onto the corrected trajectory. Ground-truth-free; the universal score (higher = tighter).
- **tag improvement** — fractional drop in the position spread of *revisited* AprilTags.
  Relocalization quality; only meaningful where a tag is seen on ≥2 separate visits.

Four algorithms: **gsc_pgo** (this branch's refined GTSAM + Scan-Context PGO — the `pgo`
column), **ivan_pgo**, **ivan_transformer**, and **unrefined_pgo** (the frozen baseline
`gsc_pgo` was refined from). gsc_pgo's loop closure is lidar Scan-Context + ICP — it does
**not** use AprilTags; tags are only the evaluation signal here.

## Headline: china_office1

china_office1 is the one recording with strong, clean tag signal (9 tags, many close
head-on revisits, 2.0 m raw spread). gsc_pgo wins it decisively on both metrics:

| algorithm | voxel | tag | closures |
|---|---|---|---|
| **gsc_pgo** (this PR) | **+0.154** | **+0.80** | 166 |
| ivan_pgo | +0.090 | +0.73 | 328 |
| ivan_transformer | +0.093 | +0.72 | 327 |
| unrefined_pgo | +0.016 | −0.02 | 32 |

### Before / after (china_office1, gsc_pgo)

![china before/after](./assets/pgo_china_before_after.png)

Top-down occupancy: scans on raw odometry (left) vs. re-anchored on the gsc_pgo-corrected
trajectory (right). The 2.0 m → 0.4 m tag-spread correction collapses doubled walls and
sharpens edges. 1543 keyframes, 157 loop closures.

## Cross-recording comparison

![mean voxel improvement](./assets/pgo_mean.png)

![per-recording voxel improvement](./assets/pgo_voxel.png)

![per-recording tag agreement](./assets/pgo_tag.png)

### Reading these honestly

- **Aggregate voxel improvement is a near-tie**, and the frozen baseline is competitive.
  Outside china most recordings are near-no-op (±0.01–0.02 voxel); the baseline "wins" those
  by making almost no closures, so it never *degrades* a map — while the refined algorithms
  take aggressive closures that pay off big where there's real drift but occasionally backfire.
- **gsc_pgo is the relocalization specialist**: it dominates the one map with real drift +
  revisit structure (china), but isn't a uniform improvement everywhere.
- **Tag agreement is only a clean benchmark on china** (and partially gir_park1). Elsewhere
  tags are seen once (no revisit constraint) or are noisy long-range detections. Each
  recording's per-tag revisit breakdown lives in its `summary.json` `april_tags.result`.

## Detailed reference tables

[reference_comparison.md](/docs/capabilities/navigation/pgo_reference_comparison.md) holds the full benchmark — tagged real
recordings, tagless hk_village (voxel-only), artificial-drift robustness, and KITTI — with
china_office1 added as the flagship clean tag entry.

## How the data was produced

- **Tag streams + revisit summaries:** `scripts/add_april.py` writes each recording's
  `raw_april_tags` (every detection) and `april_tags` (gated/clustered visits) streams plus
  the `april_tags` section of its `summary.json` (`filter_parameters` + per-tag `result`).
- **Eval:** `eval_all.py` — lockstep replay through each algorithm, voxel + tag scoring, one
  isolated LCM bus per cell. Gate thresholds are single-sourced in `eval_utils/apriltags.py`.
