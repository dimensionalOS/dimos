# Perception localize

Text queries in, verified 3D object instances out, from calibrated cameras posed by tf,
lifted through aligned depth or, for a camera without it, a lidar.
`PerceptionLocalize` (`module.py`) runs it live on a robot's ports;
`tools/tool_localize.py` runs it offline on a recording. Agents calling `state` and
`localize` over MCP: [`SKILL.md`](SKILL.md).

## Pipeline

| stage | where | model |
|---|---|---|
| index | `localize.embed_index` | embedder: sharpest posed frames, 1 Hz or 3 Hz on a mobile rig, to embeddings |
| retrieve | `localize.retrieve` | embedder: query to similarity peaks to candidate frames |
| support plane | `localize.support_planes` | one plane per camera and 2 m cell, fit from the call's candidates, for the surface-patch gate |
| detect | `localize.detect` | detector: candidate frame and queries to boxes x query scores |
| segment | `localize.segment` | segmenter: boxes to masks |
| lift | `rig.lift` | aligned depth, or the window's lidar scans merged, + tf to one cloud per sighting, gated in `localize._lift` |
| associate | `localize.associate` | sightings to objects by centroid distance in `association.Groups`, clouds fused |
| verify | `localize._finalize` | score, distinct views, rivals to `Localization` |

`localize.localize` runs retrieve and support plane once per call, then detect, segment and
associate frame by frame, so one frame's masks are in memory at a time, then verify.

## Swapping a model

`localize` sees three interfaces only, all in `dandetect.py`:

| stage | interface | registered |
|---|---|---|
| embedder | dimos `EmbeddingModel`: `embed(*images)`, `embed_text(*texts)` | `siglip`, `clip` |
| detector | `Detector.query_score_rows(image, queries, threshold) -> (boxes, scores)` | `owlv2`, `omdet`, `gdino` |
| segmenter | `Segmenter.segment(detections) -> detections` | `edgetam`, `yoloe`, `none` |

`boxes` are pixel `(x1, y1, x2, y2)` of every box whose best query clears `threshold`;
`scores` is `(n_boxes, n_queries)`. Every model also has `stop()`.

Each stage takes a registry name, a `module:attr` path to a class or factory, or a built
model. One line:

```
python -m dimos.perception.localize.tools.tool_localize ... --detector gdino --embedder clip
PerceptionLocalize.blueprint(detector="gdino", embedder="clip")
DanDetector(detector="my_pkg.my_detector:MyDetector")
```

To bring a model from outside, wrap it so it has the stage's methods and pass its path.
To register it by name, add an `Option` to `EMBEDDERS`, `DETECTORS` or `SEGMENTERS`.

## Parameters

`LocalizeParams` (`types.py`) names every threshold and what it gates. The values are per rig,
presets in `localize_params.py`, each fit with SigLIP + OWLv2 + EdgeTAM on that rig's recordings:

| preset | constant | rig | differs from `manipulation` |
|---|---|---|---|
| `manipulation` | `MANIPULATION_PARAMS` | tabletop manipulation baseline | |
| `navigation` | `NAVIGATION_PARAMS` | walking robot, room scale, 5 cm lidar map | `candidate_floor` 0.18, `accept_score` 0.32, `cluster_radius_m` 0.30, `verify_radius_m` 5.0, `max_object_extent_m` 2.0, `surface_patch_max_rise_m` 0.08, `surface_patch_min_drop_m` -0.06, `min_points` 13, `min_camera_range_m` 0.5, `fuse_voxel_m` 0.03 |
| `xarm` | `XARM_PARAMS` | wrist camera over a desk (`MANIPULATION_PARAMS`) | |
| `dual_yam` | `DUAL_YAM_PARAMS` | dual OpenYAM, D405 wrist cameras | `min_camera_range_m` 0.07: the D405s see objects from 7 cm |
| `go2` | `GO2_PARAMS` | Unitree Go2 (`NAVIGATION_PARAMS`) | same as `navigation` |
| `g1` | `G1_PARAMS` | walking robot with dense depth instead of a 5 cm lidar map | `navigation` with `min_points` 30 |

Params are set once and changed per call:

- `PerceptionLocalize` takes its rig's params at construction, `MANIPULATION_PARAMS` when not given:
  `PerceptionLocalize.blueprint(params=GO2_PARAMS)`. A rig without a preset of its own builds
  its params from the closest: the G1 and Alfred recordings run `G1_PARAMS`
  (`replace(NAVIGATION_PARAMS, min_points=30)`), room scale with a cloud floor for dense depth instead
  of a 5 cm lidar map.
- A `localize` call's `params` argument changes them for that call only. A preset name
  switches to that preset; a JSON object sets fields on the module's params, or on the preset
  its `"base"` key names: `'{"base": "xarm", "accept_score": 0.45}'`. The next call without
  one is back on the module's params.
- `tools/tool_localize --params` takes the same string, on `xarm` when omitted.

The score fields are in the models' own units:

| model | fields on its scale |
|---|---|
| embedder | `peak_prominence` |
| detector | `candidate_floor`, `accept_score` |

`DanDetector.shifted(base, fields)` applies each chosen model's registered score delta
(`Option.shift` in `dandetect.py`) relative to OWLv2 on `base`, then `fields`.
A shift puts any preset's scores on the chosen model's scale; explicit overrides in `fields`
take precedence.

| model | shift (delta vs OWLv2) | from the xArm and YAM recordings |
|---|---|---|
| `owlv2` | none | the presets' own |
| `omdet` | `candidate_floor` -0.10, `accept_score` -0.10 | its matches score 0.29 to 0.73, below OWLv2 on most objects |
| `gdino` | `candidate_floor` +0.10, `accept_score` +0.15 | its matches score about 0.2 above OWLv2; 0.35 is its published box threshold on desk scale |
| `siglip`, `clip` | none | CLIP peaks like SigLIP at the default prominence |
| `edgetam`, `yoloe`, `none` | none | |

Every combination needs its own params, tuned on the target scene before deploying:

1. Record the scene. Run `tools/tool_localize` on it with the combination and the closest
   `--params` preset, adjusting fields with `--params '{"base": ..., ...}'`. Every
   `verification` log line lists each group's best score, distinct views and sightings; the
   `.rrd` shows what was answered and from where.
2. Deploy the combination and the params that answered right, a new preset in
   `localize_params.py` for a new rig, or built from an existing one:
   `PerceptionLocalize.blueprint(detector="gdino", params=replace(XARM_PARAMS, accept_score=0.6))`.
   The `localize` skill's `params` argument still changes them per call.

## Tools

`tools/tool_localize.py` runs the stack offline on a recording, to tune params before
deploying them. `--help` lists the flags.

```
OMP_NUM_THREADS=1 uv run python -m dimos.perception.localize.tools.tool_localize \
    "label a" "label b" out.rrd --dataset <recording.db> --from <s> --duration <s> \
    --multi --params <preset or JSON>
uv run dimos-viewer out.rrd
```

- `--from` and `--duration` are seconds from the recording's first colour frame.
- `--multi` gives the labels one shared detection pass, as a live call does; without
  it each label runs alone.
- It prints the params it runs (`params: go2 {'min_points': 30}`), one `hit` line per
  verified object, and `no verified detection of ...` per miss. Exit code 0 when any
  label verified.
- `OMP_NUM_THREADS=1` makes the support-plane fit repeatable, so two runs compare.
