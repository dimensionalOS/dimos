# Perception localize

Text queries in, verified 3D object instances out, from calibrated cameras posed by tf,
lifted through aligned depth or, for a camera without it, a lidar.
`PerceptionLocalize` (`module.py`) runs it live on a robot's ports;
`tools/tool_localize.py` runs it offline on a recording.

## Pipeline

| stage | where | model |
|---|---|---|
| index | `localize.embed_index` | embedder: 1 Hz sharpest posed frames to embeddings |
| retrieve | `localize.localize` | embedder: query to similarity peaks to candidate frames |
| detect | `localize.localize` | detector: candidate frame and queries to boxes x query scores |
| segment | `localize.localize` | segmenter: boxes to masks |
| lift | `rig.lift` | aligned depth, or the window's lidar scans merged, + tf to one cloud per sighting, gated in `localize._lift` |
| associate | `association.Groups` | sightings to objects by centroid distance, clouds fused |
| verify | `localize._finalize` | score, distinct views, rivals to `Localization` |

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

## Policy

`LocalizePolicy` (`types.py`) holds every threshold. Its defaults were fit on SigLIP + OWLv2
+ EdgeTAM over the xArm recording. The score fields are in the models' own units:

| model | fields on its scale |
|---|---|
| embedder | `peak_prominence` |
| detector | `candidate_floor`, `accept_score` |

`DanDetector.policy(overrides)` starts at the defaults, applies each chosen model's
registered shift (`Option.policy` in `dandetect.py`), then the overrides. A shift only puts
the scores on the model's scale; it is a starting point, not a tuned policy.

| model | shift | from the xArm and YAM recordings |
|---|---|---|
| `owlv2` | none | the defaults |
| `omdet` | `candidate_floor` 0.15, `accept_score` 0.30 | its matches score 0.29 to 0.73, below OWLv2 on most objects |
| `gdino` | `candidate_floor` 0.35, `accept_score` 0.55 | its matches score about 0.2 above OWLv2; 0.35 is its published box threshold |
| `siglip`, `clip` | none | CLIP peaks like SigLIP at the default prominence |
| `edgetam`, `yoloe`, `none` | none | |

Every combination needs its own policy, tuned on the target scene before deploying:

1. Record the scene. Run `tools/tool_localize` on it with the combination, adjusting
   `--policy '{...}'`. Every `verification` log line lists each group's best score,
   distinct views and sightings; the `.rrd` shows what was answered and from where.
2. Deploy the combination and the policy that answered right:
   `PerceptionLocalize.blueprint(detector="gdino", policy={"accept_score": 0.6})`.
   The `localize` skill's `policy` argument still overrides per call.

## Tools

- `tools/tool_localize.py`: the offline bench above. `--help` lists the flags.
