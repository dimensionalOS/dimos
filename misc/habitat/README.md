# Habitat ground truth

`ground_truth/hssd/<scene_id>.json` lists every placed object and every wall of the
HSSD scenes used by the Habitat QA suites (`dimos/evals/suites/habitat/hssd/`), as the
JSON view of a `vision_msgs.Detection3DArray`: one entry per box with `id`, `label`,
`labels`, `center_xyz`, `size_xyz` and `orientation_xyzw`, preceded by `dataset` and
`scene_id`. The files are generated offline from the dataset's scene JSON and GLB meshes
by `dimos/simulation/habitat/hssd_ground_truth.py`; no simulator is involved.

- Frame: the DimOS ROS Z-up `world` frame the Habitat connection publishes odometry in.
  Habitat `(x, y, z)` maps to ROS `(-z, -x, y)`.
- Objects: `id` is `<template hash>@<index in scene_instance.json>`; `label` is the HSSD
  category and `labels` adds the product name, both from `semantics/objects.csv`.
- Walls: the stage's `wall_<n>` groups (each holds one wall's faces and trim), boxed as
  whole subtrees; `id` and `label` are the node name. Walls follow the objects. The two
  top-level `geometry_wall*` meshes in every stage are merged leftovers of the whole
  building and are skipped.
- Boxes are world-axis-aligned and measured from mesh vertices, so they are tight for
  axis-aligned placements and loose for rotated objects and diagonal walls. Orientation is
  always the identity. `timestamp` is 0 because the scenes are static.

| Suite | Scene id |
|---|---|
| HSSD scene 1 | 102344193 |
| HSSD scene 2 | 102344403 |
| HSSD scene 3 | 103997424_171030444 |
| HSSD scene 4 | 103997970_171031287 |
| HSSD scene 5 | 104348463_171513588 |
| HSSD scene 6 | 106366410_174226806 |
| HSSD scene 7 | 106878858_174886965 |
| HSSD scene 8 | 107734110_175999914 |
| HSSD scene 9 | 108736851_177263586 |
| HSSD scene 10 | 108736884_177263634 |

Regenerate with the HSSD dataset installed (default `target/habitat/data/hssd-hab`;
point `HSSD_DATASET_CONFIG` at another `hssd-hab.scene_dataset_config.json`):

```bash
uv run python -m dimos.simulation.habitat.hssd_ground_truth
```

For the typed message instead of the view, call
`dimos.simulation.habitat.hssd_ground_truth.scene_detections` and publish it like any
other DimOS message, e.g. `LCMTransport("/detections_3d", Detection3DArray)`.
