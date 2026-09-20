# Habitat ground truth

Ground-truth object and wall boxes for the ten HSSD scenes used by the Habitat QA suites
(`dimos/evals/suites/habitat/hssd/`), exported once from the dataset files and checked in
here as plain JSON. No simulator runs to produce them. Every file is the JSON view of a
DimOS `vision_msgs` message, so the same data can be published over LCM to anything that
consumes detections.

## Pipeline

```text
HSSD dataset (scene_instance.json + GLB meshes)
    |
    |  dimos/simulation/habitat/hssd_ground_truth.py      (needs the dataset)
    v
ground_truth/hssd/<scene_id>.json            3D boxes, Detection3DArray view
    |                                   \
    |  project_top_down.py               \  obstacle_maps/make_obstacle_map.py
    v                                     v
ground_truth/hssd/<scene_id>.top_down.json   obstacle_maps/<scene_id>.json
    |  2D footprints, Detection2DArray view     unlabeled navigation rectangles
    |  render_top_down.py
    v
ground_truth/hssd/<scene_id>.top_down.svg    reference drawing
```

Only the first step needs the HSSD dataset. Everything below it runs from the checked-in
3D file, so the 2D view, the drawings and the obstacle maps can be regenerated or re-tuned
on any machine.

## Layout

| Path | What it is |
|---|---|
| `ground_truth/hssd/<scene_id>.json` | Every placed object and every wall of the scene as 3D boxes |
| `ground_truth/hssd/<scene_id>.top_down.json` | The same boxes flattened to floor rectangles |
| `ground_truth/hssd/<scene_id>.top_down.svg` | Drawing of the 2D file on a 1 m grid |
| `obstacle_maps/` | Unlabeled obstacle rectangles for navigation, see its [README](obstacle_maps/README.md) |
| `project_top_down.py` | 3D file to 2D file |
| `render_top_down.py` | 2D file to SVG |
| `obstacle_maps/make_obstacle_map.py` | 3D file to obstacle map |

The generator itself lives in the package, `dimos/simulation/habitat/hssd_ground_truth.py`,
on top of the simulator-agnostic conversion library `dimos/simulation/object_detections.py`.

## Regenerating

From the dataset (default location `target/habitat/data/hssd-hab`; point
`HSSD_DATASET_CONFIG` at another `hssd-hab.scene_dataset_config.json`). This writes both the
3D and the 2D file for each scene:

```bash
uv run python -m dimos.simulation.habitat.hssd_ground_truth
```

Add `--scene <id>` (repeatable) for a subset and `--label-mode category|name|category+name`
to change what goes into `label`.

From the checked-in 3D files, no dataset needed:

```bash
uv run python misc/habitat/project_top_down.py misc/habitat/ground_truth/hssd/*[0-9].json
uv run python misc/habitat/render_top_down.py misc/habitat/ground_truth/hssd/*.top_down.json
uv run python misc/habitat/obstacle_maps/make_obstacle_map.py \
    misc/habitat/ground_truth/hssd/102344193.json --out misc/habitat/obstacle_maps
```

Tests:

```bash
uv run pytest dimos/simulation/test_object_detections.py dimos/simulation/habitat/test_hssd_ground_truth.py
```

## Formats

Both JSON files start with `dataset` and `scene_id`, then `frame_id`, `timestamp` (always 0,
the scenes are static), `count` and `detections`.

### 3D: `<scene_id>.json`

One entry per box with `id`, `label`, `score`, `center_xyz`, `size_xyz`, `orientation_xyzw`,
and `labels` when the box has more than one. Meters in the DimOS ROS Z-up `world` frame the
Habitat connection publishes odometry in (x forward, y left, z up). Habitat `(x, y, z)` maps
to ROS `(-z, -x, y)`.

```json
{"id": "3d779b56904aafa1c8f2412e4f5e4c4460098bb0@0", "label": "drinkware", "score": 1.0,
 "center_xyz": [3.877, 8.109, 0.525], "size_xyz": [0.128, 0.153, 0.131],
 "orientation_xyzw": [0.0, 0.0, 0.0, 1.0], "labels": ["drinkware", "Frosty Santa mug"]}
```

- Objects come first. `id` is `<template hash>@<index in scene_instance.json>`, `label` is
  the HSSD category and `labels` adds the product name, both from `semantics/objects.csv`.
- Walls follow, labelled `wall` with ids `wall_<k>` numbered by footprint position.
- Boxes are world-axis-aligned and measured from mesh vertices: tight for axis-aligned
  placements, loose for rotated objects and diagonal walls. Orientation is always the identity.

### 2D: `<scene_id>.top_down.json`

The 3D file flattened to the floor plane: `center_xy` and `size_xy` in meters in the same
`world` frame, `theta` always 0, and the same `id`, `label`, `score` and `labels`. Height is
dropped; join by `id` with the 3D file to get it back. Boxes whose footprint spans at least
90 % of the scene extent on both axes (a floor slab or roof) are left out; none of the ten
scenes has one.

```json
{"id": "670c0caf8cb7df8466c675d7c91f7877840f9513@12", "label": "chair", "score": 1.0,
 "center_xy": [7.628, 2.069], "size_xy": [0.528, 0.603], "theta": 0.0,
 "labels": ["chair", "Siena 27 Inch Bar Stool, Blue"]}
```

## How walls are found

The stage mesh is sliced horizontally 20 cm above the floor. Whatever is solid there is a
wall segment, so a doorway is a gap between two wall boxes that a robot can pass through,
and door or window headers are not boxed. Cut lines closer than 20 cm merge into one thin
segment (perpendicular walls stay separate, so L and T junctions overlap by the neighbour's
thickness), and each segment is extruded up to the top of its wall geometry so half walls
keep their height.

## Using the typed messages from Python

```python
from dimos.simulation.object_detections import read_detection3d_json, top_down

detections, provenance = read_detection3d_json("misc/habitat/ground_truth/hssd/102344193.json")
flat = top_down(detections)  # Detection2DArray
```

Or straight from the dataset with `dimos.simulation.habitat.hssd_ground_truth.scene_detections`.
Publish either like any other DimOS message, e.g.
`LCMTransport("/detections_3d", Detection3DArray).publish(detections)`.

## Scenes

| Suite | Scene id | Objects | Walls |
|---|---|---|---|
| HSSD scene 1 | 102344193 | 95 | 32 |
| HSSD scene 2 | 102344403 | 423 | 67 |
| HSSD scene 3 | 103997424_171030444 | 173 | 14 |
| HSSD scene 4 | 103997970_171031287 | 77 | 16 |
| HSSD scene 5 | 104348463_171513588 | 164 | 25 |
| HSSD scene 6 | 106366410_174226806 | 279 | 20 |
| HSSD scene 7 | 106878858_174886965 | 331 | 59 |
| HSSD scene 8 | 107734110_175999914 | 64 | 20 |
| HSSD scene 9 | 108736851_177263586 | 268 | 35 |
| HSSD scene 10 | 108736884_177263634 | 264 | 42 |
