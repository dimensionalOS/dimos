# Obstacle maps

Unlabeled top-down obstacle maps for robot navigation, derived from the ground-truth boxes
in `../ground_truth/hssd/`. `<scene_id>.json` lists axis-aligned rectangles in meters in
the ROS `world` frame (x forward, y left); everything outside them is free space.
`<scene_id>.svg` draws the same rectangles on a 1 m grid.

```json
{"dataset": "hssd-hab", "scene_id": "102344193", "frame_id": "world",
 "slice_height_m": 0.2, "units": "m", "count": 59,
 "obstacles": [{"min_xy": [0.469, -3.557], "max_xy": [0.806, 0.85]}]}
```

How a map is built (`make_obstacle_map.py`):

- Only boxes a robot meets 20 cm above the floor count. Carpets, mats and shoes are below
  that; ceiling lamps, windows, pictures, mirrors and tabletop items are above it. They
  are all free space.
- Object footprints that overlap or touch (within 5 cm) are wrapped into one rectangle
  covering all of them: a dining table with its chairs, a bed with its nightstand, a run
  of kitchen cabinets. Pass `--min-fill 0.5` to only wrap while the rectangle stays at
  least half solid.
- Wall pieces merge only while their union stays a thin strip (0.6 m), which folds
  junction duplicates and window frames into their wall; corners never chain and doorways
  remain open.

```bash
uv run python misc/habitat/obstacle_maps/make_obstacle_map.py \
    misc/habitat/ground_truth/hssd/102344193.json --out misc/habitat/obstacle_maps
```
