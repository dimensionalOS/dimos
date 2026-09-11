# Memory World demo script

A ROS 2 mcap in, a walkable world out: the ray-traced map, a timeline, and
Hyperspace answering questions in plain words with the pictures to prove it.

## Setup (one time per recording)

```bash
memworld ~/datasets/lite_recorder/grocery.mcap   # or bike.mcap / park.mcap / any .db; ~/Commands/memworld
```

- Two workarounds for the cart recordings: `camera-level-roll` (on) levels the
  camera, whose tf roll is 30-50 degrees out, and the capture-pose photos hang
  at the height the camera was, not a fixed one. Both go once tf is fixed.
- First start on a new recording builds the ray-traced replay (minutes on a
  long one); later starts take seconds. A build that places no scan (tf cannot
  reach the lidar frame, or everything is out of range) is thrown away and the
  timeline says "build failed"; the static map falls back to plain accumulation.
- Search needs `<recording>.hyperspace.db` (SigLIP2 so400m keyframes +
  patches). Without it the ☰ menu shows **Prepare search** which runs the
  ingest in the background (≈ real time on the Mac's GPU), or run it ahead:

  ```bash
  PYTHONPATH=$PWD python -m dimos.teleop.memory_world.hyperspace_ingest recording.mcap \
      --model-name ~/models/siglip2-so400m-patch16-384 --device mps
  ```

- The world frame is taken from the tf root (`odom` on the Pi rig) unless
  `--memoryworldmodule.world-frame` says otherwise. An outdoor ride makes a
  city-scale map (bike.mcap: 6.7M voxels); pass
  `--memoryworldmodule.max-points 3500000` so streets are not thinned to dots,
  and expect the first connection to take a minute while the marker photos
  are decoded from the mcap.
- Open `https://127.0.0.1:8443/memory_world?flat` (self-signed cert →
  Advanced → proceed), press **Connect**. Phone/Quest: same URL on the LAN
  address the launcher prints.

## A — The world

- Drag to look, **W A S D** walk, **Q / E** down / up, **Shift** sprint,
  wheel scales the world. Movement eases in and out.
- **O** (or *Orbit base_link*) circles the robot; the ☰ menu's **Orbit
  frame** picker orbits any tf frame (d455_link, mid360_link, …). The orbit
  target follows the timeline.
- The scrubber at the bottom replays the recording: the ray-traced map grows
  scan by scan, ▶ plays, ✕ returns to the full map.

## B — Ask it

- Type in the bar at the top (**/** focuses it) — e.g. *a traffic cone*,
  *a chair*, *a whiteboard*, *a fire extinguisher* — or hold **Hold to ask**
  and speak.
- The heat map paints the answer; the bar shows **k / N** places.
  **← →** (or ◀ ▶) fly to each place; only that place's photos stay lit,
  hung where the camera stood. **P** stands at that camera.
- **Orbit** circles the place; **Navigate** (or **N**) draws the A* route
  from where the robot ended to it, over the ray-traced map.
- Answers take ~0.2 s (text embedding on CPU) once the server is warm.

## C — Explain Hyperspace

- ☰ → **Explain Hyperspace** (or **T**). Nine stations, **← →** / Next,
  **Esc** exits:
  1. the recording (overview, roof cut away)
  2. the map: ray tracing (replay plays at 6× while orbiting the robot)
  3. seeing: keyframes and patches (eye level along the path)
  4. asking (runs the query live)
  5. hot pyramids (the frusta of the hot patches)
  6. voxel counting (the heat map)
  7. places (clusters, next/prev)
  8. navigation (the route)
  9. your turn

## Under the hood (for questions)

- Search: one fp16 matrix product over every stored patch (torch, ~5 ms),
  contrast against background prompts, the segmenter's floor/wall/ceiling
  patches gated out, pyramids rasterized in one batch, pooled per voxel
  (max per frame → log-sum-exp across frames → × √ viewing directions),
  heat kept only next to the ray-traced map, connected components ranked by
  summed score. `hyperspace_fast.py`, `hyperspace_search.py`.
- Route: dimos MLS planner (3D terrain traversability over the ray-traced
  map); on maps too large for it, a 2D costmap along the driven path with
  dimos `min_cost_astar`. `route.py`.
- Switches: `--memoryworldmodule.hyperspace-refine` picks the refinement:
  `default` (Hyperspace's own chain, the default), a chain of your own such
  as `"occupancy,prior"`, `occupancy` (heat next to the map + components
  only) or `none` (the raw map); `--memoryworldmodule.hyperspace-segments false`
  drops the segment channel.
