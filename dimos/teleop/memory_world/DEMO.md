# Memory World demo script

A ROS 2 mcap in, a walkable world out: the ray-traced map, a timeline, and
CLIP embeddings answering questions in plain words with the pictures to prove it.

## Setup (one time per recording)

```bash
memworld ~/datasets/lite_recorder/grocery.mcap   # or bike.mcap / park.mcap / any .db; ~/Commands/memworld
```

- First start on a new recording builds the ray-traced replay (minutes on a
  long one); later starts take seconds. A build that places no scan (tf cannot
  reach the lidar frame, or everything is out of range) is thrown away and the
  timeline says "build failed"; the static map falls back to plain accumulation.
- Questions are answered from the recording's own CLIP/SigLIP frame embeddings.
  The server builds that index on first start, over the colour stream, into the
  recording itself; the ☰ menu shows its progress and offers **Add embeddings**
  when there are none. One file, one tf tree, no companion.
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
- The answer lights the places it found; the bar shows **k / N** places.
  **← →** (or ◀ ▶) fly to each place; only that place's photos stay lit,
  hung where the camera stood. **P** stands at that camera.
- **Orbit** circles the place; **Navigate** (or **N**) draws the A* route
  from where the robot ended to it, over the ray-traced map.
- Answers take ~0.2 s (text embedding on CPU) once the server is warm.

## C — Dimos Spatial Reasoning

- ☰ → **Dimos Spatial Reasoning** (or **T**). Eight stations explaining how
  CLIP embeddings turn a sentence into places on a map. **← →** / Next,
  **Esc** exits:
  1. the recording (overview, roof cut away)
  2. the map: ray tracing (replay plays at 6× while orbiting the robot)
  3. embedding what it saw (eye level along the path)
  4. asking in words: a vector-database lookup (runs the query live)
  5. where it was seen from (the answer's places, next/prev)
  6. walking there (the route)
  7. asking about part of it (runs a windowed question live)
  8. your turn

## Under the hood (for questions)

- Index: every few colour frames through SigLIP 2 by DimOS's own
  `model.embed(image)` — **one vector per image** — appended to the index stream
  with `embedding=`, which puts it in the store's vector index.
  `visual_search.py`, `embed.py`.
- Search: the question through the same model's text tower, then DimOS's own
  **vector-database lookup**, `Stream.search(vector, k)` (`dimos/memory/stream.py`),
  which ranks the recorded frames by cosine. A matching frame is placed at the
  camera pose tf gives it — **where the thing was seen from**, not where the thing
  is; nothing is projected into the map. Matches within `place_radius_m` of each
  other collapse into one place, keeping the best-scoring frame.
  `visual_search.py`, `visual_answers.py`.
- Part of a recording: `find_in_memory` (and `POST /ask`) take `from_fraction` and
  `to_fraction`, fractions of the recording's length — "the first half" is 0.0 to
  0.5. A best match below `min_similarity` is reported as nothing found, which is
  what lets the answer be "no".
- Route: dimos MLS planner (3D terrain traversability over the ray-traced
  map); on maps too large for it, a 2D costmap along the driven path with
  dimos `min_cost_astar`. `route.py`.
