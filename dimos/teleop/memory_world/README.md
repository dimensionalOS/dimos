# Memory World

First-person VR exploration of a recorded robot memory. The viewer builds a
height-coloured voxel map from the recording's lidar stream and overlays image
captures, odometry, and the places an answer names.

Questions are answered from the recording's own CLIP/SigLIP image embeddings —
one vector per frame, produced by DimOS's `model.embed()` and recorded into the
store. A question goes through the model's text tower and is answered by a
vector-database lookup (`Stream.search`); each result is placed at the camera
pose of the frame that matched, so a place is somewhere the thing was **seen
from**, not the thing's own position. Nothing is projected into the map.
`DEMO.md` is the walkthrough.

## The match floor

A cosine ranking always returns its top row, so "were there any people" is answered yes
on every recording ever made unless a best match that is merely the *least bad* is
reported as no match. `min_similarity` is that floor.

Measured on `grocery.mcap` (4819 indexed frames), five things plainly in a grocery
recording against five that certainly are not:

| present | best | absent | best |
|---|---|---|---|
| a shelf of bottles | +0.1617 | a ski slope | +0.0823 |
| a refrigerator | +0.1370 | a snowmobile | +0.0633 |
| a shopping basket | +0.1246 | a giraffe | +0.0529 |
| a doorway | +0.1181 | an aeroplane cockpit | +0.0388 |
| a person | +0.1143 | a coral reef | +0.0294 |

The two ranges do not overlap: present spans +0.1143 to +0.1617, absent +0.0294 to
+0.0823. The default is the midpoint of that gap, **0.098**, which leaves about 0.016 of
margin on each side. A different camera, scene or checkpoint will move both columns, so
re-measure rather than carrying this number over.

## Running

```bash
uv run dimos run memory-world-module --memoryworldmodule.store-path recording.mcap
```

`~/Commands/memworld <recording>` wraps this with the right paths and a single
server lock.

The `memory-world-agent` blueprint starts the same viewer with an MCP server and
agent alongside it, so the question can come from a chat rather than the bar:

```bash
uv run dimos run memory-world-agent \
  --memoryworldmodule.store-path hk_building_park.db \
  --memoryworldmodule.background-mode passthrough
```

Set the API key required by the configured model, then run `uv run dimos
humancli` in another terminal. The agent's one tool is `find_in_memory`.

Useful configuration flags include `--voxel-size`, `--max-points`,
`--n-voxel-scans`, `--n-image-markers`, and
`--background-mode {black,passthrough}`. Set `--n-voxel-scans 0` to consume
every lidar frame.

## Viewing in VR

1. On the Quest 3, open the browser and go to `https://<host-ip>:8443/memory_world`.
2. Accept the self-signed cert, tap **Connect**, then enter VR.
3. Use the left thumbstick to walk/strafe and the right thumbstick to turn.
4. Hold and release the right trigger to teleport.
5. Hold both grips, or pinch both hands, and change their separation to scale.
6. Press left X to toggle images, left Y to reset, and right B to toggle voxels.

A minimap shows the current position and heading. The same cloud-derived map is
projected onto the ground.

Host and headset must be on the same Wi-Fi / LAN.
