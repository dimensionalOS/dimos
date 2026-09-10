# Memory World

First-person VR exploration and agentic analysis of a recorded robot memory.
The viewer builds a height-coloured voxel map from the recording's lidar stream
and overlays image captures, odometry, and validated query results.

## Running

The registered blueprint starts the viewer, MCP server, and MCP agent together:

```bash
uv run dimos run memory-world-agent \
  --store-path hk_building_park.db \
  --background-mode passthrough
```

Set the API key required by the configured model, then run `uv run dimos
humancli` in another terminal. The agent analyzes mem2 streams in a time-limited
subprocess and sends only validated result geometry to the viewer. Answers can
include highlighted regions and points, historical evidence paths, supporting
image observations, and a collision-aware route when the recording contains a
`global_costmap` stream.

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

A minimap shows the current position and heading.

Host and headset must be on the same Wi-Fi / LAN.
