# Memory World

First-person VR exploration and agentic analysis of a recorded robot memory.
The viewer builds a height-coloured voxel map from the recording's lidar stream
and overlays image captures, odometry, and validated query results.

## Running

The registered blueprint starts the viewer, MCP server, and MCP agent together:

```bash
uv run dimos run memory-world-agent \
  --store-path recording.db \
  --background-mode passthrough
```

The map comes from the recording: a one-time run of `memory-world-map` replays
the recording's lidar and tf into the ray tracing mapper the Go2 runs and
records the map it builds. The agent blueprint then loads that map at once and
feeds it, with the robot's final pose, to the MLS planner.

```bash
uv run dimos run memory-world-map --store-path recording.db --dataset recording.db
```

Set the API key required by the configured model, then run `uv run dimos
humancli` in another terminal. The agent analyzes mem2 streams in a time-limited
subprocess and sends only validated result geometry to the viewer. Answers can
include highlighted regions and points, historical evidence paths, supporting
image observations, and the route the MLS planner finds from the robot's last
recorded pose to a navigation goal.

Useful configuration flags include `--max-points`,
`--map-z-min`, `--map-z-max`, `--n-image-markers`, and
`--background-mode {black,passthrough}`.

## Viewing in VR

1. On the Quest 3, open the browser and go to `https://<host-ip>:8443/memory_world`.
2. Accept the self-signed cert, tap **Connect**, then enter VR.
3. Use the left thumbstick to walk/strafe and the right thumbstick to turn.
4. Hold and release the right trigger to teleport.
5. Hold both grips, or pinch both hands, and change their separation to scale.
6. Press left X to toggle images, left Y to reset, and right B to toggle voxels.

Host and headset must be on the same Wi-Fi / LAN.
