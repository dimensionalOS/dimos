# Memory World (VR)

First-person VR walkthrough of a robot's recorded point cloud — like Google Street View, but for your robot's memory. Sister to [memory_browser](../memory_browser/) (which is the curved-ribbon browser); this one drops you *inside* the map.

## Running

```bash
python scripts/run_memory_world.py --db data/go2_bigoffice.db --map data/unitree_go2_bigoffice_map.pickle
```

Flags: `--port 8443`, `--voxel-size 0.05` (m, cloud downsample), `--max-points 250000`.

Use `--background passthrough` to render with Quest passthrough outside the
opaque map base. The default is `--background black` for the original immersive
VR view.

Point-cloud source (`--cloud-source`):
- `pickle` (default) — load the prebuilt `--map` PointCloud2; keeps true RGB.
- `lidar` — accumulate a voxel map live from the store's `lidar` stream (no pickle needed, height-coloured, rebuilds from current data). First connect takes a few seconds while it accumulates ~150 scans, then it's cached.

```bash
python scripts/run_memory_world.py --cloud-source lidar
```

Density knobs:
- `--voxel-scans N` — how many lidar scans to accumulate (default 150). `--voxel-scans 0` uses **every** frame for the densest map (slower build; output stays bounded by `--voxel-size` since voxels dedupe).
- `--image-markers N` — how many capture-pose markers to sample (default 200).

## Agentic queries

Run the registered blueprint to ask spatial questions about the recording. It
starts the memory viewer, MCP server, and MCP agent together:

```bash
uv run dimos run memory-world-agent \
  --store-path hk_building_park.db \
  --cloud-source lidar \
  --background-mode passthrough
```

Set the API key required by the configured model, then run `uv run dimos
humancli` in a separate terminal. The agent analyzes the mem2 streams in a
time-limited subprocess and sends only validated result geometry to the viewer.
Answers can include highlighted regions and points, historical evidence paths,
supporting image observations, and a collision-aware route from the current VR
position when the recording has a `global_costmap` stream.

## Viewing in VR

1. On the Quest 3, open the browser and go to `https://<host-ip>:8443/memory_world`.
2. Accept the self-signed cert, tap **Connect**, then enter VR.
3. Controls:
   - **Left thumbstick** — walk / strafe (headset-relative)
   - **Right thumbstick X** — smooth yaw turn (~90°/s at full deflection)
   - **Right trigger** — hold to aim, release to teleport
   - **Both grips** (controllers) — squeeze both grips, then move controllers apart/together to scale the world
   - **Both hands pinch** (hand-tracking mode) — same gesture with thumb+index pinch
   - **Left X** — toggle image thumbnails at capture poses
   - **Left Y** — reset view to the cloud's centroid
   - **Right B** — toggle voxel rendering: solid cubes ↔ points

A GTA-style minimap sits to the lower-left of your view with a red dot for your position and a needle for your heading. The top-down density of the cloud is also pasted on the ground.

Host and headset must be on the same Wi-Fi / LAN.
