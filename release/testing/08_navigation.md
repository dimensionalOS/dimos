# 8. Navigation, mapping and localization

Most checks run in MuJoCo or on replay data from LFS. Checks with a Livox Mid-360 need the lidar on the LAN. Native modules build with nix on first use, and the Cachix cache in [native modules](../../docs/usage/native_modules.md) makes that fast. See the [navigation docs](../../docs/capabilities/navigation/index.md).

## 8.1 Go2 3D navigation in MuJoCo

```bash
# The first start builds four native navigation modules and pulls the go2_sim model.
uv run dimos run go2-sim-nav
```

* Rerun shows the office and a voxel map that grows from the simulated Mid-360.
* Click a floor point a few meters away. A path appears and the robot walks there.

Expected result: the legged robot reaches the clicked goal without crossing walls, and no module crashes.

## 8.2 Go2 premap and relocalization

Needs the `mapping` extra. The first run pulls the `go2_hongkong_office` recording.

```bash
uv run dimos map global go2_hongkong_office --pgo --export --export-pc2
uv run dimos --replay --replay-db go2_hongkong_office run unitree-go2-map --map-file go2_hongkong_office
uv run dimos --replay --replay-db go2_hongkong_office run unitree-go2
```

* `map global` opens Rerun with `world/pgo_map/pointcloud` and red lines in `world/pgo_map/pgo/loop_closures`. It writes `go2_hongkong_office.map.db` (the keyframe premap) and `go2_hongkong_office.pc2.lcm` (the point-cloud export) to the current directory. `--export` alone only writes the `.map.db` file; `--export-pc2` produces the `.pc2.lcm` file and enables PGO.
* The premap replay logs `relocalize: fix` and `PGO premap: attached`; `world/global_map` then contains the attached premap and live scan. `unitree-go2-map` saves the updated `.map.db` and backs up the previous copy.
* The PGO replay grows `world/pgo_keyframes` and publishes loop edges to `world/pgo_loops` when the walk revisits an area.

Expected result: the premap exports, relocalization finds a fix, and PGO closes loops, without tracebacks.

## 8.3 Mid-360 relocalization in replay

The first run pulls the `go2_mid360_sf_office_outdoors_2026-05-29` recording and premap. Allow about 7 GB of free disk during the download and extraction. If the Rerun viewer closes while the data is downloading, rerun the command after the data is cached to inspect the map.

```bash
uv run dimos run relocalize-mid360 --map-file go2_mid360_sf_office_outdoors_2026-05-29
uv run dimos run unitree-go2-nav-3d-relocalization-replay --map-file go2_mid360_sf_office_outdoors_2026-05-29
```

* In `relocalize-mid360`, `world/loaded_map` appears in Rerun once a fix lands and lies on top of the live map.
* In the nav replay, Rerun has no `world/loaded_map`. The premap seeds the voxel map in `world/map_regions` instead.

Expected result: both blueprints log `relocalize fix` followed by `placed, no further attempts`, without tracebacks.

## 8.4 Go2 3D navigation with a Mid-360

Needs a Go2 with a Livox Mid-360 mounted and reachable on the LAN.

```bash
uv run dimos run unitree-go2-nav-3d --lidar-ip=<mid360-ip>
```

* Drive a few meters with WASD in Rerun. The voxel map grows around the robot.
* Click a point on the floor. A path appears and the robot walks there.
* Click a point at the top of a short flight of stairs. The robot climbs them.

Expected result: the robot reaches each clicked goal, including the one up the stairs.

## 8.5 Mid-360 driver and Point-LIO

```bash
# Replay a recorded capture, no sensor needed. The first command prints the pcap path.
uv run python -c "from dimos.utils.data import get_data; print(get_data('mid360_shake_stairs/mid360_shake_stairs.pcap'))"
DIMOS_MID360_PCAP=<pcap-path> uv run dimos run mid360-pointlio-replay
# Live sensor on the LAN.
uv run dimos run mid360-pointlio-voxels --lidar-ip <lidar-ip>
```

* Replay: the capture lasts about two minutes and ends with the log line `packet source ended`.
* Live: carry the sensor around the room.

Expected result: in replay the scan follows the moving sensor while walls and stairs stay in place. Live, the log shows `mid360 configured and streaming` and `world/global_map` grows without smearing or drifting.

## 8.6 Go2 path following controllers

Needs a Go2 with 5 m of clear floor ahead of it.

```bash
uv run dimos run unitree-go2-rpp-benchmark --out-dir /tmp/rpp-benchmark
uv run python -m dimos.control.benchmarking.score /tmp/rpp-benchmark
uv run dimos run unitree-go2-hol-path-controller
```

* Benchmark: focus the Keyboard Teleop window, aim the robot at the clear floor with WASD and press ENTER. The robot walks a 5 m straight line and stops. Backspace ends the session.
* Holonomic controller: click a point in the Rerun 3D view. The robot walks to it and stops.

Expected result: each benchmark run logs `goal+stop (arrived=True, ...)`, the scorer writes `go2_benchmark_scores.json` and two plots into `/tmp/rpp-benchmark`, and the holonomic controller reaches the clicked point.

## 8.7 Habitat simulation

Needs Linux x86_64 with an NVIDIA driver that provides EGL, and nix with flakes. See [Habitat](../../docs/capabilities/navigation/habitat.md).

```bash
# One-time build, 5 to 10 minutes and about 3.5 GB.
(cd dimos/simulation/habitat/nix && nix develop path:. -c ./install.sh)
uv run dimos run habitat-nav
```

* Drive with WASD in Rerun. The scan and the voxel map grow.
* Click on the blue planner surface to set a goal.

Expected result: the camera panels show a house interior, and after the click the robot drives along a path to the clicked point.

## 8.8 dimSLAM visual odometry

Needs a RealSense D455 and an NVIDIA GPU, or `--use-gpu false` on a machine without one.

```bash
uv run dimos run demo-dim-slam-realsense
```

* Walk a loop around the room with the camera and come back to the start.

Expected result: the infrared feed shows in the cameras view, and `world/odom_hist` traces the walk and ends near the start.
