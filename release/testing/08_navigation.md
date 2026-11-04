# 8. Navigation, mapping and localization

Most checks run in MuJoCo or on replay data from LFS. Checks with a Livox Mid-360 need the lidar on the LAN. Native modules build with nix on first use, and the Cachix cache in [native modules](../../docs/usage/native_modules.md) makes that fast. See the [navigation docs](../../docs/capabilities/navigation/index.md).

## 8.1 Go2 3D navigation in MuJoCo

```bash
# The first start builds the native navigation modules and pulls the go2_sim model.
uv run dimos --simulation run unitree-go2
```

* Rerun shows the office and a voxel map that grows from the simulated lidar.
* Click a floor point a few meters away. A path appears and the robot walks there.

Expected result: the legged robot reaches the clicked goal without crossing walls, and no module crashes.

## 8.2 Go2 premap and relocalization

Needs the `mapping` extra. The first run pulls the `go2_hongkong_office` recording.

```bash
uv run dimos map global go2_hongkong_office --pgo --export --export-pc2
uv run dimos run unitree-go2 --replay.db go2_hongkong_office --localmaprelocalization.map-file go2_hongkong_office
```

* `map global` opens Rerun with `world/pgo_map/pointcloud` and red lines in `world/pgo_map/pgo/loop_closures`. It writes `go2_hongkong_office.map.db` (the keyframe premap) and `go2_hongkong_office.pc2.lcm` (the point-cloud export) to the current directory.
* The premap replay logs `relocalize fix` followed by `placed, no further attempts`.

Expected result: the premap exports and relocalization finds a fix, without tracebacks.

## 8.3 Mid-360 relocalization in replay

The first run pulls the `go2_mid360_sf_office_outdoors_2026-05-29` recording and premap. Allow about 7 GB of free disk during the download and extraction. If the Rerun viewer closes while the data is downloading, rerun the command after the data is cached to inspect the map.

```bash
uv run dimos run relocalize-mid360 --map-file go2_mid360_sf_office_outdoors_2026-05-29
```

* `world/loaded_map` appears in Rerun once a fix lands and lies on top of the live map.

Expected result: the blueprint logs `relocalize fix` followed by `placed, no further attempts`, without tracebacks.

## 8.4 Mid-360 driver and Point-LIO

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

## 8.5 Habitat simulation

Needs Linux x86_64 with an NVIDIA driver that provides EGL, and nix with flakes. See [Habitat](../../docs/capabilities/navigation/habitat.md).

```bash
# One-time build, 5 to 10 minutes and about 3.5 GB.
(cd dimos/simulation/habitat/nix && nix develop path:. -c ./install.sh)
uv run dimos run habitat-nav
```

* Drive with WASD in Rerun. The scan and the voxel map grow.
* Click on the blue planner surface to set a goal.

Expected result: the camera panels show a house interior, and after the click the robot drives along a path to the clicked point.
