# Live Loop Closure

`unitree-go2-pgo` is `unitree-go2` with a mapper that corrects odometry drift while the robot drives. When the Go2 returns to a place it has already seen, pose-graph optimization (PGO) closes the loop and the global map is moved into alignment, so a revisited wall stays one wall.

```bash
dimos --replay --replay-db go2_hongkong_office run unitree-go2-pgo   # no robot needed
dimos --robot-ip <ip> run unitree-go2-pgo
```

## How it works

`PGOVoxelMapper` replaces `VoxelGridMapper` and publishes the same `global_map`.

1. Every lidar frame goes into a voxel grid that remembers, per voxel, which frame wrote it and where.
2. The same frame feeds incremental PGO, which picks keyframes and searches for loop closures with ICP.
3. When a loop closes, the poses of old frames change. Each voxel is moved by the correction for the frame that wrote it, in one pass over the map.

The map is anchored at the robot. The area the robot is in right now stays where raw odometry puts it, and older areas shift to line up with it. Odometry, the `world` frame and the costmap need no changes.

## Configuration

| Field | Default | Effect |
|-------|---------|--------|
| `voxel_size` | `0.05` | Voxel size in meters |
| `emit_every` | `1` | Publish `global_map` every n lidar frames. The blueprint uses `5`. A rebuild always publishes. |
| `rebuild_cooldown_s` | `10.0` | Shortest gap between map rebuilds, in seconds of lidar time |
| `pgo` | `LIVE_PGO` | Keyframe spacing, loop search radius, ICP thresholds. A keyframe is taken every 0.5 m or 45° of rotation (offline PGO uses 10°). |

## Outputs

| Port | Type | Content |
|------|------|---------|
| `global_map` | `PointCloud2` | The loop-closed map, same as `VoxelGridMapper` publishes |
| `pgo_keyframes` | `PoseArray` | Pose graph nodes: optimized keyframe poses in the map frame, in order |
| `pgo_loops` | `LineSegments3D` | Loop edges in the map frame, weighted by ICP score |

The graph outputs draw in Rerun like `dimos map global` does (red keyframe points, white path, red loop edges). They are published on every rebuild, and otherwise at most once per `rebuild_cooldown_s` while new keyframes arrive.

## Cost

Measured on `go2_hongkong_office` (558 s, 405 keyframes, 37 loop closures, 465k voxels):

- A lidar frame costs 11 to 15 ms on average (voxel insert plus PGO). A frame that triggers an ICP can take up to about 0.5 s.
- A map rebuild takes 50 to 250 ms and runs at most once per `rebuild_cooldown_s`.
- The mapper runs on the CPU voxel store. Lidar frames that arrive while it is busy are dropped.

Compared with the offline rebuild (`dimos map global --pgo`) under the same pose graph, the live map is missing under 1% of voxels and has no extra ones. To reproduce:

```bash
uv run python -m dimos.navigation.go2.loop_closure.pgo_map go2_hongkong_office
```

## Limits

- Coordinates of places far from the robot shift when a loop closes. The planner does not yet move a goal or replan when that happens.
- Until a loop is detected on a revisit, new frames carve the map in drifted coordinates. This can delete thin strips at the edge of the new view, which refill when seen again.
- Only the Go2 `unitree-go2` stack is covered. `unitree-go2-nav-3d` uses a different mapper.

For a map you keep between sessions, see [Premap & Relocalization](/docs/capabilities/navigation/relocalization.md).
