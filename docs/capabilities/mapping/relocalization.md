# Relocalization

This walkthrough shows the pre-map capabilities, including how to record,
export a premap from a mem2 `.db`, and run the robot with relocalization
enabled. You can also navigate to a place the robot hasn't visited during
this run, as long as it's part of the global map.


![relocalize on the live go2 and nav_to a point in the premap](assets/reloc_and_nav_to.webp)


## 1. Record a run

```bash
dimos --robot-ip {YOUR_ROBOT_IP} run unitree-go2-memory
```

If `DIMOS_ROBOT_IP` is set in your environment (or `.env`), you can drop
the `--robot-ip` flag:

```bash
dimos run unitree-go2-memory
```

This writes `recording_go2.db` to the repo root. Run the next command
from the repo root so the bare-name lookup finds this file. In the next
steps `{DB_NAME}` refers to the stem of your recording - `recording_go2`
if you kept the default.

## 2. Export the premap

Convert the recording to a relocalization premap (`.pc2.lcm`):

```bash
# default name from step 1:
dimos export-premap recording_go2

# renamed:
dimos export-premap {DB_NAME}
```

`{DB_NAME}` can be a file name (with or without extension), or a relative / absolute path.

When a bare file name is given, the tool searches in:

1. current working directory
2. `data/` (where LFS files live)

Examples:

```bash
dimos export-premap go2_hongkong_office
dimos export-premap ./go2_hongkong_office.db
dimos export-premap data/go2_hongkong_office.db
dimos export-premap /abs/path/to/scan.db
```

Output defaults to `data/{basename}_twopass_map.pc2.lcm`. Pass `-o
<path>` to save elsewhere. Relative paths resolve against the current
working directory; parent directories are created on demand.

```bash
dimos export-premap recording_go2 -o /tmp/scan.pc2.lcm
dimos export-premap recording_go2 -o ./recording_go2_twopass_map.pc2.lcm
```

Sample log:

```
computing twopass map from /Users/dimos/Desktop/dimos-reloc/recording_go2.db (voxel_size=0.05)...
  Pass 1: 908 frames, 1 keyframes
12:54:32.025[inf][dimos/mapping/voxels.py       ] VoxelGrid using device: CPU:0
wrote /Users/dimos/Desktop/dimos-reloc/data/recording_go2_twopass_map.pc2.lcm
```

## 3. Relocalize against replay

Replay a recording and have the relocalization module localize against
the premap. Use the `unitree-go2-relocalization` blueprint — it's the
standard `unitree-go2` stack plus `RelocalizationModule`:

```bash
dimos --replay --replay-db {DB_NAME} run unitree-go2-relocalization \
  -o relocalizationmodule.map_file={DB_NAME}_twopass_map
```

`{DB_NAME}_twopass_map` is resolved the same way as in section 2: cwd
first, then `data/`.

Sample log:

```
12:58:51.469[inf][imos/mapping/relocalization.py] Relocalization module started: map_file='recording_go2_twopass_map'  loaded_map.frame_id='map'  placeholder TF 'world' -> 'map'  z_offset=20.0
12:58:56.528[war][imos/mapping/relocalization.py] relocalize skipped: n_pts=14198 < MIN_LOCAL_POINTS=20000
12:59:04.777[war][imos/mapping/relocalization.py] relocalize rejected: fitness=0.466 < threshold=0.6 time_cost=5.3s n_pts=20231
12:59:14.880[war][imos/mapping/relocalization.py] relocalize rejected: fitness=0.433 < threshold=0.6 time_cost=8.1s n_pts=37770
12:59:19.877[inf][imos/mapping/relocalization.py] relocalize: fitness=0.657 time_cost=3.0s n_pts=57385 reloc_t=[-0.007, -0.01, -0.102] TF 'world' -> 'map' published_t=[0.007, 0.009, 0.102]
12:59:27.410[inf][imos/mapping/relocalization.py] relocalize: fitness=0.684 time_cost=5.5s n_pts=64703 reloc_t=[0.001, -0.018, -0.171] TF 'world' -> 'map' published_t=[-0.004, 0.015, 0.171]
12:59:34.213[inf][imos/mapping/relocalization.py] relocalize: fitness=0.681 time_cost=4.8s n_pts=76752 reloc_t=[0.002, -0.003, -0.06] TF 'world' -> 'map' published_t=[-0.002, 0.003, 0.06]
```

`relocalize skipped` means the live submap is still warming up (fewer
than `MIN_LOCAL_POINTS` points). `relocalize rejected` means a candidate
was found but its fitness was below the configured threshold (default
`0.6`). If you want to see every candidate accepted regardless of
quality, disable the gate:

```bash
-o relocalizationmodule.fitness_threshold=0.0
```

You can watch the alignment in Rerun. The loaded premap is published on
its own entity — to compare the merged costmap (live scan + premap)
against the live scan alone, click the eye icon next to the loaded-map
entity to toggle it off.

![toggle loaded map](assets/reloc_hide_loaded_map.webp)

With the loaded map hidden you see the partial pointcloud from the
scanning replay plus the full costmap from the merged current scan +
premap.

You can also replay a different recording taken in the same physical
space against the same premap.


## 4. Relocalize on a live robot

Same flags as the replay test, but point at the live robot instead of a
recorded `.db`:

```bash
dimos --robot-ip {YOUR_ROBOT_IP} run unitree-go2-relocalization \
  -o relocalizationmodule.map_file={DB_NAME}_twopass_map
```
