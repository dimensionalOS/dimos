# Pose-graph SLAM benchmark on KITTI-360

Generic loop-closure benchmark. Drop in any pose-graph SLAM module that
exposes the standard interface and the runner will replay a KITTI-360
sequence at it, watch its loop-closure output, and score precision /
recall / F1 against KITTI's ground-truth pose trajectory.

The module under test never sees KITTI — it only sees streams. The
runner provides two helper modules:

| Module                       | Role                                                      |
|------------------------------|-----------------------------------------------------------|
| `Kitti360PlaybackModule`     | Publishes `registered_scan` + `odometry` from disk        |
| Your pose-graph SLAM module  | Consumes those, publishes `pose_graph` + `loop_closure_event` |
| `PoseGraphScoringModule`     | Subscribes to the outputs, accumulates metrics            |

`autoconnect` wires the three together by stream name.

## Required interface for the module under test

```python
class YourPoseGraphModule(Module):
    registered_scan: In[PointCloud2]
    odometry: In[Odometry]

    pose_graph: Out[Graph3D]             # nodes (keyframes) + edges (odom & loop-closure)
    loop_closure_event: Out[GraphDelta3D]  # one message per loop-closure update
```

Edge convention on `pose_graph`: loop-closure edges have
``metadata_id == 1`` (odometry edges use ``0``). Each node carries the
keyframe's *creation* timestamp in ``pose.ts``, and edges reference
nodes by ``id``; the scorer looks up ``edge.start_id`` and
``edge.end_id`` against the node table to recover endpoint frame_ids.

## Dataset

Download from <https://www.cvlibs.net/datasets/kitti360> (Test SLAM 3D
split is enough). Expected layout:

```
<kitti360-root>/
├── calibration/
├── data_3d_raw/
│   └── 2013_05_28_drive_<NNNN>_sync/velodyne_points/data/*.bin
└── data_poses/
    └── 2013_05_28_drive_<NNNN>_sync/poses.txt
```

Sequence IDs map onto the drive numbers: `2 → drive_0002`, `4 → drive_0004`,
`8 → drive_0008`, etc.

## Quickstart

```python
from pathlib import Path
from dimos.navigation.nav_stack.benchmarks.pose_graph_kitti360.runner import (
    run_benchmark,
)
from dimos.navigation.nav_stack.modules.pgo.pgo_module import PgoModule  # your module

results = run_benchmark(
    module_under_test=PgoModule.blueprint(),
    kitti360_root=Path("~/datasets/kitti360").expanduser(),
    sequence_id=2,
    max_scans=None,         # None = full sequence (~3k frames for seq 2)
    publish_interval_sec=0.1,
)

print(results)
# {
#   "true_positive": ..., "false_positive": ..., "false_negative": ...,
#   "precision": ..., "recall": ..., "f1": ...,
#   "detected_loop_edges": ..., "loop_closure_events": ...,
#   "wallclock_seconds": ..., "sequence_id": 2,
# }
```

## Ground-truth definition

A loop pair `(i, j)` counts as ground truth if:

* frame gap `|i − j| ≥ DEFAULT_MIN_FRAME_GAP` (default 50), AND
* lidar-position distance ≤ `DEFAULT_MAX_LOOP_DISTANCE_M` (default 4.0 m).

Tune via `min_frame_gap=` and `max_loop_distance_m=` on `run_benchmark`.

A detected edge `(i, j)` is a **true positive** if `j` is in the
ground-truth valid-loop set for `i` (or vice-versa). Otherwise it's a
false positive. Ground-truth queries with no detection in their valid
set become false negatives.

## Files

| File | What it does |
|------|--------------|
| `runner.py`              | `run_benchmark()` — builds the blueprint, polls playback, returns scores |
| `playback.py`            | `Kitti360PlaybackModule` — streams scan + odom messages from disk        |
| `scoring.py`             | `PoseGraphScoringModule`, `LoopMetrics` — accumulates TP/FP/FN           |
| `kitti360_loader.py`     | `load_kitti360_sequence()` — reads velodyne `.bin` + `poses.txt`         |
| `loop_groundtruth.py`    | `compute_loop_groundtruth()` + thresholds                                |

## Tips

- Start with `max_scans=200` for a smoke test; you should see playback
  hit ~95% and a couple of GT pairs before paying for the full 3000-scan
  run (~2.5 min wall on a Mac).
- Recall is bounded by your module's loop-search radius. KITTI ground
  truth uses 4 m; if your module searches a 1 m radius, recall floors
  near zero by construction even on a perfect descriptor.
- The scorer maps edge endpoints back to frame_ids via timestamps. If
  your module rewrites pose timestamps after iSAM2 optimization, keep
  the **creation** timestamp on the `PoseStamped` header so the lookup
  still works.
