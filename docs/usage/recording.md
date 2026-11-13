# Recording

`--record` writes the selected streams from a blueprint to one recording artifact.
The stable default remains the Python SQLite recorder. The Rust engine is an
explicit experimental option while it is evaluated.

```bash
dimos --record --simulation run unitree-go2
dimos --record --robot-ip 192.168.123.161 run unitree-go2
```

Bare `--record` means `--record sqlite`. SQLite lands at
`recordings/<run-id>/memory.db`; MCAP lands at
`recordings/<run-id>/memory.mcap`. The root is under the checkout, or
`~/.local/state/dimos/recordings/` for an installed package. `<run-id>` is the
same `YYYYMMDD-HHMMSS-<blueprint>` used by the run's `logs/` directory.

## Experimental Rust engine

Select the native engine explicitly:

```bash
dimos --record sqlite --record-engine rust run unitree-go2
dimos --record mcap --record-engine rust run unitree-go2
dimos --record mcap --record-engine rust --record-encoding-threads 8 run unitree-go2
```

`--record-encoding-threads` defaults to `4` and is valid only for the Rust
engine. Python remains the default because the native recorder is experimental;
MCAP recording currently requires the Rust engine.

The Rust engine records exact `LCMTransport` and `ZenohTransport` streams. It
rejects DDS, ROS, WebRTC, untyped, JPEG-transport, mixed LCM/Zenoh, and
other specialized transports before creating an artifact. Narrow
`--record-topics` or use the Python engine when a selection contains one of
those transports. Payloads must also be dimOS LCM message types.

The engine automatically prepares `dimos-memory-recorder` through the shared
[native source workflow](/docs/usage/native_modules.md#native-sources-in-pip-installations).
This works from pip installations and editable checkouts; a cache miss builds
from source. Sources come from the shared `get_project_root()` checkout (initially `main`).

Selected LCM streams must use one explicit connection URL. The recorder uses
that URL even when it differs from `LCM_DEFAULT_URL`. Conflicting URLs and
externally supplied LCM connections are rejected.

After preparation, the native process must report ready within 10 seconds.
Preparation, configuration, and subscription failures stop startup. If it exits unexpectedly after startup,
the error is logged and the rest of `dimos run` continues. Normal shutdown sends
SIGTERM and lets the existing native module runtime flush the artifact. There is
no automatic fallback to Python.

## Choosing streams

`--record-topics` takes comma-separated globs on the stream name (the blueprint name, e.g. `lidar`, not `/lidar`). Default `*`.

```bash
dimos --record --record-topics color_image run unitree-go2
dimos --record --record-topics lidar,odom,tf run unitree-go2
dimos --record --record-topics 'global_*' run unitree-go2
```

A pattern that matches no stream throws an error at startup, listing the valid stream names of the given blueprint.

Streams whose type is not a dimOS message (`Any`, `dict`) are not recorded. If
none of the selected streams is recordable, startup fails.

## Inspecting and replaying

View contents of the memory by stream:
```console
$ dimos mem summary recordings/<run-id>/memory.db

┏━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━┳━━━━━━━┳━━━━━━┳━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━┳━━━━━━━━━━━━┓
┃ Stream          ┃ Type                      ┃ Items ┃ Poses ┃   Hz ┃ Start (UTC)         ┃ Duration ┃       Size ┃
┡━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━╇━━━━━━━╇━━━━━━╇━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━╇━━━━━━━━━━━━┩
│ lidar           │ sensor_msgs.PointCloud2   │   146 │     - │  1.9 │ 2026-10-22 15:28:34 │    75.9s │  15.10 MiB │
│ color_image     │ sensor_msgs.Image         │ 1,356 │     - │ 18.5 │ 2026-10-22 15:28:36 │    73.4s │  14.93 MiB │
│ global_map      │ sensor_msgs.PointCloud2   │    29 │     - │  0.4 │ 2026-10-22 15:28:39 │    70.4s │   3.54 MiB │
│ tf              │ tf2_msgs.TFMessage        │ 3,583 │     - │ 48.8 │ 2026-10-22 15:28:36 │    73.4s │   1.04 MiB │
│ global_costmap  │ nav_msgs.OccupancyGrid    │    29 │     - │  0.4 │ 2026-10-22 15:28:39 │    70.4s │ 750.88 KiB │
│ odom            │ geometry_msgs.PoseStamped │ 3,583 │     - │ 48.8 │ 2026-10-22 15:28:36 │    73.4s │ 300.92 KiB │
│ camera_info     │ sensor_msgs.CameraInfo    │    76 │     - │  1.0 │ 2026-10-22 15:28:37 │    75.0s │  27.46 KiB │
│ mapper_frame_ms │ std_msgs.Float32          │   146 │     - │  2.0 │ 2026-10-22 15:28:37 │    72.6s │   1.71 KiB │
│ pgo_keyframes   │ geometry_msgs.PoseArray   │     1 │     - │    - │ 2026-10-22 15:28:34 │     0.0s │    90.00 B │
│ goal_request    │ geometry_msgs.PoseStamped │     1 │     - │    - │ 2026-10-22 15:29:50 │     0.0s │    86.00 B │
│ nav_cmd_vel     │ geometry_msgs.Twist       │     1 │     - │    - │ 2026-10-22 15:29:50 │     0.0s │    56.00 B │
│ pgo_loops       │ nav_msgs.LineSegments3D   │     1 │     - │    - │ 2026-10-22 15:28:34 │     0.0s │    34.00 B │
├─────────────────┼───────────────────────────┼───────┼───────┼──────┼─────────────────────┼──────────┼────────────┤
│ total           │                           │ 8,952 │       │      │                     │          │  35.66 MiB │
└─────────────────┴───────────────────────────┴───────┴───────┴──────┴─────────────────────┴──────────┴────────────┘
```
Replay a recording, SQLite or MCAP:
```bash
dimos --replay --replay-db recordings/<run-id>/memory.db run unitree-go2
dimos --replay --replay-db recordings/<run-id>/memory.mcap run unitree-go2
```

An MCAP replay loads each replayed stream into memory at start, so memory use grows with the length of the recording.

`--replay` swaps the robot connection for the recording; it needs `lidar`, `odom`, and `color_image`, so record all streams (the default) if you intend to replay. Poses are not stored per frame; `tf` is recorded like any other stream and `dimos map global` uses it to register clouds.

## Spatial queries

`stream.near(pose, radius)` matches the pose attached to each observation. `--record` attaches none, so `near()` on a fresh recording returns nothing and warns. The `Poses` column of `dimos mem summary` counts the observations that have a pose.

When `near()` follows a transformation, it checks the transformed observations. If the transformed stream is nonempty and has no poses, it warns when that stream ends; a live stream may never end. A transformation that attaches poses can make spatial queries work without changing the recording.

`dimos map pose-fill` writes a copy of a SQLite recording with poses attached to one stream:

```bash
dimos map pose-fill recordings/<run-id>/memory.db --target color_image --pose-source odom
```

- `--target` is the stream that gets the poses (default `lidar`).
- `--pose-source` is a stream of pose messages, such as `PoseStamped` or `Odometry` (default `odom`). Each target observation takes the pose nearest in time.
- The copy lands beside the source as `memory_posed.db`. Run the command on the copy to fill another stream.
- Target observations with no pose within `--tolerance` (default 0.1 s) are left out of the copy.
- MCAP recordings cannot be pose-filled. Record with `--record sqlite` for spatial queries.

## Behavior

- Off unless `--record`; never active under `--replay`.
- The Python recorder automatically subscribes to new streams when modules are
  added to a recording run through `Dimos.connect().run(...)`. The same topic
  filters apply, and messages go into the run's existing `memory.db`.
- The Python recorder uses one writer thread. Its queue holds 1000 messages, then
  drops and warns. The Rust recorder uses its existing native encoding pool and
  ordered writer pipeline.
- We also still have explicit recorder modules (`unitree-go2-memory`, `unitree-go2-mid360-record`, `unitree-g1-record`) that are unaffected and still record their own streams. These will be deprecated shortly.
