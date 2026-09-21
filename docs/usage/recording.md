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
dimos --record mcap --record-engine rust --record-topics color_image,lidar,odom,tf run unitree-go2
dimos --record mcap --record-engine rust --record-encoding-threads 8 --record-topics color_image,lidar,odom,tf run unitree-go2
```

`--record-encoding-threads` defaults to `4` and is valid only for the Rust
engine. Python remains the default because the native recorder is experimental;
MCAP recording currently requires the Rust engine.

The Rust engine records exact `LCMTransport` and `ZenohTransport` streams. It
rejects SHM, DDS, ROS, WebRTC, pickled, JPEG-transport, mixed LCM/Zenoh, and
other specialized transports before creating an artifact. Narrow
`--record-topics` or use the Python engine when a selection contains one of
those transports. Payloads must also be dimOS LCM message types.

The native process must report ready within 10 seconds, so build, configuration,
and subscription failures stop startup. If it exits unexpectedly after startup,
the error is logged and the rest of `dimos run` continues. Normal shutdown sends
SIGTERM and lets the existing native module runtime flush the artifact. There is
no automatic fallback to Python.

## MCAP format and supported messages

The Rust MCAP recorder writes a schema-aware file with profile `dimos`, indexed
Zstd chunks, and complete embedded schemas; no ROS installation or rosbag layout
is required. Install Python reading support with `uv sync --extra recording`.

| DimOS input | MCAP message encoding | Schema encoding / name |
|---|---|---|
| Image (default, including depth) | `cdr` | `ros2msg` / `sensor_msgs/msg/Image` |
| Image with explicit `jpeg` codec | `cdr` | `ros2msg` / `sensor_msgs/msg/CompressedImage` |
| PointCloud2, CameraInfo, Imu, JointState | `cdr` | `ros2msg` / matching `sensor_msgs/msg/*` |
| PoseStamped | `cdr` | `ros2msg` / `geometry_msgs/msg/PoseStamped` |
| Odometry, Path | `cdr` | `ros2msg` / matching `nav_msgs/msg/*` |
| TFMessage | `cdr` | `ros2msg` / `tf2_msgs/msg/TFMessage` |
| LineSegments3D | `json` | `jsonschema` / `dimos.LineSegments3D` |

LCM remains the transport input; Rust decodes that input and serializes CDR or
JSON directly. The recording never stores an LCM envelope. Raw images preserve
pixel data and depth precision. An explicit `stream_codecs={"color_image": "jpeg"}`
on a `RustRecorder` module enables lossy JPEG for 8-bit images; depth-to-JPEG is
rejected. The CLI uses the lossless default.

`publish_time` holds the positive source timestamp (reception time if absent),
while `log_time` holds reception time; TF batches become individually timestamped
TF messages. PointCloud2 fields, data, padding, and endianness are preserved in
the file, as are IMU and odometry covariances and camera calibration.

```bash
dimos --record mcap --record-engine rust --record-topics 'color_image,lidar,odom,camera_info,tf' run unitree-go2
dimos mem summary recordings/<run-id>/memory.mcap
dimos mem rerun recordings/<run-id>/memory.mcap --no-gui
```

`McapRecordingStore` opens these files through the Memory2 API and decodes from
embedded schemas, without passing through LCM or importing classes named by file
metadata. Known schemas become DimOS messages, unknown ROS schemas remain decoded
objects, and unknown JSON schemas yield dictionaries. The DimOS point-cloud view
loads XYZ, packed RGB, intensity, and Livox per-point attributes; other fields
remain available in the MCAP payload. Custom JSON data is inspectable in generic
MCAP tools, but specialized visualization depends on each tool's schema support.

The existing Go2 DDS MCAP reader keeps its robot-specific behavior. Legacy
schema-zero Rust recordings are not accepted by the new recording store, and
MCAP append is unsupported.

## Choosing streams

`--record-topics` takes comma-separated globs on the stream name (the blueprint name, e.g. `lidar`, not `/lidar`). Default `*`.

```bash
dimos --record --record-topics color_image run unitree-go2
dimos --record --record-topics lidar,odom,tf run unitree-go2
dimos --record --record-topics 'global_*' run unitree-go2
```

A pattern that matches no stream throws an error at startup, listing the valid stream names of the given blueprint.

SQLite skips streams whose type is not a DimOS message (`Any`, `dict`), and
fails if none of the selection is recordable. MCAP rejects every selected type
without an explicit mapping before creating or replacing the artifact.

## Inspecting and replaying

View contents of the memory by stream:
```console
$ dimos mem summary recordings/<run-id>/memory.db

┏━━━━━━━━━━━━━━━━┳━━━━━━━━┳━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━┳━━━━━━━━━━━━┓
┃ Stream         ┃  Items ┃    Hz ┃ Start (UTC)         ┃ Duration ┃       Size ┃
┡━━━━━━━━━━━━━━━━╇━━━━━━━━╇━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━╇━━━━━━━━━━━━┩
│ lidar          │     43 │   2.0 │ 2026-08-27 03:39:06 │    21.2s │   5.14 MiB │
│ color_image    │    376 │  17.3 │ 2026-08-27 03:39:06 │    21.6s │   4.26 MiB │
│ global_map     │      9 │   0.4 │ 2026-08-27 03:39:07 │    20.2s │   3.08 MiB │
│ global_costmap │      9 │   0.4 │ 2026-08-27 03:39:07 │    20.2s │ 421.03 KiB │
│ tf             │    981 │  45.2 │ 2026-08-27 03:39:06 │    21.7s │ 292.19 KiB │
│ cmd_vel        │  1,855 │  99.7 │ 2026-08-27 03:39:07 │    18.6s │ 101.45 KiB │
│ tele_cmd_vel   │  1,855 │  98.6 │ 2026-08-27 03:39:07 │    18.8s │ 101.45 KiB │
│ goal           │  1,855 │  98.9 │ 2026-08-27 03:39:07 │    18.8s │  94.20 KiB │
│ way_point      │  1,855 │  98.9 │ 2026-08-27 03:39:07 │    18.8s │  94.20 KiB │
│ odom           │    981 │  45.2 │ 2026-08-27 03:39:06 │    21.7s │  82.39 KiB │
│ stop_movement  │  1,855 │  98.9 │ 2026-08-27 03:39:07 │    18.7s │  16.30 KiB │
│ camera_info    │     24 │     - │ 2026-08-27 03:38:56 │     0.0s │   8.67 KiB │
│ nav_cmd_vel    │      3 │  32.8 │ 2026-08-27 03:39:28 │     0.1s │   168.00 B │
│ goal_request   │      1 │     - │ 2026-08-27 03:39:28 │     0.0s │    86.00 B │
│ path           │      2 │ 289.1 │ 2026-08-27 03:39:28 │     0.0s │    68.00 B │
│ goal_reached   │      1 │     - │ 2026-08-27 03:39:28 │     0.0s │     9.00 B │
├────────────────┼────────┼───────┼─────────────────────┼──────────┼────────────┤
│ total          │ 11,705 │       │                     │          │  13.66 MiB │
└────────────────┴────────┴───────┴─────────────────────┴──────────┴────────────┘
```
Replay memory from DB:
```bash
dimos --replay --replay-db recordings/<run-id>/memory.db run unitree-go2
```

`--replay` swaps the robot connection for the recording; it needs `lidar`, `odom`, and `color_image`, so record all streams (the default) if you intend to replay. Poses are not stored per frame; `tf` is recorded like any other stream and `dimos map global` uses it to register clouds. `dimos map pose-fill` instead derives poses from `odom` by default.

## Behavior

- Off unless `--record`; never active under `--replay`.
- The Python recorder uses one writer thread. Its queue holds 1000 messages, then
  drops and warns. The Rust recorder uses its existing native encoding pool and
  ordered writer pipeline.
- We also still have explicit recorder modules (`unitree-go2-memory`, `unitree-go2-mid360-record`, `unitree-g1-record`) that are unaffected and still record their own streams. These will be deprecated shortly.
