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
those transports. Legacy stream payloads must be dimOS LCM message types;
generated messages are recorded as CDR (see [CDR recordings](#cdr-recordings)).

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

### CDR recordings

Generated messages are recorded through the same Rust engine. The recorder
stores the original CDR bytes and the message's ROS 2 schema (including nested
definitions) in MCAP, so the file is self-describing:

- channel encoding is `cdr`;
- the schema is the ROS 2 definition, for example
  `geometry_msgs/msg/PoseStamped` with `ros2msg` encoding;
- each payload is an XCDR1 encapsulation (`00 01 00 00` for little-endian);
- observation timestamps use reception time; `header.stamp` stays in the
  payload.

Read a CDR recording with `CdrMcapStore`:

```python
from dimos.memory.store.cdr_mcap import CdrMcapStore

with CdrMcapStore(path="recordings/<run-id>/memory.mcap") as store:
    print(store.list_streams())
    for observation in store.stream("pose"):
        print(observation.ts, observation.data)
```

Replay republishes the recorded streams with their recorded type names:

```bash
dimos --replay-db recordings/<run-id>/memory.mcap run replay
```

Use `--zenoh-domain` to isolate the replay from other local processes. Legacy
LCM recordings stay readable through the legacy `McapStore`; the CDR store is
additive and selected by the file's channel encoding. The complete runnable
walkthrough is [`examples/cdr-recording`](/examples/cdr-recording/README.md);
API details are in [CDR messages](/docs/usage/cdr.md).

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
- The Python recorder automatically subscribes to new streams when modules are
  added to a recording run through `Dimos.connect().run(...)`. The same topic
  filters apply, and messages go into the run's existing `memory.db`.
- The Python recorder uses one writer thread. Its queue holds 1000 messages, then
  drops and warns. The Rust recorder uses its existing native encoding pool and
  ordered writer pipeline.
- We also still have explicit recorder modules (`unitree-go2-memory`, `unitree-go2-mid360-record`, `unitree-g1-record`) that are unaffected and still record their own streams. These will be deprecated shortly.
