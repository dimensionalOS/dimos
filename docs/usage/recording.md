# Recording

`--record` writes every stream a blueprint publishes to one SQLite memory store. Each stream is tapped on the transport that carries it.

```bash
dimos --record --simulation run unitree-go2
dimos --record --robot-ip 192.168.123.161 run unitree-go2
```

The file lands at `recordings/<run-id>/memory.db` under the checkout (`~/.local/state/dimos/recordings/` for an installed package). `<run-id>` is the same `YYYYMMDD-HHMMSS-<blueprint>` as the `logs/` directory of that run.

## Choosing streams

`--record-topics` takes comma-separated globs on the stream name (the blueprint name, e.g. `lidar`, not `/lidar`). Default `*`.

```bash
dimos --record --record-topics color_image run unitree-go2
dimos --record --record-topics lidar,odom,tf run unitree-go2
dimos --record --record-topics 'global_*' run unitree-go2
```

A pattern that matches no stream throws an error at startup, listing the valid stream names of the given blueprint.

Streams whose type is not a dimOS message (`Any`, `dict`) are not recorded.

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
- One writer thread; transport callbacks only enqueue. Queue holds 1000 messages, then drops and warns.
- We also still have explicit recorder modules (`unitree-go2-memory`, `unitree-go2-mid360-record`, `unitree-g1-record`) that are unaffected and still record their own streams. These will be deprecated shortly.
