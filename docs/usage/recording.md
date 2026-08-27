# Recording

`--record` writes every stream a blueprint publishes to one SQLite memory store, the way `rosbag record` captures a bus. Nothing to wire: each stream is tapped on the transport that carries it.

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

No spaces, no braces. A pattern that matches no stream is an error at startup, listing the stream names the blueprint has.

Streams whose type is not a dimOS message (`Any`, `dict`) are not recorded.

## Inspecting and replaying

```bash
dimos mem summary recordings/<run-id>/memory.db
dimos --replay --replay-db recordings/<run-id>/memory.db run unitree-go2
```

`--replay` swaps the robot connection for the recording; it needs `lidar`, `odom`, and `color_image`, so record all streams (the default) if you intend to replay. Poses are not stored per frame; `tf` is recorded like any other stream and `dimos map pose-fill` derives poses from it.

## Behaviour

- Off unless `--record`; never active under `--replay`.
- One writer thread; transport callbacks only enqueue. Queue holds 1000 messages, then drops and warns.
- Explicit recorder modules (`unitree-go2-memory`, `unitree-go2-mid360-record`, `unitree-g1-record`) are unaffected and still record their own streams.
