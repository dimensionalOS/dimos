# Pubsub benchmarks

These commands measure transport performance. They run directly with Python.
The adjacent `test_*.py` files check correctness and run in the normal test suite.

## Throughput

Measure message rate, bandwidth, CPU usage, receive drain time, and loss:

```sh skip
uv run python -m dimos.protocol.pubsub.benchmarks.throughput
```

List the backends available in the current environment, then select specific ones:

```sh skip
uv run python -m dimos.protocol.pubsub.benchmarks.throughput --list
uv run python -m dimos.protocol.pubsub.benchmarks.throughput \
    --transport lcm --transport zenoh --sizes 1024 65536 --duration 1
```

ROS and DDS cases appear when their dependencies are installed. The Cloudflare case
also needs `CF_TELEOP_APP_ID` and `CF_TELEOP_APP_SECRET` and must be selected with
`--transport webrtc_cloudflare`. The default run uses local backends.

Use `--max-messages` to cap each case and `--receive-timeout` to allow more time for
messages to arrive after publishing stops. `DIMOS_BENCH_RECEIVE_TIMEOUT_S` sets the
default receive timeout. Message sizes are approximate payload sizes before encoding.
The reported receive drain time is not per-message latency. CPU sampling covers the
benchmark process, not external daemons or kernel work.

## Image round trips

Compare raw images and JPEG over LCM and Zenoh on localhost:

```sh skip
uv run python -m dimos.protocol.pubsub.benchmarks.image_roundtrip
```

Use `--transport lcm` or `--transport zenoh` to select one backend. `--frames`,
`--width`, and `--height` control the workload. Latency includes encoding and decoding.
JPEG requires the native TurboJPEG library. Large raw frames can exceed UDP buffers
and appear as lost messages.

## Blueprint replay

Run a blueprint against recorded data and record frame arrival times, CPU usage, memory,
and loopback traffic:

```sh skip
uv run python -m dimos.protocol.pubsub.benchmarks.replay \
    --blueprint unitree-go2 --mode raw --duration 60 --out /tmp/bench/go2_raw
uv run python -m dimos.protocol.pubsub.benchmarks.replay \
    --blueprint unitree-go2 --mode codec --sinks 4 --work-ms 20 \
    --duration 60 --out /tmp/bench/go2_codec
```

Both modes use LCM and force replay mode. The blueprint must expose `color_image`.
Use a fresh output directory for each run. Each directory contains `meta.json`,
`host.jsonl`, per-consumer `sink*.jsonl` files, and a `done` marker on completion.
Recordings and any dependencies required by the selected blueprint must be available.
`--quality` controls JPEG quality in codec mode.

Frame timestamps retain their original recording time. Do not subtract them from wall
clock receive times to estimate transport latency. Use the image round-trip command
for latency measurements.
