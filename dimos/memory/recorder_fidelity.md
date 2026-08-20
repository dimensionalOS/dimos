# Recorder fidelity test

The fidelity harness checks one invariant: every observation successfully
published by the synthetic source must appear exactly once, in order and with
the same payload, in the completed mem2 recording.

The default profile models one RGB camera, two grayscale cameras, one depth
camera, 400 Hz IMU, 10 Hz PointLIO cloud, 200 Hz odometry, and 20 Hz TF. Image
and point-cloud streams use shared memory by default; small messages use LCM.
The transport can be overridden for an entire run.

Storage codecs are independent from transport. RGB uses lossy JPEG. Grayscale
and depth use lossless `lz4+lcm`. IMU, PointLIO, odometry, and TF use plain LCM
serialization.

The `shm` profile transport uses `pSHMQueueTransport`, sized to 2.5 seconds of
the configured stream rate. Ordinary `pSHMTransport` remains available for
latest-frame live consumers. A reliable-ring overflow is a Recorder failure,
not a silent drop.

```text
source worker               recorder worker                  recording.db
  independent publishers  -> reliable SHM/LCM ingress       exact oracle
  source manifest            bounded per-stream FIFO      -> count/order/data
                              parallel preparation           shared gap windows
                              64-row / 10-ms transactions
```

## Run it

Use 30 seconds for CI characterization and a longer run when investigating a
machine or storage device:

```bash
uv run python -m dimos.memory.tool_recorder_fidelity run \
  dimos/memory/testdata/recorder_production_profile.json \
  --duration 30 --output recorder-fidelity-results

uv run python -m dimos.memory.tool_recorder_fidelity run \
  dimos/memory/testdata/recorder_production_profile.json \
  --duration 300 --transport shm --output recorder-fidelity-results
```

To measure headroom instead of only testing one fixed rate, run the adaptive
capacity search. It doubles the production rates until a trial fails, refines
the passing boundary to the requested resolution, and confirms the winner with
a longer run:

```bash
uv run python -m dimos.memory.tool_recorder_fidelity capacity \
  dimos/memory/testdata/recorder_production_profile.json \
  --trial-duration 15 --confirm-duration 30 \
  --max-scale 4 --resolution 0.125 \
  --output recorder-capacity-results
```

`capacity.json.max_faithful_scale` is the optimization metric. A scale counts
only when the source is valid and every published observation is persisted
exactly once in order with the expected payload.

`--transport` accepts `lcm`, `shm`, or `zenoh`. With no override, each stream
uses the transport in the profile. A run is only interpretable when
`source_valid` is true; this proves that the publisher offered the requested
rates without a one-second source-side gap.

The report localizes loss into two stages:

| Report field | Meaning |
|---|---|
| `missing_before_receive` | transport or subscriber delivery loss |
| `missing_before_persist` | received by Recorder but absent from SQLite |
| `corrupt_sequences` | timestamp matched, decoded payload did not |
| `shared_loss_windows` | overlapping gaps across all data streams |
| `codec/*`, `append/*` | encode and complete append latency distributions |
| `recorder.queues` | accepted/completed counts and live backlog per stream |
| `recorder.writer` | committed rows, transaction size, and commit p99 |

Bandwidth is reported at distinct boundaries so “disk bandwidth” is not
confused with sensor or codec throughput:

| Metric | Boundary |
|---|---|
| `offered_raw_mib_s` | uncompressed bytes represented by published messages |
| `persisted_payload_mib_s` | encoded codec payload committed to streams |
| `sqlite_growth_mib_s` | final database bytes divided by workload duration |
| `process_character_write_mib_s` | bytes passed through Recorder-process write syscalls |
| `process_block_write_mib_s` | Linux `/proc/self/io` bytes submitted by the Recorder process |
| `process_character_write_amplification` | write-syscall bytes per persisted payload byte |
| `process_block_write_amplification` | accounted block-write bytes per persisted payload byte |
| per-stream compression ratio | raw bytes represented by persisted messages divided by their codec bytes |

The active database/WAL/SHM sizes are captured before shutdown and final sizes
after SQLite closes. Write-syscall bytes include SQLite WAL/checkpoint rewrites
and are not the same as physical media traffic. Process block writes can also
include other writes made by the dedicated Recorder process; they are marked
unavailable on platforms without `/proc/self/io` or when the kernel counter is
smaller than the resulting database and therefore demonstrably incomplete.

## Reproduce stalls and isolate hypotheses

The end-to-end modes retain the production recorder path:

```bash
# Pause depth encoding for one second.
uv run python -m dimos.memory.tool_recorder_fidelity run PROFILE \
  --mode encoder-stall --stall-duration 1

# Hold a competing SQLite writer lock for one second.
uv run python -m dimos.memory.tool_recorder_fidelity run PROFILE \
  --mode sqlite-lock --stall-duration 1
```

The storage-only controls change one factor at a time. They are measurements,
not alternate recorder implementations:

```bash
uv run python -m dimos.memory.tool_recorder_fidelity matrix PROFILE \
  --duration 60 --output recorder-fidelity-matrix
```

| Control | Question answered |
|---|---|
| `encoder-control` | Can SQLite sustain already-encoded, production-sized blobs? |
| `split-db-control` | Does sharing one database create writer contention? |
| `batch-small-control` | How much commit work is removed by batching IMU and odometry? |

To derive rates, payload sizes, shapes, and codecs from a real non-sensitive
recording, run `calibrate RECORDING PROFILE`. Review the resulting JSON before
committing it; it contains structural metadata but no recorded payloads.

## CI behavior

The self-hosted suite requires the 30-second controlled encoder-stall workload
to remain fully faithful. Source validity is checked separately, so a bad load
generator cannot be mistaken for a Recorder success or failure.

CI also runs a non-gating 30-second baseline and uploads its database, source
manifest, and JSON report. The artifact makes performance changes observable
without defining machine-specific throughput thresholds prematurely.
