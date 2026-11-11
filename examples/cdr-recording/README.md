# Record a standard ROS 2 message

This example records a `geometry_msgs/msg/PoseStamped` with the normal DimOS
CLI and the Rust recorder, then reads the MCAP back in Python, Foxglove or
Rerun. No robot, ROS installation or API key is required.

The blueprint publishes a pose moving around a one-metre circle in the `map`
frame at 10 Hz over Zenoh. The CLI starts the native recorder, which stores the
original CDR bytes together with the ROS 2 schema in MCAP. Python does not
write the recording.

For the support API behind this example (encoding, transport selection,
external schemas, viewer paths) see [CDR messages](/docs/usage/cdr.md).

## Requirements

Linux, Python 3.12, [uv](https://docs.astral.sh/uv/getting-started/installation/),
and a current stable [Rust toolchain](https://rustup.rs/). The native build also
needs a C/C++ compiler, CMake, pkg-config and NASM. On Ubuntu:

```bash
sudo apt-get install build-essential cmake pkg-config nasm libturbojpeg0 libgl1 libglib2.0-0 git-lfs
git lfs install
```

## Install

Clone the repository and check out the branch or commit you want to test (for
a PR, the head branch named in the PR). All commands below run from the
repository root.

```bash
git clone https://github.com/dimensionalOS/dimos.git
cd dimos
```

Create an isolated environment and install this checkout plus the example
package:

```bash
uv venv .venv-cdr-recording --python 3.12
DIMOS_ALLOW_MISSING_COCKPIT=1 uv pip install --python .venv-cdr-recording/bin/python -e . -e examples/cdr-recording
```

`DIMOS_ALLOW_MISSING_COCKPIT=1` allows a headless install without a prepared
web frontend. The example package registers its blueprint through the normal
`dimos.blueprints` entry point. Build the recorder binary from this checkout:

```bash
cargo build --locked \
  --manifest-path dimos/experimental/memory/rust/Cargo.toml \
  --target-dir build/cdr-recorder -j2
```

Expected output ends with `Finished` and leaves
`build/cdr-recorder/debug/dimos-memory-recorder`.

## Record

```bash
.venv-cdr-recording/bin/dimos --transport zenoh --viewer none --n-workers 1 \
  --record mcap --record-engine rust --record-topics pose \
  --record-executable "$PWD/build/cdr-recorder/debug/dimos-memory-recorder" \
  run dimos-cdr-recording-example.pose
```

If DimOS offers to raise the shared-memory limit, answer **N**; this example
works without that system change. The startup log contains:

```text
memory recorder ready ... streams=1
Experimental Rust recorder ready artifact_path=.../recordings/<timestamp>-<run-id>-dimos-cdr-recording-example-pose/memory.mcap pid=... streams=1
```

## Stop

Let it run for about 15 seconds, then press **Ctrl+C once**. Wait for the clean
shutdown. The final recorder line is the evidence that every received message
was written:

```text
memory recorder flushed ... encode_errors=0 received=<N> target=dimos_memory_recorder written=<N>
```

With `viewer=none` there is no window to close. The full MCAP path is the
`artifact_path` printed above.

## Inspect the recording

Open the printed `memory.mcap` in [Foxglove](https://foxglove.dev/):

1. Add a **Raw Messages** panel and select the `pose` topic.
2. Confirm the schema is `geometry_msgs/msg/PoseStamped` and
   `header.frame_id` is `map`; `pose.position.x`/`y` change over time.
3. Add a **3D** panel, set the display frame to `map`, enable `pose`, and play.
   The pose arrow moves around a circle. No TF topic is needed because the pose
   and display frame are the same.

The MCAP channel uses `cdr` encoding and carries the full `ros2msg` schema
definition (including nested types), so Foxglove or Rerun can decode the bytes
without the DimOS Python package. Recording timestamps are reception time; the
original `header.stamp` stays inside the payload. See
[Foxglove's supported pose types](https://docs.foxglove.dev/docs/visualization/panels/3d#pose).

Read it with Python (replace the path with the printed one):

```python
from dimos.memory.store.cdr_mcap import CdrMcapStore

with CdrMcapStore(path="/path/from/log/memory.mcap") as store:
    print(store.list_streams())              # ['pose']
    observation = next(iter(store.stream("pose")))
    print(observation.ts, observation.data)  # frame_id='map', moving x/y
```

## Record and view live

The `pose-view` blueprint composes the same `MovingPose` sample with the
standard Rerun viewer bridge (`vis_module(global_config.viewer)`), so one
process records with the Rust recorder and renders the stream at the same
time. `--viewer rerun` adds the bridge; the environment keeps the viewer
headless and saves the rendered stream next to the MCAP:

```bash
RERUNBRIDGEMODULE__RERUN_OPEN=none RERUNBRIDGEMODULE__RERUN_SAVE=true \
RERUNBRIDGEMODULE__HOST=127.0.0.1 \
.venv-cdr-recording/bin/dimos --transport zenoh --viewer rerun --n-workers 1 \
  --record mcap --record-engine rust --record-topics pose \
  --record-executable "$PWD/build/cdr-recorder/debug/dimos-memory-recorder" \
  run dimos-cdr-recording-example.pose-view
```

Stop with **Ctrl+C** as above. The run directory now contains both
`memory.mcap` (the Rust recording) and `rerun.rrd` (what the viewer rendered):

```text
Rerun bridge starting
Saving the Rerun stream path=.../recordings/<timestamp>-...-pose-view/rerun.rrd
memory recorder flushed ... encode_errors=0 received=<N> written=<N>
```

## Replay and view

Replay republishes every recorded stream over the same transport with the
recorded type and schema. Adding `--viewer rerun` connects the replayed stream
to the same Rerun bridge, so the recorded data renders while it replays:

```bash
RERUNBRIDGEMODULE__RERUN_OPEN=none RERUNBRIDGEMODULE__RERUN_SAVE=true \
RERUNBRIDGEMODULE__HOST=127.0.0.1 \
.venv-cdr-recording/bin/dimos --transport zenoh --viewer rerun \
  --zenoh-domain cdr-replay --replay-db "$PWD/recordings/<run>/memory.mcap" \
  run replay
```

Expected log:

```text
Replaying pose -> Out pose[geometry_msgs__msg__PoseStamped] @ Replay via ZenohTransport(dimos-domains/cdr-replay/pose)
Rerun bridge starting
Saving the Rerun stream path=.../recordings/<timestamp>-...-replay/rerun.rrd
```

Stop it with **Ctrl+C**. Use an isolated `--zenoh-domain` when replaying next
to other processes.

## Watch the rendered stream

Serve either saved `rerun.rrd` with the Rerun CLI from the same environment
(the flags keep it on loopback only):

```bash
.venv-cdr-recording/bin/rerun --serve-web --bind 127.0.0.1 \
  --port 9191 --web-viewer-port 9190 "$PWD/recordings/<run>/rerun.rrd"
```

Open
`http://127.0.0.1:9190/?url=rerun%2Bhttp%3A%2F%2F127.0.0.1%3A9191%2Fproxy` in a
browser. The 3D view shows `/world/pose` moving around the one-metre circle in
the `map` frame — the same values the Rust recorder wrote (check with
`rerun rrd print --entity /world/pose <rrd>`). If the machine has a display,
drop `RERUNBRIDGEMODULE__RERUN_OPEN=none` and the bridge opens a native viewer
window instead.

## Repeat

Record again with the same command; every run gets a new timestamped directory
under `recordings/`. Replay the new file the same way. Nothing outside
`recordings/` and `build/` is modified.

## Renderer demos (separate producers)

The commands above are the connected workflow: the CLI sample, its recording
and its replay all render through the standard Rerun bridge. The support layer
also ships two standalone renderer demos that publish their **own** generated
messages (16 topics / browser fixture) — they exercise the viewer paths without
the recorder and are not the workflow above (see
[CDR messages](/docs/usage/cdr.md) and [Bridge](/docs/web/bridge.md)):

```bash
# Rerun bridge, writes /tmp/cdr-demo.rrd + .mcap and shows 16 generated topics
.venv-cdr-recording/bin/python -I examples/message-viewer/demo_cdr_rerun.py /tmp/cdr-demo.rrd
```

For the browser path, start the cockpit relay and the Deno fixture in two
terminals:

```bash
.venv-cdr-recording/bin/python -I examples/message-viewer/demo_cdr_browser.py --port 8899
cd web/sdk && deno task fixture --host 127.0.0.1
```

Open the URL printed by the demo
(`http://127.0.0.1:5174/demo_cdr.html?live=1&relay=...`). The page should report
`Live relay: connected. Received 8/8 channels.` and show decoded pose, image and
custom values. These demos publish their own generated messages; they prove the
viewer paths only — the recorder workflow is the one above.

## Troubleshooting

- `cargo build --locked` fails on a lockfile mismatch: build from the same
  commit as the Python checkout; do not mix a copied `Cargo.lock`.
- `record-executable` not found: use the absolute path printed by
  `cargo build` (`$PWD/build/cdr-recorder/debug/dimos-memory-recorder`).
- `received=0`: the blueprint's topic name must match `--record-topics`; this
  example publishes the `pose` stream.
- No messages in Foxglove: keep the `map` display frame and confirm the channel
  encoding is `cdr` with a `ros2msg` schema in the MCAP metadata.
- Port 5174 already in use: stop the other Deno fixture; the relay port is
  selectable with `--port`.

## Files

- `cdr_recording_example.py` — the `MovingPose` module, the
  `dimos-cdr-recording-example.pose` blueprint and the `pose-view` composition
  with the standard viewer.
- `pyproject.toml` — example package and `dimos.blueprints` entry point.
- `../../dimos/experimental/memory/rust` — the Rust recorder crate.
- `../../dimos/memory/store/cdr_mcap.py` — Python `CdrMcapStore` reader.
