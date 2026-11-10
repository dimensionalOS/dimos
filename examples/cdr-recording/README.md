# Record a standard ROS 2 message

**WIP:** this example tests the CDR recording path. Business blueprints have not
been migrated. No robot, ROS installation, or API key is required.

The example publishes `geometry_msgs/msg/PoseStamped` at 10 Hz over Zenoh. Its
pose moves around a one-metre circle in the `map` frame. The normal DimOS CLI
starts the Rust recorder, which stores the original CDR bytes in MCAP. Python
does not write the recording.

## Install from this PR

Use Linux, Python 3.12, [uv](https://docs.astral.sh/uv/getting-started/installation/),
and a current stable [Rust toolchain](https://rustup.rs/). The native build needs
a C/C++ compiler, CMake, pkg-config, and NASM. On Ubuntu:

```bash
sudo apt-get install build-essential cmake pkg-config nasm libturbojpeg0 libgl1 libglib2.0-0 git-lfs
git lfs install
git clone --branch cc/feat/cdr-unit-recording https://github.com/dimensionalOS/dimos.git dimos-cdr-review
cd dimos-cdr-review
```

If you already checked out this PR, start at its repository root instead. All
commands below run from that root; no earlier PR checkout is needed.

```bash
uv venv .venv-cdr-recording --python 3.12
DIMOS_ALLOW_MISSING_COCKPIT=1 uv pip install --python .venv-cdr-recording/bin/python -e . -e examples/cdr-recording
source .venv-cdr-recording/bin/activate
cargo build --locked --manifest-path dimos/experimental/memory/rust/Cargo.toml --target-dir build/cdr-recorder -j2
```

The install generates the built-in messages. `DIMOS_ALLOW_MISSING_COCKPIT=1`
allows this headless example to install without a built web frontend. The small
example package registers a blueprint with the existing `dimos.blueprints` entry
point mechanism.

## Record

```bash
dimos --transport zenoh --viewer none --n-workers 1 \
  --record mcap --record-engine rust --record-topics pose \
  --record-executable "$PWD/build/cdr-recorder/debug/dimos-memory-recorder" \
  run dimos-cdr-recording-example.pose
```

If DimOS offers to raise the shared-memory limit, answer **N**. This small
example works without that system change.

Wait for `Experimental Rust recorder ready`. Let the example run for about
15 seconds, then press **Ctrl+C once**. Wait for shutdown to finish. The
`Recording saved` log prints the full `artifact_path` to `memory.mcap`.

The recorder binary comes from this checkout. `--record-executable` selects it
without invoking the default Nix build. Zenoh uses the normal local discovery
settings; this example does not connect to robot hardware.

## Inspect in Foxglove

1. Open the printed `memory.mcap` file in Foxglove.
2. Add a **Raw Messages** panel and select the `pose` topic.
3. Confirm the schema is `geometry_msgs/msg/PoseStamped`, `header.frame_id` is
   `map`, and `pose.position.x` and `pose.position.y` change during playback.
4. Add a **3D** panel. Set the display frame to `map` and enable `pose`.
5. Play the recording. The pose arrow moves around a circle. No TF topic is
   required because the pose and display frame are the same.

The MCAP contains a `ros2msg` schema with the PoseStamped definition and all its
nested definitions. Its channel uses `cdr` encoding. Foxglove reads that schema
and decodes the recorded bytes; it does not need the DimOS Python package.

Recording times use reception time. The original `header.stamp` remains in the
payload. See [Foxglove's supported pose types](https://docs.foxglove.dev/docs/visualization/panels/3d#pose).

## Read with Python

Replace the path below with the printed `artifact_path`:

```python
from dimos.memory.store.cdr_mcap import CdrMcapStore

with CdrMcapStore(path="/path/from/log/memory.mcap") as store:
    print(store.list_streams())  # ['pose']
    observation = next(iter(store.stream("pose")))
    print(observation.ts, observation.data)
```
