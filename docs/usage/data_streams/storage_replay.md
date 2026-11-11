# Sensor Storage and Replay

Record sensor streams to disk and replay them with original timing. Useful for testing, debugging, and creating reproducible datasets.

`TimedSensorStorage` writes pickle files, which `LegacyPickleStore` reads back. `TimedSensorReplay` reads one stream of a recorded dataset in the memory store (`data/<dataset>.db`).

## Quick Start

### Recording

```python skip
from dimos.utils.testing.replay import TimedSensorStorage

# Create storage (directory in data folder)
storage = TimedSensorStorage("my_recording")

# Save frames from a stream (each frame needs a .ts timestamp)
storage.consume_stream(camera_stream)

# Or save manually
storage.save(frame1, frame2, frame3)
```

### Replaying

```python skip
from dimos.utils.testing.legacy_pickle import LegacyPickleStore

# Load recording
replay = LegacyPickleStore("my_recording")

# Iterate at original speed
for frame in replay.iterate_realtime():
    process(frame)

# Or as an Observable stream
replay.stream(speed=1.0).subscribe(process)
```

## TimedSensorStorage

Stores sensor data with timestamps as pickle files. Each frame is saved as `000.pickle`, `001.pickle`, etc.

```python skip
from dimos.utils.testing.replay import TimedSensorStorage

storage = TimedSensorStorage("lidar_capture")

# Save individual frames (each uses its .ts)
storage.save(lidar_msg)

# Save multiple frames
storage.save(frame1, frame2, frame3)

# Subscribe to a stream
storage.consume_stream(lidar_stream)

# Or pipe through (emits each frame after saving it)
lidar_stream.pipe(storage.pipe_save).subscribe()
```

**Storage location:** Files are saved to the data directory under the given name (or to an absolute path). If the directory already contains pickle files, numbering continues after the last one.

**What gets stored:** Each frame as is. The `autocast` parameter transforms frames when they are read back:

```python skip
# Custom deserialization
replay = LegacyPickleStore(
    "custom_capture",
    autocast=lambda frame: frame.to_dict()
)
```

## TimedSensorReplay

Replays one stream of a dataset in the memory store with timestamp-aware iteration and seeking. Name it `"<dataset>/<stream>"`; the dataset is `data/<dataset>.db`, downloaded on first use like other data.

### Basic Iteration

```python skip
from dimos.utils.testing.replay import TimedSensorReplay

replay = TimedSensorReplay("go2_bigoffice/lidar")

# Iterate all frames (ignores timing)
for frame in replay.iterate():
    process(frame)

# Iterate with timestamps
for ts, frame in replay.iterate_ts():
    print(f"Frame at {ts}: {frame}")

# Iterate with relative timestamps (from start)
start = replay.first_timestamp()
for ts, frame in replay.iterate_ts():
    print(f"At {ts - start:.2f}s: {frame}")
```

### Realtime Playback

```python skip
# Play at original speed
replay.stream().subscribe(process)

# Play at 2x speed
replay.stream(speed=2.0).subscribe(process)

# Play at half speed
replay.stream(speed=0.5).subscribe(process)
```

`LegacyPickleStore` also has `iterate_realtime(speed=...)`, which blocks between frames.

### Seeking and Slicing

```python skip
# Start 10 seconds into the recording
for ts, frame in replay.iterate_ts(seek=10.0):
    process(frame)

# Play only 5 seconds starting at 10s
for ts, frame in replay.iterate_ts(seek=10.0, duration=5.0):
    process(frame)

# Loop forever
for ts, frame in replay.iterate_ts(loop=True):
    process(frame)
```

### Finding Specific Frames

```python skip
# Find a frame within 1s of an absolute timestamp (None if there is none)
frame = replay.find_closest(1704067200.0)

# Find the first frame at or after 30s from the start
frame = replay.find_closest_seek(30.0)

# With tolerance (returns None if no match within 0.1s)
frame = replay.find_closest(timestamp, tolerance=0.1)
```

### Observable Stream

The `.stream()` method returns an Observable that emits frames with original timing:

```python skip
# Stream at original speed
replay.stream(speed=1.0).subscribe(process)

# Stream at 2x with seeking
replay.stream(
    speed=2.0,
    seek=10.0,      # Start 10s in
    duration=30.0,  # Play for 30s
    loop=True       # Loop forever
).subscribe(process)
```

## Usage: Replay Instead of Hardware

The Go2's [`ReplayConnection`](/dimos/robot/unitree/go2/connection.py) stands in for the robot by playing a recorded dataset from the memory store. `GO2Connection` uses it when replay is on, which runs the full perception pipeline against recorded data:

```bash skip
dimos run unitree-go2 --replay --replay-db go2_short
```

## Data Format

Each pickle file of a `TimedSensorStorage` recording contains a tuple `(timestamp, data)`:

- **timestamp**: The frame's `.ts`, a Unix timestamp (float)
- **data**: The sensor data

Files are numbered sequentially: `000.pickle`, `001.pickle`, etc.

Recordings are stored in the `data/` directory. See [Data Loading](/docs/development/large_file_management.md) for how data storage works, including Git LFS handling for large datasets.

## API Reference

### TimedSensorStorage

| Method                       | Description                                       |
|------------------------------|---------------------------------------------------|
| `save(*frames)`              | Save one or more frames, using each frame's `.ts` |
| `pipe_save(observable)`      | Operator for `.pipe()`: saves and re-emits frames |
| `consume_stream(observable)` | Subscribe and save; returns the subscription      |

### TimedSensorReplay

| Method                                                | Description                       |
|-------------------------------------------------------|-----------------------------------|
| `iterate()`                                           | Iterate frames (no timing)        |
| `iterate_ts(seek, duration, from_timestamp, loop)`    | Iterate with absolute timestamps  |
| `stream(speed, seek, duration, from_timestamp, loop)` | Observable with original timing   |
| `find_closest(timestamp, tolerance)`                  | Find frame by absolute timestamp  |
| `find_closest_seek(seconds)`                          | Find frame by relative time       |
| `first()`                                             | Get first frame                   |
| `first_timestamp()`                                   | Get first timestamp               |
| `count()`                                             | Number of frames                  |
| `load_one(index)`                                     | `(timestamp, frame)` at an index  |
