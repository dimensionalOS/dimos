# Sensor Storage and Replay

Record sensor streams to disk and replay them with original timing. Useful for testing, debugging, and creating reproducible datasets.

## Quick Start

### Recording

```python skip
from dimos.utils.testing.replay import TimedSensorStorage

# Create storage (directory in data folder)
storage = TimedSensorStorage("my_recording")

# Save frames from a stream
camera_stream.subscribe(storage.save_one)

# Or save manually
storage.save(frame1, frame2, frame3)
```

### Replaying

```python skip
from dimos.utils.testing.replay import TimedSensorReplay

# Load recording
replay = TimedSensorReplay("my_recording")

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

# Save individual frames
storage.save_one(lidar_msg)  # Returns frame count

# Save multiple frames
storage.save(frame1, frame2, frame3)

# Subscribe to a stream
lidar_stream.subscribe(storage.save_one)

# Or pipe through (emits frame count)
lidar_stream.pipe(
    ops.flat_map(storage.save_stream)
).subscribe()
```

**Storage location:** Files are saved to the data directory under the given name. The directory must not already contain pickle files (prevents accidental overwrites).

**What gets stored:** By default, if a frame has a `.raw_msg` attribute, that's pickled instead of the full object. You can customize with the `autocast` parameter:

```python skip
# Custom serialization
storage = TimedSensorStorage(
    "custom_capture",
    autocast=lambda frame: frame.to_dict()
)
```

## TimedSensorReplay

Replays stored sensor data with timestamp-aware iteration and seeking.

### Basic Iteration

```python skip
from dimos.utils.testing.replay import TimedSensorReplay

replay = TimedSensorReplay("lidar_capture")

# Iterate all frames (ignores timing)
for frame in replay.iterate():
    process(frame)

# Iterate with timestamps
for ts, frame in replay.iterate_ts():
    print(f"Frame at {ts}: {frame}")

# Iterate with relative timestamps (from start)
for relative_ts, frame in replay.iterate_duration():
    print(f"At {relative_ts:.2f}s: {frame}")
```

### Realtime Playback

```python skip
# Play at original speed (blocks between frames)
for frame in replay.iterate_realtime():
    process(frame)

# Play at 2x speed
for frame in replay.iterate_realtime(speed=2.0):
    process(frame)

# Play at half speed
for frame in replay.iterate_realtime(speed=0.5):
    process(frame)
```

### Seeking and Slicing

```python skip
# Start 10 seconds into the recording
for ts, frame in replay.iterate_ts(seek=10.0):
    process(frame)

# Play only 5 seconds starting at 10s
for ts, frame in replay.iterate_ts(seek=10.0, duration=5.0):
    process(frame)

# Loop forever
for frame in replay.iterate(loop=True):
    process(frame)
```

### Finding Specific Frames

```python skip
# Find frame closest to absolute timestamp
frame = replay.find_closest(1704067200.0)

# Find frame closest to relative time (30s from start)
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

## Usage: Stub Connections for Testing

A common pattern is replaying a recording in a robot connection's place, to test without hardware. The Go2 blueprint does this with [`ReplayNative`](/dimos/memory/replay_native/native.py), a Rust module that publishes every stream of a `.db` or `.mcap`. From [`robot/unitree/go2/blueprints/unitree_go2.py`](/dimos/robot/unitree/go2/blueprints/unitree_go2.py#L37):

```python skip
class Replay(ReplayNative):
    """A Go2 recording in the connection's place: idle until ``--replay.db <x.db|x.mcap>``."""

    lidar: Out[PointCloud2]
    odom: Out[PoseStamped]
    color_image: Out[Image]
    camera_info: Out[CameraInfo]
```

This allows running the full navigation pipeline against recorded data:

```bash
dimos run unitree-go2 --replay.db go2_short
```

## Data Format

Each pickle file contains a tuple `(timestamp, data)`:

- **timestamp**: Unix timestamp (float) when the frame was captured
- **data**: The sensor data (or result of `autocast` if provided)

Files are numbered sequentially: `000.pickle`, `001.pickle`, etc.

Recordings are stored in the `data/` directory. See [Data Loading](/docs/development/large_file_management.md) for how data storage works, including Git LFS handling for large datasets.

## API Reference

### TimedSensorStorage

| Method                       | Description                              |
|------------------------------|------------------------------------------|
| `save_one(frame)`            | Save a single frame, returns frame count |
| `save(*frames)`              | Save multiple frames                     |
| `save_stream(observable)`    | Pipe an observable through storage       |
| `consume_stream(observable)` | Subscribe and save without returning     |

### TimedSensorReplay

| Method                                           | Description                           |
|--------------------------------------------------|---------------------------------------|
| `iterate(loop=False)`                            | Iterate frames (no timing)            |
| `iterate_ts(seek, duration, loop)`               | Iterate with absolute timestamps      |
| `iterate_duration(...)`                          | Iterate with relative timestamps      |
| `iterate_realtime(speed, ...)`                   | Iterate with blocking to match timing |
| `stream(speed, seek, duration, loop)`            | Observable with original timing       |
| `find_closest(timestamp, tolerance)`             | Find frame by absolute timestamp      |
| `find_closest_seek(relative_seconds, tolerance)` | Find frame by relative time           |
| `first()`                                        | Get first frame                       |
| `first_timestamp()`                              | Get first timestamp                   |
| `load(name)`                                     | Load specific frame by name/index     |
