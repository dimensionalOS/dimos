(doc-usage-data-streams-storage-replay-sensor-storage-and-replay)=

# Sensor Storage and Replay

(doc-usage-data-streams-storage-replay-sensor-storage-and-replay-1)=

# Sensor Storage and Replay

Record sensor streams to disk and replay them with original timing. Useful for testing, debugging, and creating reproducible datasets.

(doc-usage-data-streams-storage-replay-quick-start)=

## Quick Start

(doc-usage-data-streams-storage-replay-recording)=

### Recording

```python
from dimos.utils.testing.replay import TimedSensorStorage

# Create storage (directory in data folder)
storage = TimedSensorStorage("my_recording")

# Save frames from a stream
camera_stream.subscribe(storage.save_one)

# Or save manually
storage.save(frame1, frame2, frame3)
```

(doc-usage-data-streams-storage-replay-replaying)=

### Replaying

```python
from dimos.utils.testing.replay import TimedSensorReplay

# Load recording
replay = TimedSensorReplay("my_recording")

# Iterate at original speed
for frame in replay.iterate_realtime():
    process(frame)

# Or as an Observable stream
replay.stream(speed=1.0).subscribe(process)
```

(doc-usage-data-streams-storage-replay-timedsensorstorage)=

## TimedSensorStorage

Stores sensor data with timestamps as pickle files. Each frame is saved as `000.pickle`, `001.pickle`, etc.

```python
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

```python
# Custom serialization
storage = TimedSensorStorage(
    "custom_capture",
    autocast=lambda frame: frame.to_dict()
)
```

(doc-usage-data-streams-storage-replay-timedsensorreplay)=

## TimedSensorReplay

Replays stored sensor data with timestamp-aware iteration and seeking.

(doc-usage-data-streams-storage-replay-basic-iteration)=

### Basic Iteration

```python
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

(doc-usage-data-streams-storage-replay-realtime-playback)=

### Realtime Playback

```python
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

(doc-usage-data-streams-storage-replay-seeking-and-slicing)=

### Seeking and Slicing

```python
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

(doc-usage-data-streams-storage-replay-finding-specific-frames)=

### Finding Specific Frames

```python
# Find frame closest to absolute timestamp
frame = replay.find_closest(1704067200.0)

# Find frame closest to relative time (30s from start)
frame = replay.find_closest_seek(30.0)

# With tolerance (returns None if no match within 0.1s)
frame = replay.find_closest(timestamp, tolerance=0.1)
```

(doc-usage-data-streams-storage-replay-observable-stream)=

### Observable Stream

The `.stream()` method returns an Observable that emits frames with original timing:

```python
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

(doc-usage-data-streams-storage-replay-usage-stub-connections-for-testing)=

## Usage: Stub Connections for Testing

A common pattern is creating replay-based connection stubs for testing without hardware. From [robot/unitree/go2/connection.py](https://github.com/dimensionalOS/dimos/blob/main/dimos/robot/unitree/go2/connection.py#L83):

This is a bit primitive. We'd like to write a higher-order API for recording full module I/O for any module, but this is a work in progress at the moment.

```python
class ReplayConnection(UnitreeWebRTCConnection):
    dir_name = "go2_sf_office"

    def __init__(self, **kwargs) -> None:
        get_data(self.dir_name)
        self.replay_config = {
            "loop": kwargs.get("loop"),
            "seek": kwargs.get("seek"),
            "duration": kwargs.get("duration"),
        }

    def lidar_stream(self):
        lidar_store = TimedSensorReplay(f"{self.dir_name}/lidar")
        return lidar_store.stream(**self.replay_config)

    def video_stream(self):
        video_store = TimedSensorReplay(f"{self.dir_name}/video")
        return video_store.stream(**self.replay_config)
```

This allows running the full perception pipeline against recorded data:

```python
# Use replay connection instead of real hardware
connection = ReplayConnection(loop=True, seek=5.0)
robot = GO2Connection(connection=connection)
```

(doc-usage-data-streams-storage-replay-data-format)=

## Data Format

Each pickle file contains a tuple `(timestamp, data)`:

- **timestamp**: Unix timestamp (float) when the frame was captured
- **data**: The sensor data (or result of `autocast` if provided)

Files are numbered sequentially: `000.pickle`, `001.pickle`, etc.

Recordings are stored in the `data/` directory. See [Data Loading](../../development/large_file_management.md) for how data storage works, including Git LFS handling for large datasets.

(doc-usage-data-streams-storage-replay-api-reference)=

## API Reference

(doc-usage-data-streams-storage-replay-timedsensorstorage-1)=

### TimedSensorStorage

| Method                       | Description                              |
| ---------------------------- | ---------------------------------------- |
| `save_one(frame)`            | Save a single frame, returns frame count |
| `save(*frames)`              | Save multiple frames                     |
| `save_stream(observable)`    | Pipe an observable through storage       |
| `consume_stream(observable)` | Subscribe and save without returning     |

(doc-usage-data-streams-storage-replay-timedsensorreplay-1)=

### TimedSensorReplay

| Method                                           | Description                           |
| ------------------------------------------------ | ------------------------------------- |
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
