# Sensor Streams

Dimos uses reactive streams (RxPY) to handle sensor data. This approach naturally fits robotics where multiple sensors emit data asynchronously at different rates, and downstream processors may be slower than the data sources.

## Guides

| Guide                                        | Description                                                   |
|----------------------------------------------|---------------------------------------------------------------|
| [ReactiveX Fundamentals](reactivex.md)       | Observables, subscriptions, and disposables                   |
| [Advanced Streams](advanced_streams.md)      | Backpressure, parallel subscribers, synchronous getters       |
| [Quality-Based Filtering](quality_filter.md) | Select highest quality frames when downsampling streams       |
| [Temporal Alignment](temporal_alignment.md)  | Match messages from multiple sensors by timestamp             |
| [Storage & Replay](storage_replay.md)        | Record sensor streams to disk and replay with original timing |

## Quick Example

```python
from reactivex import operators as ops
from dimos.utils.reactive import backpressure
from dimos.types.timestamped import align_timestamped
from dimos.msgs.sensor_msgs.Image import sharpness_barrier

# Camera at 30fps, lidar at 10Hz
camera_stream = camera.observable()
lidar_stream = lidar.observable()

# Pipeline: filter blurry frames -> align with lidar -> handle slow consumers
processed = (
    camera_stream.pipe(
        sharpness_barrier(10.0),  # Keep sharpest frame per 100ms window (10Hz)
    )
)

aligned = align_timestamped(
    backpressure(processed),     # Camera as primary
    lidar_stream,                # Lidar as secondary
    match_tolerance=0.1,
)

aligned.subscribe(lambda pair: process_frame_with_pointcloud(*pair))
```
