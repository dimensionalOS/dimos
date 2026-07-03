---
title: "Sensor Streams"
---

Dimos uses reactive streams (RxPY) to handle sensor data. This approach naturally fits robotics where multiple sensors emit data asynchronously at different rates, and downstream processors may be slower than the data sources.

## Guides

| Guide                                        | Description                                                   |
|----------------------------------------------|---------------------------------------------------------------|
| [ReactiveX Fundamentals](/usage/sensor_streams/reactivex)       | Observables, subscriptions, and disposables                   |
| [Advanced Streams](/usage/sensor_streams/advanced_streams)      | Backpressure, parallel subscribers, synchronous getters       |
| [Quality-Based Filtering](/usage/sensor_streams/quality_filter) | Select highest quality frames when downsampling streams       |
| [Temporal Alignment](/usage/sensor_streams/temporal_alignment)  | Match messages from multiple sensors by timestamp             |
| [Storage & Replay](/usage/sensor_streams/storage_replay)        | Record sensor streams to disk and replay with original timing |

## Quick Example

```python skip
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
