(doc-usage-sensor-streams-index-sensor-streams)=

# Sensor Streams

(doc-usage-sensor-streams-index-sensor-streams-1)=

# Sensor Streams

DimOS uses reactive streams (RxPY) to handle sensor data. This approach naturally fits robotics where multiple sensors emit data asynchronously at different rates, and downstream processors may be slower than the data sources.

(doc-usage-sensor-streams-index-guides)=

## Guides

:::{list-table}
   :header-rows: 1

   * - Guide
     - Description
   * - [ReactiveX fundamentals](reactivex.md)
     - Observables, subscriptions, and disposables
   * - [Advanced streams](advanced_streams.md)
     - Backpressure, parallel subscribers, and synchronous getters
   * - [Quality-based filtering](quality_filter.md)
     - Select the highest-quality frames when downsampling streams
   * - [Temporal alignment](temporal_alignment.md)
     - Match messages from multiple sensors by timestamp
   * - [Storage and replay](storage_replay.md)
     - Record streams and replay them with their original timing
:::

(doc-usage-sensor-streams-index-quick-example)=

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

```{toctree}
:hidden: true
:maxdepth: 1

advanced_streams
reactivex
quality_filter
temporal_alignment
storage_replay
```
