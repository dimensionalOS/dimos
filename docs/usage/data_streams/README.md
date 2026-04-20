# Sensor Streams

Dimos uses reactive streams (RxPY) to handle sensor data. This approach naturally fits robotics where multiple sensors emit data asynchronously at different rates, and downstream processors may be slower than the data sources.

## Guides

| Guide                                        | Description                                                   |
|----------------------------------------------|---------------------------------------------------------------|
| [ReactiveX Fundamentals](/docs/usage/data_streams/reactivex.md)       | Observables, subscriptions, and disposables                   |
| [Advanced Streams](/docs/usage/data_streams/advanced_streams.md)      | Backpressure, parallel subscribers, synchronous getters       |
| [Quality-Based Filtering](/docs/usage/data_streams/quality_filter.md) | Select highest quality frames when downsampling streams       |
| [Temporal Alignment](/docs/usage/data_streams/temporal_alignment.md)  | Match messages from multiple sensors by timestamp             |
| [Storage & Replay](/docs/usage/data_streams/storage_replay.md)        | Record sensor streams to disk and replay with original timing |

## Quick Example

```python
from __future__ import annotations

import time

import reactivex as rx

from dimos.types.timestamped import Timestamped, align_timestamped
from dimos.utils.reactive import backpressure


class SampleMsg(Timestamped):
    def __init__(self, ts: float, label: str = "") -> None:
        super().__init__(ts)
        self.label = label


# Stand in for camera/lidar hardware streams: same pipeline shape, no device required.
camera_stream = rx.of(SampleMsg(0.00), SampleMsg(0.03), SampleMsg(0.06))
lidar_stream = rx.of(SampleMsg(0.02), SampleMsg(0.11))

processed = camera_stream
aligned = align_timestamped(
    backpressure(processed),
    lidar_stream,
    match_tolerance=0.15,
)
out: list[tuple[SampleMsg, ...]] = []
aligned.subscribe(on_next=out.append)
time.sleep(0.25)
print("pairs", len(out))
```

<!--Result:-->
```
pairs 3
```
