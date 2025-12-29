# Temporal Message Alignment

We often have multiple sensors emitting data at different rates, with different latencies etc.

For perception we'd often like to align these datapoints temporaly (to for example project RGB image into a lidar pointcloud)

```python
self.detection_stream_3d = align_timestamped(
   backpressure(self.detection_stream_2d()),
   self.pointcloud.observable(),
   match_tolerance=0.25,
   buffer_size=20.0,
).pipe(ops.map(detection2d_to_3d))
```
