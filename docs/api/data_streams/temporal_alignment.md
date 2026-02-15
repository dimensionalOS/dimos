# Temporal Message Alignment

Robots have multiple sensors emitting data at different rates and latencies. A camera might run at 30fps, while lidar scans at 10Hz, and each has different processing delays. For perception tasks like projecting 2D detections into 3D pointclouds, we need to match data from these streams by timestamp.

`align_timestamped` solves this by buffering messages and matching them within a time tolerance.

<details><summary>Pikchr</summary>

```pikchr fold output=assets/alignment_overview.svg
color = white
fill = none

Cam: box "Camera" "30 fps" rad 5px fit wid 170% ht 170%
arrow from Cam.e right 0.4in then down 0.35in then right 0.4in
Align: box "align_timestamped" rad 5px fit wid 170% ht 170%

Lidar: box "Lidar" "10 Hz" rad 5px fit wid 170% ht 170% with .s at (Cam.s.x, Cam.s.y - 0.7in)
arrow from Lidar.e right 0.4in then up 0.35in then right 0.4in

arrow from Align.e right 0.4in
Out: box "(image, pointcloud)" rad 5px fit wid 170% ht 170%
```

</details>

<!--Result:-->
![output](assets/alignment_overview.svg)


## Basic Usage

Below we set up replay of real camera and lidar data from the Unitree Go2 robot. You can check it if you're interested.

<details>
<summary>Stream Setup</summary>

You can read more about [sensor storage here](storage_replay.md) and [LFS data storage here](/docs/development/large_file_management.md).

```python session=align no-result
from reactivex import Subject
from dimos.utils.testing import TimedSensorReplay
from dimos.types.timestamped import Timestamped, align_timestamped
from reactivex import operators as ops
import reactivex as rx

# Load recorded Go2 sensor streams
video_replay = TimedSensorReplay("unitree_go2_bigoffice/video")
lidar_replay = TimedSensorReplay("unitree_go2_bigoffice/lidar")

# This is a bit tricky. We find the first video frame timestamp, then add 2 seconds to it.
seek_ts = video_replay.first_timestamp() + 2

# Lists to collect items as they flow through streams
video_frames = []
lidar_scans = []

# We are using from_timestamp=... and not seek=... because seek seeks through recording
# timestamps, from_timestamp matches actual message timestamp.
# It's possible for sensor data to come in late, but with correct capture time timestamps
video_stream = video_replay.stream(from_timestamp=seek_ts, duration=2.0).pipe(
    ops.do_action(lambda x: video_frames.append(x))
)

lidar_stream = lidar_replay.stream(from_timestamp=seek_ts, duration=2.0).pipe(
    ops.do_action(lambda x: lidar_scans.append(x))
)

```


</details>

Streams would normally come from an actual robot into your module via `In` inputs. [`detection/module3D.py`](/dimos/perception/detection/module3D.py#L11) is a good example of this.

Assume we have them. Let's align them.

```python session=align
# Align video (primary) with lidar (secondary)
# match_tolerance: max time difference for a match (seconds)
# buffer_size: how long to keep messages waiting for matches (seconds)
aligned_pairs = align_timestamped(
    video_stream,
    lidar_stream,
    match_tolerance=0.025,  # 25ms tolerance
    buffer_size=5.0, # how long to wait for match
).pipe(ops.to_list()).run()

print(f"Video: {len(video_frames)} frames, Lidar: {len(lidar_scans)} scans")
print(f"Aligned pairs: {len(aligned_pairs)} out of {len(video_frames)} video frames")

# Show a matched pair
if aligned_pairs:
    img, pc = aligned_pairs[0]
    dt = abs(img.ts - pc.ts)
    print(f"\nFirst matched pair: Δ{dt*1000:.1f}ms")
```

<!--Result:-->
```
Video: 29 frames, Lidar: 15 scans
Aligned pairs: 11 out of 29 video frames

First matched pair: Δ11.3ms
```

<details>
<summary>Visualization helper</summary>

```python session=align fold no-result
import matplotlib
import matplotlib.pyplot as plt

def plot_alignment_timeline(video_frames, lidar_scans, aligned_pairs, path):
    """Single timeline: video above axis, lidar below, green lines for matches."""
    matplotlib.use('Agg')
    plt.style.use('dark_background')

    # Get base timestamp for relative times (frames have .ts attribute)
    base_ts = video_frames[0].ts
    video_ts = [f.ts - base_ts for f in video_frames]
    lidar_ts = [s.ts - base_ts for s in lidar_scans]

    # Find matched timestamps
    matched_video_ts = set(img.ts for img, _ in aligned_pairs)
    matched_lidar_ts = set(pc.ts for _, pc in aligned_pairs)

    fig, ax = plt.subplots(figsize=(12, 2.5))

    # Video markers above axis (y=0.3) - circles, cyan when matched
    for frame in video_frames:
        rel_ts = frame.ts - base_ts
        matched = frame.ts in matched_video_ts
        ax.plot(rel_ts, 0.3, 'o', color='cyan' if matched else '#688', markersize=8)

    # Lidar markers below axis (y=-0.3) - squares, orange when matched
    for scan in lidar_scans:
        rel_ts = scan.ts - base_ts
        matched = scan.ts in matched_lidar_ts
        ax.plot(rel_ts, -0.3, 's', color='orange' if matched else '#a86', markersize=8)

    # Green lines connecting matched pairs
    for img, pc in aligned_pairs:
        img_rel = img.ts - base_ts
        pc_rel = pc.ts - base_ts
        ax.plot([img_rel, pc_rel], [0.3, -0.3], '-', color='lime', alpha=0.6, linewidth=1)

    # Axis styling
    ax.axhline(y=0, color='white', linewidth=0.5, alpha=0.3)
    ax.set_xlim(-0.1, max(video_ts + lidar_ts) + 0.1)
    ax.set_ylim(-0.6, 0.6)
    ax.set_xlabel('Time (s)')
    ax.set_yticks([0.3, -0.3])
    ax.set_yticklabels(['Video', 'Lidar'])
    ax.set_title(f'{len(aligned_pairs)} matched from {len(video_frames)} video + {len(lidar_scans)} lidar')
    plt.tight_layout()
    plt.savefig(path, transparent=True)
    plt.close()
```

</details>

```python session=align output=assets/alignment_timeline.png
plot_alignment_timeline(video_frames, lidar_scans, aligned_pairs, '{output}')
```

<!--Result:-->
![output](assets/alignment_timeline.png)

If we loosen up our match tolerance, we might get multiple pairs matching the same lidar frame.

```python session=align
aligned_pairs = align_timestamped(
    video_stream,
    lidar_stream,
    match_tolerance=0.05,  # 50ms tolerance
    buffer_size=5.0, # how long to wait for match
).pipe(ops.to_list()).run()

print(f"Video: {len(video_frames)} frames, Lidar: {len(lidar_scans)} scans")
print(f"Aligned pairs: {len(aligned_pairs)} out of {len(video_frames)} video frames")
```

<!--Result:-->
```
Video: 58 frames, Lidar: 30 scans
Aligned pairs: 23 out of 58 video frames
```


```python session=align output=assets/alignment_timeline2.png
plot_alignment_timeline(video_frames, lidar_scans, aligned_pairs, '{output}')
```

<!--Result:-->
![output](assets/alignment_timeline2.png)

## Combine Frame Alignment with a Quality Filter

More on [quality filtering here](quality_filter.md).

```python session=align
from dimos.msgs.sensor_msgs.Image import Image, sharpness_barrier

# Lists to collect items as they flow through streams
video_frames = []
lidar_scans = []

video_stream = video_replay.stream(from_timestamp=seek_ts, duration=2.0).pipe(
    sharpness_barrier(3.0),
    ops.do_action(lambda x: video_frames.append(x))
)

lidar_stream = lidar_replay.stream(from_timestamp=seek_ts, duration=2.0).pipe(
    ops.do_action(lambda x: lidar_scans.append(x))
)

aligned_pairs = align_timestamped(
    video_stream,
    lidar_stream,
    match_tolerance=0.025,  # 25ms tolerance
    buffer_size=5.0, # how long to wait for match
).pipe(ops.to_list()).run()

print(f"Video: {len(video_frames)} frames, Lidar: {len(lidar_scans)} scans")
print(f"Aligned pairs: {len(aligned_pairs)} out of {len(video_frames)} video frames")

```

<!--Result:-->
```
Video: 6 frames, Lidar: 15 scans
Aligned pairs: 1 out of 6 video frames
```

```python session=align output=assets/alignment_timeline3.png
plot_alignment_timeline(video_frames, lidar_scans, aligned_pairs, '{output}')
```

<!--Result:-->
![output](assets/alignment_timeline3.png)

We are very picky but data is high quality. Best frame, with closest lidar match in this window.

## How It Works

The primary stream (first argument) drives emissions. When a primary message arrives:

1. **Immediate match**: If matching secondaries already exist in buffers, emit immediately
2. **Deferred match**: If secondaries are missing, buffer the primary and wait

When secondary messages arrive:
1. Add to buffer for future primary matches
2. Check buffered primaries - if this completes a match, emit

<details>
<summary>diagram source</summary>

```pikchr fold output=assets/alignment_flow.svg
color = white
fill = none
linewid = 0.35in

Primary: box "Primary" "arrives" rad 5px fit wid 170% ht 170%
arrow
Check: box "Check" "secondaries" rad 5px fit wid 170% ht 170%

arrow from Check.e right 0.35in then up 0.4in then right 0.35in
Emit: box "Emit" "match" rad 5px fit wid 170% ht 170%
text "all found" at (Emit.w.x - 0.4in, Emit.w.y + 0.15in)

arrow from Check.e right 0.35in then down 0.4in then right 0.35in
Buffer: box "Buffer" "primary" rad 5px fit wid 170% ht 170%
text "waiting..." at (Buffer.w.x - 0.4in, Buffer.w.y - 0.15in)
```

</details>

<!--Result:-->
![output](assets/alignment_flow.svg)

## Parameters

| Parameter                | Type               | Default  | Description                                     |
|--------------------------|--------------------|----------|-------------------------------------------------|
| `primary_observable`     | `Observable[T]`    | required | Primary stream that drives output timing        |
| `*secondary_observables` | `Observable[S]...` | required | One or more secondary streams to align          |
| `match_tolerance`        | `float`            | 0.1      | Max time difference for a match (seconds)       |
| `buffer_size`            | `float`            | 1.0      | How long to buffer unmatched messages (seconds) |



## Usage in Modules

Every module `In` port exposes an `.observable()` method that returns a backpressured stream of incoming messages. This makes it easy to align inputs from multiple sensors.

From [`detection/module3D.py`](/dimos/perception/detection/module3D.py), projecting 2D detections into 3D pointclouds:

```python skip
class Detection3DModule(Detection2DModule):
    color_image: In[Image]
    pointcloud: In[PointCloud2]

    def start(self):
        # Align 2D detections with pointcloud data
        self.detection_stream_3d = align_timestamped(
            backpressure(self.detection_stream_2d()),
            self.pointcloud.observable(),
            match_tolerance=0.25,
            buffer_size=20.0,
        ).pipe(ops.map(detection2d_to_3d))
```

The 2D detection stream (camera + ML model) is the primary, matched with raw pointcloud data from lidar. The longer `buffer_size=20.0` accounts for variable ML inference times.
