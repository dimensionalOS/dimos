# Querying your memory in Python

Everything the run records lives in one SQLite store, `recordings/<run-id>/memory.db`, written by
DimOS `--record`. `recall.py` covers the common questions; when it does not, write the query
yourself with `dimos.memory`. Every line below was run against a real recording. Run scripts from
the repo root with `uv run python`.

## Open it

```python
import sys; sys.path.insert(0, "dimos/experimental/frank")
import recall
store = recall.open_store()            # newest recording, or SqliteStore(path=...) yourself
print(store.list_streams())            # ['color_image', 'lidar', 'odom', 'tf', ...]
img, odom, lidar = store.streams.color_image, store.streams.odom, store.streams.lidar
print(img.summary())                   # items, time range, Hz, size
```

## Streams and what is in them

| stream        | `obs.data`     | fields you use                                                          |
|---------------|----------------|-------------------------------------------------------------------------|
| `color_image` | `Image`        | `.to_opencv()` (BGR ndarray), `.width`, `.height`                        |
| `odom`        | `PoseStamped`  | `.position.x .y`, `.yaw` (radians), `.frame_id` = "world"               |
| `lidar`       | `PointCloud2`  | `.points_f32()` -> (N, 3) world-frame metres, z up (floor is about 0.3 below the lidar) |
| `tf`          | `TFMessage`    | frames world -> base_link -> camera_link, lidar_link                     |

Every observation has `obs.ts` (epoch seconds), `obs.data` (loaded lazily on first touch, so
filter before you touch it), `obs.pose_tuple` (None for recorded frames, see below), `obs.tags`.

## Filters, then one terminal

Queries are lazy; nothing runs until a terminal. Chain filters, then end with one of
`.to_list()`, `.first()`, `.last()`, `.count()`, `.exists()`, `.get_time_range()`.

```python
t0, t1 = odom.get_time_range()
odom.after(t1 - 60).to_list()                      # last minute of poses
img.at(ts, tolerance=0.5).first()                  # the frame nearest a time (None if nothing within tolerance)
img.time_range(t0 + 10, t0 + 20).count()           # frames in a window
odom.last().data.yaw                               # where I face now, radians
lidar.at(ts, tolerance=1.0).first().data.points_f32()
img.after(t).limit(20).to_list()                   # never .to_list() an hour of frames without .limit()
```

`.after(t)`, `.before(t)`, `.time_range(a, b)`, `.at(t, tolerance)`, `.limit(k)`, `.offset(n)`,
`.filter(pred)`, `.map(fn)`, `.map_data(fn)`, `.order_by(field, desc)`, `.tags(**kv)`.

## Where was I when I saw that: join frames to odometry

Recorded frames carry no pose (`pose_tuple` is None), so `.near()` on `color_image` finds nothing.
Two ways to attach one:

```python
# a. nearest odometry by time (what recall.Track does; fast, an hour scans in 2 s)
track = recall.Track(store, t0)                    # odometry from t0 on
x, y, yaw_deg = track.at(frame.ts)                 # None if no pose within a second
track.within(2.0, -1.5, radius=1.0)                # [(t_in, t_out), ...] visits to a spot

# b. the DimOS way: pose_fill re-poses a stream from another, then .near() works
from dimos.mapping.cli.pose_fill import pose_fill
posed = pose_fill(img, odom, tolerance=0.1)
posed.near((1.9, 0.1), radius=1.0).limit(5).to_list()   # frames taken within 1 m of a spot
```

`img.align(odom, tolerance=0.1)` is the raw join: each item's `.data` is an `AlignedPair`
(primary, secondary) of observations.

## Pictures and clouds from the past

```python
import cv2
frame = img.at(ts, tolerance=0.5).first()
cv2.imwrite("cache/then.jpg", frame.data.to_opencv())      # then Read it

# the room as the lidar saw it over a window, stacked
import numpy as np
pts = np.vstack([o.data.points_f32() for o in lidar.time_range(t0, t0 + 5).to_list()])
import cloud; print(cloud.encode(pts)["raster"]["rows"][:5])  # cloud.py's text raster works on any (N, 3)
```

## Search by description

```python
from dimos.models.embedding.clip import CLIPModel
clip = CLIPModel()                                  # 6 s to load, then fast
q = recall._vec(clip.embed_text("a staircase"))
frames = img.after(t1 - 600).to_list()[::30]        # about one a second
for f, e in zip(frames, clip.embed(*[f.data for f in frames])):
    score = float(recall._vec(e) @ q)
```

`recall.py find` does exactly this and draws the best tiles; write your own when you need a
different window, a different k, or a score threshold.

## Live queries

`.live()` backfills matches, then blocks for new ones. Use `.first()` or `.limit(k)` on it, never
`.to_list()` or `.last()` (they would wait forever). `stream.live().filter(pred).first()` is
"wait until this happens".

## Rules of thumb

- Time is epoch seconds; `recall.clock(ts)` prints HH:MM:SS, `recall.compass(yaw_deg)` names a heading.
- Filter by time before anything else; an hour of frames is 50k rows, and `.data` on each is a disk read.
- The store is being written while you read; that is fine (WAL). Do not append to the recorded streams.
- One script, one answer, then say it in words. Draw a picture (`recall.mosaic(tiles, out)`) when the answer is what something looked like.
