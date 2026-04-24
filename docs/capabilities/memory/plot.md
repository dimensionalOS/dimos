
## color cycle

You add streams, system auto assigns colors

```python session=plot output=none
import math
import random

from dimos.memory2.vis.plot.elements import Series
from dimos.memory2.vis.plot.plot import Plot

rng = random.Random(42)
xs = [i * 0.1 for i in range(120)]

color_check = Plot()
for i in range(14):
    phase = rng.uniform(0, 2 * math.pi)
    freq = rng.uniform(0.5, 1.8)
    amp = rng.uniform(0.6, 1.4)
    offset = i * 0.5  # vertical separation so curves don't overlap
    ys = [amp * math.sin(freq * x + phase) + offset for x in xs]

    color_check.add(Series(ts=xs, values=ys, label=f"curve {i + 1}"))

color_check.to_svg("assets/plot_colors.svg")
```

![output](assets/plot_colors.svg)

named colors can also be used explicitly. when you pin a series to one of
the named colors, the auto-cycle excludes it for the remaining series, so
you never end up with two lines that share a color by accident.

```python session=plot output=none
from dimos.memory2.vis import color
from dimos.memory2.vis.plot.elements import Series, HLine, Style

p = Plot()
# auto → blue
p.add(Series(ts=xs, values=[math.sin(x) for x in xs]))
# explicit green, dotted
p.add(Series(ts=xs, values=[math.cos(x) for x in xs], color=color.red, style=Style.dotted))
# auto → yellow (red is excluded)
p.add(Series(ts=xs, values=[math.sin(2 * x) for x in xs]))
# explicit color
p.add(HLine(y=0, style=Style.dashed, opacity=0.5, color="#ff0000"))
p.to_svg("assets/plot_named.svg")
```

![output](assets/plot_named.svg)

## speed plot

you can assign different axes to different time series, label them etc

```python session=robotdata output=none
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.transform import smooth, speed, throttle
from dimos.memory2.vis import color
from dimos.memory2.vis.plot.elements import Series
from dimos.memory2.vis.plot.plot import Plot
from dimos.utils.data import get_data

store = SqliteStore(path=get_data("go2_bigoffice.db"))
images = store.streams.color_image

plot = Plot()
plot.add(
    images.transform(speed()).transform(smooth(40)),
    label="speed (m/s)",
    opacity=0.75
)

plot.add(
    images.transform(throttle(0.5)).map_data(lambda obs: obs.data.brightness).transform(smooth(10)),
    label="brightness",
    color=color.blue,
)

plot.add(
    images.transform(throttle(0.5)).scan_data(images.first().ts, lambda state, obs: [state, obs.ts - state]),
    label="time",
    axis="time",
    opacity=0.5
)

plot.to_svg("assets/plot_robot_data.svg")
```

![output](assets/plot_robot_data.svg)

## Semantic search

Let's find some plants!

```python session=robotdata
from dimos.memory2.vis.plot.elements import Series, HLine, Style
from dimos.memory2.vis import color
from dimos.memory2.transform import normalize, smooth_time

from dimos.models.embedding.clip import CLIPModel
clip = CLIPModel()
search_vector = clip.embed_text("plant")

# we will cache this into memory since it takes a second,
# and use it to play with graphing
plantness_query = (
    store.streams.color_image_embedded
        .search(search_vector)
        # search() returns observations sorted by similarity, we re-sort by time
        .order_by("ts")
)

# we've built our query
print(plantness_query)

# we evaluate it into a in-memory stream,
# since we want to further process/plot multiple times
plantness_query_materialized = plantness_query.materialize()

print(plantness_query_materialized)
print(plantness_query_materialized.summary())

# let's create a numerical stream
plantness_similarity = plantness_query_materialized.map_data(lambda obs: obs.similarity).materialize()

plot = Plot()

plot.add(plantness_similarity,
  label="plant-ness",
  color=color.green,
)

plot.to_svg("assets/plot_plantness.svg")
```

<!--Result:-->
```
Stream("color_image_embedded") | vector_search() | order_by(ts)
Stream("materialize")
Stream("materialize"): 267 items, 2025-12-26 11:09:12 — 2025-12-26 11:14:00 (288.4s)
```

![output](assets/plot_plantness.svg)

We can be pretty sure the robot saw some plants by peaks at beginning and end of data, but this graph doesn't look great, why?

Embeddings are calculated according to some minimum picture brightness. Completely dark images are both useless and also semantically close to everything.

Let's investigate how our embedding stream relates to image brightness:

```python session=robotdata

plot = Plot()

plot.add(plantness_similarity,
  label="plant-ness",
  color=color.green,
)

plot.add(
    images.transform(throttle(0.5)).map_data(lambda obs: obs.data.brightness),
    label="brightness",
    axis="brightness"
)

plot.add(HLine(y=0.15, style=Style.dashed, color=color.red))

plot.to_svg("assets/plot_plantness_brightness.svg")
```

![output](assets/plot_plantness_brightness.svg)
We see that stuff isn't embedded below some minimum brightness.

Let's now fill the gaps in our semantic graph a bit, looks super ugly above, we will tell plotter to consider unmapped values as zero and connect values that are within 7.5 seconds, smooth with 5 second time window, and normalize the data

```python session=robotdata

plot = Plot()

plot.add(
    plantness_similarity \
      .transform(smooth_time(5.0)) \
      .transform(normalize()), \
      label="plant-ness",
      color=color.green,
      gap_fill=0.0,
      connect=7.5
)

plot.to_svg("assets/plot_plantness_gap_fill.svg")

```

![output](assets/plot_plantness_gap_fill.svg)

Looks better, these are some very obvious peaks, I'm curious let's see what was captured then.

Let's auto-detect the peaks, extract images from those moments, and run a 2D detector

```python session=robotdata
from dimos.mapping.voxels import VoxelMapTransformer
from dimos.memory2.vis.space.space import Space
from dimos.memory2.transform import peaks
from dimos.memory2.vis.color import ColorRange
from dimos.memory2.vis.plot.elements import VLine
from dimos.memory2.vis.utils import mosaic
from dimos.memory2.stream import Stream
from itertools import chain

semantic_peaks = plantness_query_materialized.transform(peaks(key=lambda obs: obs.similarity, distance=1.0))

# load all lidar frames captured in the readius around the semantic peaks
# feed them into a global mapper to get a single pointcloud around our areas of interest
global_map = semantic_peaks \
   .map(lambda obs: store.streams.lidar.near(obs.pose_stamped, radius=0.5).first()) \
   .transform(VoxelMapTransformer()) \
   .last().data

drawing = Space()
drawing.add(global_map)
drawing.add(semantic_peaks)
drawing.to_svg("assets/plot_plantness_autopeaks_map.svg")

peakColor = ColorRange("turbo")
for i, p in enumerate(semantic_peaks):
    print(f"t={p.ts - plantness_similarity.first().ts:6.1f}s score={p.similarity:.3f} prominence={p.tags['peak_prominence']:.3f}")
    plot.add(VLine(p.ts, color=peakColor(i)))

plot.to_svg("assets/plot_plantness_autopeaks.svg")

from dimos.models.vl.moondream import MoondreamVlModel
moondream = MoondreamVlModel()
moondream.start()

# peaks is still a stream of image observations (with prominence and semantic similarity metadata)
# so we can just draw it directly via mosaic that takes image streams
m = mosaic(semantic_peaks.map_data(lambda obs: moondream.query_detections(obs.data, "plant")))

m.data.save("assets/plants_auto.png")
```

<!--Result:-->
```
14:59:33.042 [inf][dimos/mapping/voxels.py       ] VoxelGrid using device: CUDA:0
t=  14.1s score=0.224 prominence=0.031
t=  26.3s score=0.225 prominence=0.033
t=  32.7s score=0.224 prominence=0.022
t=  37.0s score=0.259 prominence=0.067
t=  60.6s score=0.227 prominence=0.031
t=  61.5s score=0.218 prominence=0.026
t=  76.3s score=0.221 prominence=0.031
t=  84.0s score=0.223 prominence=0.027
t=  89.1s score=0.219 prominence=0.020
t= 162.9s score=0.224 prominence=0.041
t= 168.0s score=0.219 prominence=0.031
t= 172.4s score=0.218 prominence=0.020
t= 240.4s score=0.243 prominence=0.047
t= 245.6s score=0.224 prominence=0.028
t= 279.6s score=0.230 prominence=0.030
```


![output](assets/plot_plantness_autopeaks.svg)

![output](assets/plants_auto.png)

![output](assets/plot_plantness_autopeaks_map.svg)

## Which peaks are significant?

We got 15 peaks back, we ran a detector on all of them so we can start projecting into 3D but let's say we want some sort of pre-filter of just globally significant peaks. we can see most peaks prominence sits around 0.02–0.03 and only a couple (0.067 at t=37s, 0.047 at t=240s) really stand out. We might want to auto detect those.

`significant()` replaces that guesswork by thresholding on the distribution of prominences itself. Default outlier detection uses MAD (median absolute deviation)

Once we put the surviving peaks on the timeline we get two very obvious plants.

```python session=robotdata
from dimos.memory2.transform import significant

plot = Plot()
plot.add(
    plantness_similarity.transform(smooth_time(5.0)).transform(normalize()),
    label="plant-ness", color=color.green, gap_fill=0.0, connect=7.5,
)

meaningful_peaks = semantic_peaks.transform(significant(method="mad"))

for peak in meaningful_peaks:
    plot.add(VLine(peak.ts, color=color.red))

m = mosaic(meaningful_peaks)
m.data.save("assets/plants_meaningful.png")

plot.to_svg("assets/plot_plantness_significant.svg")
```


![output](assets/plot_plantness_significant.svg)

![output](assets/plants_meaningful.png)

Rule of thumb: keep a small absolute floor on `peaks(prominence=...)` to
reject shape-noise, then let `significant()` pick the statistical cutoff.

## Semantic peak analysis

Let's focus on those two peaks. load all images in the vicinity of a detection,

We'll also pull all lidar frames in their vicinity and reconstruct global maps for those areas.

```python session=robotdata

from dimos.memory2.vis.space.elements import Point
from dimos.memory2.transform import QualityWindow

drawing = Space()

# TODO actual near/at filters need to accept observation streams in order to easily
# reconstruct all frames in vicinity of another stream
# for now for simplicity here we are focusing only on one semantic hotspot.
meaningful_peak = meaningful_peaks.first()

# load all images captured in the readius around the semantic peak
near_images = images.near(meaningful_peak.pose_stamped, radius=2.5) \
    .filter(lambda obs: obs.data.brightness > 0.1) \
    .transform(QualityWindow(lambda img: img.sharpness, window=0.5))

# load all lidar frames captured in the readius around the semantic peak
# feed them into a global mapper to get a single pointcloud around our area of interest
global_map = store.streams.lidar.near(meaningful_peak.pose_stamped, radius=2.5) \
   .transform(VoxelMapTransformer()) \
   .last().data

# run our global mapper only on lidar frames around the POI
drawing.add(global_map)
drawing.add(meaningful_peak.pose_stamped, color=color.green)

# run a detector, filter small weird detections
detections = (near_images
    .map_data(lambda obs: moondream.query_detections(obs.data, "plant"))
    .map_data(lambda obs: obs.data.filter(lambda det: det.bbox_2d_volume() > 3000))
    .filter(lambda obs: len(obs.data) > 0)
    .materialize())
    # materialize this stream since we'll want to re-use it later

drawing.add(detections)
drawing.to_svg("assets/peak_space.svg")

m = mosaic(detections)
m.data.save("assets/plants_peak_detections.png")
```

![output](assets/peak_space.svg)
![output](assets/plants_peak_detections.png)

## 3D Projection

```python session=robotdata output=none
from dimos.perception.detection.type.detection3d.imageDetections3DPC import (
    ImageDetections3DPC,
)
from dimos.robot.unitree.go2.connection import (
    _camera_info_static as go2_camerainfo,
    BASE_TO_OPTICAL,
)
from dimos.memory2.vis.space.elements import Box3D
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3

# TODO We need a nicer way to get optical transform for image streams
# depending on the source
def world_to_optical(base_pose):
    return -(Transform.from_pose("base_link", base_pose) + BASE_TO_OPTICAL)

drawing = Space()

drawing.add(global_map)

drawing.add(detections)

camera_info = go2_camerainfo()

detections3d = (detections
    .map_data(lambda obs: ImageDetections3DPC.from_2d(
        obs.data,
        global_map,
        camera_info,
        world_to_optical(obs.pose_stamped),
    ))
    .filter(lambda obs: len(obs.data) > 0))

# TODO detection3d needs to be a natural thing to render
for obs in detections3d:
    for d3d in obs.data:
        aabb = d3d.get_bounding_box()
        c, e = aabb.get_center(), aabb.get_extent()
        drawing.add(Box3D(
            center=Pose(float(c[0]), float(c[1]), float(c[2])),
            size=Vector3(float(e[0]), float(e[1]), float(e[2])),
            color=color.green, label="plant",
        ))

drawing.to_svg("assets/peak_detections.svg")

```

![output](assets/peak_detections.svg)

# TODO further steps

- These are 3D bounding boxes with associated pointclouds, render in rerun

- Some basic statistical outlier filters - we have many overlaping detections here and we can be pretty sure there are plants right of the robot, but unclear about left.

- Now that we have 3d locations in space, we can load all camera images observing detections in space (not just rely on radius around the embedding peak) see in how many of these images we actually detect an object. (another strategy for false positive filtering)
