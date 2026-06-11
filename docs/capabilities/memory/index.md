<details>
<summary>Python</summary>

```python title="Python" fold session=mem output=none
import pickle
from dimos.mapping.pointclouds.occupancy import general_occupancy, simple_occupancy, height_cost_occupancy
from dimos.mapping.occupancy.inflation import simple_inflate
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.vis.color import Color
from dimos.memory2.transform import downsample, throttle, speed, smooth
from dimos.memory2.vis.space.space import Space
from dimos.utils.data import get_data
from dimos.memory2.vis.space.elements import Point
```

</details>

we init our recording, investigate available streams

```python title="Python" session=mem
store = SqliteStore(path=get_data("go2_bigoffice.db"))

for name, stream in store.streams.items():
   print(stream.summary())
```

```results
Stream("color_image"): 4164 items, 2025-12-26 11:09:08 — 2025-12-26 11:14:00 (292.5s)
Stream("color_image_embedded"): 267 items, 2025-12-26 11:09:12 — 2025-12-26 11:14:00 (288.4s)
Stream("lidar"): 2251 items, 2025-12-26 11:09:08 — 2025-12-26 11:14:00 (292.3s)
Stream("odom"): 5465 items, 2025-12-26 11:09:08 — 2025-12-26 11:14:00 (292.5s)
```

Any stream is drawable

```python title="Python" session=mem output=none
global_map = pickle.loads(get_data("unitree_go2_bigoffice_map.pickle").read_bytes())

drawing = Space()

# this is not necessary but we use a global map as a nice base for a drawing
drawing.add(global_map)
drawing.add(store.streams.color_image)
drawing.to_svg("assets/color_image.svg")
```

our drawing system applies turbo color scheme to timestamps by default

![output](assets/color_image.svg)

we can create new streams by querying existing streams, and we can save, further transform or draw those

```python title="Python" session=mem output=none

drawing = Space()
drawing.add(global_map)

drawing.add(
  store.streams.color_image \
  # calculate speed in m/s by checking distance between poses and timestamps of observations
  .transform(speed()) \
  # rolling window average
  .transform(smooth(50)))

drawing.to_svg("assets/speed.svg")
```

![output](assets/speed.svg)

we can do all kinds of things with this, for example map out room lighting

```python title="Python" session=mem output=none
drawing = Space()
drawing.add(global_map)

drawing.add(
  store.streams.color_image \
  # here we will take 4fps because brightness calculation loads the actual image
  # observation.data triggers another db query to fetch the data
  # otherwise observations only hold positions and timestamps
  .transform(throttle(0.25)) \
  # we calculate brightness
  .map(lambda obs: obs.derive(data=obs.data.brightness)))

drawing.to_svg("assets/brightness.svg")
```

![output](assets/brightness.svg)

So knowing above, we can create embeddings for the full stream,

```python title="Python" session=mem skip
from dimos.models.embedding.clip import CLIPModel
from dimos.msgs.sensor_msgs.Image import Image
from dimos.memory2.transform import QualityWindow
from dimos.memory2.embed import EmbedImages

embedded = store.stream("color_image_embedded", Image)
clip = CLIPModel()

# Downsample to 2Hz, filter dark images, then embed
pipeline = (
    store.streams.color_image.filter(lambda obs: obs.data.brightness > 0.1)
    .transform(QualityWindow(lambda img: img.sharpness, window=0.5))
    .transform(EmbedImages(clip))
    .save(embedded)
)

print(pipeline)

```

this pipeline is ready to execute by lazy, we can execute it by iterating, or calling .drain()

```python skip
for obs in pipeline:
    print(f"  [{count}] ts={obs.ts:.2f} pose={obs.pose}")
```

let's query it!

```python title="Python" session=mem output=none
from dimos.models.embedding.clip import CLIPModel

drawing = Space()
drawing.add(global_map)

clip = CLIPModel()
search_vector = clip.embed_text("shop")
drawing.add(store.streams.color_image_embedded.search(search_vector))

drawing.to_svg("assets/embedding.svg")
```

![output](assets/embedding.svg)

We don't really have to deal with the whole global map actually, let's get top 10 embeddings, and render only lidar around those.

```python title="Python" session=mem output=none
from dimos.models.embedding.clip import CLIPModel
from dimos.mapping.voxels import VoxelMapTransformer
drawing = Space()

# this is defined here, but not executed
matches = store.streams.color_image_embedded.search(search_vector, k=30)

print(matches) # Stream("color_image_embedded") | vector_search(k=50)

# here we execute it once, and feed it into a global mapper, then draw the map
drawing.add(
   matches.map(lambda obs: store.streams.lidar.at(obs.ts).last()) \
   .transform(VoxelMapTransformer()) \
   .last().data)

# then we add matches to the map
drawing.add(matches)

drawing.to_svg("assets/embedding_focused.svg")
```

```results
Stream("color_image_embedded") | vector_search(k=30)
13:15:15.190 [inf][dimos/mapping/voxels.py       ] VoxelGrid using device: CUDA:0
```

![output](assets/embedding_focused.svg)

<details>
<summary>Python</summary>

```python title="Python" fold session=mem
import matplotlib
import matplotlib.pyplot as plt
import math

def plot_mosaic(frames, path, cols=5):
    matplotlib.use("Agg")
    rows = math.ceil(len(frames) / cols)
    aspect = frames[0].width / frames[0].height
    fig_w, fig_h = 12, 12 * rows / (cols * aspect)

    fig, axes = plt.subplots(rows, cols, figsize=(fig_w, fig_h))
    fig.patch.set_facecolor("black")
    for i, ax in enumerate(axes.flat):
        if i < len(frames):
            ax.imshow(frames[i].data)
            for spine in ax.spines.values():
                spine.set_color("black")
                spine.set_linewidth(0)
            ax.set_xticks([])
            ax.set_yticks([])
        else:
            ax.axis("off")
    plt.subplots_adjust(wspace=0.02, hspace=0.02, left=0, right=1, top=1, bottom=0)
    plt.savefig(path, facecolor="black", dpi=100, bbox_inches="tight", pad_inches=0)
    plt.close()

```

</details>

let's view those images

```python title="Python" session=mem
plot_mosaic(matches.map(lambda obs: obs.data).to_list(), "assets/grid.png")
```

![output](assets/grid.png)

## H.264 image storage

memory2 stores `Image` streams with the default JPEG image codec unless a stream
opts into H.264. Use H.264 storage for high-rate camera streams when disk usage
matters and frame-to-frame compression is worth the dependency cost.

```python skip
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.video.h264 import H264ImageStorageConfig
from dimos.msgs.sensor_msgs.Image import Image
from dimos.protocol.video.h264 import H264Config

store = SqliteStore(path="robot_video.db")
color = store.stream(
    "color_image",
    Image,
    image_storage=H264ImageStorageConfig(
        codec=H264Config(bitrate=2_000_000, keyframe_interval=30),
    ),
)
```

Recorders can configure the same setting per input stream:

```python skip
from dimos.memory2.module import Recorder
from dimos.memory2.video.h264 import H264ImageStorageConfig
from dimos.protocol.video.h264 import H264Config

recorder = Recorder.blueprint(
    db_path="robot_video.db",
    image_storage={
        "color_image": H264ImageStorageConfig(
            codec=H264Config(bitrate=2_000_000, keyframe_interval=30),
        )
    },
)
```

H.264 storage keeps the normal memory2 shape: one observation row per source
frame. The blob for that observation stores one serialized video packet whose
payload is a complete H.264 Annex B access unit, not individual RTP fragments.
The store also writes GOP metadata so lazy decode and replay can start at the
nearest prior keyframe.

Metadata queries do not decode pixels. You can inspect timestamps, poses, tags,
frame ids, and dimensions without paying decode cost. Accessing `obs.data`
decodes lazily from the nearest usable keyframe through the requested frame and
returns a normal `Image`. Replay emits decoded `Image` values in timestamp order.

H.264 storage currently supports uint8 RGB, BGR, and grayscale images. It raises
an explicit error for depth images, 16-bit images, alpha formats, and other
unsupported pixel layouts. The default `store.stream("color_image", Image)` path
continues to use JPEG.

### Synthetic H.264 QA blueprint

The `demo-h264-video-e2e` blueprint exercises both live H.264 LCM transport and
H.264 memory2 storage without a robot or physical camera:

```bash skip
dimos run demo-h264-video-e2e --daemon
dimos log -f
```

The blueprint publishes deterministic synthetic RGB frames, records them to
`h264_video_e2e.db`, and runs a probe that logs decoded-frame count, dimensions,
timestamp monotonicity, frame id stability, and validation failures. Use it after
codec or storage changes to inspect:

- logs from the source, recorder, and probe;
- memory2 metadata queries that do not touch `obs.data`;
- lazy `obs.data` decode for both keyframe and mid-GOP observations;
- replay of the recorded stream; and
- sequence-gap behavior, if you inject packet loss in the transport tests.
