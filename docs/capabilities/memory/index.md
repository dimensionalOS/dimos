# Spatial Memory

This walkthrough creates a small synthetic CDR recording locally. Its image,
pose and cloud examples need no recording download. Semantic search additionally
loads the CLIP model and builds embeddings from these sample images; the results
demonstrate the API, not evidence from a real office.

<details>
<summary>Python</summary>

```python title="Python" fold session=mem output=none
import pickle
from dimos.mapping.pointclouds.occupancy import general_occupancy, simple_occupancy, height_cost_occupancy
from dimos.mapping.occupancy.inflation import simple_inflate
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.vis.color import Color
from dimos.memory.transform import downsample, throttle, speed, smooth
from dimos.memory.vis.space.space import Space
from pathlib import Path
from tempfile import TemporaryDirectory
from dimos.memory.demo_data import write_demo_recording
from dimos.msgs.image import image_brightness, image_sharpness, image_view, image_to_rgb
from dimos.memory.vis.space.elements import Point
```

</details>

we init our recording, investigate available streams

```python title="Python" session=mem
demo_directory = TemporaryDirectory()
store = write_demo_recording(Path(demo_directory.name) / "recording.db")

for name, stream in store.streams.items():
   print(stream.summary())
```

```results
Stream("color_image"): 720 items, 2023-11-14 22:13:20 to 2023-11-14 22:16:19 (179.8s, 4.00 Hz, 94.22 KiB)
Stream("lidar"): 720 items, 2023-11-14 22:13:20 to 2023-11-14 22:16:19 (179.8s, 4.00 Hz, 1.17 MiB)
Stream("odom"): 720 items, 2023-11-14 22:13:20 to 2023-11-14 22:16:19 (179.8s, 4.00 Hz, 55.53 KiB)
```

Any stream is drawable

```python title="Python" session=mem output=none
global_map = store.streams.lidar.first().data

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
  .map(lambda obs: obs.derive(data=image_brightness(obs.data))))

drawing.to_svg("assets/brightness.svg")
```

![output](assets/brightness.svg)

Embeddings require an explicit model. This optional larger indexing example
is not executed as part of the walkthrough:

```python title="Python" session=mem skip
from dimos.models.embedding.clip import CLIPModel
from dimos_generated.sensor_msgs.msg import Image
from dimos.memory.transform import QualityWindow
from dimos.memory.embed import EmbedImages

embedded = store.stream("color_image_embedded", Image)
clip = CLIPModel()

# Downsample to 2Hz, filter dark images, then embed
pipeline = (
    store.streams.color_image.filter(lambda obs: image_brightness(obs.data) > 0.1)
    .transform(QualityWindow(lambda img: image_sharpness(img), window=0.5))
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

clip = CLIPModel(device="cpu")
from dimos.memory.embed import EmbedImages
from dimos_generated.sensor_msgs.msg import Image

embedded = store.stream("color_image_embedded", Image, codec="lz4+cdr")
# save() is lazy: iterate to populate the index before searching it.
list(store.streams.color_image.transform(throttle(2.0)).transform(EmbedImages(clip)).save(embedded))
search_vector = clip.embed_text("shop")
drawing.add(store.streams.color_image_embedded.search(search_vector))

drawing.to_svg("assets/embedding.svg")
```

![output](assets/embedding.svg)

We don't really have to deal with the whole global map actually, let's get top 10 embeddings, and render only lidar around those.

```python title="Python" session=mem output=none
from dimos.models.embedding.clip import CLIPModel
from dimos.mapping.voxels.module import VoxelMapTransformer
drawing = Space()

# this is defined here, but not executed
matches = store.streams.color_image_embedded.search(search_vector, k=30)

print(matches) # Stream("color_image_embedded") | vector_search(k=50)

# here we execute it once, and feed it into a global mapper, then draw the map
drawing.add(
   matches.map(lambda obs: store.streams.lidar.at(obs.ts).last()) \
   .transform(VoxelMapTransformer(device="CPU:0")) \
   .last().data)

# then we add matches to the map
drawing.add(matches)

drawing.to_svg("assets/embedding_focused.svg")
```

```results
Stream("color_image_embedded") | vector_search(k=30)
16:24:39.279 [inf][dimos/mapping/voxels/grid.py  ] VoxelGrid using device: CPU:0 (packed-numpy)
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
            ax.imshow(image_to_rgb(frames[i]))
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
