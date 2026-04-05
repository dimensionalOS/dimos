
```python skip
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.data import get_data
from dimos.memory2.store.sqlite import SqliteStore


store = SqliteStore(path=get_data("go2_bigoffice.db"))

print(store.streams.color_image)

```

<!--Result:-->
```
Stream("color_image")
```

```python
     # Downsample to 2Hz, then embed
        pipeline = (
            video.filter(lambda obs: obs.data.brightness > 0.1)
            .transform(QualityWindow(lambda img: img.sharpness, window=0.5))
            .transform(EmbedImages(clip))
            .save(embedded)
        )



```

<!--Error:-->
```
File "/tmp/tmpmfb4vuia.py", line 2
    pipeline = (
IndentationError: unexpected indent

Exit code: 1
```

```python
import pickle
from dimos.mapping.pointclouds.occupancy import general_occupancy, simple_occupancy, height_cost_occupancy
from dimos.mapping.occupancy.inflation import simple_inflate
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.vis.drawing.drawing import Drawing2D
from dimos.utils.data import get_data
from dimos.memory2.vis.type import Point
from dimos.models.embedding.clip import CLIPModel

clip = CLIPModel()

#global_map = pickle.loads(get_data("unitree_go2_bigoffice_map.pickle").read_bytes())
#drawing = Drawing2D()
#costmap = simple_inflate(general_occupancy(global_map), 0.05)
#drawing.add(costmap)

store = SqliteStore(path=get_data("go2_bigoffice.db"))

store.streams.color_image \
.filter(lambda obs: obs.data.brightness > 0.1) \
.map(drawing.add)

drawing.to_svg("assets/test.svg")
```


![output](assets/test.svg)
