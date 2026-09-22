# Point-cloud agent API

Lets an agent measure a `PointCloud2` in Python instead of reading raw points: filters,
nearest returns and sweeps with shapes, top-down grids that combine into masks and
regions, and camera and top-down images whose pixels pick back to stored returns.

```python
import numpy as np

from dimos.experimental.agent_encode.pointcloud.grid.count import Count
from dimos.experimental.agent_encode.pointcloud.queries.nearest import Nearest
from dimos.experimental.agent_encode.pointcloud.queries.select import Select
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

cloud = PointCloud2.from_numpy(
    np.array([[0.5, 0.0, 0.5], [1.0, 0.2, 0.8], [0.0, 0.0, 0.0]], dtype=np.float32),
    frame_id="map",
    timestamp=0.0,
)
obstacles = Select(z=(0.15, 1.0)).run(cloud)
print(Nearest(to=(0.0, 0.0)).run(obstacles))
print(Count(0.25).run(obstacles) > 0)
```

Every operation is a frozen dataclass of its parameters with an eager
`run(cloud)`. Coordinates and heights are in the cloud's frame, in metres. No
returns, and NaN in a grid, mean unobserved, never free. Images are written under
`out_dir=`, else the run's log directory, else the DimOS state directory.

## Default summary

`cloud.agent_encode()` takes no parameters and returns the cloud's frame, timestamp,
size, bounds and centroid, and `overview`: roughly how much area was observed, where
structure stands at body height, and where the lower surface rises or drops. It is
`Overview().run(cloud)` as JSON, built from `Spacing`, `Count`, `ZPercentile`,
`Select`, the grid operators and `regions(gap=, measure=)`, so every number in it can
be reproduced with explicit calls. It measures and does not interpret: regions are
never labelled as rooms or objects.

## What the agent reads

`legend()` in [`legend.py`](/dimos/experimental/agent_encode/pointcloud/legend.py) holds
the agent's instructions: the import lines, conventions, a table with a sentence or two
per class and method, and a worked example. It is served as
`PointCloud2.agent_encode_legend()`. Edit it when the API changes; its code must run.

## Layout

| Path | Contents |
|---|---|
| `overview.py` | `Overview`, the no-argument summary, and `encode`, what `agent_encode()` returns. |
| `legend.py` | The agent instructions. |
| `constants.py` | The grid cell limit, read by more than one module. |
| `shapes/` | `Box` and `Cylinder`, each a `Shape` from `base.py`. |
| `queries/` | `Select`, `Bounds`, `Nearest`, `Sweep`, `Spacing`, each a `Query` from `base.py`. |
| `grid/` | `Grid` (`base.py`) and its constructors `Count`, `ZMin`, `ZMax`, `ZPercentile`, `Occupancy`; `regions.py` groups masks into `Regions`. |
| `image/` | `CameraView` and `grid.image()`, images (`Image` in `base.py`) that map pixels to the world and pick returns; `lib/` rasterises and draws (`Line`, `Arrow`). |

## Testing

```bash
uv run pytest dimos/experimental/agent_encode
```

Tests sit beside the code and use synthetic clouds; they need no robot or LLM API key.
