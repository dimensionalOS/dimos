# Point-cloud agent encoding

Lets an agent ask questions of a `PointCloud2` instead of reading raw points:
geometric queries (overlap, closest, sweep), grid fields (height, distance,
masks, regions), renders (depth view, occupancy map, field map) and picking
pixels in those renders back to stored returns.

```python
import numpy as np

from dimos.experimental.agent_encode.pointcloud import api as pc
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

cloud = PointCloud2.from_numpy(
    np.array([[0.5, 0.0, 0.5], [1.0, 0.2, 0.8], [0.0, 0.0, 0.0]], dtype=np.float32),
    frame_id="map",
    timestamp=0.0,
)
out = cloud.agent_encode({
    "near": pc.Closest(pc.Cylinder(center=(0, 0), radius=0.3, z_range=(0.15, 1.0))),
    "map": pc.OccupancyMap(z_range=(0.15, 1.0)),
})
print(out)
```

Coordinates and height bands use the cloud's frame, with distances in meters.
Render results include paths to generated images; pass `out_dir` to
`agent_encode()` to choose where they are written.

## Default summary

`cloud.agent_encode()` can be called without parameters and defaults to a sensible
overview of the cloud: roughly how much area was observed, where structure stands
at body height, and where the ground rises or drops, as a few compact numbers.

Use `pc.Overview(...)` in an explicit named request to customize the recipe, or
`cloud.agent_encode({})` for metadata only. The recipe is a composition of
`HeightField`, `.percentile()`, `Select`, `Threshold` and `Components`, so every
number in it can be reproduced with explicit calls; it contains no doorway detector.
`Components(mask, values=field, max_regions=n)` is the general form of its region
lists: per-region statistics of any field, with a bounded table.

Design rules:

- Explicit requests choose their poses, sizes and bands; the default recipe reports its choices.
- It measures, it does not interpret. No returns never means free space, and
  regions are never labelled as rooms or objects.
- One call evaluates shared dependencies once. Results are named, and failures
  are contained per result.
- Each request returns a frozen dataclass of what it measured; `dispatch.py` turns
  results into JSON in one place, beside an echo of the request.
- JSON responses are capped by a byte budget; the budget never lowers resolution.

## What the agent reads

`legend()` in [`runtime/dispatch.py`](/dimos/experimental/agent_encode/pointcloud/runtime/dispatch.py) holds the agent's instructions: conventions, an API table
with a sentence or two on how each entry is used, and one example. It is served as
`PointCloud2.agent_encode_legend()`. Edit it when the API changes.

## Layout

| Path | Contents |
|---|---|
| `api.py` | The request classes agents import. |
| `constants.py` | The grid cell limit, read by more than one module. |
| `fields.py` | `Select`, `Band`, `Grid`, `HeightField`, `Percentile`, `DistanceField`, `Threshold`, `Resample`, `Components`, the mask operators, and the computed `FieldData`, `Mask`, `Labels` and `Distances`. |
| `shapes/` | `Box`, `Cylinder`, `Sphere`, each a `Shape` from `base.py`. |
| `handlers/` | One output per file: `closest`, `overlap`, `sweep`, `depth_view`, `occupancy_map`, `field_outputs` (`Sample`, `Window`, `Map`), `pick`, and `overview`, the no-argument default recipe; `lib/surface.py` is what `Pick` measures on a render. |
| `render/` | Rasterising (`raster.py`) and overlays. |
| `runtime/` | `dispatch.py` (the `encode` entry point, budgets, and `legend()`, the agent instructions), the per-call context, and `recipe.py`, which describes requests as JSON. |
| `tests/` | Unit tests. |

## Testing

```bash
uv run pytest dimos/experimental/agent_encode/pointcloud/tests
```

The tests use synthetic point clouds to check geometry, field operations,
rendering, pixel picking, and response budgets. They run without robot hardware
or an LLM API key.
