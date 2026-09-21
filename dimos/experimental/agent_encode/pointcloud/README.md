# Point-cloud agent encoding

Lets an agent ask questions of a `PointCloud2` instead of reading raw points:
geometric queries (overlap, closest, sweep), grid fields (height, distance,
masks, regions), renders (depth view, occupancy map, field map) and picking
pixels in those renders back to stored returns.

```python
import numpy as np

from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2 as P

cloud = P.from_numpy(
    np.array([[0.5, 0.0, 0.5], [1.0, 0.2, 0.8], [0.0, 0.0, 0.0]], dtype=np.float32),
    frame_id="map",
    timestamp=0.0,
)
out = cloud.agent_encode({
    "near": P.Closest(P.Cylinder(center=(0, 0), radius=0.3, z_range=(0.15, 1.0))),
    "map": P.OccupancyMap(z_range=(0.15, 1.0)),
})
print(out)
```

Coordinates and height bands use the cloud's frame, with distances in meters.
Render results include paths to generated images; pass `out_dir` to
`agent_encode()` to choose where they are written.

## Default summary

`cloud.agent_encode()` returns the normal envelope and `results["overview"]`:
observed XY coverage, height-band component bounds, and lower-surface relief.
It returns compact numbers, with at most eight regions in each list and explicit
omitted counts and areas. Images and full grids require explicit requests.

Grids are internal, so their resolution serves the measurement and never the
response size: cells are multiples of the cloud's own return spacing (2 for
coverage and structure, 4 for the lower surface so each percentile pools enough
returns) and are aligned to multiples of the cell size, independent of the cloud's
extent. A stray far return therefore does not change what is measured nearby. Both
grids are reported; the cell grows, with a flag, only if the grid memory limit
requires it. What bounds the response is the region lists.

The reference height is the median of supported per-cell 10th-percentile Z values,
not a verified floor; the default band is 0.15–1.0 m above that reference. Region
bounds enclose observed cells, not solid obstacle volumes. These summaries are
most useful in a Z-up frame. Sparse evidence does not establish level ground or
free space, and areas depend on the reported cell size.

Use `P.Overview(...)` in an explicit named request to customize the recipe, or
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
- JSON responses are capped by a byte budget; the budget never lowers resolution.

## What the agent reads

`legend()` in [`runtime/dispatch.py`](/dimos/experimental/agent_encode/pointcloud/runtime/dispatch.py) holds the agent's instructions: API table, rules,
conventions and worked examples. It is served as `PointCloud2.AGENT_ENCODE_LEGEND`.
Edit it when the API changes.

## Layout

| Path | Contents |
|---|---|
| `constants.py` | The two values more than one module reads: the build form and the grid cell limit. |
| `fields.py` | `Select`, `Band`, `Grid`, `HeightField`, `Percentile`, `DistanceField`, `Threshold`, `Resample`, `Components`. |
| `shapes/` | `Box`, `Cylinder`, `Sphere`. |
| `handlers/` | One output per file: `closest`, `overlap`, `sweep`, `depth_view`, `occupancy_map`, `field_outputs` (`Sample`, `Window`, `Map`), `pick`, and `overview`, the no-argument default recipe. |
| `render/` | Rasterising (`raster.py`) and overlays. |
| `runtime/` | `dispatch.py` (the `encode` entry point, budgets, and `legend()`, the agent instructions) and the per-call context. |
| `tests/` | Unit tests. |

## Testing

```bash
uv run pytest dimos/experimental/agent_encode/pointcloud/tests
```

The tests use synthetic point clouds to check geometry, field operations,
rendering, pixel picking, and response budgets. They run without robot hardware
or an LLM API key.
