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

Design rules:

- The caller picks every pose, size and band. No robot defaults.
- It measures, it does not interpret. No returns never means free space, and
  regions are never labelled as rooms or objects.
- One call evaluates shared dependencies once. Results are named, and failures
  are contained per result.
- JSON responses are capped by a byte budget; the budget never lowers resolution.

## What the agent reads

`legend()` in [`runtime/dispatch.py`](/dimos/experimental/agent_encode/pointcloud/runtime/dispatch.py#L273) holds the agent's instructions: API table, rules,
conventions and worked examples. It is served as `PointCloud2.AGENT_ENCODE_LEGEND`.
Edit it when the API changes.

## Layout

| Path | Contents |
|---|---|
| `fields.py` | `Select`, `Band`, `Grid`, `HeightField`, `DistanceField`, `Threshold`, `Resample`, `Components`. |
| `shapes/` | `Box`, `Cylinder`, `Sphere`. |
| `handlers/` | One output per file: `closest`, `overlap`, `sweep`, `depth_view`, `occupancy_map`, `field_outputs` (`Sample`, `Window`, `Map`), `pick`. |
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
