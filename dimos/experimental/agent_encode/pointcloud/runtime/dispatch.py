# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""``agent_encode({name: request})``: run each named render or query against the
cloud and return its result under that name, plus the legend that describes
every handler and shape."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, fields, replace
import hashlib
import json
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
from pydantic import JsonValue
from pydantic_core import to_jsonable_python

from dimos.experimental.agent_encode.pointcloud.handlers.overview import Overview
from dimos.experimental.agent_encode.pointcloud.render import raster as render
from dimos.experimental.agent_encode.pointcloud.runtime.context import (
    EncodeContext,
    Request,
    Result,
)
from dimos.experimental.agent_encode.pointcloud.runtime.recipe import canonical, describe

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@dataclass(frozen=True)
class EncodeBudget:
    """Bounds a named request's response; geometry is never coarsened."""

    max_text_bytes: int = 16384
    """Maximum JSON bytes returned."""

    def __post_init__(self) -> None:
        if type(self.max_text_bytes) is not int or self.max_text_bytes < 1024:
            raise ValueError("max_text_bytes must be an integer >= 1024")


def _contains_invalid_number(value: JsonValue) -> bool:
    if isinstance(value, Mapping):
        return "invalid_number" in value or any(
            _contains_invalid_number(item) for item in value.values()
        )
    if isinstance(value, list):
        return any(_contains_invalid_number(item) for item in value)
    return False


def _json(value: object) -> JsonValue:
    """``value`` as strict JSON: dataclasses become objects of their fields, and a
    non-finite number is a ValueError."""
    converted: JsonValue = to_jsonable_python(value, fallback=lambda array: array.tolist())
    json.dumps(converted, allow_nan=False)
    return converted


def _size(value: JsonValue) -> int:
    return len(json.dumps(value, allow_nan=False, ensure_ascii=True).encode())


def _envelope(
    header: dict[str, JsonValue], results: dict[str, dict[str, JsonValue]]
) -> dict[str, JsonValue]:
    listed: dict[str, JsonValue] = {**results}
    return {**header, "results": listed}


def _unechoed(result: dict[str, JsonValue]) -> dict[str, JsonValue]:
    """A result without its ``request`` echo, which only restates the caller's input."""
    return {k: v for k, v in result.items() if k != "request"}


def _centroid(points: np.ndarray) -> JsonValue:
    return [round(float(v), 3) for v in points.mean(axis=0)] if len(points) else None


def _bounds(points: np.ndarray) -> JsonValue:
    return (
        [
            [round(float(v), 3) for v in points.min(axis=0)],
            [round(float(v), 3) for v in points.max(axis=0)],
        ]
        if len(points)
        else None
    )


def _terminal_too_large(
    header: dict[str, JsonValue], names: list[str], budget: EncodeBudget, response_bytes: int
) -> dict[str, JsonValue]:
    minimum = _envelope({**header, "status": "too_large"}, {})
    terminal: dict[str, JsonValue] = {
        "schema": header["schema"],
        "status": "too_large",
        "response_bytes": response_bytes,
        "minimum_envelope_bytes": _size(minimum),
        "max_text_bytes": budget.max_text_bytes,
        "output_count": len(names),
        "output_names": [],
        "suggestion": "use shorter metadata/output names or split the named request",
    }
    listed: list[JsonValue] = []
    omitted = 0
    for name in names:
        candidate = {**terminal, "output_names": [*listed, name]}
        if _size(candidate) <= budget.max_text_bytes:
            terminal, listed = candidate, [*listed, name]
        else:
            omitted += 1
    if omitted:
        candidate = {**terminal, "output_names_omitted": omitted}
        if _size(candidate) <= budget.max_text_bytes:
            terminal = candidate
    if _size(terminal) > budget.max_text_bytes:
        terminal = {"schema": header["schema"], "status": "too_large"}
    return terminal


def _named(
    ctx: EncodeContext, requests: Mapping[str, Request[Result]], budget: EncodeBudget
) -> dict[str, JsonValue]:
    if any(not isinstance(name, str) or not name for name in requests):
        raise ValueError("output names must be nonempty strings")
    metadata_errors: dict[str, JsonValue] = {}
    try:
        timestamp = _json(ctx.cloud.ts)
    except (ValueError, TypeError) as exc:
        timestamp = None
        metadata_errors["ts"] = str(exc)
    header: dict[str, JsonValue] = {
        "schema": "pointcloud.encode/v2",
        "frame_id": ctx.cloud.frame_id,
        "ts": timestamp,
        "num_points": len(ctx.points),
        "bounds_m": _bounds(ctx.points),
        "centroid_m": _centroid(ctx.points),
    }
    if metadata_errors:
        header["metadata_errors"] = metadata_errors
    results: dict[str, dict[str, JsonValue]] = {}
    for name, node in requests.items():
        request = describe(node)
        try:
            if _contains_invalid_number(request):
                raise ValueError("request contains a non-finite number")
            if not isinstance(node, Request):
                raise TypeError(
                    f"{type(node).__name__} is not a request; selections, shapes and grids "
                    "go inside one, as in Overlap(Box(...))"
                )
            result = ctx.evaluate(node).summary()
            measured = {f.name: _json(getattr(result, f.name)) for f in fields(result)}
            results[name] = {"status": "ok", **measured, "request": request}
        except (ValueError, TypeError, OverflowError, OSError, RuntimeError) as exc:
            results[name] = {
                "status": "invalid",
                "error": str(exc),
                "error_type": type(exc).__name__,
                "request": request,
            }

    unechoed = {name: _unechoed(result) for name, result in results.items()}
    while (response_bytes := _size(_envelope(header, unechoed))) > budget.max_text_bytes:
        candidates = [
            (_size(value), name)
            for name, value in unechoed.items()
            if value.get("status") != "too_large"
        ]
        if not candidates:
            return _terminal_too_large(header, list(results), budget, response_bytes)
        result_bytes, name = max(candidates)
        results[name] = unechoed[name] = {
            "status": "too_large",
            "result_bytes": result_bytes,
            "response_bytes": response_bytes,
            "max_text_bytes": budget.max_text_bytes,
            "suggestion": "retry this output with a small Sample first, or a smaller Window; measurement resolution is unchanged",
        }
    return _envelope(header, results)


def legend() -> str:
    """The agent guide, served as ``PointCloud2.agent_encode_legend()``."""
    return (
        r"""
# Point-cloud agent encoding

```python
from dimos.experimental.agent_encode.pointcloud import api as pc
out = cloud.agent_encode({"name": query, ...})   # one call computes shared work once
r = out["results"]["name"]                        # read r["status"] first
```

`out` holds `frame_id`, `ts`, `num_points`, `bounds_m` (`[[x,y,z]min, [x,y,z]max]`,
rounded to 0.001 m), `centroid_m` (mean of all returns) and `results`.
`cloud.agent_encode()` with no query returns `results["overview"]`, a compact first
look (see `Overview`); `cloud.agent_encode({})` returns the envelope only.

## Conventions

- Metres in the cloud's frame: x east, y north, z up; every z is absolute. Yaw 0 faces
  +x and turns toward +y; pitch up is positive.
- `r["status"]` is `ok`, `invalid` (`r["error"]` says what to change) or `too_large`
  (a response holds 16 KB: compute on the grid and read only the answer, never page
  through rows). A result holds only what was measured; `r["request"]` restates the
  call.
- No returns is not free space. An empty cell, `count == 0` or `no_return` is unobserved.
- Shapes, selections and fields are lazy recipes; only outputs, queries and renders
  return results. Most take `source=` to restrict the returns they see.
- Grid `shape=(columns, rows)`; values index `[row][column]`; cell centre =
  `origin + ([column, row] + 0.5) * cell_m`; cells are half-open.
- Image pixels are native, origin top-left, `u` right, `v` down. Print `r["image"]`
  and look at it before picking pixels.
- `view_ref`, `selection_ref` and `request` are long recipes: keep them in variables,
  never print or retype them.
- Choose sizes, bands and view heights from `bounds_m` and the overview. Answer only
  from measurements; names of files and streams are not evidence.

## API

| Kind | Call | Use |
|---|---|---|
| shape | `Box(center=(x,y,z), size=(sx,sy,sz), yaw_deg=0)` | Full extents, turned about z. |
| shape | `Cylinder(center=(x,y), radius, z_range=(lo,hi))` | Vertical; `None` = unbounded. `radius=0` is a spot, so distances are from a point; a body-sized radius gives clearance around the body. |
| shape | `Sphere(center=(x,y,z), radius)` | `radius=0` is a point. |
| shape | `Band(axis, low, high)` | A slab along `"x"`, `"y"` or `"z"`; `None` = unbounded. |
| source | `Select(include=shape or tuple, exclude=(...), source=None)` | The returns inside every include and outside every exclude, passed as `source=`. `Select(include=Band("z", 0.15, 1.0), exclude=(body,))` is obstacles without floor, ceiling or the robot itself. |
| query | `Overlap(shape, source=None)` | `count` of returns inside and their `bounds_m`; `.selection` reuses them as a source. Stack them over z slices to see where returns sit. |
| query | `Closest(shape, source=None)` | `distance_m` from the shape's surface to the nearest return (0 when one is inside) and its `point_m`. A Cylinder measures horizontally among returns in its `z_range`. |
| query | `Sweep(shape, direction_deg= or direction=(dx,dy,dz), max_distance=1.0, step_m=0.05, source=None)` | Steps the shape along a heading: `hit`, `distance_m`, `point_m`, `start_inside`. Sweep a body-sized shape at least `step_m` thick, never a thin ray. Returns already inside (`start_inside > 0`) give distance 0: start the shape just ahead of the robot instead of shrinking it. Contacts between steps can be missed. |
| grid | `Grid(origin=(u,v), shape=(cols,rows), cell_m, plane="xy")` | Fixed cells, at most 262144. `plane="xz"` or `"yz"` slices vertically. Cover everything the question could involve: for paths and connectivity that is all of `bounds_m` plus a margin, since a grid cut short closes off the routes that go around. |
| field | `HeightField(grid, source=None)` | Per cell `.count`, `.min`, `.max` of the remaining axis (stored z, not a fitted floor) and `.percentile(q, min_count=4)`. |
| field | `DistanceField(grid, source=None)` | Metres from each cell centre to the nearest target: a mask's true cells, or a selection's returns. This is clearance; no targets gives null. |
| field | `Threshold(field, op, value)` | A mask of 1 / 0 / null (no data) for `>` `>=` `<` `<=` `==` `!=`. Masks combine with `&`, `\|`, `~`, and `a - b` subtracts fields, on identical grids. |
| field | `Resample(field, grid)` | The field on another grid, by containing cell, so fields from different grids combine. |
| field | `Components(mask, connectivity=8, gap_cells=0, values=None, max_regions=None)` | Connected true cells as `regions` with `id`, `cells`, `centroid`, `bounds`. `gap_cells` links cells that far apart, `values=field` adds each region's `min p10 p50 p90 max` of that field, `max_regions=n` lists the n largest and counts the rest. Its labels read like any field: two places are connected when `Sample` finds the same non-zero label at both. Sample each place with a `radius` of about 1.5 cells, because a place's own cell often fails a clearance mask without the place being cut off. |
| output | `Sample(field, at=[(u,v), ...], fields=None, decimals=None, radius=0)` | Values at points, about 170 B each. `radius` summarises a disc: min/max, and the `distinct` values of masks and labels. |
| output | `Window(field, cells=(col,row,w,h), fields=None, decimals=None)` | A block of cells as `values[channel][row][col]`, about 7 B per cell. |
| output | `Overview(max_regions=8, percentile=10, min_count=4, band=(0.15,1.0), relief_m=0.15, cell_spacings=2, relief_cell_spacings=4, relief_gap_cells=1, min_supported_cells=8, min_supported_fraction=0.5)` | The no-argument default, as numbers only. See below. |
| render | `Map(field, value_range=None, overlays=(), max_side=1024)` | An image of one channel, mask or label grid (`Map(h.max)`, not `Map(h)`); grey is no data. The point (a, b) on the grid's plane is at pixel `u = (a - origin[0]) / cell_m * pixels_per_cell - 0.5`, `v = (rows - (b - origin[1]) / cell_m) * pixels_per_cell - 0.5`. |
| render | `DepthView(view=(x,y,z,yaw_deg,pitch_deg), fov_deg=90, size=(768,480), max_depth=None, point_size_m=None, source=None, overlays=())` | A perspective depth image. `colour.stops` give depths from near to far on a log scale; black is no return along that ray, depth unknown. |
| render | `OccupancyMap(z_range=(lo,hi), max_cells=256, zoom=None or (x,y,r), mark=None or (x,y,yaw_deg), free_radius=0, colour="flat" or "height", grid=None, source=None, overlays=())` | A top-down image. Returns below `z_range` mark a cell free (white), inside it occupied (black, or coloured by height), none unseen (grey); grey lines every metre, `mark` in red. `origin_xy` is the south-west corner and `cell_m` the cell used; `height_scale.stops` read a height colour. |
| pick | `Pick(view_ref, uv=(u,v) or rect=(u,v,w,h) or polygon=, radius_px=0, max_items=16)` | What lies under pixels of a render you have looked at: depth `hits` with `point_m`, or map `cells` with the rendered `value` and the returns' `count`, `min_m`, `max_m`. `outcome` is `hit`, `ambiguous`, `no_return`, `outside_image` or `field_value`; `.selection` is a source. |
| pick | `SelectionRef(selection_ref)` | A pick's returns as a source in a later call on the same cloud. |

Renders return `image` and a `view_ref`. `overlays=` draws shapes, `Overlap`, `Closest`,
`Sweep` and `Pick` queries, or `Segment(((x,y,z), ...))` paths on them.

`Overview` sizes its cells from the cloud's own return spacing (`spacing_m`) and reports
the grids it used. `lower_surface` is the per-cell `percentile` of z; its median is
`reference_z_m`, a reference height, not a verified floor. `structure` lists bounds of
connected occupied cells in the band `reference_z_m + band`: they enclose returns, and
are not rooms or solid objects. `relief` lists lower-surface patches further than
`relief_m` from the reference, with `offset_quantiles_m`; a patch of a few cells is
weak evidence, and no patch does not prove level ground. Each list holds the largest
`max_regions` and counts the rest.

## Example

```python
grid = pc.Grid(origin=(-4, -4), shape=(80, 80), cell_m=0.1)
body = pc.Cylinder(center=(0, 0), radius=0.35, z_range=(None, None))
obstacles = pc.Select(include=pc.Band("z", 0.15, 1.0), exclude=(body,))
blocked = pc.Threshold(pc.HeightField(grid, source=obstacles).count, ">", 0)
free = pc.Threshold(pc.DistanceField(grid, source=blocked), ">", 0.3)   # 0.3 m from anything
out = cloud.agent_encode({
    "near": pc.Closest(pc.Cylinder((0, 0), 0, (0.15, 1.0)), source=obstacles),
    "ahead": pc.Sweep(pc.Box((0.5, 0, 0.55), (0.1, 0.6, 0.8)), direction_deg=0, max_distance=2.0),
    "joined": pc.Sample(pc.Components(free), at=[(0, 0), (2.5, 1.0)], radius=0.15),
    "map": pc.OccupancyMap(z_range=(0.15, 1.0), mark=(0, 0, 0)),
})
for name, r in out["results"].items():
    print(name, r["status"], r.get("error"))
```
""".strip()
        + "\n"
    )


def encode(
    cloud: PointCloud2,
    requests: Mapping[str, Request[Result]] | None = None,
    out_dir: str | Path | None = None,
    budget: EncodeBudget | None = None,
) -> dict[str, JsonValue]:
    points = np.asarray(cloud.points_f32(), dtype=np.float32).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    if requests is None:
        requests = {"overview": Overview()}
    if not isinstance(requests, Mapping):
        raise TypeError("agent_encode takes one {name: request} mapping")
    ctx = EncodeContext(cloud, points, render.output_dir(out_dir), "pointcloud")
    digest = hashlib.sha256(f"{ctx.fingerprint}{canonical(describe(requests))}".encode())
    ctx = replace(ctx, stem=f"pointcloud_{digest.hexdigest()[:24]}")
    return _named(ctx, requests, budget or EncodeBudget())
