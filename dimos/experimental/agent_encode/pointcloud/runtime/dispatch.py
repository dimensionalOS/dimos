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

"""``agent_encode(*handlers)``: run each requested render or query against the
cloud and return their results in the order asked, plus the legend that
describes every handler and shape the build offers."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, fields, is_dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.experimental.agent_encode.pointcloud import constants
from dimos.experimental.agent_encode.pointcloud.fields import FieldData
from dimos.experimental.agent_encode.pointcloud.handlers.overview import Overview
from dimos.experimental.agent_encode.pointcloud.render import raster as render
from dimos.experimental.agent_encode.pointcloud.runtime.context import EncodeContext

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


def _request_spec(value: Any, _depth: int = 0) -> Any:
    """Return a deterministic JSON-safe description without executing a request."""
    if _depth > 96:
        return {"invalid_number": "request nesting exceeds 96 levels"}
    if is_dataclass(value) and not isinstance(value, type):
        return {
            "type": type(value).__name__,
            **{f.name: _request_spec(getattr(value, f.name), _depth + 1) for f in fields(value)},
        }
    if isinstance(value, Mapping):
        if all(isinstance(key, str) for key in value):
            return {key: _request_spec(item, _depth + 1) for key, item in value.items()}
        return {
            "type": type(value).__name__,
            "items": [[_request_spec(key), _request_spec(item)] for key, item in value.items()],
        }
    if isinstance(value, np.ndarray):
        return _request_spec(value.tolist())
    if isinstance(value, np.generic):
        return _request_spec(value.item())
    if isinstance(value, (tuple, list)):
        return [_request_spec(item, _depth + 1) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return {"invalid_number": "nan" if math.isnan(value) else ("inf" if value > 0 else "-inf")}
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return {"type": type(value).__name__}


def _contains_invalid_number(value: Any) -> bool:
    if isinstance(value, Mapping):
        return "invalid_number" in value or any(
            _contains_invalid_number(item) for item in value.values()
        )
    if isinstance(value, list):
        return any(_contains_invalid_number(item) for item in value)
    return False


def _json_safe(value: Any) -> Any:
    """Normalize handler output to values accepted by strict JSON serialization."""
    if isinstance(value, Mapping):
        if any(not isinstance(key, str) for key in value):
            raise TypeError("JSON result keys must be strings")
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, np.ndarray):
        return _json_safe(value.tolist())
    if isinstance(value, np.generic):
        return _json_safe(value.item())
    if isinstance(value, (tuple, list)):
        return [_json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError("result contains a non-finite number")
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    raise TypeError(f"result contains non-JSON value {type(value).__name__}")


def _size(value: Any) -> int:
    return len(json.dumps(value, allow_nan=False, ensure_ascii=True).encode())


def _billed(out: dict[str, Any]) -> int:
    """Response size without the ``request`` echoes, which only restate the caller's input."""
    results = {
        name: {k: v for k, v in value.items() if k != "request"}
        for name, value in out["results"].items()
    }
    return _size({**out, "results": results})


def _stem(cloud: PointCloud2, points: np.ndarray, requests: Any) -> str:
    digest = hashlib.sha256()
    digest.update(np.ascontiguousarray(points, dtype="<f4").tobytes())
    digest.update(
        json.dumps(
            {
                "frame_id": cloud.frame_id,
                "ts": _request_spec(cloud.ts),
                "form": constants.FORM,
                "requests": _request_spec(requests),
            },
            ensure_ascii=True,
            sort_keys=True,
            separators=(",", ":"),
        ).encode()
    )
    return f"pointcloud_{digest.hexdigest()[:24]}"


def _centroid(points: np.ndarray) -> list[float] | None:
    return [round(float(v), 3) for v in points.mean(axis=0)] if len(points) else None


def _bounds(points: np.ndarray) -> list[list[float]] | None:
    return (
        [
            [round(float(v), 3) for v in points.min(axis=0)],
            [round(float(v), 3) for v in points.max(axis=0)],
        ]
        if len(points)
        else None
    )


def _terminal_too_large(
    out: dict[str, Any], budget: EncodeBudget, response_bytes: int
) -> dict[str, Any]:
    minimum = {
        "schema": out["schema"],
        "frame_id": out["frame_id"],
        "ts": out["ts"],
        "num_points": out["num_points"],
        "bounds_m": out["bounds_m"],
        "centroid_m": out["centroid_m"],
        "form": out["form"],
        "status": "too_large",
        "results": {},
    }
    terminal: dict[str, Any] = {
        "schema": out["schema"],
        "status": "too_large",
        "response_bytes": response_bytes,
        "minimum_envelope_bytes": _size(minimum),
        "max_text_bytes": budget.max_text_bytes,
        "output_count": len(out["results"]),
        "output_names": [],
        "suggestion": "use shorter metadata/output names or split the named request",
    }
    omitted = 0
    for name in out["results"]:
        candidate = {**terminal, "output_names": [*terminal["output_names"], name]}
        if _size(candidate) <= budget.max_text_bytes:
            terminal = candidate
        else:
            omitted += 1
    if omitted:
        candidate = {**terminal, "output_names_omitted": omitted}
        if _size(candidate) <= budget.max_text_bytes:
            terminal = candidate
    if _size(terminal) > budget.max_text_bytes:
        terminal = {"schema": out["schema"], "status": "too_large"}
    return terminal


def _named(ctx: EncodeContext, requests: Mapping[str, Any], budget: EncodeBudget) -> dict[str, Any]:
    if any(not isinstance(name, str) or not name for name in requests):
        raise ValueError("output names must be nonempty strings")
    metadata_errors: dict[str, str] = {}
    try:
        timestamp = _json_safe(ctx.cloud.ts)
    except (ValueError, TypeError) as exc:
        timestamp = None
        metadata_errors["ts"] = str(exc)
    out: dict[str, Any] = {
        "schema": "pointcloud.encode/v2",
        "frame_id": ctx.cloud.frame_id,
        "ts": timestamp,
        "num_points": len(ctx.points),
        "bounds_m": _bounds(ctx.points),
        "centroid_m": _centroid(ctx.points),
        "form": constants.FORM,
        "results": {},
    }
    if metadata_errors:
        out["metadata_errors"] = metadata_errors
    for name, node in requests.items():
        request = _request_spec(node)
        try:
            if _contains_invalid_number(request):
                raise ValueError("request contains a non-finite number")
            result = ctx.evaluate(node)
            if isinstance(result, FieldData):
                result = {
                    "grid": result.grid.describe(ctx),
                    "kind": result.kind,
                    "channels": list(result.values),
                    **result.metadata,
                }
            if not isinstance(result, dict):
                raise TypeError("named outputs must be measurements or renders, not selections")
            result = _json_safe(result)
            measurement_status = result.pop("status", None)
            out["results"][name] = {
                "status": "ok",
                **result,
                **({"measurement_status": measurement_status} if measurement_status else {}),
                "request": request,
            }
        except (ValueError, TypeError, OverflowError, OSError, RuntimeError) as exc:
            out["results"][name] = {
                "status": "invalid",
                "error": str(exc),
                "error_type": type(exc).__name__,
                "request": request,
            }

    while (response_bytes := _billed(out)) > budget.max_text_bytes:
        candidates = [
            (_size({k: v for k, v in value.items() if k != "request"}), name)
            for name, value in out["results"].items()
            if value.get("status") != "too_large"
        ]
        if not candidates:
            return _terminal_too_large(out, budget, response_bytes)
        result_bytes, name = max(candidates)
        out["results"][name] = {
            "status": "too_large",
            "result_bytes": result_bytes,
            "response_bytes": response_bytes,
            "max_text_bytes": budget.max_text_bytes,
            "suggestion": "retry this output with a small Sample first, or a smaller Window; measurement resolution is unchanged",
        }
    return out


def legend() -> str:
    """The agent guide, served as ``PointCloud2.AGENT_ENCODE_LEGEND``; text builds
    append their grid key."""
    text = (
        r"""
# Point-cloud agent encoding

```python
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2 as P
out = cloud.agent_encode({"name": query, ...})   # one call computes shared work once
r = out["results"]["name"]                        # read r["status"] first
```

`out` holds `frame_id`, `ts`, `num_points`, `bounds_m` (`[[x,y,z]min, [x,y,z]max]`,
rounded to 0.001 m), `centroid_m` (mean of all returns), `form` and `results`.
`cloud.agent_encode()` with no query returns `results["overview"]`, a compact first
look (see `Overview`); `cloud.agent_encode({})` returns the envelope only.

## Conventions

- Metres in the cloud's frame: x east, y north, z up; every z is absolute. Yaw 0 faces
  +x and turns toward +y; pitch up is positive.
- `r["status"]` is `ok`, `invalid` (`r["error"]` says what to change) or `too_large`
  (a response holds 16 KB: compute on the grid and read only the answer, never page
  through rows). The geometric outcome is separate: `r["measurement_status"]`.
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
| render | `Map(field, value_range=None, overlays=(), max_side=1024)` | An image of one channel, mask or label grid (`Map(h.max)`, not `Map(h)`) with its colour scale and pixel transform. |
| render | `DepthView(view=(x,y,z,yaw_deg,pitch_deg), fov_deg=90, size=(768,480), max_depth=None, point_size_m=None, source=None, overlays=())` | A perspective depth image; `colour` gives the log depth scale and its stops. |
| render | `OccupancyMap(z_range=(lo,hi), max_cells=256, zoom=None or (x,y,r), mark=None or (x,y,yaw_deg), free_radius=0, colour="flat" or "height", grid=None, source=None, overlays=())` | A top-down image. Returns below `z_range` mark a cell free (white), inside it occupied (black, or coloured by height), none unseen (grey); grey lines every metre, `mark` in red. `origin_xy` is the south-west corner and `cell_m` the cell used. |
| pick | `Pick(view_ref, uv=(u,v) or rect=(u,v,w,h) or polygon=, radius_px=0, max_items=16)` | What lies under pixels of a render you have looked at: depth `hits` with `point_m`, or map `cells` with counts and heights. `measurement_status` is `hit`, `ambiguous`, `no_return` or `outside_image`; `.selection` is a source. |
| pick | `SelectionRef(selection_ref)` | A pick's returns as a source in a later call on the same cloud. |

Renders return `image` and a `view_ref`. `overlays=` draws shapes, query results or
`{"segment_m": [[x,y,z], ...]}` paths on them.

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
grid = P.Grid(origin=(-4, -4), shape=(80, 80), cell_m=0.1)
body = P.Cylinder(center=(0, 0), radius=0.35, z_range=(None, None))
obstacles = P.Select(include=P.Band("z", 0.15, 1.0), exclude=(body,))
blocked = P.Threshold(P.HeightField(grid, source=obstacles).count, ">", 0)
free = P.Threshold(P.DistanceField(grid, source=blocked), ">", 0.3)   # 0.3 m from anything
out = cloud.agent_encode({
    "near": P.Closest(P.Cylinder((0, 0), 0, (0.15, 1.0)), source=obstacles),
    "ahead": P.Sweep(P.Box((0.5, 0, 0.55), (0.1, 0.6, 0.8)), direction_deg=0, max_distance=2.0),
    "joined": P.Sample(P.Components(free), at=[(0, 0), (2.5, 1.0)], radius=0.15),
    "map": P.OccupancyMap(z_range=(0.15, 1.0), mark=(0, 0, 0)),
})
for name, r in out["results"].items():
    print(name, r["status"], r.get("error"))
```
""".strip()
        + "\n"
    )
    if constants.FORM != "image":
        text += (
            "\nText forms are ascii grids. depth ascii: rows top-to-bottom, columns "
            "left-to-right, digit 0..9 = near..far by ascii_formula, '.' = no return (depth "
            "unknown, treat as infinity). occupancy ascii: rows run north (top) to south, "
            "columns west to east, '#' occupied (colour='flat') or digit 0..9 = z_low..z_high "
            "of the highest return (colour='height'), '.' free, '?' unseen, the mark is drawn "
            "as ^ > v < for north east south west when given; mark_cell is (column, row) in "
            "the ascii. "
        )
    return text


def encode(
    cloud: PointCloud2,
    *handlers: Any,
    out_dir: str | Path | None = None,
    budget: EncodeBudget | None = None,
) -> dict[str, Any]:
    points = np.asarray(cloud.points_f32(), dtype=np.float32).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    stem = _stem(cloud, points, handlers)
    ctx = EncodeContext(cloud=cloud, points=points, out_dir=render.output_dir(out_dir), stem=stem)
    if not handlers:
        return _named(ctx, {"overview": Overview()}, budget or EncodeBudget())
    if len(handlers) == 1 and isinstance(handlers[0], Mapping):
        return _named(ctx, handlers[0], budget or EncodeBudget())
    if budget is not None:
        raise ValueError("budget applies to the named mapping API")
    results = []
    for i, handler in enumerate(handlers):
        if not hasattr(handler, "run"):
            raise TypeError(
                f"handler {i} is {type(handler).__name__}, not a query such as P.Overlap(...)"
            )
        sub = EncodeContext(
            cloud=ctx.cloud,
            points=ctx.points,
            out_dir=ctx.out_dir,
            stem=f"{stem}_{i}",
            cache=ctx.cache,
        )
        results.append(handler.run(sub))
    return {
        "frame_id": cloud.frame_id,
        "ts": cloud.ts,
        "num_points": len(points),
        "bounds_m": _bounds(points),
        "centroid_m": _centroid(points),
        "form": constants.FORM,
        "results": results,
    }
