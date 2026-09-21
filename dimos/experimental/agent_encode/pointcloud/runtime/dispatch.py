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
out = cloud.agent_encode({"name": query, ...})
r = out["results"]["name"]
```

Public query classes are reachable as `P.<Name>`. Explicit queries choose their
sizes, poses and bands. No-argument calls select the compact Overview recipe below.

Envelope: `schema`, `frame_id`, `ts`, `num_points`, `bounds_m` (return bounds
`[[x,y,z]min, [x,y,z]max]`, rounded to 0.001 m), `centroid_m` (mean `[x,y,z]` of
all returns; the centre of the cloud, unlike the middle of `bounds_m`, which a few
outliers move), `form`, `results`.
Positional form `cloud.agent_encode(h1, h2)` returns `results` as a list in order.
`cloud.agent_encode({})` returns the envelope only.
`cloud.agent_encode()` returns `results["overview"]`: compact numeric coverage,
height-band region bounds, and signed lower-surface relief. No images or grids
are emitted. Print this compact result directly; use explicit queries for details.
`P.Overview()` can also be requested by name to customize its defaults.

Overview sizes its XY cells from the cloud's own return spacing (`spacing_m`, the
median gap between neighbouring returns), never from the cloud's extent: 2 spacings
for coverage and structure, 4 for the lower surface so each cell pools enough returns.
Cells are aligned to multiples of the cell size, so one scene measures the same in
any frame that holds it. The reported grids (`origin`, `shape`, `cell_m`) are
authoritative; `limited_by` appears only if the grid memory limit forced larger cells.
Coverage area is occupied XY cells times cell area, so it depends on `cell_m`.
Lower-surface values are per-cell 10th-percentile Z (linear interpolation, at least
4 returns). Their median is `reference_z_m`, not a verified floor. The structure
band is reference + [0.15, 1.0] m; relief lists patches more than 0.15 m above/below
reference, with `offset_quantiles_m` relative to it. Patches one cell apart
(`relief_gap_cells`) are linked whatever lies between them, and only their own cells
count towards area. With fewer than 8 supported cells, or under half of the observed
cells supported, `status` is `insufficient_support` and both lists are left out.
At most 8 regions per list, largest first; omitted counts/areas are explicit, and a
patch of a few cells is weak evidence. Bounds enclose occupied cells, not solid
obstacles. Use Closest/DistanceField on selected returns for distances. Sparse
support is reported, and no reported patch does not prove level ground. These XY/Z
summaries are most useful in a Z-up cloud frame.

Conventions:

- Metres in the cloud's frame. x east, y north, z up. Every z is absolute.
- Yaw 0 faces +x, positive yaw turns toward +y. Pitch positive looks up.
- Grid `shape=(columns, rows)`; arrays index `[row][column]`;
  cell centre = `origin + ([column, row] + 0.5) * cell_m`. Cells are half-open.
- Image pixels: native size, top-left origin, u right, v down.

## API

| Kind | Name | Signature | Returns |
|---|---|---|---|
| shape | `Box` | `Box(center=(x,y,z), size=(sx,sy,sz), yaw_deg=0)` full extents | |
| shape | `Cylinder` | `Cylinder(center=(x,y), radius, z_range=(lo,hi))`; `None` = unbounded | |
| shape | `Sphere` | `Sphere(center=(x,y,z), radius)` | |
| shape | `Band` | `Band(axis, low, high)`; `None` = unbounded | |
| source | `Select` | `Select(include=shape\|tuple, exclude=(...), source=None)` | lazy selection |
| query | `Overlap` | `Overlap(shape, source=None)` | `count`, `bounds_m`; `.selection` |
| query | `Closest` | `Closest(shape, source=None)`; Cylinder: only returns in its `z_range`, horizontal distance | `distance_m`, `point_m` |
| query | `Sweep` | `Sweep(shape, direction_deg=\|direction=(dx,dy,dz), max_distance, step_m=0.05)`; shape must be at least `step_m` thick (body-sized) | `hit`, `distance_m`, `point_m`, `start_inside` |
| grid | `Grid` | `Grid(origin=(u,v), shape=(cols,rows), cell_m, plane="xy")` | |
| field | `HeightField` | `HeightField(grid, source=None)` | `.min` `.max` `.count`, `.percentile(q, min_count=4)` |
| field | `DistanceField` | `DistanceField(grid, source=None)` | metres to nearest target |
| field | `Threshold` | `Threshold(field, ">", value)` | mask; combine with `&` `\|` |
| field | `Resample` | `Resample(field, grid)` | the field on another grid (containing-cell lookup) |
| field | `Components` | `Components(mask, connectivity=8, gap_cells=0, values=None, max_regions=None)`; `gap_cells` links true cells that far apart without filling between them; `values=field` adds each region's statistics of that field; `max_regions=n` keeps the n largest in the table | `regions` table; `.label` per cell |
| output | `Overview` | `Overview(max_regions=8, percentile=10, min_count=4, band=(0.15,1.0), relief_m=0.15, cell_spacings=2, relief_cell_spacings=4, relief_gap_cells=1, min_supported_cells=8, min_supported_fraction=0.5)` | compact default recipe; band is relative to measured reference Z |
| output | `Sample` | `Sample(field, at=[(u,v)], fields=None, decimals=None, radius=0)` — a few points, ~170 B each; `radius` summarises the disc: min/max, and `distinct` values for masks/labels | `samples[i]["values"][ch]`: a number when `radius=0`, `{"min", "max"(, "distinct")}` when `radius>0` |
| output | `Window` | `Window(field, cells=(col,row,w,h), fields=None, decimals=None)` — ~7 B/cell, ~4 for counts/masks/labels | `values[channel][row][col]` |
| render | `Map` | `Map(field_or_channel, value_range=None, overlays=())` | image + `view_ref` |
| render | `DepthView` | `DepthView(view=(x,y,z,yaw_deg,pitch_deg), fov_deg=90, source=None, overlays=())` | image + `view_ref` |
| render | `OccupancyMap` | `OccupancyMap(z_range=(lo,hi), zoom=None\|(x,y,r), mark=None\|(x,y,yaw), grid=None, colour="flat"\|"height")` | image + `view_ref` |
| pick | `Pick` | `Pick(view_ref, uv=\|rect=\|polygon=, radius_px=0, max_items=16)` | hits/cells; `.selection` |
| pick | `SelectionRef` | `SelectionRef(selection_ref)` | reusable source across processes |

Rules:

- Check `r["status"]` (`ok` / `invalid` / `too_large`) before reading anything.
  If it is not `ok`, print `r["error"]`: it says what to change. If your last call
  failed or came back `too_large`, fix it before answering.
  Geometry outcome is separate: `r["measurement_status"]`.
- Zero returns ≠ free space. Empty cells, `count == 0`, `no_return`: unobserved, not empty.
- Sweep samples positions; a miss is not a continuous clearance guarantee.
- Images: print `r["image"]`, look at it, then pick. Never pick pixels from a view
  you have not seen. Pixels are native `(u right, v down)`.
- One call computes shared dependencies once; prefer several named outputs in one call.
- The response budget is fixed (16 KB: about 2000 `Window` cells of distances or
  heights, 4000 of counts/masks/labels, or 90 `Sample` points). Compute on the grid and read only the answer; see Fields. `too_large`
  means the question is being asked the wrong way, not that it needs paging: never
  loop calls over rows or chunks.
- Print compact fields. Never print whole results: `view_ref`, `selection_ref`
  and `request` are long recipes. Keep them in variables or save them to a JSON
  file; never retype them.
- Start from `bounds_m`: choose z bands and view heights from it. To find where
  returns sit in z, stack `Overlap`s over z slices of one Box.
- Sweep `start_inside > 0` means returns overlap the shape where it starts, so
  `distance_m == 0`. Around the robot those returns are beside or behind it: start
  the shape just ahead of the robot instead (see "How far can I go along a heading?").
  Never shrink it.
- Sample rows with status `outside_grid` have no `cell`/`values`. `no_targets` ≠ `outside_grid`.
- Fields (`HeightField` etc.) returned directly give metadata only; use `Sample`/`Window` for numbers.
- Answer only from measurements. File, stream and dataset names are not evidence.
  If the data does not show it, say so (none / unmeasured).

Reading images:

- Depth: colour = depth on a log scale. The result's `colour` gives near/far
  colours, sampled stops, and `depth_m = near_m * (far_m / near_m) ** f`;
  `colour_none` = no return.
- Occupancy: returns below `z_range` mark a cell free, inside it occupied
  (occupied wins), above it are ignored. White = free, grey = unseen, black =
  occupied (`colour="height"`: coloured by highest in-band return, see
  `height_scale`). Red = `mark` with a heading line. Grey lines every 1 m.
  `origin_xy` is the south-west corner; `cell_m` is the cell size used.
- Map: the result reports its colour scale and pixel transform.

Helper used below:

```python
def ok(out, name):
    r = out["results"][name]
    if r["status"] != "ok":
        raise ValueError((name, r["status"], r.get("error")))
    return r
```

## Fields

A field is a lazy grid of numbers; nothing is computed or returned until an
output reads it. Build the whole question as fields, then read the least you need.

- **Height:** `h = HeightField(grid, source)` gives `h.count` (returns per cell),
  `h.min`, `h.max` (stored z along the remaining axis, not a fitted floor).
  Empty cells: count 0, null min/max.
- **Percentiles:** `h.percentile(10, min_count=4)` is a scalar field of per-cell
  return heights, using linear interpolation. Cells below support are null.
- **Masks:** `Threshold(channel, op, value)` → 1 / 0 / null (null = no data).
  `a & b`, `a | b`: false AND null = 0, true OR null = 1; otherwise null.
  `a - b` subtracts fields.
- **Obstacles:** `Threshold(HeightField(grid, source=Select(include=Band("z", lo, hi),
  exclude=(body,))).count, ">", 0)`: cells holding a return in the band.
- **Clearance:** `DistanceField(grid, source=mask)` = metres from each cell centre to
  the nearest mask cell centre. `source=Select(...)` measures to the returns
  themselves instead (including ones outside the grid). No targets → null, `no_targets`.
  `Threshold(clearance, ">", r)` = cells at least `r` from anything.
- **Same grid:** `&`, `|`, `-` and `DistanceField(source=mask)` need identical grids
  (origin, shape, cell_m, plane). `Resample(field, grid)` puts a field on another grid:
  each target cell takes the source cell containing its centre; outside → null.
- **Components:** `Components(mask, connectivity=8)` labels connected 1-cells.
  Returned directly: `regions` (id, cells, centroid, bounds), no per-cell data.
  `values=field` (same grid) adds each region's `valid_cells`, `min`, `p10`, `p50`,
  `p90`, `max` of that field over its finite cells, e.g. a patch's height offsets.
  The table lists every region; on a fragmented mask pass `max_regions=n` for the
  n largest plus `omitted_regions` / `omitted_cells`. Labels always cover them all.
  `Sample(Components(mask), at=[a, b])` → equal non-zero labels = connected.
- **Reading:** `Sample` for a few points. `Window(field, (col, row, w, h), fields=("max",))`
  for a block of cells; values are `[row][col]`, null = no data. Window only the
  block you need and one channel; a 40x40 window of distances fits.
- **Precision:** `Sample`/`Window` round to a thousandth of the cell size by default
  (0.1 m cells → 0.001 m; the result's `decimals` says which). Whole numbers stay
  integers. Pass `decimals=` only if you need more.
- **Seeing:** `Map` renders one channel, mask or label grid: `Map(h.max)`, not `Map(h)`.

Recipes:

```python
grid = P.Grid(origin=(-4, -4), shape=(80, 80), cell_m=0.1)
height = P.HeightField(grid)
raised = P.Threshold(height.max, ">", 0.3)
out = cloud.agent_encode({
    "pts": P.Sample(height, at=[(1.0, 0.5), (2.0, 0.0)]),     # surface height at points
    "blobs": P.Components(raised),                            # raised regions, as a table
    "block": P.Window(height.max, (55, 35, 10, 10)),          # 1 m x 1 m of heights
    "img": P.Map(raised),
})
for row in ok(out, "pts")["samples"]:
    print(row["status"], row.get("values"))
print(ok(out, "blobs")["regions"])   # cells, bounds, centroids; you decide what they are
print(ok(out, "block")["values"]["max"])
```

### Characterising regions

`Components` finds regions; it does not say what they are. Decide by measuring:

1. **Local floor:** the typical `h.min` of well-covered cells (high `h.count`) near
   the area. State heights relative to it, not as absolute z. Floors are not flat:
   measure the range of `h.min` (max minus min) over well-covered cells within about
   1 m of the robot. That range is your noise: any offset inside it, up or down, is
   the same level.
2. **Walkable level:** `h.min` is the lowest thing in a cell, so it is the floor
   there. A cell whose `h.min` is further above or below the local floor than
   that noise range is on another level, whatever stands on top of it; judge it by `h.min` and coverage,
   not by `h.max - h.min`. Coverage is relative: read `h.count` on floor cells around
   the robot and call a cell well-covered when its count is comparable (about a
   quarter of that or more); recompute whenever you change `cell_m`. A floor is a
   surface: count only returns near the cell's floor height, with `HeightField(grid,
   source=P.Select(include=P.Band("z", z - 0.05, z + 0.05))).count` at the candidate
   height `z`. A cell whose `h.min` sits far below the floor but has few returns near
   that height is stray returns under something, not a sunken floor. For level
   questions use cells of about 0.2-0.25 m so each cell pools enough returns.
   Raised `h.max` over floor-level `h.min` is something standing on the floor.
   Do not cut z bands before looking: a band can hide the raised or sunken floor
   you are looking for.
3. **Slices:** `Threshold(h.max, ">", z)` at several heights, `Components` at each.
   Same footprint at every level = one upright thing; a footprint that shrinks and
   shifts level by level = stepped; present at one level only = a flat surface there.
4. **Gaps:** the space between two regions is measurable: their `bounds`, or
   `DistanceField(grid, source=region_mask)` sampled at the other region.
5. **Check before claiming:** show the defining measurement (treads, a flat top above
   the floor, an opening's width). If you cannot, answer none / unmeasured. Before
   answering none or level, list every covered patch your masks returned with its
   offset, and name the criterion that rejects each one you drop. Report only
   patches whose offset exceeds the noise range from step 1.
6. **Stairs and steps:** a step is a tread, a flat patch of well-covered cells whose
   `h.min` sits above the floor next to it by more than the noise range; stairs are
   treads, each above the last by more than that, stepping in one direction. One raised patch with things on it,
   or scattered cells of mixed height, is not stairs. The middle is the centre of
   the treads; the rise is the top tread's `h.min` minus the bottom floor's. Treads
   further from the sensor get fewer returns: judge a tread's coverage against the
   cells around it at its own height, not against the floor at the robot. Before
   answering none, list every raised patch with its offset and why it is not a tread.
7. **Doorways:** an opening is a gap between wall runs that line up. Take `Components`
   of cells holding returns at wall height (for example `h.max` above 0.5 m on the
   local floor), read their `bounds`, and measure the gap between runs along the same
   line. A gap about as wide as a door (0.7-1.2 m) with walls on both sides is a
   doorway; its centre is the middle of the gap. Do not require a lintel: the cloud's
   top may be cut off.

```python
floor_z = -0.1                                  # measured local floor (step 1)
level = 0.05                                    # measured h.min variation on level floor
covered = P.Threshold(height.count, ">=", 5)
up = P.Threshold(height.min, ">", floor_z + level) & covered    # raised floor
down = P.Threshold(height.min, "<", floor_z - level) & covered  # sunken floor
out = cloud.agent_encode({
    f"z{z}": P.Components(P.Threshold(height.max, ">", z)) for z in (0.1, 0.3, 0.5)
} | {"up": P.Components(up), "down": P.Components(down)})
for k, r in out["results"].items():
    print(k, [(g["cells"], [round(c, 2) for c in g["centroid"]]) for g in r["regions"]])
```

## Scenarios

### Orient: what does the cloud contain?

```python
out = cloud.agent_encode({
    "top": P.OccupancyMap(z_range=(0.15, 1.0)),
    "ahead": P.DepthView(view=(0, 0, 0.5, 0, 0)),
})
print(out["frame_id"], out["num_points"], out["bounds_m"])
print(ok(out, "top")["image"], ok(out, "ahead")["image"])
```

### Is anything inside this volume?

```python
r = ok(cloud.agent_encode({"box": P.Overlap(P.Box(center=(2, 0, 0.5), size=(1, 1, 1)))}), "box")
print(r["count"], r.get("bounds_m"))  # count 0: nothing returned, not proven empty
```

### How far is the nearest obstacle?

"How far from me / the robot / a point" is measured from that point: use a
`radius=0` cylinder. `distance_m` is always from the shape's surface, so a
body-sized shape gives the gap to the body instead; use that only when the
question asks about clearance around the body.

```python
here = P.Cylinder(center=(0, 0), radius=0, z_range=(0.15, 1.0))
body = P.Cylinder(center=(0, 0), radius=0.35, z_range=(None, None))
obstacles = P.Select(include=P.Band("z", 0.15, 1.0), exclude=(body,))  # not the robot itself
r = ok(cloud.agent_encode({"near": P.Closest(here, source=obstacles)}), "near")
print(r["distance_m"], r["point_m"])   # horizontal metres from (0, 0)
```

### Can I drive forward 2 m?

```python
body = P.Cylinder(center=(0, 0), radius=0.3, z_range=(0.15, 1.0))
r = ok(cloud.agent_encode({"go": P.Sweep(body, direction_deg=0, max_distance=2.0)}), "go")
print(r["hit"], r["distance_m"], r["point_m"], r["start_inside"])
```

Check several headings in one call:

```python
out = cloud.agent_encode({
    f"h{d}": P.Sweep(body, direction_deg=d, max_distance=2.0) for d in range(0, 360, 45)
})
print({k: v["distance_m"] for k, v in out["results"].items()})
```

### How far can I go along a heading?

The robot needs a lane as wide as its body, and the lane must be measured, not
just empty. Sweep a thin body-wide slab that starts just ahead of the robot, facing
the heading, so returns beside or behind it do not count; check the lane is observed
with `Overlap` boxes in 0.2 m bins. Four headings fit in one call, so two calls
cover all eight.

```python
import math
robot, reach, width = (0.0, 0.0), 2.0, 0.6          # lane: body plus margin
def lane(d, along, size_along, z=0.55, height=0.8):  # z 0.15-0.95: not the floor
    c, s = math.cos(math.radians(d)), math.sin(math.radians(d))
    return P.Box(center=(robot[0] + c * along, robot[1] + s * along, z),
                 size=(size_along, width, height), yaw_deg=d)
headings = list(range(0, 360, 45))
bins = [0.2 * i + 0.1 for i in range(int(reach / 0.2))]
for half in (headings[:4], headings[4:]):
    q = {f"s{d}": P.Sweep(lane(d, 0.2, 0.1), direction_deg=d, max_distance=reach) for d in half}
    q |= {f"c{d}_{i}": P.Overlap(lane(d, b, 0.2, z=0.0, height=4.0))   # every return, floor included
          for d in half for i, b in enumerate(bins)}
    out = cloud.agent_encode(q)
    for d in half:
        r = ok(out, f"s{d}")
        unseen = [b for i, b in enumerate(bins) if ok(out, f"c{d}_{i}")["count"] == 0]
        print(d, r["hit"], r["distance_m"], r["start_inside"], "first unmeasured:", unseen[:1])
```

`start_inside > 0` here means something stands right in front of the robot on that
heading: blocked there, not a reason to shrink the slab. Clear along a heading means
no hit within the distance and no unmeasured bin before it. If no heading meets the
question's distance, answer none.

### Work at the robot's scale

Questions about passing, driving or separate things are about a body, not a point.

- **Travel is a lane as wide as the body plus margin.** Sweep a shape that size;
  never shrink it to get past a hit. A single cell or a thin ray says nothing
  about whether the robot fits.
- **Separate things are separated by a gap the body could use.** Returns closer
  together than that belong to one thing. Merge them on the grid first, then count:

```python
occ = P.Threshold(P.HeightField(grid, source=obstacles).count, ">", 0)
link = 0.4                                         # gaps narrower than this don't separate
merged = P.Threshold(P.DistanceField(grid, source=occ), "<=", link / 2)
out = cloud.agent_encode({"things": P.Components(merged)})
```

  Ignore regions of a few cells: specks are not things. The gap between two
  things is `DistanceField(grid, source=thing_a)` sampled with a `radius` over thing b.
- **Keep your own criterion.** If no candidate meets the question's threshold,
  the answer is none; say so.

### Ignore the floor, ceiling, or the robot itself

```python
obstacles = P.Select(include=P.Band("z", 0.15, 1.0), exclude=(body,))
out = cloud.agent_encode({
    "near": P.Closest(body, source=obstacles),
    "view": P.DepthView(view=(0, 0, 0.5, 0, 0), source=obstacles),
})
```

### Reachability: can I get from A to B?

Build the answer on the grid; read the labels under the start and goal.

```python
robot, goal = (0.0, 0.0), (-2.5, 1.0)
(x0, y0, _), (x1, y1, _) = cloud.agent_encode({})["bounds_m"]
lo_x, lo_y = min(x0, robot[0], goal[0]) - 0.5, min(y0, robot[1], goal[1]) - 0.5
hi_x, hi_y = max(x1, robot[0], goal[0]) + 0.5, max(y1, robot[1], goal[1]) + 0.5
grid = P.Grid(origin=(lo_x, lo_y), shape=(int((hi_x - lo_x) / 0.1) + 1, int((hi_y - lo_y) / 0.1) + 1), cell_m=0.1)
coarse = P.Grid(origin=(lo_x, lo_y), shape=(int((hi_x - lo_x) / 0.2) + 1, int((hi_y - lo_y) / 0.2) + 1), cell_m=0.2)

body = P.Cylinder(center=robot, radius=0.35, z_range=(None, None))
blocked = P.Threshold(P.HeightField(grid, source=P.Select(
    include=P.Band("z", 0.15, None), exclude=(body,))).count, ">", 0)   # body excluded from obstacles only
free = P.Threshold(P.DistanceField(grid, source=blocked), ">", 0.2)       # 0.2 m margin
seen = P.Resample(P.Threshold(P.HeightField(coarse).count, ">", 0), grid)  # all returns, body included
out = cloud.agent_encode({
    k: P.Sample(P.Components(mask), at=[robot, goal], radius=0.15)         # cells touching each end
    for k, mask in (("free", free), ("seen", free & seen))
})
for k in ("free", "seen"):
    start, end = (set(s["values"]["label"].get("distinct") or []) - {0}
                  for s in ok(out, k)["samples"])
    print(k, "connected" if start & end else "not connected")
```

Label 0 = not in the mask. Start and goal are points, not cells; their own cells are
often not passable (the goal may sit near something). A path reaches an end when it
reaches a passable cell touching that end's cell, so sample with a `radius` of about
1.5 cells, never `radius=0`; the goal's own cell failing the test does not make it
blocked. Connected means some start cell and some goal cell share a non-zero label.

Keep the 0.5 m margin past the cloud's bounds: unmeasured space beyond the edge is
where a path can go around a wall, and a grid cut at the edge closes it off.

If the goal is unmapped (the `free & seen` label at the goal is 0 and `h.count` there
is 0), the `seen` test cannot reach it. Instead take `Components` of the unmapped
cells (`count == 0`), find the patch holding the goal, and call it reachable when
that patch touches the robot's `free & seen` component (sample the patch's cells, or
dilate one with `DistanceField`, and look for the robot's label).

### Top-down map around a pose, with the robot marked

```python
r = ok(cloud.agent_encode({"m": P.OccupancyMap(
    z_range=(0.15, 1.0), zoom=(0, 0, 3), mark=(0, 0, 90), colour="height"
)}), "m")
print(r["image"], r["cell_m"], r["origin_xy"], r["occupied_cells"])
```

### Draw queries or a path on a render

```python
path = {"segment_m": [[0, 0, 0.3], [1, 0, 0.3], [1, 1, 0.3]]}
near = P.Closest(body, source=obstacles)
r = ok(cloud.agent_encode({"v": P.OccupancyMap(z_range=(0.15, 1.0), overlays=(near, path))}), "v")
```

### Measure something you see in an image (two calls)

Call 1: render and look.

```python
shown = ok(cloud.agent_encode({"v": P.DepthView(view=(0, 0, 1, 0, 0))}), "v")
print(shown["image"])  # inspect, choose (u, v) in native pixels
```

Call 2: pick.

```python
u, v = 120, 80  # chosen from the image
hit = ok(cloud.agent_encode({"h": P.Pick(shown["view_ref"], uv=(u, v), max_items=4)}), "h")
print(hit["measurement_status"])  # hit / no_return / ambiguous / outside_image
if hit["measurement_status"] in ("hit", "ambiguous"):
    print([h["point_m"] for h in hit["hits"]], hit["depth_span_m"])
```

Patches: `rect=(u, v, w, h)` or `polygon=((u0, v0), (u1, v1), (u2, v2))`.

### Pick a map cell and read its numbers

```python
shown_map = ok(cloud.agent_encode({"m": P.Map(height.max)}), "m")
print(shown_map["image"])
# after inspecting:
cell_hit = ok(cloud.agent_encode({"h": P.Pick(shown_map["view_ref"], uv=(u, v), max_items=1)}), "h")
if cell_hit["measurement_status"] == "hit":
    cell = cell_hit["cells"][0]
    print(cell["centre_m"], cell["count"], cell["min_m"], cell["max_m"])
```

### What else is near the thing I picked?

```python
point = hit["hits"][0]["point_m"]  # from a depth pick
r = ok(cloud.agent_encode({"n": P.Overlap(P.Sphere(center=point, radius=0.2))}), "n")
print(r["count"], r.get("bounds_m"))
```

Or treat the pick itself as a source:

```python
pick = P.Pick(shown["view_ref"], uv=(u, v))
out = cloud.agent_encode({"near": P.Closest(body, source=pick.selection)})
```

### Save a selection and reuse it later

```python
import json
saved = json.dumps({"ref": hit["selection_ref"]})
# later, same cloud:
src = P.SelectionRef(json.loads(saved)["ref"])
out = cloud.agent_encode({"n": P.Overlap(P.Sphere((0, 0, 0), 100), source=src)})
```

References bind the exact cloud (points, frame, timestamp); a different cloud is rejected.

### Vertical slice (walls, doorways)

```python
side = P.Grid(origin=(-4, 0), shape=(80, 25), cell_m=0.1, plane="xz")
r = ok(cloud.agent_encode({"m": P.Map(P.HeightField(side).count)}), "m")
```

## Limits

Fields ≤ 262144 cells; Sample ≤ 4096 points (the 16 KB response fits ~90); Map `max_side` ≤ 2048 px; Pick
`max_items` 1..64, `radius_px` ≤ 64, region ≤ 65536 px, polygon 3..32 vertices.
Images go to `out_dir`, else `$AGENT_ENCODE_DIR`, the run directory, or the state directory.
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
