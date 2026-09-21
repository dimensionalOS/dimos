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

"""Portable view and selection references: bounded recipes bound to the exact finite cloud."""

from __future__ import annotations

from dataclasses import fields, is_dataclass
import hashlib
import json
from typing import Any

import numpy as np

from dimos.experimental.agent_encode.pointcloud import fields as field_nodes
from dimos.experimental.agent_encode.pointcloud.render import raster as render
from dimos.experimental.agent_encode.pointcloud.runtime.context import EncodeContext
from dimos.experimental.agent_encode.pointcloud.shapes.box import Box
from dimos.experimental.agent_encode.pointcloud.shapes.cylinder import Cylinder
from dimos.experimental.agent_encode.pointcloud.shapes.sphere import Sphere

MAX_REF_BYTES = 8192


def _json(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def _bounded(value: Any, depth: int = 0, count: list[int] | None = None) -> None:
    count = [0] if count is None else count
    count[0] += 1
    if depth > 80 or count[0] > 4096:
        raise ValueError("reference exceeds nesting/item limit")
    if isinstance(value, dict):
        if any(not isinstance(k, str) for k in value):
            raise ValueError("reference keys must be strings")
        for item in value.values():
            _bounded(item, depth + 1, count)
    elif isinstance(value, (list, tuple)):
        for item in value:
            _bounded(item, depth + 1, count)
    elif not isinstance(value, (str, int, float, bool, type(None))):
        raise ValueError("reference must contain only JSON values")


def _spec(node: Any, depth: int = 0) -> Any:
    if depth > 64:
        raise ValueError("view recipe exceeds 64 levels")
    if is_dataclass(node) and not isinstance(node, type):
        return {
            "node": type(node).__name__,
            "args": {
                f.name: _spec(getattr(node, f.name), depth + 1)
                for f in fields(node)
                if f.name not in ("overlays", "mark")
            },
        }
    if isinstance(node, dict):
        return {k: _spec(v, depth + 1) for k, v in node.items()}
    if isinstance(node, (tuple, list)):
        return [_spec(v, depth + 1) for v in node]
    if isinstance(node, np.generic):
        return node.item()
    if isinstance(node, (str, int, float, bool, type(None))):
        return node
    raise ValueError("unsupported node in portable view recipe")


def _restore(spec: Any) -> Any:
    # These imports resolve the handlers -> reference -> handlers dependency. The registry is closed.
    from dimos.experimental.agent_encode.pointcloud.handlers.depth_view import DepthView
    from dimos.experimental.agent_encode.pointcloud.handlers.field_outputs import Map
    from dimos.experimental.agent_encode.pointcloud.handlers.occupancy_map import OccupancyMap
    from dimos.experimental.agent_encode.pointcloud.handlers.pick import (
        Pick,
        PickSelection,
        SelectionRef,
    )

    classes = (
        DepthView,
        OccupancyMap,
        Map,
        Box,
        Cylinder,
        Sphere,
        Pick,
        PickSelection,
        SelectionRef,
        render.View,
        field_nodes.Grid,
        field_nodes.Select,
        field_nodes.Band,
        field_nodes.HeightField,
        field_nodes.Percentile,
        field_nodes.Channel,
        field_nodes.DistanceField,
        field_nodes.Binary,
        field_nodes.Resample,
        field_nodes.Threshold,
        field_nodes.Components,
    )
    registry = {cls.__name__: cls for cls in classes}
    if isinstance(spec, list):
        return tuple(_restore(v) for v in spec)
    if not isinstance(spec, dict):
        return spec
    if "schema" in spec:
        # Nested references are opaque JSON; their recipes are restored only by resolve().
        return spec
    if set(spec) == {"node", "args"}:
        if spec["node"] not in registry or not isinstance(spec["args"], dict):
            raise ValueError("unknown portable recipe node")
        return registry[spec["node"]](**{k: _restore(v) for k, v in spec["args"].items()})
    return {k: _restore(v) for k, v in spec.items()}


def _cloud_digest(ctx: EncodeContext) -> str:
    root = ctx.root
    key = ("pick_cloud_digest", id(root.points))
    if key not in ctx.cache:
        digest = hashlib.sha256(np.ascontiguousarray(root.points, dtype="<f4").tobytes())
        digest.update(
            _json({"frame": root.cloud.frame_id, "ts": root.cloud.ts, "form": render.FORM}).encode()
        )
        ctx.cache[key] = digest.hexdigest()
    return str(ctx.cache[key])


def reference(node: Any, ctx: EncodeContext, kind: str = "view") -> dict[str, Any]:
    payload = {
        "schema": f"pointcloud.{kind}/v1",
        "cloud": _cloud_digest(ctx),
        "recipe": _spec(node),
    }
    _bounded(payload)
    result = {**payload, "digest": hashlib.sha256(_json(payload).encode()).hexdigest()}
    if len(_json(result).encode()) > MAX_REF_BYTES:
        raise ValueError("reference exceeds 8192 bytes; simplify its source recipe")
    return result


def resolve(ref: Any, ctx: EncodeContext, kind: str = "view") -> Any:
    _bounded(ref)
    if not isinstance(ref, dict) or set(ref) != {"schema", "cloud", "recipe", "digest"}:
        raise ValueError("invalid reference envelope")
    if len(_json(ref).encode()) > MAX_REF_BYTES:
        raise ValueError("reference exceeds 8192 bytes")
    payload = {k: v for k, v in ref.items() if k != "digest"}
    if (
        ref["schema"] != f"pointcloud.{kind}/v1"
        or ref["digest"] != hashlib.sha256(_json(payload).encode()).hexdigest()
    ):
        raise ValueError("reference schema or digest mismatch")
    if ref["cloud"] != _cloud_digest(ctx):
        raise ValueError("stale reference: finite cloud, frame, or timestamp differs")
    return _restore(ref["recipe"])
