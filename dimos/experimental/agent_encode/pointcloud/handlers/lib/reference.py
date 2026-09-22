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

from collections.abc import Callable
import hashlib

from pydantic import JsonValue

from dimos.experimental.agent_encode.pointcloud import fields as field_nodes
from dimos.experimental.agent_encode.pointcloud.render import raster as render
from dimos.experimental.agent_encode.pointcloud.runtime.context import EncodeContext
from dimos.experimental.agent_encode.pointcloud.runtime.recipe import canonical, describe
from dimos.experimental.agent_encode.pointcloud.shapes.box import Box
from dimos.experimental.agent_encode.pointcloud.shapes.cylinder import Cylinder
from dimos.experimental.agent_encode.pointcloud.shapes.sphere import Sphere


def _small(ref: dict[str, JsonValue]) -> None:
    if len(canonical(ref).encode()) > 8192:
        raise ValueError("reference exceeds 8192 bytes; simplify its source recipe")


def _bounded(value: object, depth: int = 0, count: list[int] | None = None) -> None:
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


def _restore(spec: JsonValue) -> object:
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
        field_nodes.Difference,
        field_nodes.And,
        field_nodes.Or,
        field_nodes.Not,
        field_nodes.Resample,
        field_nodes.Threshold,
        field_nodes.Components,
    )
    registry: dict[str, Callable[..., object]] = {cls.__name__: cls for cls in classes}
    if isinstance(spec, list):
        return tuple(_restore(v) for v in spec)
    if not isinstance(spec, dict):
        return spec
    if "schema" in spec:
        # Nested references are opaque JSON; their recipes are restored only by resolve().
        return spec
    if "type" in spec:
        name = spec["type"]
        if not isinstance(name, str) or name not in registry:
            raise ValueError("unknown portable recipe node")
        args = {k: _restore(v) for k, v in spec.items() if k != "type"}
        return registry[name](**args)
    return {k: _restore(v) for k, v in spec.items()}


def reference(node: object, ctx: EncodeContext, kind: str = "view") -> dict[str, JsonValue]:
    payload: dict[str, JsonValue] = {
        "schema": f"pointcloud.{kind}/v1",
        "cloud": ctx.fingerprint,
        "recipe": describe(node, omit=("overlays", "mark")),
    }
    _bounded(payload)
    result = {**payload, "digest": hashlib.sha256(canonical(payload).encode()).hexdigest()}
    _small(result)
    return result


def resolve(ref: object, ctx: EncodeContext, kind: str = "view") -> object:
    _bounded(ref)
    if not isinstance(ref, dict) or set(ref) != {"schema", "cloud", "recipe", "digest"}:
        raise ValueError("invalid reference envelope")
    _small(ref)
    payload = {k: v for k, v in ref.items() if k != "digest"}
    if (
        ref["schema"] != f"pointcloud.{kind}/v1"
        or ref["digest"] != hashlib.sha256(canonical(payload).encode()).hexdigest()
    ):
        raise ValueError("reference schema or digest mismatch")
    if ref["cloud"] != ctx.fingerprint:
        raise ValueError("stale reference: finite cloud, frame, or timestamp differs")
    return _restore(ref["recipe"])
