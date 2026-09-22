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

from dimos.experimental.agent_encode.pointcloud.fields import Band, Grid
from dimos.experimental.agent_encode.pointcloud.runtime.context import (
    EncodeContext,
    Request,
    Selection,
)
from dimos.experimental.agent_encode.pointcloud.runtime.recipe import canonical, describe
from dimos.experimental.agent_encode.pointcloud.shapes.base import Shape


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


def _classes(base: type) -> dict[str, Callable[..., object]]:
    """``base`` and every subclass defined so far, by name."""
    found: dict[str, Callable[..., object]] = {base.__name__: base}
    for sub in base.__subclasses__():
        found |= _classes(sub)
    return found


def _restore(spec: JsonValue, classes: dict[str, Callable[..., object]]) -> object:
    """Rebuild the recipe objects ``describe`` wrote; nested references stay JSON until
    their own ``resolve``."""
    if isinstance(spec, list):
        return tuple(_restore(v, classes) for v in spec)
    if not isinstance(spec, dict) or "schema" in spec:
        return spec
    restored = {k: _restore(v, classes) for k, v in spec.items() if k != "type"}
    name = spec.get("type")
    if name is None:
        return restored
    if not isinstance(name, str) or name not in classes:
        raise ValueError("unknown portable recipe node")
    return classes[name](**restored)


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
    classes = {**_classes(Request), **_classes(Selection), **_classes(Shape)}
    return _restore(ref["recipe"], {**classes, "Grid": Grid, "Band": Band})
