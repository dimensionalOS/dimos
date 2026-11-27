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

"""Spawn points: a scene's ``DimosSpawnAPI`` prims (schema/generatedSchema.usda), each a pose plus the guide-purpose volume it accepts."""

from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from dimos.robot.assets.usd_utils.pxr import Layer, Prim, Pxr, Stage, layer_from, load_pxr

SPAWN_API = "DimosSpawnAPI"
_SCHEMA = Path(__file__).parent / "schema"
# Volumes are float scales, so a part exactly as big as its spot may overshoot it by float32 rounding.
_FIT_SLACK = float(np.finfo(np.float32).eps)


@dataclass(frozen=True)
class SpawnPoint:
    """A spawn point: its frame (its prim path without the leading "/") and the box it accepts, in its own frame."""

    frame: str
    low: tuple[float, float, float]
    high: tuple[float, float, float]

    @property
    def name(self) -> str:
        return self.frame.rpartition("/")[2]


def register_spawn_schema() -> None:
    """Register DimosSpawnAPI with USD, so ``prim.HasAPI("DimosSpawnAPI")`` works; it has to run before the process opens its first stage."""
    from pxr import Plug

    Plug.Registry().RegisterPlugins(str(_SCHEMA))


def is_spawn_point(prim: Prim) -> bool:
    """Whether ``prim`` applies DimosSpawnAPI, read off its apiSchemas, so it holds whether or not the schema was registered in time."""
    if SPAWN_API in prim.GetAppliedSchemas():
        return True
    listed = prim.GetMetadata("apiSchemas")
    return listed is not None and SPAWN_API in listed.ApplyOperations([])


def spawn_points(stage: Stage) -> list[SpawnPoint]:
    """Every spawn point of ``stage``, each with the volume its guide children bound."""
    pxr = load_pxr()
    cache = pxr.UsdGeom.BBoxCache(pxr.Usd.TimeCode.Default(), [pxr.UsdGeom.Tokens.guide])
    points = []
    for prim in stage.Traverse():
        if not is_spawn_point(prim):
            continue
        volumes = [
            child
            for child in prim.GetChildren()
            if child.IsA(pxr.UsdGeom.Gprim)
            and pxr.UsdGeom.Imageable(child).ComputePurpose() == pxr.UsdGeom.Tokens.guide
        ]
        if not volumes:
            raise ValueError(
                f"the spawn point {prim.GetPath()} has no volume: give it a guide-purpose "
                f"Cube, Sphere, Cylinder or Capsule child bounding what it accepts"
            )
        box = pxr.Gf.Range3d()
        for volume in volumes:
            box.UnionWith(cache.ComputeRelativeBound(volume, prim).ComputeAlignedRange())
        points.append(
            SpawnPoint(str(prim.GetPath())[1:], _triple(box.GetMin()), _triple(box.GetMax()))
        )
    return points


def scene_spawn_points(usd: str, root: Path | str | None = None) -> list[SpawnPoint]:
    """A scene's spawn points; a scene with none is refused."""
    pxr = load_pxr()
    stage = pxr.Usd.Stage.Open(layer_from(pxr, usd, "a scene", root))
    points = spawn_points(stage)
    if not points:
        raise ValueError(
            f"a scene needs spawn points (prims with {SPAWN_API}) for its robots and objects "
            f"to start on, and {stage.GetDefaultPrim().GetPath() or 'this one'} has none"
        )
    return points


def spawn_frames_in_file(path: Path) -> list[str]:
    """The spawn points' frames in a scene file's own layer, read without composing it (so without opening a mesh)."""
    from dimos.robot.assets.usd_utils.lfs import pull_lfs_files

    pull_lfs_files([path])
    pxr = load_pxr()
    layer = pxr.Sdf.Layer.FindOrOpen(str(path))
    if layer is None:
        raise ValueError(f"{path} is not a USD layer")
    return sorted(str(spec.path)[1:] for spec in _prim_specs(layer) if _spec_is_spawn(spec))


def spawn_volume_paths(layer: Path) -> set[str]:
    """The prim paths of every spawn point's volume in a layer file, read without composing it; a renderer or physics engine leaves these out."""
    pxr = load_pxr()
    opened = pxr.Sdf.Layer.FindOrOpen(str(layer))
    if opened is None:
        return set()
    return {
        str(child.path)
        for spec in _prim_specs(opened)
        if _spec_is_spawn(spec)
        for child in spec.nameChildren
        if child.attributes.get("purpose") is not None
        and child.attributes["purpose"].default == pxr.UsdGeom.Tokens.guide
    }


def with_spawn_frames(usd: str, root: Path | str | None = None) -> str:
    """``usd`` with each spawn point made a tf frame: a rigid body welded where it stands to the rigid body it is under."""
    pxr = load_pxr()
    layer = layer_from(pxr, usd, "a scene", root)
    stage = pxr.Usd.Stage.Open(layer)
    cache = pxr.UsdGeom.XformCache()
    for prim in stage.Traverse():
        if not is_spawn_point(prim) or prim.HasAPI(pxr.UsdPhysics.RigidBodyAPI):
            continue
        _weld_where_it_stands(pxr, stage, cache, prim)
    return str(layer.ExportToString())


def check_spawn_fits(root_usd: str, mounted: Sequence[tuple[str, str]]) -> None:
    """Refuse a mounted (usd, mount frame) whose bounding box does not fit in the spawn point it is mounted on."""
    pxr = load_pxr()
    spots = {
        point.frame: point
        for point in spawn_points(pxr.Usd.Stage.Open(layer_from(pxr, root_usd, "a scene", None)))
    }
    too_big = []
    for usd, frame in mounted:
        spot = spots.get(frame)
        if spot is None:
            continue
        size = _bound(pxr, usd)
        if size is None:
            continue
        low, high = size
        if any(
            low[i] < spot.low[i] - _FIT_SLACK or high[i] > spot.high[i] + _FIT_SLACK
            for i in range(3)
        ):
            too_big.append(
                f"{frame}: takes {_box(spot.low, spot.high)}, what is mounted there spans {_box(low, high)}"
            )
    if too_big:
        raise ValueError("too big for the spawn points they are on:\n  " + "\n  ".join(too_big))


def _bound(pxr: Pxr, usd: str) -> tuple[tuple[float, ...], tuple[float, ...]] | None:
    """A namespaced USD's bounding box, which is in its mount's frame since its namespace prims carry no transform."""
    stage = pxr.Usd.Stage.Open(layer_from(pxr, usd, "a mounted part", None))
    cache = pxr.UsdGeom.BBoxCache(
        pxr.Usd.TimeCode.Default(),
        [pxr.UsdGeom.Tokens.default_, pxr.UsdGeom.Tokens.render, pxr.UsdGeom.Tokens.proxy],
    )
    box = pxr.Gf.Range3d()
    for prim in stage.GetPseudoRoot().GetChildren():
        box.UnionWith(cache.ComputeWorldBound(prim).ComputeAlignedRange())
    if box.IsEmpty():
        return None
    return tuple(box.GetMin()), tuple(box.GetMax())


def _triple(vector: Any) -> tuple[float, float, float]:
    return float(vector[0]), float(vector[1]), float(vector[2])


def _box(low: Iterable[float], high: Iterable[float]) -> str:
    return "x".join(f"{b - a:.2f}" for a, b in zip(low, high, strict=True)) + " m"


def _weld_where_it_stands(pxr: Pxr, stage: Stage, cache: Any, prim: Prim) -> None:
    body = prim.GetParent()
    while body and not body.IsPseudoRoot() and not body.HasAPI(pxr.UsdPhysics.RigidBodyAPI):
        body = body.GetParent()
    pxr.UsdPhysics.RigidBodyAPI.Apply(prim)
    joint = pxr.UsdPhysics.FixedJoint.Define(stage, prim.GetPath().AppendChild("spawn_joint"))
    if body and not body.IsPseudoRoot():
        joint.CreateBody0Rel().SetTargets([body.GetPath()])
        relative, _ = cache.ComputeRelativeTransform(prim, body)
    else:
        relative = cache.GetLocalToWorldTransform(prim)
    joint.CreateBody1Rel().SetTargets([prim.GetPath()])
    transform = pxr.Gf.Transform(relative)
    joint.CreateLocalPos0Attr(pxr.Gf.Vec3f(transform.GetTranslation()))
    joint.CreateLocalRot0Attr(pxr.Gf.Quatf(transform.GetRotation().GetQuat()))
    joint.CreateLocalPos1Attr(pxr.Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr(pxr.Gf.Quatf(1.0, 0.0, 0.0, 0.0))


def _prim_specs(layer: Layer) -> Iterable[Any]:
    stack = list(layer.rootPrims)
    while stack:
        spec = stack.pop()
        yield spec
        stack.extend(spec.nameChildren)


def _spec_is_spawn(spec: Any) -> bool:
    listed = spec.GetInfo("apiSchemas") if spec.HasInfo("apiSchemas") else None
    return listed is not None and SPAWN_API in listed.ApplyOperations([])


register_spawn_schema()
