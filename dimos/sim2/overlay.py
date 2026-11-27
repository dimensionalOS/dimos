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

"""Overlaying namespaced layers into one world: each already sits at its tf frames."""

from collections.abc import Sequence
from typing import Any

from dimos.robot.assets.usd_utils.pxr import (
    Layer,
    Prim,
    Pxr,
    Stage,
    frame_name,
    layer_from,
    load_pxr,
    prim_identifier,
)


def overlay_usds(root_usd: str, mounted: Sequence[tuple[str, str]]) -> str:
    """The root layer with each namespaced (usd, mount frame) layer flattened in at its frames: the root's opinions and metadata win, only its PhysicsScene survives, and each mounted rigid part is welded to its mount."""
    from pxr import UsdUtils

    pxr = load_pxr()
    root_layer = layer_from(pxr, root_usd, "the root", None)
    # Kept in a name: a sublayer is found by its identifier, and an anonymous layer lives only as long as its handle.
    mounted_layers = [(_without_physics_scenes(pxr, usd), frame) for usd, frame in mounted]
    root_layer.subLayerPaths = [layer.identifier for layer, _ in mounted_layers]
    flattened = UsdUtils.FlattenLayerStack(pxr.Usd.Stage.Open(root_layer))
    world = pxr.Usd.Stage.Open(flattened)
    for layer, frame in mounted_layers:
        _weld_to_mount(pxr, world, pxr.Usd.Stage.Open(layer), frame)
    return str(flattened.ExportToString())


def _without_physics_scenes(pxr: Pxr, usd: str) -> Layer:
    # A mounted part's PhysicsScene would be a second one for the world, so it goes.
    layer = layer_from(pxr, usd, "a mounted part", None)
    stage = pxr.Usd.Stage.Open(layer)
    for path in [prim.GetPath() for prim in stage.Traverse() if prim.IsA(pxr.UsdPhysics.Scene)]:
        stage.RemovePrim(path)
    return layer


def _weld_to_mount(pxr: Pxr, world: Stage, part: Stage, mount_frame: str) -> None:
    mount_path = pxr.Sdf.Path(f"/{mount_frame}")
    root_link = _first_link(pxr, part, mount_path)
    if root_link is None:
        return
    # The part's own top prim, the child of its mount on the way to its first link.
    top = next(
        path for path in root_link.GetPath().GetPrefixes() if path.GetParentPath() == mount_path
    )
    name = prim_identifier(pxr, f"{mount_frame}_mount")
    joint = pxr.UsdPhysics.Joint.Define(world, top.AppendChild(name))
    joint.CreateBody0Rel().SetTargets([mount_path])
    joint.CreateBody1Rel().SetTargets([root_link.GetPath()])
    joint.CreateLocalPos0Attr(pxr.Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr(pxr.Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalPos1Attr(pxr.Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr(pxr.Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.GetPrim().SetDisplayName(f"{frame_name(root_link)}_mount")


def _first_link(pxr: Pxr, stage: Stage, path: Any) -> Prim:
    root = stage.GetPrimAtPath(path)
    if not root:
        return None
    for prim in pxr.Usd.PrimRange(root):
        if prim.HasAPI(pxr.UsdPhysics.RigidBodyAPI):
            return prim
    return None
