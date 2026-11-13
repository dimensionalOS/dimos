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

"""Offline authoring of native USD scenes from existing URDF components.

This is preparation tooling, not a simulation backend or runtime scene contract.
URDF inertias, limits and meshes remain authoritative; placement, drives and
collision exclusions are explicit authoring inputs in SI units.
"""

from collections.abc import Mapping
from dataclasses import dataclass, field
import hashlib
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation
import trimesh

from dimos.robot.assets.model import RobotModel, UsdModel
from dimos.robot.assets.usd import (
    layer_from,
    link_prim_paths,
    load_pxr,
    usd_matrix,
    usd_with_drives,
)


@dataclass(frozen=True)
class Component:
    model: RobotModel
    xyz: tuple[float, float, float] = (0, 0, 0)
    rpy: tuple[float, float, float] = (0, 0, 0)
    floating: bool = False
    drives: Mapping[str, tuple[float, float, float]] = field(default_factory=dict)
    armature: float = 0.02
    friction: float = 0.0
    collision_exclusions: tuple[tuple[str, str], ...] = ()


def prepare_scene(
    components: Mapping[str, Component], *, ground_plane: bool = True, mesh_dir: Path | None = None
) -> UsdModel:
    """Author a portable scene; callers save the resulting layer before launching."""
    if mesh_dir is not None:
        mesh_dir = mesh_dir.resolve()
        mesh_dir.mkdir(parents=True, exist_ok=True)
    pxr = load_pxr()
    stage = pxr.Usd.Stage.CreateInMemory()
    mesh_library = pxr.Usd.Stage.CreateInMemory()
    mesh_references: list[tuple[str, str]] = []
    mesh_paths: dict[Path, str] = {}
    world = pxr.UsdGeom.Xform.Define(stage, "/world")
    stage.SetDefaultPrim(world.GetPrim())
    pxr.UsdGeom.SetStageUpAxis(stage, "Z")
    pxr.UsdGeom.SetStageMetersPerUnit(stage, 1)
    for name, component in components.items():
        model = component.model.load()
        source = component.model.to_usd()
        layer = layer_from(pxr, usd_with_drives(source.usd, component.drives), name, None)
        child_stage = pxr.Usd.Stage.Open(layer)
        root = child_stage.GetDefaultPrim().GetPath()
        target = world.GetPath().AppendChild(name)
        pxr.Sdf.CopySpec(layer, root, stage.GetRootLayer(), target)
        # CopySpec does not relocate relationship targets into the new namespace.
        for prim in pxr.Usd.PrimRange(stage.GetPrimAtPath(target)):
            for rel in prim.GetRelationships():
                paths = rel.GetTargets()
                if paths:
                    rel.SetTargets([p.ReplacePrefix(root, target) for p in paths])
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_euler("xyz", component.rpy).as_matrix()
        transform[:3, 3] = component.xyz
        pxr.UsdGeom.Xformable(stage.GetPrimAtPath(target)).AddTransformOp().Set(
            usd_matrix(pxr, transform)
        )
        links = link_prim_paths(stage)
        if not component.floating:
            joint = pxr.UsdPhysics.FixedJoint.Define(stage, target.AppendChild("world_mount"))
            joint.CreateBody1Rel().SetTargets([links[model.root_link]])
            joint.CreateLocalPos0Attr(pxr.Gf.Vec3f(*component.xyz))
            quat = Rotation.from_euler("xyz", component.rpy).as_quat()
            joint.CreateLocalRot0Attr(pxr.Gf.Quatf(float(quat[3]), *map(float, quat[:3])))
        for first, second in component.collision_exclusions:
            api = pxr.UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(links[first]))
            api.CreateFilteredPairsRel().AddTarget(links[second])
        joints = {j.name: j for j in model.joints}
        for prim in list(pxr.Usd.PrimRange(stage.GetPrimAtPath(target))):
            if prim.IsA(pxr.UsdGeom.Xformable):
                xf = pxr.UsdGeom.Xformable(prim)
                xf.SetXformOpOrder(
                    sorted(
                        xf.GetOrderedXformOps(),
                        key=lambda op: op.GetOpType() == pxr.UsdGeom.XformOp.TypeScale,
                    )
                )
            if prim.IsA(pxr.UsdGeom.Mesh):
                asset = prim.GetAssetInfo()["identifier"]
                source_path = Path(asset.resolvedPath or asset.path)
                if source_path not in mesh_paths or mesh_dir is None:
                    mesh = trimesh.load_mesh(source_path, process=False)
                    key = "/mesh_" + hashlib.sha256(source_path.read_bytes()).hexdigest()
                    geom = (
                        pxr.UsdGeom.Mesh(prim)
                        if mesh_dir is None
                        else pxr.UsdGeom.Mesh.Define(mesh_library, key)
                    )
                    geom.CreatePointsAttr(pxr.Vt.Vec3fArray(mesh.vertices.tolist()))
                    geom.CreateFaceVertexCountsAttr([3] * len(mesh.faces))
                    geom.CreateFaceVertexIndicesAttr(mesh.faces.ravel().tolist())
                    geom.CreateSubdivisionSchemeAttr("none")
                    mesh_paths[source_path] = key
                if mesh_dir is not None:
                    mesh_references.append((str(prim.GetPath()), mesh_paths[source_path]))
            joint_name = str(prim.GetDisplayName() or prim.GetName())
            if joint_name not in component.drives or not prim.IsA(pxr.UsdPhysics.Joint):
                continue
            prim.CreateAttribute("newton:armature", pxr.Sdf.ValueTypeNames.Float).Set(
                component.armature
            )
            prim.CreateAttribute("newton:friction", pxr.Sdf.ValueTypeNames.Float).Set(
                component.friction
            )
            kp, kd, _ = component.drives[joint_name]
            actuator = stage.DefinePrim(
                target.AppendChild("actuator_" + str(prim.GetName())), "MjcActuator"
            )
            actuator.CreateRelationship("mjc:target").SetTargets([prim.GetPath()])
            attributes: dict[str, tuple[Any, Any]] = {
                "gainPrm": (pxr.Sdf.ValueTypeNames.DoubleArray, [kp] + [0.0] * 9),
                "biasPrm": (pxr.Sdf.ValueTypeNames.DoubleArray, [0.0, -kp, -kd] + [0.0] * 7),
                "biasType": (pxr.Sdf.ValueTypeNames.Token, "affine"),
            }
            effort = joints[joint_name].effort
            if effort is not None and effort > 0:
                attributes.update(
                    {
                        "forceLimited": (pxr.Sdf.ValueTypeNames.Token, "true"),
                        "forceRange:min": (pxr.Sdf.ValueTypeNames.Double, -effort),
                        "forceRange:max": (pxr.Sdf.ValueTypeNames.Double, effort),
                    }
                )
            for attribute, (kind, value) in attributes.items():
                actuator.CreateAttribute("mjc:" + attribute, kind).Set(value)
    if mesh_dir is not None and mesh_references:
        pending = mesh_dir / "meshes.pending.usdc"
        mesh_library.GetRootLayer().Export(str(pending))
        digest = hashlib.sha256(pending.read_bytes()).hexdigest()
        destination = mesh_dir / (digest + ".usdc")
        pending.replace(destination)
        # Author references together; repeated recomposition of a large office is expensive.
        with pxr.Sdf.ChangeBlock():
            for path, key in mesh_references:
                spec = stage.GetRootLayer().GetPrimAtPath(path)
                spec.referenceList.explicitItems = [pxr.Sdf.Reference(str(destination), key)]
                spec.assetInfo = {
                    "identifier": pxr.Sdf.AssetPath(str(destination)),
                    "version": "sha256:" + digest,
                }
    if ground_plane:
        floor = pxr.UsdGeom.Plane.Define(stage, "/world/floor")
        floor.CreateAxisAttr("Z")
        floor.CreateWidthAttr(100)
        floor.CreateLengthAttr(100)
        floor.CreateDisplayColorAttr([(0.25, 0.3, 0.35)])
        pxr.UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
    return UsdModel.from_layer(str(stage.GetRootLayer().ExportToString()))
