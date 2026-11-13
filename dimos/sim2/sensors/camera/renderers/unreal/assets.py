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

"""USD visuals in body-local metres; only the Unreal wire uses left-handed cm."""

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from dimos_lcm.geometry_msgs import Pose
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation
import trimesh

from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd import layer_from, load_pxr
from dimos.sim2.geometry.poses import pose_array

BASIS = np.diag([1.0, -1.0, 1.0])
OPTICAL_AXES = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])


def unreal_pose(transform: NDArray[np.float64], *, camera: bool = False) -> dict[str, Any]:
    rotation = BASIS @ transform[:3, :3] @ (OPTICAL_AXES if camera else BASIS)
    return {
        "position": (100 * BASIS @ transform[:3, 3]).tolist(),
        "quaternion": Rotation.from_matrix(rotation).as_quat().tolist(),
    }


def pose_matrix(pose: Pose) -> NDArray[np.float64]:
    values = pose_array(pose)
    result = np.eye(4)
    result[:3, 3] = values[:3]
    result[:3, :3] = Rotation.from_quat(values[3:]).as_matrix()
    return result


@dataclass(frozen=True)
class Visual:
    id: int
    body: str | None
    local: NDArray[np.float64]


def export_visuals(
    scene: UsdModel,
    authored_bodies: tuple[str, ...],
    native_visuals: dict[str, str] | None = None,
) -> tuple[dict[str, Any], list[Visual], set[str]]:
    """Export visible USD geometry once, attached to exact rigid-body frame IDs.

    Authored backgrounds must be fixed at their saved origin. Native mesh
    overrides keep the office's cooked materials on independently moving props.
    """
    pxr = load_pxr()
    root = scene.source_path.parent if scene.source_path else None
    stage = pxr.Usd.Stage.Open(layer_from(pxr, scene.usd, "Unreal visuals", root))
    bodies = {
        str(p.GetPath()): str(p.GetDisplayName() or p.GetName())
        for p in stage.Traverse()
        if p.HasAPI(pxr.UsdPhysics.RigidBodyAPI)
    }
    if not set(authored_bodies) <= set(bodies.values()):
        raise ValueError("authored body is absent from the physics scene")
    native_visuals = native_visuals or {}
    if not set(native_visuals) <= set(bodies.values()) - set(authored_bodies):
        raise ValueError(
            "native visual body is missing or belongs to the static authored background"
        )
    cache = pxr.UsdGeom.XformCache()
    manifest: dict[str, Any] = {"version": 2, "meshes": [], "geoms": []}
    visuals: list[Visual] = []
    for path, body in bodies.items():
        prim = stage.GetPrimAtPath(path)
        if body in authored_bodies:
            if not np.allclose(
                np.array(cache.GetLocalToWorldTransform(prim)).T, np.eye(4), atol=1e-7
            ):
                raise ValueError("authored background must stay at its saved origin")
        if body in native_visuals:
            asset = native_visuals[body]
            if not asset.startswith("/Game/") or "." not in asset:
                raise ValueError("native visuals require a full Unreal content object path")
            index = len(visuals)
            visuals.append(Visual(index, body, np.eye(4)))
            manifest["geoms"].append(
                {"id": index, "static_mesh": asset, "scale": [1, 1, 1], **unreal_pose(np.eye(4))}
            )
    for prim in stage.Traverse():
        if not prim.IsA(pxr.UsdGeom.Gprim):
            continue
        imageable = pxr.UsdGeom.Imageable(prim)
        group = prim.GetAttribute("mjc:group").Get()
        if imageable.ComputeVisibility() == "invisible" or (group is not None and group >= 3):
            continue
        ancestor = prim.GetParent()
        while ancestor and str(ancestor.GetPath()) not in bodies:
            ancestor = ancestor.GetParent()
        visual_body = bodies[str(ancestor.GetPath())] if ancestor else None
        if visual_body in authored_bodies or visual_body in native_visuals:
            continue
        kind = str(prim.GetTypeName())
        if kind == "Mesh":
            geom = pxr.UsdGeom.Mesh(prim)
            points = geom.GetPointsAttr().Get()
            if points is not None and len(points):
                counts = geom.GetFaceVertexCountsAttr().Get()
                indices = geom.GetFaceVertexIndicesAttr().Get()
                faces: list[tuple[int, int, int]] = []
                offset = 0
                for count in counts:
                    faces.extend(
                        (indices[offset], indices[offset + i], indices[offset + i + 1])
                        for i in range(1, count - 1)
                    )
                    offset += count
                mesh = trimesh.Trimesh(vertices=np.asarray(points), faces=faces, process=False)
            else:
                asset = prim.GetAssetInfo().get("identifier")
                if asset is None:
                    raise ValueError(f"mesh has neither points nor an asset: {prim.GetPath()}")
                mesh_path = Path(asset.resolvedPath or asset.path)
                if not mesh_path.is_absolute() and root is not None:
                    mesh_path = root / mesh_path
                mesh = trimesh.load_mesh(mesh_path)
        elif kind == "Cube":
            mesh = trimesh.creation.box([pxr.UsdGeom.Cube(prim).GetSizeAttr().Get()] * 3)
        elif kind == "Sphere":
            mesh = trimesh.creation.icosphere(radius=pxr.UsdGeom.Sphere(prim).GetRadiusAttr().Get())
        elif kind in ("Cylinder", "Capsule"):
            geom = pxr.UsdGeom.Cylinder(prim) if kind == "Cylinder" else pxr.UsdGeom.Capsule(prim)
            factory = trimesh.creation.cylinder if kind == "Cylinder" else trimesh.creation.capsule
            mesh = factory(radius=geom.GetRadiusAttr().Get(), height=geom.GetHeightAttr().Get())
            axis = str(geom.GetAxisAttr().Get())
            if axis != "Z":
                rotation = np.eye(4)
                rotation[:3, :3] = Rotation.from_rotvec(
                    [0, np.pi / 2, 0] if axis == "X" else [-np.pi / 2, 0, 0]
                ).as_matrix()
                mesh.apply_transform(rotation)
        elif kind == "Plane":
            geom = pxr.UsdGeom.Plane(prim)
            if str(geom.GetAxisAttr().Get()) != "Z":
                raise ValueError("Unreal plane export currently requires a Z-up plane")
            x, y = geom.GetWidthAttr().Get() / 2, geom.GetLengthAttr().Get() / 2
            mesh = trimesh.Trimesh(
                vertices=[[-x, -y, 0], [x, -y, 0], [x, y, 0], [-x, y, 0]],
                faces=[[0, 1, 2], [0, 2, 3]],
                process=False,
            )
        else:
            raise ValueError(f"unsupported USD visual geometry: {kind}")
        world = np.array(cache.GetLocalToWorldTransform(prim)).T
        body_world = np.array(cache.GetLocalToWorldTransform(ancestor)).T if ancestor else np.eye(4)
        # Bake all affine geometry transforms into vertices, leaving a rigid body pose on the wire.
        mesh.apply_transform(np.linalg.inv(body_world) @ world)
        colors = pxr.UsdGeom.Gprim(prim).GetDisplayColorAttr().Get()
        rgba = [*(list(colors[0]) if colors else [0.7, 0.7, 0.7]), 1.0]
        index = len(visuals)
        visuals.append(Visual(index, visual_body, np.eye(4)))
        manifest["meshes"].append(
            {
                "id": str(index),
                "vertices": (100 * mesh.vertices @ BASIS).tolist(),
                "normals": (mesh.vertex_normals @ BASIS).tolist(),
                "triangles": mesh.faces.ravel().tolist(),
            }
        )
        manifest["geoms"].append(
            {
                "id": index,
                "mesh_id": str(index),
                "rgba": rgba,
                "roughness": 0.7,
                **unreal_pose(np.eye(4)),
            }
        )
    return manifest, visuals, set(authored_bodies)
