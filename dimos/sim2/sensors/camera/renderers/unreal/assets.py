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

"""URDF visuals in body-local metres; only the Unreal wire uses left-handed cm."""

from dataclasses import dataclass
from typing import Any
import xml.etree.ElementTree as ET

from dimos_lcm.geometry_msgs import Pose
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation
import trimesh

from dimos.sim2.description import SceneConfig
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
    scene: SceneConfig, authored_instances: tuple[str, ...]
) -> tuple[dict[str, Any], list[Visual], set[str]]:
    """Load once, keeping each visual attached to the shared model's named body.

    Authored instances already exist in the native level. They must be fixed,
    single-link models at the authored origin; their poses are checked on every
    sample. URDF mesh textures are not translated; native level materials remain
    untouched and procedural robot visuals use the URDF's diffuse color.
    """
    if not set(authored_instances) <= {i.id for i in scene.instances}:
        raise ValueError("authored instance is absent from the physics scene")
    manifest: dict[str, Any] = {"version": 2, "meshes": [], "geoms": []}
    visuals: list[Visual] = []
    static_bodies: set[str] = set()
    if scene.ground_plane:
        # The camera's far plane is 1 km; this quad covers its visible ground.
        manifest["meshes"].append(
            {
                "id": "ground",
                "vertices": [
                    [x, y, 0]
                    for x, y in [
                        (-200000, -200000),
                        (200000, -200000),
                        (200000, 200000),
                        (-200000, 200000),
                    ]
                ],
                "normals": [[0, 0, 1]] * 4,
                "triangles": [0, 1, 2, 0, 2, 3],
            }
        )
        manifest["geoms"].append(
            {
                "id": 0,
                "mesh_id": "ground",
                "rgba": [0.25, 0.3, 0.35, 1],
                "roughness": 0.7,
                **unreal_pose(np.eye(4)),
            }
        )
        visuals.append(Visual(0, None, np.eye(4)))
    for instance in scene.instances:
        model = instance.description
        if instance.id in authored_instances:
            if instance.floating or model.joints or any((*instance.xyz, *instance.rpy)):
                raise ValueError("authored background must be one fixed body at its saved origin")
            static_bodies.add(model.root_link)
            continue
        root = ET.fromstring(model.xml)
        materials = {m.get("name"): m for m in root.findall("material")}
        for link in root.findall("link"):
            for shape in link.findall("visual"):
                geom = shape.find("geometry")
                if geom is None or len(geom) != 1:
                    raise ValueError("visual must have exactly one geometry")
                primitive = geom[0]
                match primitive.tag:
                    case "mesh":
                        mesh = trimesh.load_mesh(primitive.attrib["filename"])
                        mesh.vertices = mesh.vertices * np.fromstring(
                            primitive.get("scale", "1 1 1"), sep=" "
                        )
                    case "box":
                        mesh = trimesh.creation.box(
                            np.fromstring(primitive.attrib["size"], sep=" ")
                        )
                    case "cylinder":
                        mesh = trimesh.creation.cylinder(
                            float(primitive.attrib["radius"]), float(primitive.attrib["length"])
                        )
                    case "sphere":
                        mesh = trimesh.creation.icosphere(radius=float(primitive.attrib["radius"]))
                    case _:
                        raise ValueError(f"unsupported URDF geometry: {primitive.tag}")
                local = np.eye(4)
                origin = shape.find("origin")
                if origin is not None:
                    local[:3, 3] = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
                    local[:3, :3] = Rotation.from_euler(
                        "xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
                    ).as_matrix()
                material = shape.find("material")
                if material is not None and material.find("color") is None:
                    material = materials.get(material.get("name"))
                color = None if material is None else material.find("color")
                rgba = (
                    [0.7, 0.7, 0.7, 1.0]
                    if color is None
                    else [float(v) for v in color.attrib["rgba"].split()]
                )
                if rgba[3] != 1:
                    raise ValueError("procedural Unreal visuals currently require opaque materials")
                index = len(visuals)
                visuals.append(Visual(index, link.attrib["name"], local))
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
                        **unreal_pose(local),
                    }
                )
    return manifest, visuals, static_bodies
