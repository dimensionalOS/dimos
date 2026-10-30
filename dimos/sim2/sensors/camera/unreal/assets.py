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

"""Convert neutral visual assets and body poses into Unreal's rendering wire.

The wire uses left-handed centimetres and xyzw quaternions. Physics engines do
not appear here: geometry stays local to a stable body ID, whose pose comes from
WorldState. Only this rendering boundary changes coordinate convention.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.sim2.physics import WorldState
from dimos.sim2.visuals import VisualAttachment, VisualCamera, VisualScene

_BASIS = np.diag([1.0, -1.0, 1.0])
_OPTICAL_AXES = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])


def attachment_pose(
    attachment: VisualAttachment, state: WorldState
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Compose a body's world pose and an authored local attachment."""
    if attachment.body == "world":
        position, rotation = np.zeros(3), np.eye(3)
    else:
        body = state.bodies[attachment.body]
        position = np.asarray(body.position)
        rotation = Rotation.from_quat(body.orientation).as_matrix()
    return (
        position + rotation @ np.asarray(attachment.position),
        rotation @ Rotation.from_quat(attachment.orientation).as_matrix(),
    )


def _pose(position: NDArray[np.float64], rotation: NDArray[np.float64]) -> dict[str, Any]:
    return {
        "position": (100.0 * (_BASIS @ position)).tolist(),
        "quaternion": Rotation.from_matrix(rotation).as_quat().tolist(),
    }


def geom_poses(
    visuals: VisualScene, state: WorldState, geom_ids: Sequence[int] | None = None
) -> list[dict[str, Any]]:
    """Resolve visual placements from body state without reconstructing physics."""
    selected = None if geom_ids is None else set(geom_ids)
    poses = []
    for geometry in visuals.geometries:
        if selected is not None and geometry.id not in selected:
            continue
        position, rotation = attachment_pose(geometry.attachment, state)
        poses.append({"id": geometry.id, **_pose(position, _BASIS @ rotation @ _BASIS)})
    return poses


def camera_pose(position: NDArray[np.float64], optical: NDArray[np.float64]) -> dict[str, Any]:
    """Convert world optical axes (+X right,+Y down,+Z forward) to Unreal."""
    return _pose(np.asarray(position), _BASIS @ optical @ _OPTICAL_AXES)


def camera_parameters(
    camera: VisualCamera, state: WorldState, width: int, height: int
) -> tuple[dict[str, Any], NDArray[np.float64], NDArray[np.float64]]:
    """Return calibrated perspective parameters and the world optical transform."""
    if width <= 0 or height <= 0:
        raise ValueError("camera image dimensions must be positive")
    if not camera.perspective:
        raise ValueError("Unreal camera bridge currently requires a perspective camera")
    if not camera.fixed:
        raise ValueError("tracking cameras require a backend-neutral camera trajectory")
    position, optical = attachment_pose(camera.attachment, state)
    if camera.focal_ratio is None:
        fx = fy = height / (2 * np.tan(np.deg2rad(camera.fovy) / 2))
    else:
        fx, fy = np.asarray(camera.focal_ratio) * (width, height)
    cx, cy = (np.array((width, height)) - 1) / 2 + np.asarray(camera.principal_offset) * (
        width,
        height,
    )
    calibration = {
        **camera_pose(position, optical),
        "width": width,
        "height": height,
        "fx": float(fx),
        "fy": float(fy),
        "cx": float(cx),
        "cy": float(cy),
    }
    return calibration, position, Rotation.from_matrix(optical).as_quat()


def export_scene(visuals: VisualScene, state: WorldState) -> dict[str, Any]:
    """Build the Unreal manifest; native asset materials stay owned by Unreal."""
    meshes = [
        {
            "id": name,
            "vertices": (100.0 * (mesh.vertices @ _BASIS)).tolist(),
            "triangles": mesh.triangles.ravel().tolist(),
            "normals": (mesh.normals @ _BASIS).tolist(),
        }
        for name, mesh in visuals.meshes.items()
    ]
    poses = {pose["id"]: pose for pose in geom_poses(visuals, state)}
    geometries = []
    for geometry in visuals.geometries:
        asset = geometry.native_asset
        if asset is not None:
            if asset.renderer != "unreal":
                raise ValueError(f"native asset belongs to renderer {asset.renderer!r}")
            geometries.append(
                {**poses[geometry.id], "static_mesh": asset.reference, "scale": list(asset.scale)}
            )
        else:
            if geometry.mesh_id not in visuals.meshes:
                raise ValueError(f"visual {geometry.id} has no mesh asset")
            geometries.append(
                {
                    **poses[geometry.id],
                    "mesh_id": geometry.mesh_id,
                    "rgba": list(geometry.rgba),
                    "roughness": geometry.roughness,
                }
            )
    return {"version": 1, "meshes": meshes, "geoms": geometries}
