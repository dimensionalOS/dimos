# Copyright 2025-2026 Dimensional Inc.
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

from __future__ import annotations

import mimetypes
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

WS_MSG_CAMERA = 0x01


def dimos_joint_to_mjcf(name: str) -> str:
    parts = name.split("/", 1)
    suffix = parts[1] if len(parts) > 1 else parts[0]
    if suffix.endswith("_joint"):
        return suffix
    return f"{suffix}_joint"


def canonical_joint_name(name: str) -> str:
    if "/" in name:
        name = name.split("/", 1)[1]
    if name.endswith("_joint"):
        name = name[: -len("_joint")]
    return name


def compose_scene_mesh_wxyz(
    *, y_up: bool, rotation_zyx_deg: tuple[float, float, float]
) -> tuple[float, float, float, float]:
    matrix = np.eye(3, dtype=np.float64)
    if y_up:
        matrix = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=np.float64)

    rz, ry, rx = (np.deg2rad(angle) for angle in rotation_zyx_deg)
    cz, sz = np.cos(rz), np.sin(rz)
    cy, sy = np.cos(ry), np.sin(ry)
    cx, sx = np.cos(rx), np.sin(rx)
    rotate_z = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=np.float64)
    rotate_y = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=np.float64)
    rotate_x = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=np.float64)
    matrix = rotate_z @ rotate_y @ rotate_x @ matrix

    x, y, z, w = Rotation.from_matrix(matrix).as_quat()
    return (float(w), float(x), float(y), float(z))


def path_contains(parent: Path, child: Path) -> bool:
    try:
        child.resolve().relative_to(parent.resolve())
    except ValueError:
        return False
    return True


def media_type(path: Path) -> str | None:
    match path.suffix.lower():
        case ".glb":
            return "model/gltf-binary"
        case ".gltf":
            return "model/gltf+json"
        case _:
            return mimetypes.guess_type(path.name)[0]
