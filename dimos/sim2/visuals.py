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

"""Static visual assets and local frames, independent of the physics engine.

Mesh vertices and attachments use right-handed metres and xyzw quaternions.
Camera attachment frames use optical axes: +X right, +Y down, +Z forward.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from dimos.sim2.physics import Quat, Vec3


@dataclass(frozen=True)
class VisualAttachment:
    body: str
    position: Vec3 = (0.0, 0.0, 0.0)
    orientation: Quat = (0.0, 0.0, 0.0, 1.0)


@dataclass(frozen=True)
class VisualMesh:
    vertices: NDArray[np.float64]
    triangles: NDArray[np.int64]
    normals: NDArray[np.float64]


@dataclass(frozen=True)
class NativeVisualAsset:
    renderer: str
    reference: str
    scale: Vec3


@dataclass(frozen=True)
class VisualGeometry:
    id: int
    attachment: VisualAttachment
    mesh_id: str | None = None
    rgba: tuple[float, float, float, float] = (0.5, 0.5, 0.5, 1.0)
    roughness: float = 0.6
    native_asset: NativeVisualAsset | None = None


@dataclass(frozen=True)
class VisualCamera:
    attachment: VisualAttachment
    fovy: float
    focal_ratio: tuple[float, float] | None = None
    principal_offset: tuple[float, float] = (0.0, 0.0)
    perspective: bool = True
    fixed: bool = True


@dataclass(frozen=True)
class VisualScene:
    revision: int
    meshes: dict[str, VisualMesh]
    geometries: tuple[VisualGeometry, ...]
    frames: dict[str, VisualAttachment]
    cameras: dict[str, VisualCamera]
