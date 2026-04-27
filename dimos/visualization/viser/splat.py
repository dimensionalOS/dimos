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

"""3DGS PLY loader for Viser.

Reads a Gaussian splat PLY (3DGS-format), applies an alignment transform
(rotation + translation + uniform scale), and returns the
``(centers, covariances, rgbs, opacities)`` arrays Viser's
``add_gaussian_splats`` expects.

The alignment transform is needed because 3DGS splats trained from
COLMAP come out in the COLMAP world frame, which is Y-up and in
arbitrary scale.  Dimos uses Z-up metric.  See
``data/scenes/<scene>.yaml`` for per-scene alignment values.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from pathlib import Path

# 3DGS spherical harmonic DC coefficient.
_SH_C0 = 0.28209479177387814

# Y-up (3DGS / COLMAP) -> Z-up (dimos).  (x, y, z) -> (x, z, -y).
_Y_UP_TO_Z_UP = np.array(
    [[1, 0, 0], [0, 0, 1], [0, -1, 0]],
    dtype=np.float32,
)


@dataclass
class SplatAlignment:
    """Per-scene splat alignment.

    Loaded from a YAML next to the PLY:

        scale: 1.0           # multiplicative; COLMAP units -> meters
        translation: [0, 0, 0]  # post-rotation offset in world frame, meters
        rotation_zyx: [0, 0, 0] # extra yaw/pitch/roll in degrees, applied after Y->Z
        y_up: true           # apply standard Y-up -> Z-up
    """

    scale: float = 1.0
    translation: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rotation_zyx_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)
    y_up: bool = True

    @classmethod
    def from_yaml(cls, path: str | Path) -> SplatAlignment:
        import yaml

        with open(path) as f:
            d = yaml.safe_load(f) or {}
        return cls(
            scale=float(d.get("scale", 1.0)),
            translation=tuple(d.get("translation", [0.0, 0.0, 0.0])),  # type: ignore[arg-type]
            rotation_zyx_deg=tuple(d.get("rotation_zyx", [0.0, 0.0, 0.0])),  # type: ignore[arg-type]
            y_up=bool(d.get("y_up", True)),
        )

    def world_from_splat(self) -> np.ndarray:
        """3x3 rotation: scaled rotation that maps splat-frame -> world-frame."""
        R = _Y_UP_TO_Z_UP if self.y_up else np.eye(3, dtype=np.float32)
        rz, ry, rx = (np.deg2rad(a) for a in self.rotation_zyx_deg)
        cz, sz = np.cos(rz), np.sin(rz)
        cy, sy = np.cos(ry), np.sin(ry)
        cx, sx = np.cos(rx), np.sin(rx)
        Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=np.float32)
        Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=np.float32)
        Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=np.float32)
        return (Rz @ Ry @ Rx @ R).astype(np.float32)


@dataclass
class SplatData:
    centers: np.ndarray  # (N, 3) float32, world frame
    covariances: np.ndarray  # (N, 3, 3) float32, world frame.  For viser.
    rgbs: np.ndarray  # (N, 3) float32, [0, 1]
    opacities: np.ndarray  # (N, 1) float32, [0, 1]
    # Primitive form for renderers that prefer it (e.g. gsplat).
    # All in the same world frame as ``centers``/``covariances``.
    quats_wxyz: np.ndarray  # (N, 4) float32
    scales: np.ndarray  # (N, 3) float32, post-alignment uniform-scaled


def load_splat(ply_path: str | Path, alignment: SplatAlignment | None = None) -> SplatData:
    """Load a 3DGS PLY and bake the alignment into centers + covariances.

    Returns arrays in dimos world frame, ready to feed straight into
    ``viser.scene.add_gaussian_splats``.  Heavy: ~1-2 GB for a 250 MB PLY,
    so do this once at module start, not per frame.
    """
    if alignment is None:
        alignment = SplatAlignment()
    from plyfile import PlyData

    ply = PlyData.read(str(ply_path))
    v = ply["vertex"]
    props = {p.name for p in v.properties}

    pos = np.stack(
        [v["x"], v["y"], v["z"]],
        axis=-1,
    ).astype(np.float32)

    if "rot_0" in props:
        rw = v["rot_0"].astype(np.float32)
        rx = v["rot_1"].astype(np.float32)
        ry = v["rot_2"].astype(np.float32)
        rz = v["rot_3"].astype(np.float32)
        n = np.sqrt(rw * rw + rx * rx + ry * ry + rz * rz).clip(1e-12)
        rw, rx, ry, rz = rw / n, rx / n, ry / n, rz / n
        R = np.empty((len(rw), 3, 3), dtype=np.float32)
        R[:, 0, 0] = 1 - 2 * (ry * ry + rz * rz)
        R[:, 0, 1] = 2 * (rx * ry - rw * rz)
        R[:, 0, 2] = 2 * (rx * rz + rw * ry)
        R[:, 1, 0] = 2 * (rx * ry + rw * rz)
        R[:, 1, 1] = 1 - 2 * (rx * rx + rz * rz)
        R[:, 1, 2] = 2 * (ry * rz - rw * rx)
        R[:, 2, 0] = 2 * (rx * rz - rw * ry)
        R[:, 2, 1] = 2 * (ry * rz + rw * rx)
        R[:, 2, 2] = 1 - 2 * (rx * rx + ry * ry)
    else:
        R = np.tile(np.eye(3, dtype=np.float32), (len(pos), 1, 1))

    if "scale_0" in props:
        s = np.exp(np.stack([v["scale_0"], v["scale_1"], v["scale_2"]], axis=-1).astype(np.float32))
    else:
        s = np.ones((len(pos), 3), dtype=np.float32)

    if "opacity" in props:
        op = (1.0 / (1.0 + np.exp(-v["opacity"].astype(np.float32)))).reshape(-1, 1)
    else:
        op = np.ones((len(pos), 1), dtype=np.float32)

    if all(f"f_dc_{i}" in props for i in range(3)):
        sh_dc = np.stack(
            [v["f_dc_0"], v["f_dc_1"], v["f_dc_2"]],
            axis=-1,
        ).astype(np.float32)
        rgbs = (_SH_C0 * sh_dc + 0.5).clip(0.0, 1.0).astype(np.float32)
    else:
        rgbs = np.ones((len(pos), 3), dtype=np.float32)

    # Covariance in original splat frame: C = R diag(s^2) R^T.
    cov_splat = np.einsum("nij,nj,nkj->nik", R, s * s, R)

    # Apply alignment.  Centers: world = scale * (M @ splat) + t.  Covariances:
    # transform as bilinear forms: C' = scale^2 * M C M^T (translation drops).
    M = alignment.world_from_splat()
    centers = (alignment.scale * pos @ M.T) + np.asarray(alignment.translation, dtype=np.float32)
    covariances = (alignment.scale * alignment.scale) * np.einsum("ij,njk,lk->nil", M, cov_splat, M)

    # Same alignment applied to the primitive (quat, scale) form.  Renderers
    # like gsplat operate directly on these instead of full covariances.
    quats_splat = np.stack([rw, rx, ry, rz], axis=-1) if "rot_0" in props else None
    if quats_splat is None:
        quats_world = np.tile(np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32), (len(pos), 1))
    else:
        # Rotate every Gaussian's local frame by M: q_world = q_M ⊗ q_splat.
        qM = _mat3_to_wxyz(M)
        quats_world = _quat_mul_left(qM, quats_splat).astype(np.float32)
    scales_world = (alignment.scale * s).astype(np.float32)

    return SplatData(
        centers=centers,
        covariances=covariances,
        rgbs=rgbs,
        opacities=op,
        quats_wxyz=quats_world,
        scales=scales_world,
    )


def _mat3_to_wxyz(R: np.ndarray) -> np.ndarray:
    """3x3 rotation matrix -> (w, x, y, z), via mujoco for numerical stability."""
    import mujoco

    out = np.zeros(4, dtype=np.float64)
    mujoco.mju_mat2Quat(out, np.asarray(R, dtype=np.float64).flatten())
    return out.astype(np.float32)


def _quat_mul_left(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Left-multiply ``q1 ⊗ q2`` for q1: (4,) wxyz and q2: (N, 4) wxyz.

    Vectorised — the per-Gaussian alternative through ``mujoco.mju_mulQuat``
    in a Python loop is ~50x slower for 1M Gaussians.
    """
    aw, ax, ay, az = q1[0], q1[1], q1[2], q1[3]
    bw, bx, by, bz = q2[:, 0], q2[:, 1], q2[:, 2], q2[:, 3]
    return np.stack(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        axis=-1,
    )
