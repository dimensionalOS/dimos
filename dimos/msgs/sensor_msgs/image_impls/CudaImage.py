# Copyright 2025 Dimensional Inc.
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

import time
from dataclasses import dataclass, field
from typing import Optional, Tuple

import cv2
import numpy as np

from dimos.msgs.sensor_msgs.image_impls.AbstractImage import (
    AbstractImage,
    ImageFormat,
    HAS_CUDA,
    _is_cu,
    _to_cpu,
    _ascontig,
)
from dimos.msgs.sensor_msgs.image_impls.NumpyImage import NumpyImage

try:
    import cupy as cp  # type: ignore
    from cupyx.scipy import ndimage as cndimage  # type: ignore
    from cupyx.scipy import signal as csignal  # type: ignore
except Exception:  # pragma: no cover
    cp = None  # type: ignore
    cndimage = None  # type: ignore
    csignal = None  # type: ignore


def _bgr_to_rgb_cuda(img):
    return img[..., ::-1]


def _rgb_to_bgr_cuda(img):
    return img[..., ::-1]


def _bgra_to_rgba_cuda(img):
    out = img.copy()
    out[..., 0], out[..., 2] = img[..., 2], img[..., 0]
    return out


def _rgba_to_bgra_cuda(img):
    out = img.copy()
    out[..., 0], out[..., 2] = img[..., 2], img[..., 0]
    return out


def _gray_to_rgb_cuda(gray):
    return cp.stack([gray, gray, gray], axis=-1)  # type: ignore


def _rgb_to_gray_cuda(rgb):
    r = rgb[..., 0].astype(cp.float32)  # type: ignore
    g = rgb[..., 1].astype(cp.float32)  # type: ignore
    b = rgb[..., 2].astype(cp.float32)  # type: ignore
    y = 0.299 * r + 0.587 * g + 0.114 * b
    if rgb.dtype == cp.uint8:  # type: ignore
        y = cp.clip(y, 0, 255).astype(cp.uint8)  # type: ignore
    return y


def _resize_bilinear_hwc_cuda(img, out_h: int, out_w: int):
    if cp is None or cndimage is None:
        raise RuntimeError("CuPy/CUDA not available")
    if img.ndim not in (2, 3):
        raise ValueError("Expected HxW or HxWxC array")

    work = img[..., None] if img.ndim == 2 else img
    squeezed = work is not img
    in_h, in_w = work.shape[:2]
    if (in_h, in_w) == (out_h, out_w):
        return img.copy()

    zoom = (out_h / in_h, out_w / in_w, 1.0)
    out = cndimage.zoom(
        work.astype(cp.float32, copy=False),
        zoom=zoom,
        order=1,
        mode="nearest",
        prefilter=False,
        grid_mode=True,
    )

    if squeezed:
        out = out[..., 0]
    if img.dtype == cp.uint8:
        out = cp.clip(out, 0, 255).astype(cp.uint8, copy=False)
    elif out.dtype != img.dtype:
        out = out.astype(img.dtype, copy=False)
    return out


def _rodrigues(x, inverse: bool = False):
    """Unified Rodrigues transform (vector<->matrix) for NumPy/CuPy arrays."""

    if cp is not None and (
        isinstance(x, cp.ndarray)  # type: ignore[arg-type]
        or getattr(x, "__cuda_array_interface__", None) is not None
    ):
        xp = cp
    else:
        xp = np
    arr = xp.asarray(x, dtype=xp.float64)

    if not inverse and arr.ndim >= 2 and arr.shape[-2:] == (3, 3):
        inverse = True

    if not inverse:
        vec = arr
        if vec.ndim >= 2 and vec.shape[-1] == 1:
            vec = vec[..., 0]
        if vec.shape[-1] != 3:
            raise ValueError("Rodrigues expects vectors of shape (..., 3)")
        orig_shape = vec.shape[:-1]
        vec = vec.reshape(-1, 3)
        n = vec.shape[0]
        theta = xp.linalg.norm(vec, axis=1)
        small = theta < 1e-12

        def _skew(v):
            vx, vy, vz = v[:, 0], v[:, 1], v[:, 2]
            O = xp.zeros_like(vx)
            return xp.stack(
                [
                    xp.stack([O, -vz, vy], axis=-1),
                    xp.stack([vz, O, -vx], axis=-1),
                    xp.stack([-vy, vx, O], axis=-1),
                ],
                axis=-2,
            )

        K = _skew(vec)
        theta2 = theta * theta
        theta4 = theta2 * theta2
        theta_safe = xp.where(small, 1.0, theta)
        theta2_safe = xp.where(small, 1.0, theta2)
        A = xp.where(small, 1.0 - theta2 / 6.0 + theta4 / 120.0, xp.sin(theta) / theta_safe)[:, None, None]
        B = xp.where(
            small,
            0.5 - theta2 / 24.0 + theta4 / 720.0,
            (1.0 - xp.cos(theta)) / theta2_safe,
        )[:, None, None]
        I = xp.eye(3, dtype=arr.dtype)
        I = I[None, :, :] if n == 1 else xp.broadcast_to(I, (n, 3, 3))
        KK = xp.matmul(K, K)
        out = I + A * K + B * KK
        return out.reshape(orig_shape + (3, 3)) if orig_shape else out[0]

    mat = arr
    if mat.shape[-2:] != (3, 3):
        raise ValueError("Rodrigues expects rotation matrices of shape (..., 3, 3)")
    orig_shape = mat.shape[:-2]
    mat = mat.reshape(-1, 3, 3)
    trace = xp.trace(mat, axis1=1, axis2=2)
    trace = xp.clip((trace - 1.0) / 2.0, -1.0, 1.0)
    theta = xp.arccos(trace)
    v = xp.stack(
        [
            mat[:, 2, 1] - mat[:, 1, 2],
            mat[:, 0, 2] - mat[:, 2, 0],
            mat[:, 1, 0] - mat[:, 0, 1],
        ],
        axis=1,
    )
    norm_v = xp.linalg.norm(v, axis=1)
    small = theta < 1e-7
    eps = 1e-8
    norm_safe = xp.where(norm_v < eps, 1.0, norm_v)
    r_general = theta[:, None] * v / norm_safe[:, None]
    r_small = 0.5 * v
    r = xp.where(small[:, None], r_small, r_general)
    pi_mask = xp.abs(theta - xp.pi) < 1e-4
    if (np.any(pi_mask) if xp is np else bool(cp.asnumpy(pi_mask).any())):
        diag = xp.diagonal(mat, axis1=1, axis2=2)
        axis_candidates = xp.clip((diag + 1.0) / 2.0, 0.0, None)
        axis = xp.sqrt(axis_candidates)
        signs = xp.sign(v)
        axis = xp.where(signs == 0, axis, xp.copysign(axis, signs))
        axis_norm = xp.linalg.norm(axis, axis=1)
        axis_norm = xp.where(axis_norm < eps, 1.0, axis_norm)
        axis = axis / axis_norm[:, None]
        r_pi = theta[:, None] * axis
        r = xp.where(pi_mask[:, None], r_pi, r)
    out = r.reshape(orig_shape + (3,)) if orig_shape else r[0]
    return out


def _undistort_points_cuda(
    img_px: "cp.ndarray", K: "cp.ndarray", dist: "cp.ndarray", iterations: int = 8
) -> "cp.ndarray":
    """Iteratively undistort pixel coordinates on device (Brown–Conrady).

    Returns pixel coordinates after undistortion (fx*xu+cx, fy*yu+cy).
    """
    N = img_px.shape[0]
    ones = cp.ones((N, 1), dtype=cp.float64)
    uv1 = cp.concatenate([img_px.astype(cp.float64), ones], axis=1)
    Kinv = cp.linalg.inv(K)
    xdyd1 = uv1 @ Kinv.T
    xd = xdyd1[:, 0]
    yd = xdyd1[:, 1]
    xu = xd.copy()
    yu = yd.copy()
    k1 = dist[0]
    k2 = dist[1] if dist.size > 1 else 0.0
    p1 = dist[2] if dist.size > 2 else 0.0
    p2 = dist[3] if dist.size > 3 else 0.0
    k3 = dist[4] if dist.size > 4 else 0.0
    for _ in range(iterations):
        r2 = xu * xu + yu * yu
        r4 = r2 * r2
        r6 = r4 * r2
        radial = 1.0 + k1 * r2 + k2 * r4 + k3 * r6
        delta_x = 2.0 * p1 * xu * yu + p2 * (r2 + 2.0 * xu * xu)
        delta_y = p1 * (r2 + 2.0 * yu * yu) + 2.0 * p2 * xu * yu
        xu = (xd - delta_x) / radial
        yu = (yd - delta_y) / radial
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    return cp.stack([fx * xu + cx, fy * yu + cy], axis=1)


@dataclass
class CudaImage(AbstractImage):
    data: any  # cupy.ndarray
    format: ImageFormat = field(default=ImageFormat.BGR)
    frame_id: str = field(default="")
    ts: float = field(default_factory=time.time)

    def __post_init__(self):
        if not HAS_CUDA or cp is None:
            raise RuntimeError("CuPy/CUDA not available")
        if not _is_cu(self.data):
            # Accept NumPy arrays and move to device automatically
            try:
                self.data = cp.asarray(self.data)
            except Exception as e:
                raise ValueError("CudaImage requires a CuPy array") from e
        if self.data.ndim < 2:
            raise ValueError("Image data must be at least 2D")
        self.data = _ascontig(self.data)

    @property
    def is_cuda(self) -> bool:
        return True

    def to_opencv(self) -> np.ndarray:
        if self.format in (ImageFormat.BGR, ImageFormat.RGB, ImageFormat.RGBA, ImageFormat.BGRA):
            return _to_cpu(self.to_bgr().data)
        return _to_cpu(self.data)

    def to_rgb(self) -> "CudaImage":
        if self.format == ImageFormat.RGB:
            return self.copy()  # type: ignore
        if self.format == ImageFormat.BGR:
            return CudaImage(_bgr_to_rgb_cuda(self.data), ImageFormat.RGB, self.frame_id, self.ts)
        if self.format == ImageFormat.RGBA:
            return self.copy()  # type: ignore
        if self.format == ImageFormat.BGRA:
            return CudaImage(
                _bgra_to_rgba_cuda(self.data), ImageFormat.RGBA, self.frame_id, self.ts
            )
        if self.format == ImageFormat.GRAY:
            return CudaImage(_gray_to_rgb_cuda(self.data), ImageFormat.RGB, self.frame_id, self.ts)
        if self.format in (ImageFormat.GRAY16, ImageFormat.DEPTH16):
            gray8 = (self.data.astype(cp.float32) / 256.0).clip(0, 255).astype(cp.uint8)  # type: ignore
            return CudaImage(_gray_to_rgb_cuda(gray8), ImageFormat.RGB, self.frame_id, self.ts)
        return self.copy()  # type: ignore

    def to_bgr(self) -> "CudaImage":
        if self.format == ImageFormat.BGR:
            return self.copy()  # type: ignore
        if self.format == ImageFormat.RGB:
            return CudaImage(_rgb_to_bgr_cuda(self.data), ImageFormat.BGR, self.frame_id, self.ts)
        if self.format == ImageFormat.RGBA:
            return CudaImage(
                _rgba_to_bgra_cuda(self.data)[..., :3], ImageFormat.BGR, self.frame_id, self.ts
            )
        if self.format == ImageFormat.BGRA:
            return CudaImage(self.data[..., :3], ImageFormat.BGR, self.frame_id, self.ts)
        if self.format in (ImageFormat.GRAY, ImageFormat.DEPTH):
            return CudaImage(
                _rgb_to_bgr_cuda(_gray_to_rgb_cuda(self.data)),
                ImageFormat.BGR,
                self.frame_id,
                self.ts,
            )
        if self.format in (ImageFormat.GRAY16, ImageFormat.DEPTH16):
            gray8 = (self.data.astype(cp.float32) / 256.0).clip(0, 255).astype(cp.uint8)  # type: ignore
            return CudaImage(
                _rgb_to_bgr_cuda(_gray_to_rgb_cuda(gray8)), ImageFormat.BGR, self.frame_id, self.ts
            )
        return self.copy()  # type: ignore

    def to_grayscale(self) -> "CudaImage":
        if self.format in (ImageFormat.GRAY, ImageFormat.GRAY16, ImageFormat.DEPTH):
            return self.copy()  # type: ignore
        if self.format == ImageFormat.BGR:
            return CudaImage(
                _rgb_to_gray_cuda(_bgr_to_rgb_cuda(self.data)),
                ImageFormat.GRAY,
                self.frame_id,
                self.ts,
            )
        if self.format == ImageFormat.RGB:
            return CudaImage(_rgb_to_gray_cuda(self.data), ImageFormat.GRAY, self.frame_id, self.ts)
        if self.format in (ImageFormat.RGBA, ImageFormat.BGRA):
            rgb = (
                self.data[..., :3]
                if self.format == ImageFormat.RGBA
                else _bgra_to_rgba_cuda(self.data)[..., :3]
            )
            return CudaImage(_rgb_to_gray_cuda(rgb), ImageFormat.GRAY, self.frame_id, self.ts)
        raise ValueError(f"Unsupported format: {self.format}")

    def resize(self, width: int, height: int, interpolation: int = cv2.INTER_LINEAR) -> "CudaImage":
        return CudaImage(
            _resize_bilinear_hwc_cuda(self.data, height, width), self.format, self.frame_id, self.ts
        )

    def sharpness(self) -> float:
        if cp is None:
            return 0.0
        try:
            from cupyx.scipy import ndimage as cndimage  # type: ignore

            gray = self.to_grayscale().data.astype(cp.float32)
            deriv5 = cp.asarray([1, 2, 0, -2, -1], dtype=cp.float32)
            smooth5 = cp.asarray([1, 4, 6, 4, 1], dtype=cp.float32)
            gx = cndimage.convolve1d(gray, deriv5, axis=1, mode="reflect")  # type: ignore
            gx = cndimage.convolve1d(gx, smooth5, axis=0, mode="reflect")  # type: ignore
            gy = cndimage.convolve1d(gray, deriv5, axis=0, mode="reflect")  # type: ignore
            gy = cndimage.convolve1d(gy, smooth5, axis=1, mode="reflect")  # type: ignore
            magnitude = cp.hypot(gx, gy)  # type: ignore
            mean_mag = float(cp.asnumpy(magnitude.mean()))  # type: ignore
        except Exception:
            return 0.0
        if mean_mag <= 0:
            return 0.0
        return float(np.clip((np.log10(mean_mag + 1) - 1.7) / 2.0, 0.0, 1.0))

    # CUDA tracker (template NCC with small scale pyramid)
    @dataclass
    class BBox:
        x: int
        y: int
        w: int
        h: int
    def create_csrt_tracker(self, bbox: BBox):
        if csignal is None:
            raise RuntimeError("cupyx.scipy.signal not available for CUDA tracker")
        x, y, w, h = map(int, bbox)
        gray = self.to_grayscale().data.astype(cp.float32)
        tmpl = gray[y : y + h, x : x + w]
        if tmpl.size == 0:
            raise ValueError("Invalid bbox for CUDA tracker")
        return _CudaTemplateTracker(tmpl, x0=x, y0=y)

    def csrt_update(self, tracker) -> Tuple[bool, Tuple[int, int, int, int]]:
        if not isinstance(tracker, _CudaTemplateTracker):
            raise TypeError("Expected CUDA tracker instance")
        gray = self.to_grayscale().data.astype(cp.float32)
        x, y, w, h = tracker.update(gray)
        return True, (int(x), int(y), int(w), int(h))

    # PnP – Gauss–Newton (no distortion in batch), iterative per-instance
    def solve_pnp(
        self,
        object_points: np.ndarray,
        image_points: np.ndarray,
        camera_matrix: np.ndarray,
        dist_coeffs: Optional[np.ndarray] = None,
        flags: int = cv2.SOLVEPNP_ITERATIVE,
    ) -> Tuple[bool, np.ndarray, np.ndarray]:
        obj = cp.asarray(object_points, dtype=cp.float64)
        img = cp.asarray(image_points, dtype=cp.float64)
        K = cp.asarray(camera_matrix, dtype=cp.float64)
        if dist_coeffs is not None:
            dist = cp.asarray(dist_coeffs, dtype=cp.float64)
            img = _undistort_points_cuda(img, K, dist)
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        rvec = cp.zeros((3,), dtype=cp.float64)
        tvec = cp.array([0.0, 0.0, 2.0], dtype=cp.float64)

        def skew(v):
            vx, vy, vz = v[..., 0], v[..., 1], v[..., 2]
            O = cp.zeros_like(vx)
            return cp.stack(
                [
                    cp.stack([O, -vz, vy], axis=-1),
                    cp.stack([vz, O, -vx], axis=-1),
                    cp.stack([-vy, vx, O], axis=-1),
                ],
                axis=-2,
            )

        for _ in range(30):
            R = _rodrigues(rvec)
            Xc = (obj @ R.T) + tvec
            X, Y, Z = Xc[:, 0], Xc[:, 1], Xc[:, 2]
            invZ = 1.0 / cp.clip(Z, 1e-9, None)
            u_hat = fx * X * invZ + cx
            v_hat = fy * Y * invZ + cy
            r_u = img[:, 0] - u_hat
            r_v = img[:, 1] - v_hat
            r = cp.stack([r_u, r_v], axis=1).reshape(-1)
            du_dX, du_dY, du_dZ = fx * invZ, cp.zeros_like(invZ), -fx * X * invZ * invZ
            dv_dX, dv_dY, dv_dZ = cp.zeros_like(invZ), fy * invZ, -fy * Y * invZ * invZ
            Xi = obj
            Xi_skew = skew(Xi)
            R_rep = R[None, :, :].repeat(Xi.shape[0], axis=0)
            dXc_drot = -cp.matmul(R_rep, Xi_skew)
            J = cp.zeros((2 * Xi.shape[0], 6), dtype=cp.float64)
            u_grad = cp.stack([du_dX, du_dY, du_dZ], axis=1)[:, None, :]
            v_grad = cp.stack([dv_dX, dv_dY, dv_dZ], axis=1)[:, None, :]
            Ju = cp.matmul(u_grad, dXc_drot).reshape(Xi.shape[0], 3)
            Jv = cp.matmul(v_grad, dXc_drot).reshape(Xi.shape[0], 3)
            J[0::2, 0:3] = Ju
            J[1::2, 0:3] = Jv
            J[0::2, 3:6] = cp.stack([du_dX, du_dY, du_dZ], axis=1)
            J[1::2, 3:6] = cp.stack([dv_dX, dv_dY, dv_dZ], axis=1)
            JT = J.T
            JTJ = JT @ J
            JTr = JT @ r
            delta = cp.linalg.solve(JTJ + 1e-6 * cp.eye(6, dtype=cp.float64), JTr)
            rvec = rvec + delta[0:3]
            tvec = tvec + delta[3:6]
            if cp.linalg.norm(delta) < 1e-9:
                break
        return True, cp.asnumpy(rvec).reshape(3, 1), cp.asnumpy(tvec).reshape(3, 1)

    def solve_pnp_batch(
        self,
        object_points_batch: np.ndarray,
        image_points_batch: np.ndarray,
        camera_matrix: np.ndarray,
        dist_coeffs: Optional[np.ndarray] = None,
        iterations: int = 15,
        damping: float = 1e-6,
    ) -> Tuple[np.ndarray, np.ndarray]:
        obj_b = cp.asarray(object_points_batch, dtype=cp.float64)
        img_b = cp.asarray(image_points_batch, dtype=cp.float64)
        if camera_matrix.ndim == 2:
            K_b = cp.broadcast_to(
                cp.asarray(camera_matrix, dtype=cp.float64), (obj_b.shape[0], 3, 3)
            )
        else:
            K_b = cp.asarray(camera_matrix, dtype=cp.float64)
        if dist_coeffs is not None:
            raise NotImplementedError("Batch PnP with distortion not yet supported")
        B, N, _ = obj_b.shape
        fx, fy = K_b[:, 0, 0], K_b[:, 1, 1]
        cx, cy = K_b[:, 0, 2], K_b[:, 1, 2]
        rvecs = cp.zeros((B, 3), dtype=cp.float64)
        tvecs = cp.tile(cp.array([[0.0, 0.0, 2.0]], dtype=cp.float64), (B, 1))

        def skew_batch(v):
            vx, vy, vz = v[..., 0], v[..., 1], v[..., 2]
            O = cp.zeros_like(vx)
            return cp.stack(
                [
                    cp.stack([O, -vz, vy], axis=-1),
                    cp.stack([vz, O, -vx], axis=-1),
                    cp.stack([-vy, vx, O], axis=-1),
                ],
                axis=-2,
            )

        for _ in range(int(iterations)):
            Rb = _rodrigues(rvecs)
            Xc = cp.einsum("bij,bnj->bni", Rb, obj_b) + tvecs[:, None, :]
            X, Y, Z = Xc[:, :, 0], Xc[:, :, 1], Xc[:, :, 2]
            invZ = 1.0 / cp.clip(Z, 1e-9, None)
            u_hat = fx[:, None] * X * invZ + cx[:, None]
            v_hat = fy[:, None] * Y * invZ + cy[:, None]
            r_u = img_b[:, :, 0] - u_hat
            r_v = img_b[:, :, 1] - v_hat
            du_dX = fx[:, None] * invZ
            du_dY = cp.zeros_like(du_dX)
            du_dZ = -fx[:, None] * X * invZ * invZ
            dv_dX = cp.zeros_like(du_dX)
            dv_dY = fy[:, None] * invZ
            dv_dZ = -fy[:, None] * Y * invZ * invZ
            Xi = obj_b
            Xi_skew = skew_batch(Xi.reshape(-1, 3)).reshape(B, N, 3, 3)
            R_rep = Rb[:, None, :, :].repeat(N, axis=1)
            dXc_drot = -cp.matmul(R_rep, Xi_skew)
            u_grad = cp.stack([du_dX, du_dY, du_dZ], axis=2)[:, :, None, :]
            v_grad = cp.stack([dv_dX, dv_dY, dv_dZ], axis=2)[:, :, None, :]
            Ju = cp.matmul(u_grad, dXc_drot).reshape(B, N, 3)
            Jv = cp.matmul(v_grad, dXc_drot).reshape(B, N, 3)
            Jut = cp.stack([du_dX, du_dY, du_dZ], axis=2)
            Jvt = cp.stack([dv_dX, dv_dY, dv_dZ], axis=2)
            J = cp.zeros((B, 2 * N, 6), dtype=cp.float64)
            J[:, 0::2, 0:3] = Ju
            J[:, 1::2, 0:3] = Jv
            J[:, 0::2, 3:6] = Jut
            J[:, 1::2, 3:6] = Jvt
            r = cp.stack([r_u, r_v], axis=2).reshape(B, -1)
            JT = cp.transpose(J, (0, 2, 1))
            JTJ = cp.matmul(JT, J)
            JTr = cp.einsum("bji,bi->bj", JT, r)
            I6 = cp.eye(6, dtype=cp.float64)
            deltas = cp.zeros((B, 6), dtype=cp.float64)
            for b in range(B):
                A = JTJ[b] + damping * I6
                deltas[b] = cp.linalg.solve(A, JTr[b])
            rvecs = rvecs + deltas[:, 0:3]
            tvecs = tvecs + deltas[:, 3:6]
            if cp.linalg.norm(deltas) < 1e-7:
                break
        return cp.asnumpy(rvecs).reshape(B, 3, 1), cp.asnumpy(tvecs).reshape(B, 3, 1)

    def solve_pnp_ransac(
        self,
        object_points: np.ndarray,
        image_points: np.ndarray,
        camera_matrix: np.ndarray,
        dist_coeffs: Optional[np.ndarray] = None,
        iterations_count: int = 100,
        reprojection_error: float = 3.0,
        confidence: float = 0.99,
        min_sample: int = 6,
    ) -> Tuple[bool, np.ndarray, np.ndarray, np.ndarray]:
        obj_all = cp.asarray(object_points, dtype=cp.float64)
        img_all = cp.asarray(image_points, dtype=cp.float64)
        K = cp.asarray(camera_matrix, dtype=cp.float64)
        if dist_coeffs is not None:
            img_all = _undistort_points_cuda(img_all, K, cp.asarray(dist_coeffs, dtype=cp.float64))
        B = obj_all.shape[0] if obj_all.ndim == 3 else None
        N = obj_all.shape[0] if obj_all.ndim == 2 else obj_all.shape[1]
        if obj_all.ndim == 2:
            obj_all = obj_all[None, ...]
            img_all = img_all[None, ...]
        rng = cp.random.RandomState(12345)
        best_inliers = -1
        best_r, best_t, best_mask = None, None, None
        iters = int(iterations_count)

        def gn(obj, img):
            # Small GN on device; returns rvec,tvec (cp)
            fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
            rvec = cp.zeros((3,), dtype=cp.float64)
            tvec = cp.array([0.0, 0.0, 2.0], dtype=cp.float64)

            def skew(v):
                vx, vy, vz = v[..., 0], v[..., 1], v[..., 2]
                O = cp.zeros_like(vx)
                return cp.stack(
                    [
                        cp.stack([O, -vz, vy], axis=-1),
                        cp.stack([vz, O, -vx], axis=-1),
                        cp.stack([-vy, vx, O], axis=-1),
                    ],
                    axis=-2,
                )

            for _ in range(20):
                R = _rodrigues(rvec)
                Xc = (obj @ R.T) + tvec
                X, Y, Z = Xc[:, 0], Xc[:, 1], Xc[:, 2]
                invZ = 1.0 / cp.clip(Z, 1e-9, None)
                u_hat = fx * X * invZ + cx
                v_hat = fy * Y * invZ + cy
                r_u = img[:, 0] - u_hat
                r_v = img[:, 1] - v_hat
                r = cp.stack([r_u, r_v], axis=1).reshape(-1)
                du_dX, du_dY, du_dZ = fx * invZ, cp.zeros_like(invZ), -fx * X * invZ * invZ
                dv_dX, dv_dY, dv_dZ = cp.zeros_like(invZ), fy * invZ, -fy * Y * invZ * invZ
                Xi_skew = skew(obj)
                R_rep = R[None, :, :].repeat(obj.shape[0], axis=0)
                dXc_drot = -cp.matmul(R_rep, Xi_skew)
                J = cp.zeros((2 * obj.shape[0], 6), dtype=cp.float64)
                u_grad = cp.stack([du_dX, du_dY, du_dZ], axis=1)[:, None, :]
                v_grad = cp.stack([dv_dX, dv_dY, dv_dZ], axis=1)[:, None, :]
                Ju = cp.matmul(u_grad, dXc_drot).reshape(obj.shape[0], 3)
                Jv = cp.matmul(v_grad, dXc_drot).reshape(obj.shape[0], 3)
                J[0::2, 0:3] = Ju
                J[1::2, 0:3] = Jv
                J[0::2, 3:6] = cp.stack([du_dX, du_dY, du_dZ], axis=1)
                J[1::2, 3:6] = cp.stack([dv_dX, dv_dY, dv_dZ], axis=1)
                delta = cp.linalg.solve(J.T @ J + 1e-6 * cp.eye(6, dtype=cp.float64), J.T @ r)
                rvec = rvec + delta[0:3]
                tvec = tvec + delta[3:6]
                if cp.linalg.norm(delta) < 1e-8:
                    break
            return rvec, tvec

        for _ in range(iters):
            idx = rng.choice(N, size=min_sample, replace=False)
            obj_s = obj_all[0, idx]
            img_s = img_all[0, idx]
            rvec, tvec = gn(obj_s, img_s)
            # Score all points
            fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
            Xc = obj_all[0] @ (cp.eye(3, dtype=cp.float64) + 0)  # dummy to ensure dtype
            # Project
            R = None
            # Rodrigues for scoring
            R = _rodrigues(rvec)
            Xc = (obj_all[0] @ R.T) + tvec
            invZ = 1.0 / cp.clip(Xc[:, 2], 1e-9, None)
            u_hat = fx * Xc[:, 0] * invZ + cx
            v_hat = fy * Xc[:, 1] * invZ + cy
            err = cp.sqrt((img_all[0][:, 0] - u_hat) ** 2 + (img_all[0][:, 1] - v_hat) ** 2)
            mask = (err < float(reprojection_error)).astype(cp.uint8)
            inliers = int(cp.asnumpy(mask.sum()))
            if inliers > best_inliers:
                best_inliers = inliers
                best_r, best_t, best_mask = rvec, tvec, mask
                if inliers >= int(confidence * N):
                    break

        if best_inliers <= 0:
            return False, np.zeros((3, 1)), np.zeros((3, 1)), np.zeros((N,), dtype=np.uint8)

        in_idx = cp.nonzero(best_mask)[0]
        rvec, tvec = gn(obj_all[0][in_idx], img_all[0][in_idx])
        return (
            True,
            cp.asnumpy(rvec).reshape(3, 1),
            cp.asnumpy(tvec).reshape(3, 1),
            cp.asnumpy(best_mask),
        )


class _CudaTemplateTracker:
    def __init__(
        self,
        tmpl: "cp.ndarray",
        scale_step: float = 1.05,
        lr: float = 0.1,
        search_radius: int = 16,
        x0: int = 0,
        y0: int = 0,
    ):
        self.tmpl = tmpl.astype(cp.float32)
        self.h, self.w = int(tmpl.shape[0]), int(tmpl.shape[1])
        self.scale_step = float(scale_step)
        self.lr = float(lr)
        self.search_radius = int(search_radius)
        # Cosine window
        wy = cp.hanning(self.h).astype(cp.float32)
        wx = cp.hanning(self.w).astype(cp.float32)
        self.window = wy[:, None] * wx[None, :]
        self.tmpl = self.tmpl * self.window
        self.y = int(y0)
        self.x = int(x0)

    def update(self, gray: "cp.ndarray"):
        H, W = int(gray.shape[0]), int(gray.shape[1])
        r = self.search_radius
        x0 = max(0, self.x - r)
        y0 = max(0, self.y - r)
        x1 = min(W, self.x + self.w + r)
        y1 = min(H, self.y + self.h + r)
        search = gray[y0:y1, x0:x1]
        if search.shape[0] < self.h or search.shape[1] < self.w:
            search = gray
            x0 = y0 = 0
        best = (self.x, self.y, self.w, self.h)
        best_score = -1e9
        for s in (1.0 / self.scale_step, 1.0, self.scale_step):
            th = max(1, int(round(self.h * s)))
            tw = max(1, int(round(self.w * s)))
            tmpl_s = _resize_bilinear_hwc_cuda(self.tmpl, th, tw)
            if tmpl_s.ndim == 3:
                tmpl_s = tmpl_s[..., 0]
            tmpl_s = tmpl_s.astype(cp.float32)
            tmpl_zm = tmpl_s - tmpl_s.mean()
            tmpl_energy = cp.sqrt(cp.sum(tmpl_zm * tmpl_zm)) + 1e-6
            # NCC via correlate2d and local std
            ones = cp.ones((th, tw), dtype=cp.float32)
            num = csignal.correlate2d(search, tmpl_zm, mode="valid")  # type: ignore
            sumS = csignal.correlate2d(search, ones, mode="valid")  # type: ignore
            sumS2 = csignal.correlate2d(search * search, ones, mode="valid")  # type: ignore
            n = float(th * tw)
            meanS = sumS / n
            varS = cp.clip(sumS2 - n * meanS * meanS, 0.0, None)
            stdS = cp.sqrt(varS) + 1e-6
            res = num / (stdS * tmpl_energy)
            ij = cp.unravel_index(cp.argmax(res), res.shape)
            dy, dx = int(ij[0].get()), int(ij[1].get())  # type: ignore
            score = float(res[ij].get())  # type: ignore
            if score > best_score:
                best_score = score
                best = (x0 + dx, y0 + dy, tw, th)
        x, y, w, h = best
        patch = gray[y : y + h, x : x + w]
        if patch.shape[0] != self.h or patch.shape[1] != self.w:
            patch = _resize_bilinear_hwc_cuda(patch, self.h, self.w)
            if patch.ndim == 3:
                patch = patch[..., 0]
        patch = patch.astype(cp.float32) * self.window
        self.tmpl = (1.0 - self.lr) * self.tmpl + self.lr * patch
        self.x, self.y, self.w, self.h = x, y, w, h
        return x, y, w, h
