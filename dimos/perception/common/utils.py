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

from typing import Union

import cv2
from dimos_lcm.sensor_msgs import CameraInfo  # type: ignore[import-untyped]
from dimos_lcm.vision_msgs import (  # type: ignore[import-untyped]
    BoundingBox2D,
    Detection2D,
    Detection3D,
)
import numpy as np
import torch
import yaml

from dimos.msgs.geometry_msgs import Pose, Quaternion, Vector3
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.std_msgs import Header
from dimos.types.manipulation import ObjectData
from dimos.types.vector import Vector
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Optional CuPy support
try:  # pragma: no cover - optional dependency
    import cupy as cp  # type: ignore

    _HAS_CUDA = True
except Exception:  # pragma: no cover - optional dependency
    cp = None
    _HAS_CUDA = False


def _is_cu_array(x) -> bool:  # type: ignore[no-untyped-def]
    return _HAS_CUDA and cp is not None and isinstance(x, cp.ndarray)


def _to_numpy(x):  # type: ignore[no-untyped-def]
    return cp.asnumpy(x) if _is_cu_array(x) else x


def _to_cupy(x):  # type: ignore[no-untyped-def]
    if _HAS_CUDA and cp is not None and isinstance(x, np.ndarray):
        try:
            return cp.asarray(x)
        except Exception:
            return x
    return x


def load_camera_info(yaml_path: str, frame_id: str = "camera_link") -> CameraInfo:
    """
    Load ROS-style camera_info YAML file and convert to CameraInfo LCM message.

    Args:
        yaml_path: Path to camera_info YAML file (ROS format)
        frame_id: Frame ID for the camera (default: "camera_link")

    Returns:
        CameraInfo: LCM CameraInfo message with all calibration data
    """
    with open(yaml_path) as f:
        camera_info_data = yaml.safe_load(f)

    # Extract image dimensions
    width = camera_info_data.get("image_width", 1280)
    height = camera_info_data.get("image_height", 720)

    # Extract camera matrix (K) - already in row-major format
    K = camera_info_data["camera_matrix"]["data"]

    # Extract distortion coefficients
    D = camera_info_data["distortion_coefficients"]["data"]

    # Extract rectification matrix (R) if available, else use identity
    R = camera_info_data.get("rectification_matrix", {}).get("data", [1, 0, 0, 0, 1, 0, 0, 0, 1])

    # Extract projection matrix (P) if available
    P = camera_info_data.get("projection_matrix", {}).get("data", None)

    # If P not provided, construct from K
    if P is None:
        fx = K[0]
        fy = K[4]
        cx = K[2]
        cy = K[5]
        P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]

    # Create header
    header = Header(frame_id)

    # Create and return CameraInfo message
    return CameraInfo(
        D_length=len(D),
        header=header,
        height=height,
        width=width,
        distortion_model=camera_info_data.get("distortion_model", "plumb_bob"),
        D=D,
        K=K,
        R=R,
        P=P,
        binning_x=0,
        binning_y=0,
    )


def load_camera_info_opencv(yaml_path: str) -> tuple[np.ndarray, np.ndarray]:  # type: ignore[type-arg]
    """
    Load ROS-style camera_info YAML file and convert to OpenCV camera matrix and distortion coefficients.

    Args:
        yaml_path: Path to camera_info YAML file (ROS format)

    Returns:
        K: 3x3 camera intrinsic matrix
        dist: 1xN distortion coefficients array (for plumb_bob model)
    """
    with open(yaml_path) as f:
        camera_info = yaml.safe_load(f)

    # Extract camera matrix (K)
    camera_matrix_data = camera_info["camera_matrix"]["data"]
    K = np.array(camera_matrix_data).reshape(3, 3)

    # Extract distortion coefficients
    dist_coeffs_data = camera_info["distortion_coefficients"]["data"]
    dist = np.array(dist_coeffs_data)

    # Ensure dist is 1D array for OpenCV compatibility
    if dist.ndim == 2:
        dist = dist.flatten()

    return K, dist


def rectify_image_cpu(image: Image, camera_matrix: np.ndarray, dist_coeffs: np.ndarray) -> Image:  # type: ignore[type-arg]
    """CPU rectification using OpenCV. Preserves backend by caller.

    Returns an Image with numpy or cupy data depending on caller choice.
    """
    src = _to_numpy(image.data)  # type: ignore[no-untyped-call]
    rect = cv2.undistort(src, camera_matrix, dist_coeffs)
    # Caller decides whether to convert back to GPU.
    return Image(data=rect, format=image.format, frame_id=image.frame_id, ts=image.ts)


def rectify_image_cuda(image: Image, camera_matrix: np.ndarray, dist_coeffs: np.ndarray) -> Image:  # type: ignore[type-arg]
    """GPU rectification using CuPy bilinear sampling.

    Generates an undistorted output grid and samples from the distorted source.
    Falls back to CPU if CUDA not available.
    """
    if not _HAS_CUDA or cp is None or not image.is_cuda:
        return rectify_image_cpu(image, camera_matrix, dist_coeffs)

    xp = cp

    # Source (distorted) image on device
    src = image.data
    if src.ndim not in (2, 3):
        raise ValueError("Unsupported image rank for rectification")
    H, W = int(src.shape[0]), int(src.shape[1])

    # Extract intrinsics and distortion as float64
    K = xp.asarray(camera_matrix, dtype=xp.float64)
    dist = xp.asarray(dist_coeffs, dtype=xp.float64).reshape(-1)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    k1 = dist[0] if dist.size > 0 else 0.0
    k2 = dist[1] if dist.size > 1 else 0.0
    p1 = dist[2] if dist.size > 2 else 0.0
    p2 = dist[3] if dist.size > 3 else 0.0
    k3 = dist[4] if dist.size > 4 else 0.0

    # Build undistorted target grid (pixel coords)
    u = xp.arange(W, dtype=xp.float64)
    v = xp.arange(H, dtype=xp.float64)
    uu, vv = xp.meshgrid(u, v, indexing="xy")

    # Convert to normalized undistorted coords
    xu = (uu - cx) / fx
    yu = (vv - cy) / fy

    # Apply forward distortion model to get distorted normalized coords
    r2 = xu * xu + yu * yu
    r4 = r2 * r2
    r6 = r4 * r2
    radial = 1.0 + k1 * r2 + k2 * r4 + k3 * r6
    delta_x = 2.0 * p1 * xu * yu + p2 * (r2 + 2.0 * xu * xu)
    delta_y = p1 * (r2 + 2.0 * yu * yu) + 2.0 * p2 * xu * yu
    xd = xu * radial + delta_x
    yd = yu * radial + delta_y

    # Back to pixel coordinates in the source (distorted) image
    us = fx * xd + cx
    vs = fy * yd + cy

    # Bilinear sample from src at (vs, us)
    def _bilinear_sample_cuda(img, x_src, y_src):  # type: ignore[no-untyped-def]
        h, w = int(img.shape[0]), int(img.shape[1])
        # Base integer corners (not clamped)
        x0i = xp.floor(x_src).astype(xp.int32)
        y0i = xp.floor(y_src).astype(xp.int32)
        x1i = x0i + 1
        y1i = y0i + 1

        # Masks for in-bounds neighbors (BORDER_CONSTANT behavior)
        m00 = (x0i >= 0) & (x0i < w) & (y0i >= 0) & (y0i < h)
        m10 = (x1i >= 0) & (x1i < w) & (y0i >= 0) & (y0i < h)
        m01 = (x0i >= 0) & (x0i < w) & (y1i >= 0) & (y1i < h)
        m11 = (x1i >= 0) & (x1i < w) & (y1i >= 0) & (y1i < h)

        # Clamp indices for safe gather, but multiply contributions by masks
        x0 = xp.clip(x0i, 0, w - 1)
        y0 = xp.clip(y0i, 0, h - 1)
        x1 = xp.clip(x1i, 0, w - 1)
        y1 = xp.clip(y1i, 0, h - 1)

        # Weights
        wx = (x_src - x0i).astype(xp.float64)
        wy = (y_src - y0i).astype(xp.float64)
        w00 = (1.0 - wx) * (1.0 - wy)
        w10 = wx * (1.0 - wy)
        w01 = (1.0 - wx) * wy
        w11 = wx * wy

        # Cast masks for arithmetic
        m00f = m00.astype(xp.float64)
        m10f = m10.astype(xp.float64)
        m01f = m01.astype(xp.float64)
        m11f = m11.astype(xp.float64)

        if img.ndim == 2:
            Ia = img[y0, x0].astype(xp.float64)
            Ib = img[y0, x1].astype(xp.float64)
            Ic = img[y1, x0].astype(xp.float64)
            Id = img[y1, x1].astype(xp.float64)
            out = w00 * m00f * Ia + w10 * m10f * Ib + w01 * m01f * Ic + w11 * m11f * Id
        else:
            Ia = img[y0, x0].astype(xp.float64)
            Ib = img[y0, x1].astype(xp.float64)
            Ic = img[y1, x0].astype(xp.float64)
            Id = img[y1, x1].astype(xp.float64)
            # Expand weights and masks for channel broadcasting
            w00e = (w00 * m00f)[..., None]
            w10e = (w10 * m10f)[..., None]
            w01e = (w01 * m01f)[..., None]
            w11e = (w11 * m11f)[..., None]
            out = w00e * Ia + w10e * Ib + w01e * Ic + w11e * Id

        # Cast back to original dtype with clipping for integers
        if img.dtype == xp.uint8:
            out = xp.clip(xp.rint(out), 0, 255).astype(xp.uint8)
        elif img.dtype == xp.uint16:
            out = xp.clip(xp.rint(out), 0, 65535).astype(xp.uint16)
        elif img.dtype == xp.int16:
            out = xp.clip(xp.rint(out), -32768, 32767).astype(xp.int16)
        else:
            out = out.astype(img.dtype, copy=False)
        return out

    rect = _bilinear_sample_cuda(src, us, vs)  # type: ignore[no-untyped-call]
    return Image(data=rect, format=image.format, frame_id=image.frame_id, ts=image.ts)


def rectify_image(image: Image, camera_matrix: np.ndarray, dist_coeffs: np.ndarray) -> Image:  # type: ignore[type-arg]
    """
    Rectify (undistort) an image using camera calibration parameters.

    Args:
        image: Input Image object to rectify
        camera_matrix: 3x3 camera intrinsic matrix (K)
        dist_coeffs: Distortion coefficients array

    Returns:
        Image: Rectified Image object with same format and metadata
    """
    if image.is_cuda and _HAS_CUDA:
        return rectify_image_cuda(image, camera_matrix, dist_coeffs)
    return rectify_image_cpu(image, camera_matrix, dist_coeffs)


def project_3d_points_to_2d_cuda(
    points_3d: "cp.ndarray", camera_intrinsics: Union[list[float], "cp.ndarray"]
) -> "cp.ndarray":
    xp = cp
    pts = points_3d.astype(xp.float64, copy=False)
    mask = pts[:, 2] > 0
    if not bool(xp.any(mask)):
        return xp.zeros((0, 2), dtype=xp.int32)
    valid = pts[mask]
    if isinstance(camera_intrinsics, list) and len(camera_intrinsics) == 4:
        fx, fy, cx, cy = [xp.asarray(v, dtype=xp.float64) for v in camera_intrinsics]
    else:
        K = camera_intrinsics.astype(xp.float64, copy=False)  # type: ignore[union-attr]
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    u = (valid[:, 0] * fx / valid[:, 2]) + cx
    v = (valid[:, 1] * fy / valid[:, 2]) + cy
    return xp.stack([u, v], axis=1).astype(xp.int32)


def project_3d_points_to_2d_cpu(
    points_3d: np.ndarray,  # type: ignore[type-arg]
    camera_intrinsics: list[float] | np.ndarray,  # type: ignore[type-arg]
) -> np.ndarray:  # type: ignore[type-arg]
    pts = np.asarray(points_3d, dtype=np.float64)
    valid_mask = pts[:, 2] > 0
    if not np.any(valid_mask):
        return np.zeros((0, 2), dtype=np.int32)
    valid_points = pts[valid_mask]
    if isinstance(camera_intrinsics, list) and len(camera_intrinsics) == 4:
        fx, fy, cx, cy = [float(v) for v in camera_intrinsics]
    else:
        K = np.array(camera_intrinsics, dtype=np.float64)
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    u = (valid_points[:, 0] * fx / valid_points[:, 2]) + cx
    v = (valid_points[:, 1] * fy / valid_points[:, 2]) + cy
    return np.column_stack([u, v]).astype(np.int32)


def project_3d_points_to_2d(
    points_3d: Union[np.ndarray, "cp.ndarray"],  # type: ignore[type-arg]
    camera_intrinsics: Union[list[float], np.ndarray, "cp.ndarray"],  # type: ignore[type-arg]
) -> Union[np.ndarray, "cp.ndarray"]:  # type: ignore[type-arg]
    """
    Project 3D points to 2D image coordinates using camera intrinsics.

    Args:
        points_3d: Nx3 array of 3D points (X, Y, Z)
        camera_intrinsics: Camera parameters as [fx, fy, cx, cy] list or 3x3 matrix

    Returns:
        Nx2 array of 2D image coordinates (u, v)
    """
    if len(points_3d) == 0:
        return (
            cp.zeros((0, 2), dtype=cp.int32)
            if _is_cu_array(points_3d)
            else np.zeros((0, 2), dtype=np.int32)
        )

    # Filter out points with zero or negative depth
    if _is_cu_array(points_3d) or _is_cu_array(camera_intrinsics):
        xp = cp
        pts = points_3d if _is_cu_array(points_3d) else xp.asarray(points_3d)
        K = camera_intrinsics if _is_cu_array(camera_intrinsics) else camera_intrinsics
        return project_3d_points_to_2d_cuda(pts, K)
    return project_3d_points_to_2d_cpu(np.asarray(points_3d), np.asarray(camera_intrinsics))


def project_2d_points_to_3d_cuda(
    points_2d: "cp.ndarray",
    depth_values: "cp.ndarray",
    camera_intrinsics: Union[list[float], "cp.ndarray"],
) -> "cp.ndarray":
    xp = cp
    pts = points_2d.astype(xp.float64, copy=False)
    depths = depth_values.astype(xp.float64, copy=False)
    valid = depths > 0
    if not bool(xp.any(valid)):
        return xp.zeros((0, 3), dtype=xp.float32)
    uv = pts[valid]
    Z = depths[valid]
    if isinstance(camera_intrinsics, list) and len(camera_intrinsics) == 4:
        fx, fy, cx, cy = [xp.asarray(v, dtype=xp.float64) for v in camera_intrinsics]
    else:
        K = camera_intrinsics.astype(xp.float64, copy=False)  # type: ignore[union-attr]
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    X = (uv[:, 0] - cx) * Z / fx
    Y = (uv[:, 1] - cy) * Z / fy
    return xp.stack([X, Y, Z], axis=1).astype(xp.float32)


def project_2d_points_to_3d_cpu(
    points_2d: np.ndarray,  # type: ignore[type-arg]
    depth_values: np.ndarray,  # type: ignore[type-arg]
    camera_intrinsics: list[float] | np.ndarray,  # type: ignore[type-arg]
) -> np.ndarray:  # type: ignore[type-arg]
    pts = np.asarray(points_2d, dtype=np.float64)
    depths = np.asarray(depth_values, dtype=np.float64)
    valid_mask = depths > 0
    if not np.any(valid_mask):
        return np.zeros((0, 3), dtype=np.float32)
    valid_points_2d = pts[valid_mask]
    valid_depths = depths[valid_mask]
    if isinstance(camera_intrinsics, list) and len(camera_intrinsics) == 4:
        fx, fy, cx, cy = [float(v) for v in camera_intrinsics]
    else:
        camera_matrix = np.array(camera_intrinsics, dtype=np.float64)
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]
    X = (valid_points_2d[:, 0] - cx) * valid_depths / fx
    Y = (valid_points_2d[:, 1] - cy) * valid_depths / fy
    Z = valid_depths
    return np.column_stack([X, Y, Z]).astype(np.float32)


def project_2d_points_to_3d(
    points_2d: Union[np.ndarray, "cp.ndarray"],  # type: ignore[type-arg]
    depth_values: Union[np.ndarray, "cp.ndarray"],  # type: ignore[type-arg]
    camera_intrinsics: Union[list[float], np.ndarray, "cp.ndarray"],  # type: ignore[type-arg]
) -> Union[np.ndarray, "cp.ndarray"]:  # type: ignore[type-arg]
    """
    Project 2D image points to 3D coordinates using depth values and camera intrinsics.

    Args:
        points_2d: Nx2 array of 2D image coordinates (u, v)
        depth_values: N-length array of depth values (Z coordinates) for each point
        camera_intrinsics: Camera parameters as [fx, fy, cx, cy] list or 3x3 matrix

    Returns:
        Nx3 array of 3D points (X, Y, Z)
    """
    if len(points_2d) == 0:
        return (
            cp.zeros((0, 3), dtype=cp.float32)
            if _is_cu_array(points_2d)
            else np.zeros((0, 3), dtype=np.float32)
        )

    # Ensure depth_values is a numpy array
    if _is_cu_array(points_2d) or _is_cu_array(depth_values) or _is_cu_array(camera_intrinsics):
        xp = cp
        pts = points_2d if _is_cu_array(points_2d) else xp.asarray(points_2d)
        depths = depth_values if _is_cu_array(depth_values) else xp.asarray(depth_values)
        K = camera_intrinsics if _is_cu_array(camera_intrinsics) else camera_intrinsics
        return project_2d_points_to_3d_cuda(pts, depths, K)
    return project_2d_points_to_3d_cpu(
        np.asarray(points_2d), np.asarray(depth_values), np.asarray(camera_intrinsics)
    )


def colorize_depth(
    depth_img: Union[np.ndarray, "cp.ndarray"],  # type: ignore[type-arg]
    max_depth: float = 5.0,
    overlay_stats: bool = True,
) -> Union[np.ndarray, "cp.ndarray"] | None:  # type: ignore[type-arg]
    """
    Normalize and colorize depth image using COLORMAP_JET with optional statistics overlay.

    Args:
        depth_img: Depth image (H, W) in meters
        max_depth: Maximum depth value for normalization
        overlay_stats: Whether to overlay depth statistics on the image

    Returns:
        Colorized depth image (H, W, 3) in RGB format, or None if input is None
    """
    if depth_img is None:
        return None

    was_cu = _is_cu_array(depth_img)
    xp = cp if was_cu else np
    depth = depth_img if was_cu else np.asarray(depth_img)

    valid_mask = xp.isfinite(depth) & (depth > 0)
    depth_norm = xp.zeros_like(depth, dtype=xp.float32)
    if bool(valid_mask.any() if not was_cu else xp.any(valid_mask)):
        depth_norm = xp.where(valid_mask, xp.clip(depth / max_depth, 0, 1), depth_norm)

    # Use CPU for colormap/text; convert back to GPU if needed
    depth_norm_np = _to_numpy(depth_norm)  # type: ignore[no-untyped-call]
    depth_colored = cv2.applyColorMap((depth_norm_np * 255).astype(np.uint8), cv2.COLORMAP_JET)
    depth_rgb_np = cv2.cvtColor(depth_colored, cv2.COLOR_BGR2RGB)
    depth_rgb_np = (depth_rgb_np * 0.6).astype(np.uint8)

    if overlay_stats and (np.any(_to_numpy(valid_mask))):  # type: ignore[no-untyped-call]
        valid_depths = _to_numpy(depth)[_to_numpy(valid_mask)]  # type: ignore[no-untyped-call]
        min_depth = float(np.min(valid_depths))
        max_depth_actual = float(np.max(valid_depths))
        h, w = depth_rgb_np.shape[:2]
        center_y, center_x = h // 2, w // 2
        center_region = _to_numpy(
            depth
        )[  # type: ignore[no-untyped-call]
            max(0, center_y - 2) : min(h, center_y + 3), max(0, center_x - 2) : min(w, center_x + 3)
        ]
        center_mask = np.isfinite(center_region) & (center_region > 0)
        if center_mask.any():
            center_depth = float(np.median(center_region[center_mask]))
        else:
            depth_np = _to_numpy(depth)  # type: ignore[no-untyped-call]
            vm_np = _to_numpy(valid_mask)  # type: ignore[no-untyped-call]
            center_depth = float(depth_np[center_y, center_x]) if vm_np[center_y, center_x] else 0.0

        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        thickness = 1
        line_type = cv2.LINE_AA
        text_color = (255, 255, 255)
        bg_color = (0, 0, 0)
        padding = 5

        min_text = f"Min: {min_depth:.2f}m"
        (text_w, text_h), _ = cv2.getTextSize(min_text, font, font_scale, thickness)
        cv2.rectangle(
            depth_rgb_np,
            (padding, padding),
            (padding + text_w + 4, padding + text_h + 6),
            bg_color,
            -1,
        )
        cv2.putText(
            depth_rgb_np,
            min_text,
            (padding + 2, padding + text_h + 2),
            font,
            font_scale,
            text_color,
            thickness,
            line_type,
        )

        max_text = f"Max: {max_depth_actual:.2f}m"
        (text_w, text_h), _ = cv2.getTextSize(max_text, font, font_scale, thickness)
        cv2.rectangle(
            depth_rgb_np,
            (w - padding - text_w - 4, padding),
            (w - padding, padding + text_h + 6),
            bg_color,
            -1,
        )
        cv2.putText(
            depth_rgb_np,
            max_text,
            (w - padding - text_w - 2, padding + text_h + 2),
            font,
            font_scale,
            text_color,
            thickness,
            line_type,
        )

        if center_depth > 0:
            center_text = f"{center_depth:.2f}m"
            (text_w, text_h), _ = cv2.getTextSize(center_text, font, font_scale, thickness)
            center_text_x = center_x - text_w // 2
            center_text_y = center_y + text_h // 2
            cross_size = 10
            cross_color = (255, 255, 255)
            cv2.line(
                depth_rgb_np,
                (center_x - cross_size, center_y),
                (center_x + cross_size, center_y),
                cross_color,
                1,
            )
            cv2.line(
                depth_rgb_np,
                (center_x, center_y - cross_size),
                (center_x, center_y + cross_size),
                cross_color,
                1,
            )
            cv2.rectangle(
                depth_rgb_np,
                (center_text_x - 2, center_text_y - text_h - 2),
                (center_text_x + text_w + 2, center_text_y + 2),
                bg_color,
                -1,
            )
            cv2.putText(
                depth_rgb_np,
                center_text,
                (center_text_x, center_text_y),
                font,
                font_scale,
                text_color,
                thickness,
                line_type,
            )

    return _to_cupy(depth_rgb_np) if was_cu else depth_rgb_np  # type: ignore[no-untyped-call]


def draw_bounding_box(
    image: Union[np.ndarray, "cp.ndarray"],  # type: ignore[type-arg]
    bbox: list[float],
    color: tuple[int, int, int] = (0, 255, 0),
    thickness: int = 2,
    label: str | None = None,
    confidence: float | None = None,
    object_id: int | None = None,
    font_scale: float = 0.6,
) -> Union[np.ndarray, "cp.ndarray"]:  # type: ignore[type-arg]
    """
    Draw a bounding box with optional label on an image.

    Args:
        image: Image to draw on (H, W, 3)
        bbox: Bounding box [x1, y1, x2, y2]
        color: RGB color tuple for the box
        thickness: Line thickness for the box
        label: Optional class label
        confidence: Optional confidence score
        object_id: Optional object ID
        font_scale: Font scale for text

    Returns:
        Image with bounding box drawn
    """
    was_cu = _is_cu_array(image)
    img_np = _to_numpy(image)  # type: ignore[no-untyped-call]
    x1, y1, x2, y2 = map(int, bbox)
    cv2.rectangle(img_np, (x1, y1), (x2, y2), color, thickness)

    # Create label text
    text_parts = []
    if label is not None:
        text_parts.append(str(label))
    if object_id is not None:
        text_parts.append(f"ID: {object_id}")
    if confidence is not None:
        text_parts.append(f"({confidence:.2f})")

    if text_parts:
        text = ", ".join(text_parts)

        # Draw text background
        text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, 1)[0]
        cv2.rectangle(
            img_np,
            (x1, y1 - text_size[1] - 5),
            (x1 + text_size[0], y1),
            (0, 0, 0),
            -1,
        )

        # Draw text
        cv2.putText(
            img_np,
            text,
            (x1, y1 - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            (255, 255, 255),
            1,
        )

    return _to_cupy(img_np) if was_cu else img_np  # type: ignore[no-untyped-call]


def draw_segmentation_mask(
    image: Union[np.ndarray, "cp.ndarray"],  # type: ignore[type-arg]
    mask: Union[np.ndarray, "cp.ndarray"],  # type: ignore[type-arg]
    color: tuple[int, int, int] = (0, 200, 200),
    alpha: float = 0.5,
    draw_contours: bool = True,
    contour_thickness: int = 2,
) -> Union[np.ndarray, "cp.ndarray"]:  # type: ignore[type-arg]
    """
    Draw segmentation mask overlay on an image.

    Args:
        image: Image to draw on (H, W, 3)
        mask: Segmentation mask (H, W) - boolean or uint8
        color: RGB color for the mask
        alpha: Transparency factor (0.0 = transparent, 1.0 = opaque)
        draw_contours: Whether to draw mask contours
        contour_thickness: Thickness of contour lines

    Returns:
        Image with mask overlay drawn
    """
    if mask is None:
        return image

    was_cu = _is_cu_array(image)
    img_np = _to_numpy(image)  # type: ignore[no-untyped-call]
    mask_np = _to_numpy(mask)  # type: ignore[no-untyped-call]

    try:
        mask_np = mask_np.astype(np.uint8)
        colored_mask = np.zeros_like(img_np)
        colored_mask[mask_np > 0] = color
        mask_area = mask_np > 0
        img_np[mask_area] = cv2.addWeighted(
            img_np[mask_area], 1 - alpha, colored_mask[mask_area], alpha, 0
        )
        if draw_contours:
            contours, _ = cv2.findContours(mask_np, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(img_np, contours, -1, color, contour_thickness)
    except Exception as e:
        logger.warning(f"Error drawing segmentation mask: {e}")

    return _to_cupy(img_np) if was_cu else img_np  # type: ignore[no-untyped-call]


def draw_object_detection_visualization(
    image: Union[np.ndarray, "cp.ndarray"],  # type: ignore[type-arg]
    objects: list[ObjectData],
    draw_masks: bool = False,
    bbox_color: tuple[int, int, int] = (0, 255, 0),
    mask_color: tuple[int, int, int] = (0, 200, 200),
    font_scale: float = 0.6,
) -> Union[np.ndarray, "cp.ndarray"]:  # type: ignore[type-arg]
    """
    Create object detection visualization with bounding boxes and optional masks.

    Args:
        image: Base image to draw on (H, W, 3)
        objects: List of ObjectData with detection information
        draw_masks: Whether to draw segmentation masks
        bbox_color: Default color for bounding boxes
        mask_color: Default color for segmentation masks
        font_scale: Font scale for text labels

    Returns:
        Image with detection visualization
    """
    was_cu = _is_cu_array(image)
    viz_image = _to_numpy(image).copy()  # type: ignore[no-untyped-call]

    for obj in objects:
        try:
            # Draw segmentation mask first (if enabled and available)
            if draw_masks and "segmentation_mask" in obj and obj["segmentation_mask"] is not None:
                viz_image = draw_segmentation_mask(
                    viz_image, obj["segmentation_mask"], color=mask_color, alpha=0.5
                )

            # Draw bounding box
            if "bbox" in obj and obj["bbox"] is not None:
                # Use object's color if available, otherwise default
                color = bbox_color
                if "color" in obj and obj["color"] is not None:
                    obj_color = obj["color"]
                    if isinstance(obj_color, np.ndarray):
                        color = tuple(int(c) for c in obj_color)  # type: ignore[assignment]
                    elif isinstance(obj_color, list | tuple):
                        color = tuple(int(c) for c in obj_color[:3])

                viz_image = draw_bounding_box(
                    viz_image,
                    obj["bbox"],
                    color=color,
                    label=obj.get("label"),
                    confidence=obj.get("confidence"),
                    object_id=obj.get("object_id"),
                    font_scale=font_scale,
                )

        except Exception as e:
            logger.warning(f"Error drawing object visualization: {e}")

    return _to_cupy(viz_image) if was_cu else viz_image  # type: ignore[no-untyped-call]


def detection_results_to_object_data(
    bboxes: list[list[float]],
    track_ids: list[int],
    class_ids: list[int],
    confidences: list[float],
    names: list[str],
    masks: list[np.ndarray] | None = None,  # type: ignore[type-arg]
    source: str = "detection",
) -> list[ObjectData]:
    """
    Convert detection/segmentation results to ObjectData format.

    Args:
        bboxes: List of bounding boxes [x1, y1, x2, y2]
        track_ids: List of tracking IDs
        class_ids: List of class indices
        confidences: List of detection confidences
        names: List of class names
        masks: Optional list of segmentation masks
        source: Source type ("detection" or "segmentation")

    Returns:
        List of ObjectData dictionaries
    """
    objects = []

    for i in range(len(bboxes)):
        # Calculate basic properties from bbox
        bbox = bboxes[i]
        width = bbox[2] - bbox[0]
        height = bbox[3] - bbox[1]
        bbox[0] + width / 2
        bbox[1] + height / 2

        # Create ObjectData
        object_data: ObjectData = {
            "object_id": track_ids[i] if i < len(track_ids) else i,
            "bbox": bbox,
            "depth": -1.0,  # Will be populated by depth estimation or point cloud processing
            "confidence": confidences[i] if i < len(confidences) else 1.0,
            "class_id": class_ids[i] if i < len(class_ids) else 0,
            "label": names[i] if i < len(names) else f"{source}_object",
            "movement_tolerance": 1.0,  # Default to freely movable
            "segmentation_mask": masks[i].cpu().numpy()  # type: ignore[attr-defined, typeddict-item]
            if masks and i < len(masks) and isinstance(masks[i], torch.Tensor)
            else masks[i]
            if masks and i < len(masks)
            else None,
            # Initialize 3D properties (will be populated by point cloud processing)
            "position": Vector(0, 0, 0),  # type: ignore[arg-type]
            "rotation": Vector(0, 0, 0),  # type: ignore[arg-type]
            "size": {
                "width": 0.0,
                "height": 0.0,
                "depth": 0.0,
            },
        }
        objects.append(object_data)

    return objects


def combine_object_data(
    list1: list[ObjectData], list2: list[ObjectData], overlap_threshold: float = 0.8
) -> list[ObjectData]:
    """
    Combine two ObjectData lists, removing duplicates based on segmentation mask overlap.
    """
    combined = list1.copy()
    used_ids = set(obj.get("object_id", 0) for obj in list1)
    next_id = max(used_ids) + 1 if used_ids else 1

    for obj2 in list2:
        obj_copy = obj2.copy()

        # Handle duplicate object_id
        if obj_copy.get("object_id", 0) in used_ids:
            obj_copy["object_id"] = next_id
            next_id += 1
        used_ids.add(obj_copy["object_id"])

        # Check mask overlap
        mask2 = obj2.get("segmentation_mask")
        m2 = _to_numpy(mask2) if mask2 is not None else None  # type: ignore[no-untyped-call]
        if m2 is None or np.sum(m2 > 0) == 0:
            combined.append(obj_copy)
            continue

        mask2_area = np.sum(m2 > 0)
        is_duplicate = False

        for obj1 in list1:
            mask1 = obj1.get("segmentation_mask")
            if mask1 is None:
                continue

            m1 = _to_numpy(mask1)  # type: ignore[no-untyped-call]
            intersection = np.sum((m1 > 0) & (m2 > 0))
            if intersection / mask2_area >= overlap_threshold:
                is_duplicate = True
                break

        if not is_duplicate:
            combined.append(obj_copy)

    return combined


def point_in_bbox(point: tuple[int, int], bbox: list[float]) -> bool:
    """
    Check if a point is inside a bounding box.

    Args:
        point: (x, y) coordinates
        bbox: Bounding box [x1, y1, x2, y2]

    Returns:
        True if point is inside bbox
    """
    x, y = point
    x1, y1, x2, y2 = bbox
    return x1 <= x <= x2 and y1 <= y <= y2


def bbox2d_to_corners(bbox_2d: BoundingBox2D) -> tuple[float, float, float, float]:
    """
    Convert BoundingBox2D from center format to corner format.

    Args:
        bbox_2d: BoundingBox2D with center and size

    Returns:
        Tuple of (x1, y1, x2, y2) corner coordinates
    """
    center_x = bbox_2d.center.position.x
    center_y = bbox_2d.center.position.y
    half_width = bbox_2d.size_x / 2.0
    half_height = bbox_2d.size_y / 2.0

    x1 = center_x - half_width
    y1 = center_y - half_height
    x2 = center_x + half_width
    y2 = center_y + half_height

    return x1, y1, x2, y2


def find_clicked_detection(
    click_pos: tuple[int, int], detections_2d: list[Detection2D], detections_3d: list[Detection3D]
) -> Detection3D | None:
    """
    Find which detection was clicked based on 2D bounding boxes.

    Args:
        click_pos: (x, y) click position
        detections_2d: List of Detection2D objects
        detections_3d: List of Detection3D objects (must be 1:1 correspondence)

    Returns:
        Corresponding Detection3D object if found, None otherwise
    """
    click_x, click_y = click_pos

    for i, det_2d in enumerate(detections_2d):
        if det_2d.bbox and i < len(detections_3d):
            x1, y1, x2, y2 = bbox2d_to_corners(det_2d.bbox)

            if x1 <= click_x <= x2 and y1 <= click_y <= y2:
                return detections_3d[i]

    return None


def extract_pose_from_detection3d(detection3d: Detection3D):  # type: ignore[no-untyped-def]
    """Extract PoseStamped from Detection3D message.

    Args:
        detection3d: Detection3D message

    Returns:
        Pose or None if no valid detection
    """
    if not detection3d or not detection3d.bbox or not detection3d.bbox.center:
        return None

    # Extract position
    pos = detection3d.bbox.center.position
    position = Vector3(pos.x, pos.y, pos.z)

    # Extract orientation
    orient = detection3d.bbox.center.orientation
    orientation = Quaternion(orient.x, orient.y, orient.z, orient.w)

    pose = Pose(position=position, orientation=orientation)
    return pose
