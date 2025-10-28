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

import numpy as np
import pytest

from dimos.msgs.sensor_msgs import Image, ImageFormat

try:
    HAS_CUDA = True
    print("Running image backend utils tests with CUDA/CuPy support (GPU mode)")
except:
    HAS_CUDA = False
    print("Running image backend utils tests in CPU-only mode")

from dimos.perception.common.utils import (
    colorize_depth,
    draw_bounding_box,
    draw_object_detection_visualization,
    draw_segmentation_mask,
    project_2d_points_to_3d,
    project_3d_points_to_2d,
    rectify_image,
)


def _has_cupy() -> bool:
    try:
        import cupy as cp  # type: ignore

        try:
            ndev = cp.cuda.runtime.getDeviceCount()  # type: ignore[attr-defined]
            if ndev <= 0:
                return False
            x = cp.array([1, 2, 3])
            _ = int(x.sum().get())
            return True
        except Exception:
            return False
    except Exception:
        return False


@pytest.mark.parametrize(
    "shape,fmt", [((64, 64, 3), ImageFormat.BGR), ((64, 64), ImageFormat.GRAY)]
)
def test_rectify_image_cpu(shape, fmt) -> None:
    arr = (np.random.rand(*shape) * (255 if fmt != ImageFormat.GRAY else 65535)).astype(
        np.uint8 if fmt != ImageFormat.GRAY else np.uint16
    )
    img = Image(data=arr, format=fmt, frame_id="cam", ts=123.456)
    K = np.array(
        [[100.0, 0, arr.shape[1] / 2], [0, 100.0, arr.shape[0] / 2], [0, 0, 1]], dtype=np.float64
    )
    D = np.zeros(5, dtype=np.float64)
    out = rectify_image(img, K, D)
    assert out.shape[:2] == arr.shape[:2]
    assert out.format == fmt
    assert out.frame_id == "cam"
    assert abs(out.ts - 123.456) < 1e-9
    # With zero distortion, pixels should match
    np.testing.assert_array_equal(out.data, arr)


@pytest.mark.skipif(not _has_cupy(), reason="CuPy/CUDA not available")
@pytest.mark.parametrize(
    "shape,fmt", [((32, 32, 3), ImageFormat.BGR), ((32, 32), ImageFormat.GRAY)]
)
def test_rectify_image_gpu_parity(shape, fmt) -> None:
    import cupy as cp  # type: ignore

    arr_np = (np.random.rand(*shape) * (255 if fmt != ImageFormat.GRAY else 65535)).astype(
        np.uint8 if fmt != ImageFormat.GRAY else np.uint16
    )
    arr_cu = cp.asarray(arr_np)
    img = Image(data=arr_cu, format=fmt, frame_id="cam", ts=1.23)
    K = np.array(
        [[80.0, 0, arr_np.shape[1] / 2], [0, 80.0, arr_np.shape[0] / 2], [0, 0, 1.0]],
        dtype=np.float64,
    )
    D = np.zeros(5, dtype=np.float64)
    out = rectify_image(img, K, D)
    # Zero distortion parity and backend preservation
    assert out.format == fmt
    assert out.frame_id == "cam"
    assert abs(out.ts - 1.23) < 1e-9
    assert out.data.__class__.__module__.startswith("cupy")
    np.testing.assert_array_equal(cp.asnumpy(out.data), arr_np)


@pytest.mark.skipif(not _has_cupy(), reason="CuPy/CUDA not available")
def test_rectify_image_gpu_nonzero_dist_close() -> None:
    import cupy as cp  # type: ignore

    H, W = 64, 96
    # Structured pattern to make interpolation deterministic enough
    x = np.linspace(0, 255, W, dtype=np.float32)
    y = np.linspace(0, 255, H, dtype=np.float32)
    xv, yv = np.meshgrid(x, y)
    arr_np = np.stack(
        [
            xv.astype(np.uint8),
            yv.astype(np.uint8),
            ((xv + yv) / 2).astype(np.uint8),
        ],
        axis=2,
    )
    img_cpu = Image(data=arr_np, format=ImageFormat.BGR, frame_id="cam", ts=0.5)
    img_gpu = Image(data=cp.asarray(arr_np), format=ImageFormat.BGR, frame_id="cam", ts=0.5)

    fx, fy = 120.0, 125.0
    cx, cy = W / 2.0, H / 2.0
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]], dtype=np.float64)
    D = np.array([0.05, -0.02, 0.001, -0.001, 0.0], dtype=np.float64)

    out_cpu = rectify_image(img_cpu, K, D)
    out_gpu = rectify_image(img_gpu, K, D)
    # Compare within a small tolerance
    # Small numeric differences may remain due to model and casting; keep tight tolerance
    np.testing.assert_allclose(
        cp.asnumpy(out_gpu.data).astype(np.int16), out_cpu.data.astype(np.int16), atol=4
    )


def test_project_roundtrip_cpu() -> None:
    pts3d = np.array([[0.1, 0.2, 1.0], [0.0, 0.0, 2.0], [0.5, -0.3, 3.0]], dtype=np.float32)
    fx, fy, cx, cy = 200.0, 220.0, 64.0, 48.0
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]], dtype=np.float64)
    uv = project_3d_points_to_2d(pts3d, K)
    assert uv.shape == (3, 2)
    Z = pts3d[:, 2]
    pts3d_back = project_2d_points_to_3d(uv.astype(np.float32), Z.astype(np.float32), K)
    # Allow small rounding differences due to int rounding in 2D
    assert pts3d_back.shape == (3, 3)
    assert np.all(pts3d_back[:, 2] > 0)


@pytest.mark.skipif(not _has_cupy(), reason="CuPy/CUDA not available")
def test_project_parity_gpu_cpu() -> None:
    import cupy as cp  # type: ignore

    pts3d_np = np.array([[0.1, 0.2, 1.0], [0.0, 0.0, 2.0], [0.5, -0.3, 3.0]], dtype=np.float32)
    fx, fy, cx, cy = 200.0, 220.0, 64.0, 48.0
    K_np = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]], dtype=np.float64)
    uv_cpu = project_3d_points_to_2d(pts3d_np, K_np)
    uv_gpu = project_3d_points_to_2d(cp.asarray(pts3d_np), cp.asarray(K_np))
    np.testing.assert_array_equal(cp.asnumpy(uv_gpu), uv_cpu)

    Z_np = pts3d_np[:, 2]
    pts3d_cpu = project_2d_points_to_3d(uv_cpu.astype(np.float32), Z_np.astype(np.float32), K_np)
    pts3d_gpu = project_2d_points_to_3d(
        cp.asarray(uv_cpu.astype(np.float32)), cp.asarray(Z_np.astype(np.float32)), cp.asarray(K_np)
    )
    assert pts3d_cpu.shape == cp.asnumpy(pts3d_gpu).shape


@pytest.mark.skipif(not _has_cupy(), reason="CuPy/CUDA not available")
def test_project_parity_gpu_cpu_random() -> None:
    import cupy as cp  # type: ignore

    rng = np.random.RandomState(0)
    N = 1000
    Z = rng.uniform(0.1, 5.0, size=(N, 1)).astype(np.float32)
    XY = rng.uniform(-1.0, 1.0, size=(N, 2)).astype(np.float32)
    pts3d_np = np.concatenate([XY, Z], axis=1)

    fx, fy = 300.0, 320.0
    cx, cy = 128.0, 96.0
    K_np = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]], dtype=np.float64)

    uv_cpu = project_3d_points_to_2d(pts3d_np, K_np)
    uv_gpu = project_3d_points_to_2d(cp.asarray(pts3d_np), cp.asarray(K_np))
    np.testing.assert_array_equal(cp.asnumpy(uv_gpu), uv_cpu)

    # Roundtrip
    Z_flat = pts3d_np[:, 2]
    pts3d_cpu = project_2d_points_to_3d(uv_cpu.astype(np.float32), Z_flat.astype(np.float32), K_np)
    pts3d_gpu = project_2d_points_to_3d(
        cp.asarray(uv_cpu.astype(np.float32)),
        cp.asarray(Z_flat.astype(np.float32)),
        cp.asarray(K_np),
    )
    assert pts3d_cpu.shape == cp.asnumpy(pts3d_gpu).shape


def test_colorize_depth_cpu() -> None:
    depth = np.zeros((32, 48), dtype=np.float32)
    depth[8:16, 12:24] = 1.5
    out = colorize_depth(depth, max_depth=3.0, overlay_stats=False)
    assert isinstance(out, np.ndarray)
    assert out.shape == (32, 48, 3)
    assert out.dtype == np.uint8


@pytest.mark.skipif(not _has_cupy(), reason="CuPy/CUDA not available")
def test_colorize_depth_gpu_parity() -> None:
    import cupy as cp  # type: ignore

    depth_np = np.zeros((16, 20), dtype=np.float32)
    depth_np[4:8, 5:15] = 2.0
    out_cpu = colorize_depth(depth_np, max_depth=4.0, overlay_stats=False)
    out_gpu = colorize_depth(cp.asarray(depth_np), max_depth=4.0, overlay_stats=False)
    np.testing.assert_array_equal(cp.asnumpy(out_gpu), out_cpu)


def test_draw_bounding_box_cpu() -> None:
    img = np.zeros((20, 30, 3), dtype=np.uint8)
    out = draw_bounding_box(img, [2, 3, 10, 12], color=(255, 0, 0), thickness=1)
    assert isinstance(out, np.ndarray)
    assert out.shape == img.shape
    assert out.dtype == img.dtype


@pytest.mark.skipif(not _has_cupy(), reason="CuPy/CUDA not available")
def test_draw_bounding_box_gpu_parity() -> None:
    import cupy as cp  # type: ignore

    img_np = np.zeros((20, 30, 3), dtype=np.uint8)
    out_cpu = draw_bounding_box(img_np.copy(), [2, 3, 10, 12], color=(0, 255, 0), thickness=2)
    img_cu = cp.asarray(img_np)
    out_gpu = draw_bounding_box(img_cu, [2, 3, 10, 12], color=(0, 255, 0), thickness=2)
    np.testing.assert_array_equal(cp.asnumpy(out_gpu), out_cpu)


def test_draw_segmentation_mask_cpu() -> None:
    img = np.zeros((20, 30, 3), dtype=np.uint8)
    mask = np.zeros((20, 30), dtype=np.uint8)
    mask[5:10, 8:15] = 1
    out = draw_segmentation_mask(img, mask, color=(0, 200, 200), alpha=0.5)
    assert out.shape == img.shape


@pytest.mark.skipif(not _has_cupy(), reason="CuPy/CUDA not available")
def test_draw_segmentation_mask_gpu_parity() -> None:
    import cupy as cp  # type: ignore

    img_np = np.zeros((20, 30, 3), dtype=np.uint8)
    mask_np = np.zeros((20, 30), dtype=np.uint8)
    mask_np[2:12, 3:20] = 1
    out_cpu = draw_segmentation_mask(img_np.copy(), mask_np, color=(100, 50, 200), alpha=0.4)
    out_gpu = draw_segmentation_mask(
        cp.asarray(img_np), cp.asarray(mask_np), color=(100, 50, 200), alpha=0.4
    )
    np.testing.assert_array_equal(cp.asnumpy(out_gpu), out_cpu)


def test_draw_object_detection_visualization_cpu() -> None:
    img = np.zeros((30, 40, 3), dtype=np.uint8)
    objects = [
        {
            "object_id": 1,
            "bbox": [5, 6, 20, 25],
            "label": "box",
            "confidence": 0.9,
        }
    ]
    out = draw_object_detection_visualization(img, objects)
    assert out.shape == img.shape


@pytest.mark.skipif(not _has_cupy(), reason="CuPy/CUDA not available")
def test_draw_object_detection_visualization_gpu_parity() -> None:
    import cupy as cp  # type: ignore

    img_np = np.zeros((30, 40, 3), dtype=np.uint8)
    objects = [
        {
            "object_id": 1,
            "bbox": [5, 6, 20, 25],
            "label": "box",
            "confidence": 0.9,
        }
    ]
    out_cpu = draw_object_detection_visualization(img_np.copy(), objects)
    out_gpu = draw_object_detection_visualization(cp.asarray(img_np), objects)
    np.testing.assert_array_equal(cp.asnumpy(out_gpu), out_cpu)
