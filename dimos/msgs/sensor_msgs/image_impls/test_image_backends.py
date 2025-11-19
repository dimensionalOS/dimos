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

import time

import cv2
import numpy as np
import pytest

from dimos.msgs.sensor_msgs.Image import HAS_CUDA, Image, ImageFormat
from dimos.utils.data import get_data

IMAGE_PATH = get_data("chair-image.png")

if HAS_CUDA:
    print("Running image backend tests with CUDA/CuPy support (GPU mode)")
else:
    print("Running image backend tests in CPU-only mode")


def _load_chair_image() -> np.ndarray:
    img = cv2.imread(IMAGE_PATH, cv2.IMREAD_UNCHANGED)
    if img is None:
        raise FileNotFoundError(f"unable to load test image at {IMAGE_PATH}")
    return img


_CHAIR_BGRA = _load_chair_image()


def _prepare_image(fmt: ImageFormat, shape=None) -> np.ndarray:
    base = _CHAIR_BGRA
    if fmt == ImageFormat.BGR:
        arr = cv2.cvtColor(base, cv2.COLOR_BGRA2BGR)
    elif fmt == ImageFormat.RGB:
        arr = cv2.cvtColor(base, cv2.COLOR_BGRA2RGB)
    elif fmt == ImageFormat.BGRA:
        arr = base.copy()
    elif fmt == ImageFormat.GRAY:
        arr = cv2.cvtColor(base, cv2.COLOR_BGRA2GRAY)
    else:
        raise ValueError(f"unsupported image format {fmt}")

    if shape is None:
        return arr.copy()

    if len(shape) == 2:
        height, width = shape
        orig_h, orig_w = arr.shape[:2]
        interp = cv2.INTER_AREA if height <= orig_h and width <= orig_w else cv2.INTER_LINEAR
        resized = cv2.resize(arr, (width, height), interpolation=interp)
        return resized.copy()

    if len(shape) == 3:
        height, width, channels = shape
        orig_h, orig_w = arr.shape[:2]
        interp = cv2.INTER_AREA if height <= orig_h and width <= orig_w else cv2.INTER_LINEAR
        resized = cv2.resize(arr, (width, height), interpolation=interp)
        if resized.ndim == 2:
            resized = np.repeat(resized[:, :, None], channels, axis=2)
        elif resized.shape[2] != channels:
            if channels == 4 and resized.shape[2] == 3:
                alpha = np.full((height, width, 1), 255, dtype=resized.dtype)
                resized = np.concatenate([resized, alpha], axis=2)
            elif channels == 3 and resized.shape[2] == 4:
                resized = resized[:, :, :3]
            else:
                raise ValueError(f"cannot adjust image to {channels} channels")
        return resized.copy()

    raise ValueError("shape must be a tuple of length 2 or 3")


@pytest.fixture
def alloc_timer(request):
    """Helper fixture for adaptive testing with optional GPU support."""

    def _alloc(
        arr: np.ndarray, fmt: ImageFormat, *, to_cuda: bool | None = None, label: str | None = None
    ):
        tag = label or request.node.name

        # Always create CPU image
        start = time.perf_counter()
        cpu = Image.from_numpy(arr, format=fmt, to_cuda=False)
        cpu_time = time.perf_counter() - start

        # Optionally create GPU image if CUDA is available
        gpu = None
        gpu_time = None
        if to_cuda is None:
            to_cuda = HAS_CUDA

        if to_cuda and HAS_CUDA:
            arr_gpu = np.array(arr, copy=True)
            start = time.perf_counter()
            gpu = Image.from_numpy(arr_gpu, format=fmt, to_cuda=True)
            gpu_time = time.perf_counter() - start

        if gpu_time is not None:
            print(f"[alloc {tag}] cpu={cpu_time:.6f}s gpu={gpu_time:.6f}s")
        else:
            print(f"[alloc {tag}] cpu={cpu_time:.6f}s")
        return cpu, gpu, cpu_time, gpu_time

    return _alloc


@pytest.mark.parametrize(
    "shape,fmt",
    [
        ((64, 64, 3), ImageFormat.BGR),
        ((64, 64, 4), ImageFormat.BGRA),
        ((64, 64, 3), ImageFormat.RGB),
        ((64, 64), ImageFormat.GRAY),
    ],
)
def test_color_conversions(shape, fmt, alloc_timer) -> None:
    """Test color conversions with NumpyImage always, add CudaImage parity when available."""
    arr = _prepare_image(fmt, shape)
    cpu, gpu, _, _ = alloc_timer(arr, fmt)

    # Always test CPU backend
    cpu_round = cpu.to_rgb().to_bgr().to_opencv()
    assert cpu_round.shape[0] == shape[0]
    assert cpu_round.shape[1] == shape[1]
    assert cpu_round.shape[2] == 3  # to_opencv always returns BGR (3 channels)
    assert cpu_round.dtype == np.uint8

    # Optionally test GPU parity when CUDA is available
    if gpu is not None:
        gpu_round = gpu.to_rgb().to_bgr().to_opencv()
        assert gpu_round.shape == cpu_round.shape
        assert gpu_round.dtype == cpu_round.dtype
        # Exact match for uint8 color ops
        assert np.array_equal(cpu_round, gpu_round)


def test_grayscale(alloc_timer) -> None:
    """Test grayscale conversion with NumpyImage always, add CudaImage parity when available."""
    arr = _prepare_image(ImageFormat.BGR, (48, 32, 3))
    cpu, gpu, _, _ = alloc_timer(arr, ImageFormat.BGR)

    # Always test CPU backend
    cpu_gray = cpu.to_grayscale().to_opencv()
    assert cpu_gray.shape == (48, 32)  # Grayscale has no channel dimension in OpenCV
    assert cpu_gray.dtype == np.uint8

    # Optionally test GPU parity when CUDA is available
    if gpu is not None:
        gpu_gray = gpu.to_grayscale().to_opencv()
        assert gpu_gray.shape == cpu_gray.shape
        assert gpu_gray.dtype == cpu_gray.dtype
        # Allow tiny rounding differences (<=1 LSB) — visually indistinguishable
        diff = np.abs(cpu_gray.astype(np.int16) - gpu_gray.astype(np.int16))
        assert diff.max() <= 1


@pytest.mark.parametrize("fmt", [ImageFormat.BGR, ImageFormat.RGB, ImageFormat.BGRA])
def test_resize(fmt, alloc_timer) -> None:
    """Test resize with NumpyImage always, add CudaImage parity when available."""
    shape = (60, 80, 3) if fmt in (ImageFormat.BGR, ImageFormat.RGB) else (60, 80, 4)
    arr = _prepare_image(fmt, shape)
    cpu, gpu, _, _ = alloc_timer(arr, fmt)

    new_w, new_h = 37, 53

    # Always test CPU backend
    cpu_res = cpu.resize(new_w, new_h).to_opencv()
    assert (
        cpu_res.shape == (53, 37, 3) if fmt != ImageFormat.BGRA else (53, 37, 3)
    )  # to_opencv drops alpha
    assert cpu_res.dtype == np.uint8

    # Optionally test GPU parity when CUDA is available
    if gpu is not None:
        gpu_res = gpu.resize(new_w, new_h).to_opencv()
        assert gpu_res.shape == cpu_res.shape
        assert gpu_res.dtype == cpu_res.dtype
        # Allow small tolerance due to float interpolation differences
        assert np.max(np.abs(cpu_res.astype(np.int16) - gpu_res.astype(np.int16))) <= 1


def test_perf_alloc(alloc_timer) -> None:
    """Test allocation performance with NumpyImage always, add CudaImage when available."""
    arr = _prepare_image(ImageFormat.BGR, (480, 640, 3))
    alloc_timer(arr, ImageFormat.BGR, label="test_perf_alloc-setup")

    runs = 5

    # Always test CPU allocation
    t0 = time.perf_counter()
    for _ in range(runs):
        _ = Image.from_numpy(arr, format=ImageFormat.BGR, to_cuda=False)
    cpu_t = (time.perf_counter() - t0) / runs
    assert cpu_t > 0

    # Optionally test GPU allocation when CUDA is available
    if HAS_CUDA:
        t0 = time.perf_counter()
        for _ in range(runs):
            _ = Image.from_numpy(arr, format=ImageFormat.BGR, to_cuda=True)
        gpu_t = (time.perf_counter() - t0) / runs
        print(f"alloc (avg per call) cpu={cpu_t:.6f}s gpu={gpu_t:.6f}s")
        assert gpu_t > 0
    else:
        print(f"alloc (avg per call) cpu={cpu_t:.6f}s")


def test_sharpness(alloc_timer) -> None:
    """Test sharpness computation with NumpyImage always, add CudaImage parity when available."""
    arr = _prepare_image(ImageFormat.BGR, (64, 64, 3))
    cpu, gpu, _, _ = alloc_timer(arr, ImageFormat.BGR)

    # Always test CPU backend
    s_cpu = cpu.sharpness
    assert s_cpu >= 0  # Sharpness should be non-negative
    assert s_cpu < 1000  # Reasonable upper bound

    # Optionally test GPU parity when CUDA is available
    if gpu is not None:
        s_gpu = gpu.sharpness
        # Values should be very close; minor border/rounding differences allowed
        assert abs(s_cpu - s_gpu) < 5e-2


def test_to_opencv(alloc_timer) -> None:
    """Test to_opencv conversion with NumpyImage always, add CudaImage parity when available."""
    # BGRA should drop alpha and produce BGR
    arr = _prepare_image(ImageFormat.BGRA, (32, 32, 4))
    cpu, gpu, _, _ = alloc_timer(arr, ImageFormat.BGRA)

    # Always test CPU backend
    cpu_bgr = cpu.to_opencv()
    assert cpu_bgr.shape == (32, 32, 3)
    assert cpu_bgr.dtype == np.uint8

    # Optionally test GPU parity when CUDA is available
    if gpu is not None:
        gpu_bgr = gpu.to_opencv()
        assert gpu_bgr.shape == cpu_bgr.shape
        assert gpu_bgr.dtype == cpu_bgr.dtype
        assert np.array_equal(cpu_bgr, gpu_bgr)


def test_solve_pnp(alloc_timer) -> None:
    """Test solve_pnp with NumpyImage always, add CudaImage parity when available."""
    # Synthetic camera and 3D points
    K = np.array([[400.0, 0.0, 32.0], [0.0, 400.0, 24.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    dist = None
    obj = np.array(
        [
            [-0.5, -0.5, 0.0],
            [0.5, -0.5, 0.0],
            [0.5, 0.5, 0.0],
            [-0.5, 0.5, 0.0],
            [0.0, 0.0, 0.5],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float32,
    )

    rvec_true = np.zeros((3, 1), dtype=np.float64)
    tvec_true = np.array([[0.0], [0.0], [2.0]], dtype=np.float64)
    img_pts, _ = cv2.projectPoints(obj, rvec_true, tvec_true, K, dist)
    img_pts = img_pts.reshape(-1, 2).astype(np.float32)

    # Build images using deterministic fixture content
    base_bgr = _prepare_image(ImageFormat.BGR, (48, 64, 3))
    cpu, gpu, _, _ = alloc_timer(base_bgr, ImageFormat.BGR)

    # Always test CPU backend
    ok_cpu, r_cpu, t_cpu = cpu.solve_pnp(obj, img_pts, K, dist)
    assert ok_cpu

    # Validate reprojection error for CPU solver
    proj_cpu, _ = cv2.projectPoints(obj, r_cpu, t_cpu, K, dist)
    proj_cpu = proj_cpu.reshape(-1, 2)
    err_cpu = np.linalg.norm(proj_cpu - img_pts, axis=1)
    assert err_cpu.mean() < 1e-3
    assert err_cpu.max() < 1e-2

    # Optionally test GPU parity when CUDA is available
    if gpu is not None:
        ok_gpu, r_gpu, t_gpu = gpu.solve_pnp(obj, img_pts, K, dist)
        assert ok_gpu

        # Validate reprojection error for GPU solver
        proj_gpu, _ = cv2.projectPoints(obj, r_gpu, t_gpu, K, dist)
        proj_gpu = proj_gpu.reshape(-1, 2)
        err_gpu = np.linalg.norm(proj_gpu - img_pts, axis=1)
        assert err_gpu.mean() < 1e-3
        assert err_gpu.max() < 1e-2


def test_perf_grayscale(alloc_timer) -> None:
    """Test grayscale performance with NumpyImage always, add CudaImage when available."""
    arr = _prepare_image(ImageFormat.BGR, (480, 640, 3))
    cpu, gpu, _, _ = alloc_timer(arr, ImageFormat.BGR, label="test_perf_grayscale-setup")

    runs = 10

    # Always test CPU performance
    t0 = time.perf_counter()
    for _ in range(runs):
        _ = cpu.to_grayscale()
    cpu_t = (time.perf_counter() - t0) / runs
    assert cpu_t > 0

    # Optionally test GPU performance when CUDA is available
    if gpu is not None:
        t0 = time.perf_counter()
        for _ in range(runs):
            _ = gpu.to_grayscale()
        gpu_t = (time.perf_counter() - t0) / runs
        print(f"grayscale (avg per call) cpu={cpu_t:.6f}s gpu={gpu_t:.6f}s")
        assert gpu_t > 0
    else:
        print(f"grayscale (avg per call) cpu={cpu_t:.6f}s")


def test_perf_resize(alloc_timer) -> None:
    """Test resize performance with NumpyImage always, add CudaImage when available."""
    arr = _prepare_image(ImageFormat.BGR, (480, 640, 3))
    cpu, gpu, _, _ = alloc_timer(arr, ImageFormat.BGR, label="test_perf_resize-setup")

    runs = 5

    # Always test CPU performance
    t0 = time.perf_counter()
    for _ in range(runs):
        _ = cpu.resize(320, 240)
    cpu_t = (time.perf_counter() - t0) / runs
    assert cpu_t > 0

    # Optionally test GPU performance when CUDA is available
    if gpu is not None:
        t0 = time.perf_counter()
        for _ in range(runs):
            _ = gpu.resize(320, 240)
        gpu_t = (time.perf_counter() - t0) / runs
        print(f"resize (avg per call) cpu={cpu_t:.6f}s gpu={gpu_t:.6f}s")
        assert gpu_t > 0
    else:
        print(f"resize (avg per call) cpu={cpu_t:.6f}s")


def test_perf_sharpness(alloc_timer) -> None:
    """Test sharpness performance with NumpyImage always, add CudaImage when available."""
    arr = _prepare_image(ImageFormat.BGR, (480, 640, 3))
    cpu, gpu, _, _ = alloc_timer(arr, ImageFormat.BGR, label="test_perf_sharpness-setup")

    runs = 3

    # Always test CPU performance
    t0 = time.perf_counter()
    for _ in range(runs):
        _ = cpu.sharpness
    cpu_t = (time.perf_counter() - t0) / runs
    assert cpu_t > 0

    # Optionally test GPU performance when CUDA is available
    if gpu is not None:
        t0 = time.perf_counter()
        for _ in range(runs):
            _ = gpu.sharpness
        gpu_t = (time.perf_counter() - t0) / runs
        print(f"sharpness (avg per call) cpu={cpu_t:.6f}s gpu={gpu_t:.6f}s")
        assert gpu_t > 0
    else:
        print(f"sharpness (avg per call) cpu={cpu_t:.6f}s")


def test_perf_solvepnp(alloc_timer) -> None:
    """Test solve_pnp performance with NumpyImage always, add CudaImage when available."""
    K = np.array([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    dist = None
    rng = np.random.default_rng(123)
    obj = rng.standard_normal((200, 3)).astype(np.float32)
    rvec_true = np.array([[0.1], [-0.2], [0.05]])
    tvec_true = np.array([[0.0], [0.0], [3.0]])
    img_pts, _ = cv2.projectPoints(obj, rvec_true, tvec_true, K, dist)
    img_pts = img_pts.reshape(-1, 2).astype(np.float32)
    base_bgr = _prepare_image(ImageFormat.BGR, (480, 640, 3))
    cpu, gpu, _, _ = alloc_timer(base_bgr, ImageFormat.BGR, label="test_perf_solvepnp-setup")

    runs = 5

    # Always test CPU performance
    t0 = time.perf_counter()
    for _ in range(runs):
        _ = cpu.solve_pnp(obj, img_pts, K, dist)
    cpu_t = (time.perf_counter() - t0) / runs
    assert cpu_t > 0

    # Optionally test GPU performance when CUDA is available
    if gpu is not None:
        t0 = time.perf_counter()
        for _ in range(runs):
            _ = gpu.solve_pnp(obj, img_pts, K, dist)
        gpu_t = (time.perf_counter() - t0) / runs
        print(f"solvePnP (avg per call) cpu={cpu_t:.6f}s gpu={gpu_t:.6f}s")
        assert gpu_t > 0
    else:
        print(f"solvePnP (avg per call) cpu={cpu_t:.6f}s")


# this test is failing with
#  raise RuntimeError("OpenCV CSRT tracker not available")
@pytest.mark.skip
def test_perf_tracker(alloc_timer) -> None:
    """Test tracker performance with NumpyImage always, add CudaImage when available."""
    # Don't check - just let it fail if CSRT isn't available

    H, W = 240, 320
    img_base = _prepare_image(ImageFormat.BGR, (H, W, 3))
    img1 = img_base.copy()
    img2 = img_base.copy()
    bbox0 = (80, 60, 40, 30)
    x0, y0, w0, h0 = bbox0
    cv2.rectangle(img1, (x0, y0), (x0 + w0, y0 + h0), (255, 255, 255), thickness=-1)
    dx, dy = 8, 5
    cv2.rectangle(
        img2,
        (x0 + dx, y0 + dy),
        (x0 + dx + w0, y0 + dy + h0),
        (255, 255, 255),
        thickness=-1,
    )
    cpu1, gpu1, _, _ = alloc_timer(img1, ImageFormat.BGR, label="test_perf_tracker-frame1")
    cpu2, gpu2, _, _ = alloc_timer(img2, ImageFormat.BGR, label="test_perf_tracker-frame2")

    # Always test CPU tracker
    trk_cpu = cpu1.create_csrt_tracker(bbox0)

    runs = 10
    t0 = time.perf_counter()
    for _ in range(runs):
        _ = cpu2.csrt_update(trk_cpu)
    cpu_t = (time.perf_counter() - t0) / runs
    assert cpu_t > 0

    # Optionally test GPU performance when CUDA is available
    if gpu1 is not None and gpu2 is not None:
        trk_gpu = gpu1.create_csrt_tracker(bbox0)
        t0 = time.perf_counter()
        for _ in range(runs):
            _ = gpu2.csrt_update(trk_gpu)
        gpu_t = (time.perf_counter() - t0) / runs
        print(f"tracker (avg per call) cpu={cpu_t:.6f}s gpu={gpu_t:.6f}s")
        assert gpu_t > 0
    else:
        print(f"tracker (avg per call) cpu={cpu_t:.6f}s")


# this test is failing with
#  raise RuntimeError("OpenCV CSRT tracker not available")
@pytest.mark.skip
def test_csrt_tracker(alloc_timer) -> None:
    """Test CSRT tracker with NumpyImage always, add CudaImage parity when available."""
    # Don't check - just let it fail if CSRT isn't available

    H, W = 100, 100
    # Create two frames with a moving rectangle
    img_base = _prepare_image(ImageFormat.BGR, (H, W, 3))
    img1 = img_base.copy()
    img2 = img_base.copy()
    bbox0 = (30, 30, 20, 15)
    x0, y0, w0, h0 = bbox0
    # draw rect in img1
    cv2.rectangle(img1, (x0, y0), (x0 + w0, y0 + h0), (255, 255, 255), thickness=-1)
    # shift by (dx,dy)
    dx, dy = 5, 3
    cv2.rectangle(
        img2,
        (x0 + dx, y0 + dy),
        (x0 + dx + w0, y0 + dy + h0),
        (255, 255, 255),
        thickness=-1,
    )

    cpu1, gpu1, _, _ = alloc_timer(img1, ImageFormat.BGR, label="test_csrt_tracker-frame1")
    cpu2, gpu2, _, _ = alloc_timer(img2, ImageFormat.BGR, label="test_csrt_tracker-frame2")

    # Always test CPU tracker
    trk_cpu = cpu1.create_csrt_tracker(bbox0)
    ok_cpu, bbox_cpu = cpu2.csrt_update(trk_cpu)
    assert ok_cpu

    # Compare to ground-truth expected bbox
    expected = (x0 + dx, y0 + dy, w0, h0)
    err_cpu = sum(abs(a - b) for a, b in zip(bbox_cpu, expected, strict=False))
    assert err_cpu <= 8

    # Optionally test GPU parity when CUDA is available
    if gpu1 is not None and gpu2 is not None:
        trk_gpu = gpu1.create_csrt_tracker(bbox0)
        ok_gpu, bbox_gpu = gpu2.csrt_update(trk_gpu)
        assert ok_gpu

        err_gpu = sum(abs(a - b) for a, b in zip(bbox_gpu, expected, strict=False))
        assert err_gpu <= 10  # allow some slack for scale/window effects


def test_solve_pnp_ransac(alloc_timer) -> None:
    """Test solve_pnp_ransac with NumpyImage always, add CudaImage when available."""
    # Camera with distortion
    K = np.array([[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    dist = np.array([0.1, -0.05, 0.001, 0.001, 0.0], dtype=np.float64)
    rng = np.random.default_rng(202)
    obj = rng.uniform(-1.0, 1.0, size=(200, 3)).astype(np.float32)
    obj[:, 2] = np.abs(obj[:, 2]) + 2.0  # keep in front of camera
    rvec_true = np.array([[0.1], [-0.15], [0.05]], dtype=np.float64)
    tvec_true = np.array([[0.2], [-0.1], [3.0]], dtype=np.float64)
    img_pts, _ = cv2.projectPoints(obj, rvec_true, tvec_true, K, dist)
    img_pts = img_pts.reshape(-1, 2)
    # Add outliers
    n_out = 20
    idx = rng.choice(len(img_pts), size=n_out, replace=False)
    img_pts[idx] += rng.uniform(-50, 50, size=(n_out, 2))
    img_pts = img_pts.astype(np.float32)

    base_bgr = _prepare_image(ImageFormat.BGR, (480, 640, 3))
    cpu, gpu, _, _ = alloc_timer(base_bgr, ImageFormat.BGR, label="test_solve_pnp_ransac-setup")

    # Always test CPU backend
    ok_cpu, r_cpu, t_cpu, mask_cpu = cpu.solve_pnp_ransac(
        obj, img_pts, K, dist, iterations_count=150, reprojection_error=3.0
    )
    assert ok_cpu
    inlier_ratio = mask_cpu.mean()
    assert inlier_ratio > 0.7

    # Reprojection error on inliers
    in_idx = np.nonzero(mask_cpu)[0]
    proj_cpu, _ = cv2.projectPoints(obj[in_idx], r_cpu, t_cpu, K, dist)
    proj_cpu = proj_cpu.reshape(-1, 2)
    err = np.linalg.norm(proj_cpu - img_pts[in_idx], axis=1)
    assert err.mean() < 1.5
    assert err.max() < 4.0

    # Optionally test GPU parity when CUDA is available
    if gpu is not None:
        ok_gpu, r_gpu, t_gpu, mask_gpu = gpu.solve_pnp_ransac(
            obj, img_pts, K, dist, iterations_count=150, reprojection_error=3.0
        )
        assert ok_gpu
        inlier_ratio_gpu = mask_gpu.mean()
        assert inlier_ratio_gpu > 0.7

        # Reprojection error on inliers for GPU
        in_idx_gpu = np.nonzero(mask_gpu)[0]
        proj_gpu, _ = cv2.projectPoints(obj[in_idx_gpu], r_gpu, t_gpu, K, dist)
        proj_gpu = proj_gpu.reshape(-1, 2)
        err_gpu = np.linalg.norm(proj_gpu - img_pts[in_idx_gpu], axis=1)
        assert err_gpu.mean() < 1.5
        assert err_gpu.max() < 4.0


def test_solve_pnp_batch(alloc_timer) -> None:
    """Test solve_pnp batch processing with NumpyImage always, add CudaImage when available."""
    # Note: Batch processing is primarily a GPU feature, but we can still test CPU loop
    # Generate batched problems
    B, N = 8, 50
    rng = np.random.default_rng(99)
    obj = rng.uniform(-1.0, 1.0, size=(B, N, 3)).astype(np.float32)
    obj[:, :, 2] = np.abs(obj[:, :, 2]) + 2.0
    K = np.array([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    r_true = np.zeros((B, 3, 1), dtype=np.float64)
    t_true = np.tile(np.array([[0.0], [0.0], [3.0]], dtype=np.float64), (B, 1, 1))
    img = []
    for b in range(B):
        ip, _ = cv2.projectPoints(obj[b], r_true[b], t_true[b], K, None)
        img.append(ip.reshape(-1, 2))
    img = np.stack(img, axis=0).astype(np.float32)

    base_bgr = _prepare_image(ImageFormat.BGR, (10, 10, 3))
    cpu, gpu, _, _ = alloc_timer(base_bgr, ImageFormat.BGR, label="test_solve_pnp_batch-setup")

    # Always test CPU loop
    t0 = time.perf_counter()
    r_list = []
    t_list = []
    for b in range(B):
        ok, r, t = cpu.solve_pnp(obj[b], img[b], K, None)
        assert ok
        r_list.append(r)
        t_list.append(t)
    cpu_total = time.perf_counter() - t0
    cpu_t = cpu_total / B

    # Check reprojection for CPU results
    for b in range(min(B, 2)):
        proj, _ = cv2.projectPoints(obj[b], r_list[b], t_list[b], K, None)
        err = np.linalg.norm(proj.reshape(-1, 2) - img[b], axis=1)
        assert err.mean() < 1e-2
        assert err.max() < 1e-1

    # Optionally test GPU batch when CUDA is available
    if gpu is not None and hasattr(gpu._impl, "solve_pnp_batch"):
        t0 = time.perf_counter()
        r_b, t_b = gpu.solve_pnp_batch(obj, img, K)
        gpu_total = time.perf_counter() - t0
        gpu_t = gpu_total / B
        print(f"solvePnP-batch (avg per pose) cpu={cpu_t:.6f}s gpu={gpu_t:.6f}s (B={B}, N={N})")

        # Check reprojection for GPU batches
        for b in range(min(B, 4)):
            proj, _ = cv2.projectPoints(obj[b], r_b[b], t_b[b], K, None)
            err = np.linalg.norm(proj.reshape(-1, 2) - img[b], axis=1)
            assert err.mean() < 1e-2
            assert err.max() < 1e-1
    else:
        print(f"solvePnP-batch (avg per pose) cpu={cpu_t:.6f}s (GPU batch not available)")


def test_nvimgcodec_flag_and_fallback(monkeypatch) -> None:
    # Test that to_base64() works with and without nvimgcodec by patching runtime flags
    import dimos.msgs.sensor_msgs.image_impls.AbstractImage as AbstractImageMod

    arr = _prepare_image(ImageFormat.BGR, (32, 32, 3))

    # Save original values
    original_has_nvimgcodec = AbstractImageMod.HAS_NVIMGCODEC
    original_nvimgcodec = AbstractImageMod.nvimgcodec

    try:
        # Test 1: Simulate nvimgcodec not available
        monkeypatch.setattr(AbstractImageMod, "HAS_NVIMGCODEC", False)
        monkeypatch.setattr(AbstractImageMod, "nvimgcodec", None)

        # Should work via cv2 fallback for CPU
        img_cpu = Image.from_numpy(arr, format=ImageFormat.BGR, to_cuda=False)
        b64_cpu = img_cpu.to_base64()
        assert isinstance(b64_cpu, str) and len(b64_cpu) > 0

        # If CUDA available, test GPU fallback to CPU encoding
        if HAS_CUDA:
            img_gpu = Image.from_numpy(arr, format=ImageFormat.BGR, to_cuda=True)
            b64_gpu = img_gpu.to_base64()
            assert isinstance(b64_gpu, str) and len(b64_gpu) > 0
            # Should have fallen back to CPU encoding
            assert not AbstractImageMod.NVIMGCODEC_LAST_USED

        # Test 2: Restore nvimgcodec if it was originally available
        if original_has_nvimgcodec:
            monkeypatch.setattr(AbstractImageMod, "HAS_NVIMGCODEC", True)
            monkeypatch.setattr(AbstractImageMod, "nvimgcodec", original_nvimgcodec)

            # Test it still works with nvimgcodec "available"
            img2 = Image.from_numpy(arr, format=ImageFormat.BGR, to_cuda=HAS_CUDA)
            b64_2 = img2.to_base64()
            assert isinstance(b64_2, str) and len(b64_2) > 0

    finally:
        pass


@pytest.mark.skipif(not HAS_CUDA, reason="CuPy/CUDA not available")
def test_nvimgcodec_gpu_path(monkeypatch) -> None:
    """Test nvimgcodec GPU encoding path when CUDA is available.

    This test specifically verifies that when nvimgcodec is available,
    GPU images can be encoded directly without falling back to CPU.
    """
    import dimos.msgs.sensor_msgs.image_impls.AbstractImage as AbstractImageMod

    # Check if nvimgcodec was originally available
    if not AbstractImageMod.HAS_NVIMGCODEC:
        pytest.skip("nvimgcodec library not available")

    # Save original nvimgcodec module reference

    # Create a CUDA image and encode using the actual nvimgcodec if available
    arr = _prepare_image(ImageFormat.BGR, (32, 32, 3))

    # Test with nvimgcodec enabled (should be the default if available)
    img = Image.from_numpy(arr, format=ImageFormat.BGR, to_cuda=True)
    b64 = img.to_base64()
    assert isinstance(b64, str) and len(b64) > 0

    # Check if GPU encoding was actually used
    # Some builds may import nvimgcodec but not support CuPy device buffers
    if not getattr(AbstractImageMod, "NVIMGCODEC_LAST_USED", False):
        pytest.skip("nvimgcodec present but encode fell back to CPU in this environment")

    # Now test that we can disable nvimgcodec and still encode via fallback
    monkeypatch.setattr(AbstractImageMod, "HAS_NVIMGCODEC", False)
    monkeypatch.setattr(AbstractImageMod, "nvimgcodec", None)

    # Create another GPU image - should fall back to CPU encoding
    img2 = Image.from_numpy(arr, format=ImageFormat.BGR, to_cuda=True)
    b64_2 = img2.to_base64()
    assert isinstance(b64_2, str) and len(b64_2) > 0
    # Should have fallen back to CPU encoding
    assert not AbstractImageMod.NVIMGCODEC_LAST_USED


@pytest.mark.skipif(not HAS_CUDA, reason="CuPy/CUDA not available")
def test_to_cpu_format_preservation() -> None:
    """Test that to_cpu() preserves image format correctly.

    This tests the fix for the bug where to_cpu() was using to_opencv()
    which always returns BGR, but keeping the original format label.
    """
    # Test RGB format preservation
    rgb_array = np.random.randint(0, 255, (100, 100, 3), dtype=np.uint8)
    gpu_img_rgb = Image.from_numpy(rgb_array, format=ImageFormat.RGB, to_cuda=True)
    cpu_img_rgb = gpu_img_rgb.to_cpu()

    # Verify format is preserved
    assert cpu_img_rgb.format == ImageFormat.RGB, (
        f"Format mismatch: expected RGB, got {cpu_img_rgb.format}"
    )
    # Verify data is actually in RGB format (not BGR)
    np.testing.assert_array_equal(cpu_img_rgb.data, rgb_array)

    # Test RGBA format preservation
    rgba_array = np.random.randint(0, 255, (100, 100, 4), dtype=np.uint8)
    gpu_img_rgba = Image.from_numpy(rgba_array, format=ImageFormat.RGBA, to_cuda=True)
    cpu_img_rgba = gpu_img_rgba.to_cpu()

    assert cpu_img_rgba.format == ImageFormat.RGBA, (
        f"Format mismatch: expected RGBA, got {cpu_img_rgba.format}"
    )
    np.testing.assert_array_equal(cpu_img_rgba.data, rgba_array)

    # Test BGR format (should be unchanged since to_opencv returns BGR)
    bgr_array = np.random.randint(0, 255, (100, 100, 3), dtype=np.uint8)
    gpu_img_bgr = Image.from_numpy(bgr_array, format=ImageFormat.BGR, to_cuda=True)
    cpu_img_bgr = gpu_img_bgr.to_cpu()

    assert cpu_img_bgr.format == ImageFormat.BGR, (
        f"Format mismatch: expected BGR, got {cpu_img_bgr.format}"
    )
    np.testing.assert_array_equal(cpu_img_bgr.data, bgr_array)

    # Test BGRA format
    bgra_array = np.random.randint(0, 255, (100, 100, 4), dtype=np.uint8)
    gpu_img_bgra = Image.from_numpy(bgra_array, format=ImageFormat.BGRA, to_cuda=True)
    cpu_img_bgra = gpu_img_bgra.to_cpu()

    assert cpu_img_bgra.format == ImageFormat.BGRA, (
        f"Format mismatch: expected BGRA, got {cpu_img_bgra.format}"
    )
    np.testing.assert_array_equal(cpu_img_bgra.data, bgra_array)

    # Test GRAY format
    gray_array = np.random.randint(0, 255, (100, 100), dtype=np.uint8)
    gpu_img_gray = Image.from_numpy(gray_array, format=ImageFormat.GRAY, to_cuda=True)
    cpu_img_gray = gpu_img_gray.to_cpu()

    assert cpu_img_gray.format == ImageFormat.GRAY, (
        f"Format mismatch: expected GRAY, got {cpu_img_gray.format}"
    )
    np.testing.assert_array_equal(cpu_img_gray.data, gray_array)

    # Test DEPTH format (float32)
    depth_array = np.random.uniform(0.5, 10.0, (100, 100)).astype(np.float32)
    gpu_img_depth = Image.from_numpy(depth_array, format=ImageFormat.DEPTH, to_cuda=True)
    cpu_img_depth = gpu_img_depth.to_cpu()

    assert cpu_img_depth.format == ImageFormat.DEPTH, (
        f"Format mismatch: expected DEPTH, got {cpu_img_depth.format}"
    )
    np.testing.assert_array_equal(cpu_img_depth.data, depth_array)

    # Test DEPTH16 format (uint16)
    depth16_array = np.random.randint(100, 65000, (100, 100), dtype=np.uint16)
    gpu_img_depth16 = Image.from_numpy(depth16_array, format=ImageFormat.DEPTH16, to_cuda=True)
    cpu_img_depth16 = gpu_img_depth16.to_cpu()

    assert cpu_img_depth16.format == ImageFormat.DEPTH16, (
        f"Format mismatch: expected DEPTH16, got {cpu_img_depth16.format}"
    )
    np.testing.assert_array_equal(cpu_img_depth16.data, depth16_array)

    # Test GRAY16 format (uint16)
    gray16_array = np.random.randint(0, 65535, (100, 100), dtype=np.uint16)
    gpu_img_gray16 = Image.from_numpy(gray16_array, format=ImageFormat.GRAY16, to_cuda=True)
    cpu_img_gray16 = gpu_img_gray16.to_cpu()

    assert cpu_img_gray16.format == ImageFormat.GRAY16, (
        f"Format mismatch: expected GRAY16, got {cpu_img_gray16.format}"
    )
    np.testing.assert_array_equal(cpu_img_gray16.data, gray16_array)
