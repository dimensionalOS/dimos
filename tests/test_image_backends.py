import os
import numpy as np
import pytest

from dimos.msgs.sensor_msgs.Image import Image, ImageFormat, HAS_CUDA
import cv2


def _rand_uint8(shape, seed=1337):
    rng = np.random.default_rng(seed)
    return rng.integers(0, 256, size=shape, dtype=np.uint8)


@pytest.mark.parametrize(
    "shape,fmt",
    [
        ((64, 64, 3), ImageFormat.BGR),
        ((64, 64, 4), ImageFormat.BGRA),
        ((64, 64, 3), ImageFormat.RGB),
        ((64, 64), ImageFormat.GRAY),
    ],
)
@pytest.mark.skipif(not HAS_CUDA, reason="CuPy/CUDA not available")
def test_color_conversions_parity(shape, fmt):
    arr = _rand_uint8(shape)
    # Build CPU and CUDA images with same logical content
    cpu = Image.from_numpy(arr, format=fmt)
    gpu = Image.from_numpy(arr, format=fmt, to_cuda=True)

    # Test to_rgb -> to_bgr parity
    cpu_round = cpu.to_rgb().to_bgr().to_opencv()
    gpu_round = gpu.to_rgb().to_bgr().to_opencv()

    assert cpu_round.shape == gpu_round.shape
    assert cpu_round.dtype == gpu_round.dtype
    # Exact match for uint8 color ops
    assert np.array_equal(cpu_round, gpu_round)


@pytest.mark.skipif(not HAS_CUDA, reason="CuPy/CUDA not available")
def test_grayscale_parity():
    arr = _rand_uint8((48, 32, 3), seed=7)
    cpu = Image.from_numpy(arr, format=ImageFormat.BGR)
    gpu = Image.from_numpy(arr, format=ImageFormat.BGR, to_cuda=True)

    cpu_gray = cpu.to_grayscale().to_opencv()
    gpu_gray = gpu.to_grayscale().to_opencv()

    assert cpu_gray.shape == gpu_gray.shape
    assert cpu_gray.dtype == gpu_gray.dtype
    # Allow tiny rounding differences (<=1 LSB) — visually indistinguishable
    diff = np.abs(cpu_gray.astype(np.int16) - gpu_gray.astype(np.int16))
    assert diff.max() <= 1


@pytest.mark.parametrize("fmt", [ImageFormat.BGR, ImageFormat.RGB, ImageFormat.BGRA])
@pytest.mark.skipif(not HAS_CUDA, reason="CuPy/CUDA not available")
def test_resize_parity(fmt):
    shape = (60, 80, 3) if fmt in (ImageFormat.BGR, ImageFormat.RGB) else (60, 80, 4)
    arr = _rand_uint8(shape, seed=9)
    cpu = Image.from_numpy(arr, format=fmt)
    gpu = Image.from_numpy(arr, format=fmt, to_cuda=True)

    new_w, new_h = 37, 53
    cpu_res = cpu.resize(new_w, new_h).to_opencv()
    gpu_res = gpu.resize(new_w, new_h).to_opencv()

    assert cpu_res.shape == gpu_res.shape
    assert cpu_res.dtype == gpu_res.dtype
    # Allow small tolerance due to float interpolation differences
    assert np.max(np.abs(cpu_res.astype(np.int16) - gpu_res.astype(np.int16))) <= 1


@pytest.mark.skipif(not HAS_CUDA, reason="CuPy/CUDA not available")
def test_sharpness_parity():
    arr = _rand_uint8((64, 64, 3), seed=42)
    cpu = Image.from_numpy(arr, format=ImageFormat.BGR)
    gpu = Image.from_numpy(arr, format=ImageFormat.BGR, to_cuda=True)

    s_cpu = cpu.sharpness()
    s_gpu = gpu.sharpness()

    # Values should be very close; minor border/rounding differences allowed
    assert abs(s_cpu - s_gpu) < 5e-2


@pytest.mark.skipif(not HAS_CUDA, reason="CuPy/CUDA not available")
def test_to_opencv_parity():
    # BGRA should drop alpha and produce BGR
    arr = _rand_uint8((32, 32, 4), seed=21)
    cpu = Image.from_numpy(arr, format=ImageFormat.BGRA)
    gpu = Image.from_numpy(arr, format=ImageFormat.BGRA, to_cuda=True)

    cpu_bgr = cpu.to_opencv()
    gpu_bgr = gpu.to_opencv()

    assert cpu_bgr.shape == (32, 32, 3)
    assert gpu_bgr.shape == (32, 32, 3)
    assert np.array_equal(cpu_bgr, gpu_bgr)


@pytest.mark.skipif(not HAS_CUDA, reason="CuPy/CUDA not available")
def test_solve_pnp_parity():
    # Synthetic camera and 3D points
    K = np.array([[400.0, 0.0, 32.0], [0.0, 400.0, 24.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    dist = None
    obj = np.array([
        [-0.5, -0.5, 0.0],
        [0.5, -0.5, 0.0],
        [0.5, 0.5, 0.0],
        [-0.5, 0.5, 0.0],
        [0.0, 0.0, 0.5],
        [0.0, 0.0, 1.0],
    ], dtype=np.float32)

    rvec_true = np.zeros((3, 1), dtype=np.float64)
    tvec_true = np.array([[0.0], [0.0], [2.0]], dtype=np.float64)
    img_pts, _ = cv2.projectPoints(obj, rvec_true, tvec_true, K, dist)
    img_pts = img_pts.reshape(-1, 2).astype(np.float32)

    # Build images (content irrelevant for solvePnP)
    cpu = Image.from_numpy(np.zeros((48, 64, 3), dtype=np.uint8), format=ImageFormat.BGR)
    gpu = Image.from_numpy(np.zeros((48, 64, 3), dtype=np.uint8), format=ImageFormat.BGR, to_cuda=True)

    ok_cpu, r_cpu, t_cpu = cpu.solve_pnp(obj, img_pts, K, dist)
    ok_gpu, r_gpu, t_gpu = gpu.solve_pnp(obj, img_pts, K, dist)

    assert ok_cpu and ok_gpu
    # Validate reprojection error for CUDA solver
    proj_cpu, _ = cv2.projectPoints(obj, r_cpu, t_cpu, K, dist)
    proj_gpu, _ = cv2.projectPoints(obj, r_gpu, t_gpu, K, dist)
    proj_cpu = proj_cpu.reshape(-1, 2)
    proj_gpu = proj_gpu.reshape(-1, 2)
    err_gpu = np.linalg.norm(proj_gpu - img_pts, axis=1)
    assert err_gpu.mean() < 1e-3
    assert err_gpu.max() < 1e-2


@pytest.mark.skipif(not HAS_CUDA, reason="CuPy/CUDA not available")
def test_perf_compare_grayscale():
    arr = _rand_uint8((480, 640, 3), seed=3)
    cpu = Image.from_numpy(arr, format=ImageFormat.BGR)
    gpu = Image.from_numpy(arr, format=ImageFormat.BGR, to_cuda=True)
    import time
    t0 = time.perf_counter();
    for _ in range(10):
        _ = cpu.to_grayscale()
    cpu_t = time.perf_counter() - t0
    t0 = time.perf_counter();
    for _ in range(10):
        _ = gpu.to_grayscale()
    gpu_t = time.perf_counter() - t0
    print(f"grayscale cpu={cpu_t:.6f}s gpu={gpu_t:.6f}s")
    assert cpu_t > 0 and gpu_t > 0


@pytest.mark.skipif(not HAS_CUDA, reason="CuPy/CUDA not available")
def test_perf_compare_resize():
    arr = _rand_uint8((480, 640, 3), seed=4)
    cpu = Image.from_numpy(arr, format=ImageFormat.BGR)
    gpu = Image.from_numpy(arr, format=ImageFormat.BGR, to_cuda=True)
    import time
    t0 = time.perf_counter();
    for _ in range(5):
        _ = cpu.resize(320, 240)
    cpu_t = time.perf_counter() - t0
    t0 = time.perf_counter();
    for _ in range(5):
        _ = gpu.resize(320, 240)
    gpu_t = time.perf_counter() - t0
    print(f"resize cpu={cpu_t:.6f}s gpu={gpu_t:.6f}s")
    assert cpu_t > 0 and gpu_t > 0


@pytest.mark.skipif(not HAS_CUDA, reason="CuPy/CUDA not available")
def test_perf_compare_sharpness():
    arr = _rand_uint8((480, 640, 3), seed=5)
    cpu = Image.from_numpy(arr, format=ImageFormat.BGR)
    gpu = Image.from_numpy(arr, format=ImageFormat.BGR, to_cuda=True)
    import time
    t0 = time.perf_counter();
    for _ in range(3):
        _ = cpu.sharpness()
    cpu_t = time.perf_counter() - t0
    t0 = time.perf_counter();
    for _ in range(3):
        _ = gpu.sharpness()
    gpu_t = time.perf_counter() - t0
    print(f"sharpness cpu={cpu_t:.6f}s gpu={gpu_t:.6f}s")
    assert cpu_t > 0 and gpu_t > 0


@pytest.mark.skipif(not HAS_CUDA, reason="CuPy/CUDA not available")
def test_perf_compare_solvepnp():
    K = np.array([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    dist = None
    rng = np.random.default_rng(123)
    obj = rng.standard_normal((200, 3)).astype(np.float32)
    rvec_true = np.array([[0.1], [-0.2], [0.05]])
    tvec_true = np.array([[0.0], [0.0], [3.0]])
    img_pts, _ = cv2.projectPoints(obj, rvec_true, tvec_true, K, dist)
    img_pts = img_pts.reshape(-1, 2).astype(np.float32)
    cpu = Image.from_numpy(np.zeros((480, 640, 3), dtype=np.uint8), format=ImageFormat.BGR)
    gpu = Image.from_numpy(np.zeros((480, 640, 3), dtype=np.uint8), format=ImageFormat.BGR, to_cuda=True)
    import time
    t0 = time.perf_counter();
    for _ in range(5):
        _ = cpu.solve_pnp(obj, img_pts, K, dist)
    cpu_t = time.perf_counter() - t0
    t0 = time.perf_counter();
    for _ in range(5):
        _ = gpu.solve_pnp(obj, img_pts, K, dist)
    gpu_t = time.perf_counter() - t0
    print(f"solvePnP cpu={cpu_t:.6f}s gpu={gpu_t:.6f}s")
    assert cpu_t > 0 and gpu_t > 0


@pytest.mark.skipif(not HAS_CUDA, reason="CuPy/CUDA not available")
def test_perf_compare_tracker():
    H, W = 240, 320
    img1 = np.zeros((H, W, 3), dtype=np.uint8)
    img2 = np.zeros((H, W, 3), dtype=np.uint8)
    bbox0 = (80, 60, 40, 30)
    x0, y0, w0, h0 = bbox0
    img1[y0 : y0 + h0, x0 : x0 + w0] = 255
    dx, dy = 8, 5
    img2[y0 + dy : y0 + dy + h0, x0 + dx : x0 + dx + w0] = 255
    cpu1 = Image.from_numpy(img1, format=ImageFormat.BGR)
    cpu2 = Image.from_numpy(img2, format=ImageFormat.BGR)
    gpu1 = Image.from_numpy(img1, format=ImageFormat.BGR, to_cuda=True)
    gpu2 = Image.from_numpy(img2, format=ImageFormat.BGR, to_cuda=True)
    trk_cpu = cpu1.create_csrt_tracker(bbox0)
    trk_gpu = gpu1.create_csrt_tracker(bbox0)
    import time
    t0 = time.perf_counter();
    for _ in range(10):
        _ = cpu2.csrt_update(trk_cpu)
    cpu_t = time.perf_counter() - t0
    t0 = time.perf_counter();
    for _ in range(10):
        _ = gpu2.csrt_update(trk_gpu)
    gpu_t = time.perf_counter() - t0
    print(f"tracker cpu={cpu_t:.6f}s gpu={gpu_t:.6f}s")
    assert cpu_t > 0 and gpu_t > 0


@pytest.mark.skipif(not HAS_CUDA, reason="CuPy/CUDA not available")
def test_csrt_tracker_parity():
    # Check tracker availability
    has_csrt = False
    if hasattr(cv2, "legacy") and hasattr(cv2.legacy, "TrackerCSRT_create"):
        has_csrt = True
    elif hasattr(cv2, "TrackerCSRT_create"):
        has_csrt = True
    if not has_csrt:
        pytest.skip("OpenCV CSRT tracker not available")

    H, W = 100, 100
    # Create two frames with a moving rectangle
    img1 = np.zeros((H, W, 3), dtype=np.uint8)
    img2 = np.zeros((H, W, 3), dtype=np.uint8)
    bbox0 = (30, 30, 20, 15)
    x0, y0, w0, h0 = bbox0
    # draw rect in img1
    img1[y0 : y0 + h0, x0 : x0 + w0] = 255
    # shift by (dx,dy)
    dx, dy = 5, 3
    img2[y0 + dy : y0 + dy + h0, x0 + dx : x0 + dx + w0] = 255

    cpu1 = Image.from_numpy(img1, format=ImageFormat.BGR)
    cpu2 = Image.from_numpy(img2, format=ImageFormat.BGR)
    gpu1 = Image.from_numpy(img1, format=ImageFormat.BGR, to_cuda=True)
    gpu2 = Image.from_numpy(img2, format=ImageFormat.BGR, to_cuda=True)

    trk_cpu = cpu1.create_csrt_tracker(bbox0)
    ok_cpu, bbox_cpu = cpu2.csrt_update(trk_cpu)
    trk_gpu = gpu1.create_csrt_tracker(bbox0)
    ok_gpu, bbox_gpu = gpu2.csrt_update(trk_gpu)

    assert ok_cpu and ok_gpu
    # Compare both to ground-truth expected bbox
    expected = (x0 + dx, y0 + dy, w0, h0)
    err_cpu = sum(abs(a - b) for a, b in zip(bbox_cpu, expected))
    err_gpu = sum(abs(a - b) for a, b in zip(bbox_gpu, expected))
    assert err_cpu <= 8
    assert err_gpu <= 10  # allow some slack for scale/window effects


@pytest.mark.skipif(not HAS_CUDA, reason="CuPy/CUDA not available")
def test_solve_pnp_ransac_with_outliers_and_distortion():
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

    cpu = Image.from_numpy(np.zeros((480, 640, 3), dtype=np.uint8), format=ImageFormat.BGR)
    gpu = Image.from_numpy(np.zeros((480, 640, 3), dtype=np.uint8), format=ImageFormat.BGR, to_cuda=True)

    ok_gpu, r_gpu, t_gpu, mask_gpu = gpu.solve_pnp_ransac(obj, img_pts, K, dist, iterations_count=150, reprojection_error=3.0)
    assert ok_gpu
    inlier_ratio = mask_gpu.mean()
    assert inlier_ratio > 0.7
    # Reprojection error on inliers
    in_idx = np.nonzero(mask_gpu)[0]
    proj_gpu, _ = cv2.projectPoints(obj[in_idx], r_gpu, t_gpu, K, dist)
    proj_gpu = proj_gpu.reshape(-1, 2)
    err = np.linalg.norm(proj_gpu - img_pts[in_idx], axis=1)
    assert err.mean() < 1.5
    assert err.max() < 4.0


@pytest.mark.skipif(not HAS_CUDA, reason="CuPy/CUDA not available")
def test_solve_pnp_batch_correctness_and_perf():
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

    cpu = Image.from_numpy(np.zeros((10, 10, 3), dtype=np.uint8), format=ImageFormat.BGR)
    gpu = Image.from_numpy(np.zeros((10, 10, 3), dtype=np.uint8), format=ImageFormat.BGR, to_cuda=True)

    # CPU loop
    import time
    t0 = time.perf_counter();
    r_list = []
    t_list = []
    for b in range(B):
        ok, r, t = cpu.solve_pnp(obj[b], img[b], K, None)
        assert ok
        r_list.append(r)
        t_list.append(t)
    cpu_t = time.perf_counter() - t0

    # CUDA batched
    t0 = time.perf_counter();
    r_b, t_b = gpu.solve_pnp_batch(obj, img, K)
    gpu_t = time.perf_counter() - t0
    print(f"solvePnP-batch cpu={cpu_t:.6f}s gpu={gpu_t:.6f}s (B={B}, N={N})")

    # Check reprojection for a couple of batches
    for b in range(min(B, 4)):
        proj, _ = cv2.projectPoints(obj[b], r_b[b], t_b[b], K, None)
        err = np.linalg.norm(proj.reshape(-1, 2) - img[b], axis=1)
        assert err.mean() < 1e-2
        assert err.max() < 1e-1

def test_nvimgcodec_flag_and_fallback(monkeypatch):
    # Force nvimgcodec flag on, then reload Image and ensure fallback works
    monkeypatch.setenv("USE_NVIMGCODEC", "1")
    import importlib as _importlib
    ImageMod = _importlib.import_module('dimos.msgs.sensor_msgs.Image')
    _importlib.reload(ImageMod)
    # Even if nvimgcodec missing, to_base64 should work (fallback)
    arr = _rand_uint8((32, 32, 3))
    img = ImageMod.Image.from_numpy(arr, format=ImageMod.ImageFormat.BGR, to_cuda=bool(ImageMod.HAS_CUDA))
    b64 = img.to_base64()
    assert isinstance(b64, str) and len(b64) > 0
    # Turn flag off and reload
    monkeypatch.setenv("USE_NVIMGCODEC", "0")
    _importlib.reload(ImageMod)
    img2 = ImageMod.Image.from_numpy(arr, format=ImageMod.ImageFormat.BGR)
    b64_2 = img2.to_base64()
    assert isinstance(b64_2, str) and len(b64_2) > 0


@pytest.mark.skipif(not HAS_CUDA, reason="CuPy/CUDA not available")
def test_nvimgcodec_gpu_path(monkeypatch):
    # Enable flag and reload; skip if nvimgcodec not present
    monkeypatch.setenv("USE_NVIMGCODEC", "1")
    import importlib as _importlib
    ImageMod = _importlib.import_module('dimos.msgs.sensor_msgs.Image')
    _importlib.reload(ImageMod)
    if not ImageMod.HAS_NVIMGCODEC:
        pytest.skip("nvimgcodec library not available")
    # Create a CUDA image and encode
    arr = _rand_uint8((32, 32, 3))
    img = ImageMod.Image.from_numpy(arr, format=ImageMod.ImageFormat.BGR, to_cuda=True)
    b64 = img.to_base64()
    assert isinstance(b64, str) and len(b64) > 0
    # Some builds may import nvimgcodec but not support CuPy device buffers; allow skip
    if not getattr(ImageMod, 'NVIMGCODEC_LAST_USED', False):
        pytest.skip("nvimgcodec present but encode fell back to CPU in this environment")
