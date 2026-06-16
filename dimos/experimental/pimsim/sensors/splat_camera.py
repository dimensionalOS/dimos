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

"""Splat-rendered camera image stream for sim.

Publishes ``color_image: Out[Image]`` and ``camera_info: Out[CameraInfo]``
by rendering the Gaussian splat scene from the robot's camera pose each
tick.  Consumers (perception, memory, anything subscribing to
``Image`` / ``CameraInfo``) get the same wire format real cameras use,
so the rest of the stack can run unmodified against splat-rendered
images.

Backend selection:
  * Linux + CUDA: ``GsplatBackend`` (real splat rasterization).
  * macOS: ``MacosBackend`` stub publishing a black placeholder until
    a real cross-platform renderer (Brush via wgpu, MLX-based splat,
    etc.) is wired in.

Backends share the ``SplatCameraBackend`` Protocol so additional
backends drop in by name without touching the module.
"""

from __future__ import annotations

import atexit
import os
from pathlib import Path as FilePath
import platform
import sys
import threading
import time
from typing import Any, ClassVar, Protocol

import mujoco
import numpy as np
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.experimental.pimsim.entity import EntityStateBatch
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger
from dimos.visualization.viser.camera import CameraSpec, g1_d435_default, world_pose
from dimos.visualization.viser.robot_meshes import (
    GeomInstance,
    RobotMeshes,
    apply_state,
    dimos_joint_to_mjcf,
    load_robot_meshes,
)
from dimos.visualization.viser.splat import SplatAlignment, SplatData, load_splat

logger = setup_logger()


# =============================================================================
# Backend Protocol + implementations
# =============================================================================


class SplatCameraBackend(Protocol):
    """Renders a Gaussian splat scene from a camera pose to an RGB image."""

    def render(self, cam_world_pos: np.ndarray, cam_world_wxyz: np.ndarray) -> np.ndarray:
        """Render to an HxWx3 uint8 RGB image at the given world-frame camera pose.

        Args:
            cam_world_pos: (3,) camera position in world meters.
            cam_world_wxyz: (4,) camera orientation, image convention
                (+Z forward, +Y down, +X right), wxyz quaternion.
        """
        ...


def _wxyz_to_rotmat_args(w: float, x: float, y: float, z: float) -> np.ndarray:
    return _wxyz_to_rotmat(np.array([w, x, y, z], dtype=np.float64))


def _wxyz_to_rotmat(wxyz: np.ndarray) -> np.ndarray:
    """(4,) wxyz quaternion -> (3, 3) rotation matrix."""
    w, x, y, z = (float(c) for c in wxyz)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float32,
    )


def _world_to_camera_viewmat(cam_world_pos: np.ndarray, cam_world_wxyz: np.ndarray) -> np.ndarray:
    """4x4 world->camera transform from a camera world pose (image convention)."""
    R = _wxyz_to_rotmat(cam_world_wxyz)
    Rt = R.T
    t = np.asarray(cam_world_pos, dtype=np.float32)
    out = np.eye(4, dtype=np.float32)
    out[:3, :3] = Rt
    out[:3, 3] = -Rt @ t
    return out


class GsplatBackend:
    """gsplat-based renderer for Linux + CUDA.

    All splat data lives on the GPU; ``render`` builds a 4x4 view matrix
    from the camera pose and calls ``gsplat.rasterization`` with the
    pinhole intrinsics from the spec.  Inference-only — gradients
    disabled so memory stays flat across frames.
    """

    def __init__(self, splat: SplatData, spec: CameraSpec, device: str = "cuda") -> None:
        try:
            import gsplat
            import torch
        except ImportError as e:
            raise ImportError(
                "gsplat is not installed.  Add the splat extra: "
                "uv pip install -e '.[splat]'  (Linux + CUDA only)"
            ) from e
        self._torch = torch
        self._gsplat = gsplat
        self._device = device
        self._spec = spec

        self._means = torch.from_numpy(splat.centers).to(device).float()
        self._quats = torch.from_numpy(splat.quats_wxyz).to(device).float()
        self._scales = torch.from_numpy(splat.scales).to(device).float()
        self._opacities = torch.from_numpy(splat.opacities.flatten()).to(device).float()
        self._colors = torch.from_numpy(splat.rgbs).to(device).float()

        K = np.eye(3, dtype=np.float32)
        K[0, 0] = spec.focal_pixels()
        K[1, 1] = spec.focal_pixels()
        K[0, 2] = spec.cx()
        K[1, 2] = spec.cy()
        self._K = torch.from_numpy(K).to(device).float().unsqueeze(0)  # (1, 3, 3)

        logger.info(
            f"GsplatBackend ready: {len(splat.centers)} Gaussians on {device}, "
            f"{spec.width}x{spec.height} @ vfov={spec.vfov_deg}°"
        )

    def render(self, cam_world_pos: np.ndarray, cam_world_wxyz: np.ndarray) -> np.ndarray:
        torch = self._torch
        viewmat = _world_to_camera_viewmat(cam_world_pos, cam_world_wxyz)
        viewmats = torch.from_numpy(viewmat).to(self._device).unsqueeze(0)
        with torch.no_grad():
            colors, _alphas, _info = self._gsplat.rasterization(
                means=self._means,
                quats=self._quats,
                scales=self._scales,
                opacities=self._opacities,
                colors=self._colors,
                viewmats=viewmats,
                Ks=self._K,
                width=self._spec.width,
                height=self._spec.height,
            )
        # colors: (1, H, W, 3) float in [0, 1].
        return (colors[0].clamp(0, 1) * 255.0).byte().cpu().numpy()


class _SplatAppleCamera:
    """Adapter for splat-apple's expected camera duck-type.

    splat-apple's ``mlx_gs.renderer.renderer.render`` reads
    ``camera.H/W/fx/fy/cx/cy/W2C`` directly off the object.  We build
    one of these per ``render()`` call so the W2C reflects the latest
    robot pose.
    """

    __slots__ = ("H", "W", "W2C", "cx", "cy", "fx", "fy")

    def __init__(self, spec: CameraSpec, viewmat_mx: object) -> None:
        self.H = spec.height
        self.W = spec.width
        self.fx = spec.focal_pixels()
        self.fy = spec.focal_pixels()
        self.cx = spec.cx()
        self.cy = spec.cy()
        self.W2C = viewmat_mx


def _mlx_subprocess_main(
    conn: Any,
    splat_pkl_path: str,
    spec_pkl_path: str,
    rasterizer_type: str,
) -> None:
    """Standalone subprocess entrypoint for MLX rendering.

    Lives in its own ``spawn``-started Python interpreter so MLX/Metal
    has a clean process state — independent of whatever the dimos
    worker happens to have loaded.  Communication: a single
    ``multiprocessing.Pipe``.  Protocol:

        parent -> child: 4x4 viewmat as a flat 16-tuple, or ``None`` to
                         request termination.
        child  -> parent: ``"ready"`` after init, then per-frame
                          ``("ok", uint8 rgb ndarray)`` or
                          ``("err", traceback string)``.
    """
    import pickle
    import traceback as _tb

    import numpy as _np

    try:
        import mlx.core as mx
        from mlx_gs.core.gaussians import Gaussians
        from mlx_gs.renderer import renderer as _renderer
    except Exception as e:  # pragma: no cover - subprocess-only path
        conn.send(f"import-failed: {e!r}")
        return

    try:
        with open(splat_pkl_path, "rb") as f:
            splat = pickle.load(f)
        with open(spec_pkl_path, "rb") as f:
            spec = pickle.load(f)

        # Optional 3D scale pre-filter.  Default 0 (disabled) because
        # the patched ``mlx_gs/renderer/projection.py`` already does
        # gsplat-style screen-rect + max-radii culling at the rendering
        # stage, which handles training-divergence outliers correctly.
        # Set ``DIMOS_MLX_MAX_SCALE=<meters>`` to additionally drop any
        # gaussians whose 3D max-axis scale exceeds the threshold (e.g.
        # 0.5 to be conservative on noisy splats).
        max_scale_m = float(os.environ.get("DIMOS_MLX_MAX_SCALE", "0"))
        if max_scale_m > 0:
            keep = splat.scales.max(axis=1) < max_scale_m
            n_dropped = int(len(splat.centers) - keep.sum())
            if n_dropped > 0:
                splat = type(splat)(
                    centers=splat.centers[keep],
                    covariances=splat.covariances[keep],
                    rgbs=splat.rgbs[keep],
                    opacities=splat.opacities[keep],
                    quats_wxyz=splat.quats_wxyz[keep],
                    scales=splat.scales[keep],
                )

        scales_raw = _np.log(_np.maximum(splat.scales, 1e-10))
        op_clip = _np.clip(splat.opacities.flatten(), 1e-6, 1.0 - 1e-6)
        opacities_raw = _np.log(op_clip / (1.0 - op_clip)).reshape(-1, 1)
        sh_dc = ((splat.rgbs - 0.5) / 0.28209479177387814).reshape(-1, 1, 3)

        gaussians = Gaussians(
            means=mx.array(splat.centers, dtype=mx.float32),
            scales=mx.array(scales_raw, dtype=mx.float32),
            quaternions=mx.array(splat.quats_wxyz, dtype=mx.float32),
            opacities=mx.array(opacities_raw, dtype=mx.float32),
            sh_coeffs=mx.array(sh_dc, dtype=mx.float32),
        )
        mx.eval(
            gaussians.means,
            gaussians.scales,
            gaussians.quaternions,
            gaussians.opacities,
            gaussians.sh_coeffs,
        )

        intrinsics_K = _np.eye(3, dtype=_np.float32)
        intrinsics_K[0, 0] = spec.focal_pixels()
        intrinsics_K[1, 1] = spec.focal_pixels()
        intrinsics_K[0, 2] = spec.cx()
        intrinsics_K[1, 2] = spec.cy()
        K_mx = mx.array(intrinsics_K, dtype=mx.float32)
        mx.eval(K_mx)
    except Exception:  # pragma: no cover
        conn.send(f"init-failed:\n{_tb.format_exc()}")
        return

    conn.send("ready")

    class _Cam:
        H = spec.height
        W = spec.width
        fx = spec.focal_pixels()
        fy = spec.focal_pixels()
        cx = spec.cx()
        cy = spec.cy()
        W2C = K_mx  # placeholder; overwritten per frame

    cam = _Cam()

    while True:
        msg = conn.recv()
        if msg is None:
            return
        try:
            viewmat_flat = msg
            viewmat = _np.asarray(viewmat_flat, dtype=_np.float32).reshape(4, 4)
            cam.W2C = mx.array(viewmat, dtype=mx.float32)
            img = _renderer.render(gaussians, cam, rasterizer_type=rasterizer_type)
            mx.eval(img)
            rgb_u8 = (_np.clip(_np.array(img), 0.0, 1.0) * 255.0).astype(_np.uint8)
            conn.send(("ok", rgb_u8))
        except Exception:
            conn.send(("err", _tb.format_exc()))


class MlxBackend:
    """MLX-based renderer for macOS Apple Silicon (via ghif/splat-apple).

    Architecture: the actual rasterization runs in a **separate spawned
    subprocess**, not in the dimos worker.  Why:

      * The dimos worker is itself a forkserver child with a long
        chain of imports (rpyc, structlog, mujoco, viser_render_module,
        cv2, ...).  At least one of those leaves the Metal compiler XPC
        connection in a state where MLX cannot compile its kernels —
        ``Compiler encountered XPC_ERROR_CONNECTION_INVALID``.
      * Spawn-starting a brand-new Python interpreter for the rasterizer
        side-steps that entirely.  The subprocess imports only what it
        needs (``mlx``, ``mlx_gs``, ``numpy``, ``pickle``); its Metal
        context is born clean.

    The parent (``MlxBackend``) holds a ``multiprocessing.Pipe`` to the
    subprocess.  Per ``render()`` call: send the 4x4 viewmat, receive
    the rendered uint8 image.  Per-frame overhead is dominated by the
    pickle of a 320x180x3 ndarray (~170 KB; sub-millisecond on Mac).

    Install requirements:
        git clone https://github.com/ghif/splat-apple
        echo /path/to/splat-apple > .venv/lib/pythonX.Y/site-packages/splat_apple.pth

    Optional perf upgrade — build the C++ Metal kernel for ~2x speedup:
        cd /path/to/splat-apple && python setup_mlx.py build_ext --inplace
        # then run dimos with DIMOS_MLX_RASTERIZER=cpp

    NOTE on a required local patch to splat-apple:
        ``mlx_gs/renderer/projection.py`` ships with ``radii = clip(radii,
        0, 1000)`` which doesn't cull anything — it just changes tile-
        binning extent.  Training-divergence "fog blob" gaussians (centers
        thousands of meters off-screen and/or 2D radii larger than the
        image) still get evaluated at every pixel and saturate alpha,
        producing the cubish-edge / blurry-foreground artifact.  We
        replace that line with proper culls (AABB-vs-screen-rect intersect
        + max-2D-radius cull).  The patch is in-place at the cloned splat-
        apple checkout — re-apply if you ever ``git pull`` it.
    """

    _RECV_TIMEOUT_SEC = 30.0

    def __init__(
        self,
        splat: SplatData,
        spec: CameraSpec,
        *,
        rasterizer_type: str = "python",
    ) -> None:
        # The rasterizer runs in a subprocess (see class docstring); we
        # just stash splat + spec to disk so it can pick them up after
        # spawn (which doesn't inherit memory from the parent).
        self._spec = spec
        self._rasterizer_type = os.environ.get("DIMOS_MLX_RASTERIZER", rasterizer_type)
        self._proc: Any = None
        self._pipe: Any = None
        self._proc_lock = threading.Lock()

        import pickle as _pickle
        import tempfile

        self._tmpdir = tempfile.mkdtemp(prefix="dimos-mlx-")
        self._splat_pkl = os.path.join(self._tmpdir, "splat.pkl")
        self._spec_pkl = os.path.join(self._tmpdir, "spec.pkl")
        with open(self._splat_pkl, "wb") as f:
            _pickle.dump(splat, f, protocol=_pickle.HIGHEST_PROTOCOL)
        with open(self._spec_pkl, "wb") as f:
            _pickle.dump(spec, f, protocol=_pickle.HIGHEST_PROTOCOL)

        # Verify mlx_gs is importable in the parent (cheap reachability
        # check) but don't actually use mlx here — that would poison the
        # parent's Metal state for everyone.
        import importlib.util

        if importlib.util.find_spec("mlx_gs") is None:
            raise ImportError(
                "mlx_gs not importable.  Clone "
                "https://github.com/ghif/splat-apple and add it to the "
                "venv via a .pth file (see MlxBackend docstring)."
            )

        atexit.register(self._shutdown)

        logger.info(
            f"MlxBackend (splat-apple/{self._rasterizer_type}) configured: "
            f"{len(splat.centers)} Gaussians, {spec.width}x{spec.height} "
            f"@ vfov={spec.vfov_deg}° (subprocess starts on first render)"
        )

    def _ensure_subprocess(self) -> None:
        """Spawn the rasterizer subprocess on first render call."""
        if self._proc is not None:
            return
        with self._proc_lock:
            if self._proc is not None:
                return
            import multiprocessing as _mp

            ctx = _mp.get_context("spawn")
            parent_conn, child_conn = ctx.Pipe()
            proc = ctx.Process(
                target=_mlx_subprocess_main,
                args=(
                    child_conn,
                    self._splat_pkl,
                    self._spec_pkl,
                    self._rasterizer_type,
                ),
                daemon=False,
            )
            # The dimos worker process is itself started with
            # ``daemon=True`` (see ``python_worker.py:218``).
            # ``Process.start()`` asserts that daemonic processes can't
            # have children, full stop — regardless of the child's own
            # daemon flag.  Temporarily lie about our daemon-ness for
            # the duration of the spawn; multiprocessing only consults
            # this flag at start-time.  Cleanup of the rasterizer
            # subprocess happens via atexit + ``stop()``.
            current = _mp.current_process()
            old_daemon = current._config.get("daemon")  # type: ignore[attr-defined]
            current._config["daemon"] = False  # type: ignore[attr-defined]
            try:
                proc.start()
            finally:
                if old_daemon is None:
                    current._config.pop("daemon", None)  # type: ignore[attr-defined]
                else:
                    current._config["daemon"] = old_daemon  # type: ignore[attr-defined]

            # Wait up to 60s for the child to finish loading + Metal init.
            if not parent_conn.poll(60.0):
                proc.terminate()
                raise RuntimeError("MlxBackend subprocess did not signal ready")
            ready = parent_conn.recv()
            if ready != "ready":
                proc.terminate()
                raise RuntimeError(f"MlxBackend subprocess init failed: {ready}")

            self._proc = proc
            self._pipe = parent_conn
            logger.info(
                f"MlxBackend subprocess ready (pid={proc.pid}, rasterizer={self._rasterizer_type})"
            )

    def render(self, cam_world_pos: np.ndarray, cam_world_wxyz: np.ndarray) -> np.ndarray:
        try:
            self._ensure_subprocess()
            viewmat = _world_to_camera_viewmat(cam_world_pos, cam_world_wxyz)
            self._pipe.send(viewmat.tolist())
            if not self._pipe.poll(self._RECV_TIMEOUT_SEC):
                raise TimeoutError(
                    f"MlxBackend subprocess did not respond within "
                    f"{self._RECV_TIMEOUT_SEC}s (likely hung)"
                )
            result = self._pipe.recv()
            if not isinstance(result, tuple) or len(result) != 2:
                raise RuntimeError(f"MlxBackend subprocess unexpected reply: {result!r}")
            tag, payload = result
            if tag == "ok":
                return payload  # already (H, W, 3) uint8
            if tag == "err":
                raise RuntimeError(f"MlxBackend subprocess render failed:\n{payload}")
            raise RuntimeError(f"MlxBackend subprocess unknown tag: {tag!r}")
        except Exception:
            # The render loop in SplatCameraModule swallows render
            # failures at DEBUG level — escalate here so a silent
            # black image isn't impossible to diagnose.
            logger.exception("MlxBackend.render failed")
            raise

    def _shutdown(self) -> None:
        """Best-effort cleanup of the rasterizer subprocess."""
        if self._proc is None:
            return
        try:
            if self._pipe is not None:
                try:
                    self._pipe.send(None)
                except (BrokenPipeError, OSError):
                    pass
            self._proc.join(timeout=5.0)
            if self._proc.is_alive():
                self._proc.terminate()
                self._proc.join(timeout=2.0)
            if self._proc.is_alive():
                self._proc.kill()
        except Exception:
            logger.warning("MlxBackend: error shutting down subprocess", exc_info=True)
        finally:
            self._proc = None
            self._pipe = None
            try:
                import shutil

                shutil.rmtree(self._tmpdir, ignore_errors=True)
            except Exception:
                pass

    # Public alias matching the SplatCameraModule's stop() expectation if
    # one is ever added later — currently the atexit hook is the only
    # cleanup path.
    def stop(self) -> None:
        self._shutdown()


class MacosBackend:
    """Cross-platform stub backend.

    Currently publishes a black placeholder so ``SplatCameraModule``'s
    output ports stay live for downstream consumers — useful for
    confirming the wiring (camera pose subscribed, image+info
    published) without a real renderer available.

    To wire a real Mac renderer, replace ``render`` with a call into
    your renderer of choice:

      * **Brush** (Rust + wgpu, cross-platform via Metal/Vulkan): pip
        install + minimal Python binding on top of the splat data
        already loaded in this module.
      * **MLX-based splat**: Apple-Silicon-only, near-zero copy
        through unified memory.
      * **Headless browser** (Three.js + gsplat.js in headless
        Chromium): heaviest integration but truly portable.

    The splat data this backend was constructed with lives on
    ``self._splat`` (a ``SplatData``) — it has both the covariance
    form (for raster engines) and the primitive (means, quats, scales,
    rgbs, opacities) form (for tile-based rasterizers).  Pick whichever
    matches the renderer's input shape.
    """

    def __init__(self, splat: SplatData, spec: CameraSpec) -> None:
        self._spec = spec
        self._splat = splat
        self._placeholder = np.zeros((spec.height, spec.width, 3), dtype=np.uint8)
        logger.info(
            f"MacosBackend stub ready: {len(splat.centers)} Gaussians cached but not "
            f"rendered — black placeholder image only.  See module docstring to wire a "
            f"real renderer."
        )

    def render(self, cam_world_pos: np.ndarray, cam_world_wxyz: np.ndarray) -> np.ndarray:
        # TODO: replace with Brush / MLX / etc.  The (cam_world_pos,
        # cam_world_wxyz) pair is image-convention (+Z forward).
        return self._placeholder


def make_backend(splat: SplatData, spec: CameraSpec) -> SplatCameraBackend:
    """Pick a backend based on platform + import availability.

    Order:
      1. macOS + Apple Silicon + ``mlx_gs`` (splat-apple) importable
         -> ``MlxBackend``.
      2. macOS Intel, or arm64 Mac without splat-apple installed
         -> ``MacosBackend`` stub.
      3. Linux + ``gsplat`` installed -> ``GsplatBackend``.
      4. Otherwise -> ``MacosBackend`` stub.
    """
    if sys.platform == "darwin" and platform.machine() == "arm64":
        try:
            import mlx_gs  # noqa: F401

            return MlxBackend(splat, spec)
        except ImportError:
            logger.warning(
                "SplatCamera: mlx_gs (splat-apple) not importable on Apple "
                "Silicon — falling back to stub backend.  Clone "
                "https://github.com/ghif/splat-apple and expose it via a "
                ".pth file in the venv (see MlxBackend docstring)."
            )
            return MacosBackend(splat, spec)

    if sys.platform == "darwin":
        logger.info("SplatCamera: Intel macOS detected, using stub backend")
        return MacosBackend(splat, spec)

    try:
        import gsplat  # noqa: F401

        return GsplatBackend(splat, spec)
    except ImportError:
        logger.warning(
            "SplatCamera: gsplat not installed — falling back to stub backend.  "
            "Install dimos[splat] for real splat-rendered images on Linux+CUDA."
        )
        return MacosBackend(splat, spec)


# =============================================================================
# Asset loading without mujoco_playground (so MLX-Metal stays alive)
# =============================================================================
#
# ``dimos.simulation.mujoco.model.get_assets`` walks several directories
# and reads mesh bytes, but it does so via ``mjx_env.update_assets``.
# Importing ``mjx_env`` pulls in ``mujoco_playground._src.wrapper_torch``
# which imports NVIDIA Warp, which initializes Metal aggressively on
# macOS — corrupting the Metal compiler XPC connection that MLX needs
# to compile splat-camera kernels.  This local re-implementation does
# the same directory walking with stdlib only and locates
# ``mujoco_playground``'s bundled menagerie via ``importlib.util.find_spec``
# (which finds the package on disk without executing its ``__init__``).


def _update_assets_no_warp(assets: dict[str, bytes], root: FilePath, pattern: str = "*") -> None:
    if not root.exists():
        return
    for f in root.glob(pattern):
        if f.is_file():
            assets[f.name] = f.read_bytes()


def _menagerie_path() -> FilePath | None:
    import importlib.util

    spec = importlib.util.find_spec("mujoco_playground")
    if spec is None or not spec.submodule_search_locations:
        return None
    return (
        FilePath(next(iter(spec.submodule_search_locations))) / "external_deps" / "mujoco_menagerie"
    )


def _load_robot_assets(data_dir: FilePath, person_dir: FilePath) -> dict[str, bytes]:
    """No-warp replacement for ``dimos.simulation.mujoco.model.get_assets``.

    Mirrors the asset set that the original loads, but without dragging
    in ``mujoco_playground`` (and through it ``warp`` and ``torch``)
    which clobbers Metal for MLX.  See module-level note above.
    """
    assets: dict[str, bytes] = {}
    _update_assets_no_warp(assets, data_dir, "*.xml")
    _update_assets_no_warp(assets, data_dir, "*.obj")
    _update_assets_no_warp(assets, data_dir / "scene_office1" / "textures", "*.png")
    _update_assets_no_warp(assets, data_dir / "scene_office1" / "office_split", "*.obj")
    menagerie = _menagerie_path()
    if menagerie is not None:
        _update_assets_no_warp(assets, menagerie / "unitree_go1" / "assets")
        _update_assets_no_warp(assets, menagerie / "unitree_g1" / "assets")
    _update_assets_no_warp(assets, person_dir, "*.obj")
    _update_assets_no_warp(assets, person_dir, "*.png")
    return assets


# =============================================================================
# Module
# =============================================================================


_ARM_BODY_KEYWORDS = ("shoulder", "elbow", "wrist", "hand")
_HULL_MAX_POINTS = 64
# Unit-box corner/face tables for primitive entity overlays.
_BOX_CORNERS = np.array(
    [
        [-1, -1, -1],
        [1, -1, -1],
        [1, 1, -1],
        [-1, 1, -1],
        [-1, -1, 1],
        [1, -1, 1],
        [1, 1, 1],
        [-1, 1, 1],
    ],
    dtype=np.float64,
)
_BOX_FACES = np.array(
    [
        [0, 1, 2],
        [0, 2, 3],
        [4, 6, 5],
        [4, 7, 6],
        [0, 4, 5],
        [0, 5, 1],
        [1, 5, 6],
        [1, 6, 2],
        [2, 6, 7],
        [2, 7, 3],
        [3, 7, 4],
        [3, 4, 0],
    ],
    dtype=np.int32,
)


def _arm_hull_geoms(robot: RobotMeshes | None) -> list[GeomInstance]:
    """Convex-hulled copies of the robot's arm/hand geoms for compositing.

    The visual STLs total ~750k triangles across both arms — far too heavy
    for per-frame cv2 fills. Hulls are a few hundred faces per link, which
    is plenty for "where are my arms" feedback.
    """
    if robot is None:
        return []
    try:
        from scipy.spatial import ConvexHull  # type: ignore[import-untyped]
    except Exception:
        return []

    hulls: list[GeomInstance] = []
    for geom in robot.geoms:
        if not any(k in geom.body_name for k in _ARM_BODY_KEYWORDS):
            continue
        if len(geom.vertices) < 4:
            continue
        # Hull of the full vertex set tracks every surface curve — thousands
        # of faces per link, which murders the per-frame fill loop. Hull a
        # deterministic subsample instead: ≤64 points → ≤~124 faces, visually
        # identical at camera resolution.
        vertices = geom.vertices
        if len(vertices) > _HULL_MAX_POINTS:
            idx = np.unique(np.linspace(0, len(vertices) - 1, _HULL_MAX_POINTS).astype(np.int64))
            vertices = vertices[idx]
        try:
            hull = ConvexHull(vertices)
        except Exception:
            continue
        remap = np.full(len(vertices), -1, dtype=np.int32)
        remap[hull.vertices] = np.arange(len(hull.vertices), dtype=np.int32)
        hulls.append(
            GeomInstance(
                body_name=geom.body_name,
                vertices=vertices[hull.vertices],
                faces=remap[hull.simplices],
                local_pos=geom.local_pos,
                local_wxyz=geom.local_wxyz,
                rgba=geom.rgba,
            )
        )
    return hulls


def _primitive_mesh(shape: str, extents: tuple[float, ...]) -> tuple[np.ndarray, np.ndarray] | None:
    """(vertices, faces) for a primitive entity shape, origin-centered.

    Spheres/cylinders render as their bounding boxes — close enough for an
    overlay whose job is "the object is here".
    """
    if shape == "box" and len(extents) == 3:
        half = np.asarray(extents, dtype=np.float64) / 2.0
    elif shape == "sphere" and len(extents) == 1:
        half = np.full(3, float(extents[0]))
    elif shape == "cylinder" and len(extents) == 2:
        r, h = float(extents[0]), float(extents[1])
        half = np.array([r, r, h / 2.0])
    else:
        return None
    return _BOX_CORNERS * half, _BOX_FACES


class SplatCameraModule(Module):
    """Publishes splat-rendered camera images at the robot's camera pose.

    Subscribes to the same joint_state + odom topics the viser viewer
    uses, runs MuJoCo FK to find where the configured camera body sits,
    composes the camera mount into world coords, and asks the active
    backend for an image.

    Inputs:
        joint_state: per-joint q values from the coordinator.
        odom: base pose from the sim adapter.

    Outputs:
        color_image: rendered RGB at ``camera_spec`` resolution.
        camera_info: pinhole intrinsics matching the spec.
    """

    # Splat rendering is expensive enough that it should never share a Python
    # worker with lidar / mapping modules that need to stay responsive.
    dedicated_worker: ClassVar[bool] = True

    color_image: Out[Image]
    camera_info: Out[CameraInfo]
    joint_state: In[JointState]
    odom: In[PoseStamped]
    # Live entity poses (e.g. from MuJoCo in sim mode). Primitive-shaped
    # entities are composited onto the render; mesh entities are skipped —
    # scanned furniture is already in the splat itself.
    entity_states: In[EntityStateBatch]

    def __init__(
        self,
        splat_path: str | FilePath,
        mjcf_path: str | FilePath,
        *,
        alignment_yaml: str | FilePath | None = None,
        camera_spec: CameraSpec | None = None,
        render_hz: float = 10.0,
        info_hz: float = 1.0,
        frame_id: str = "splat_camera_optical_frame",
        composite_robot: bool = True,
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)
        self._splat_path = FilePath(splat_path)
        self._mjcf_path = FilePath(mjcf_path)
        self._alignment_yaml = FilePath(alignment_yaml) if alignment_yaml else None
        self._camera_spec = camera_spec if camera_spec is not None else g1_d435_default()
        self._render_dt = 1.0 / float(render_hz)
        self._info_dt = 1.0 / float(info_hz)
        self._frame_id = frame_id
        # The splat doesn't contain the robot, so in sim the operator can't
        # see their own arms in the camera. Composite convex hulls of the
        # arm/hand meshes at FK poses (full STLs are ~750k tris; hulls a
        # few thousand).
        self._composite_robot = composite_robot
        self._robot_overlay_geoms: list[GeomInstance] = []

        self._state_lock = threading.Lock()
        self._latest_joints: dict[str, float] = {}
        self._latest_base_pos: np.ndarray | None = None
        self._latest_base_wxyz: np.ndarray | None = None
        self._latest_entity_batch: EntityStateBatch | None = None

        self._robot: RobotMeshes | None = None
        self._backend: SplatCameraBackend | None = None
        self._cam_body_id: int | None = None
        self._cam_info_msg: CameraInfo | None = None
        self._render_thread: threading.Thread | None = None
        self._info_thread: threading.Thread | None = None
        self._stop_event = threading.Event()

    @rpc
    def start(self) -> None:
        super().start()

        alignment = (
            SplatAlignment.from_yaml(self._alignment_yaml)
            if self._alignment_yaml and self._alignment_yaml.exists()
            else SplatAlignment()
        )

        logger.info(f"SplatCamera: loading splat from {self._splat_path}")
        splat = load_splat(self._splat_path, alignment=alignment)
        logger.info(f"SplatCamera: loaded {len(splat.centers)} Gaussians")

        # IMPORTANT: do NOT use ``dimos.simulation.mujoco.model.get_assets``
        # here.  That import chain pulls ``mujoco_playground._src.wrapper_torch``
        # which loads NVIDIA Warp, which grabs Metal in a way that corrupts
        # the XPC compiler connection MLX needs to render splats.  Use the
        # local no-warp loader instead — it produces the same asset bytes.
        from dimos.utils.data import get_data

        data_dir = FilePath(str(get_data("mujoco_sim")))
        person_dir = FilePath(str(get_data("person")))
        self._robot = load_robot_meshes(
            self._mjcf_path, assets=_load_robot_assets(data_dir, person_dir)
        )

        cam_body_id = mujoco.mj_name2id(
            self._robot.model, mujoco.mjtObj.mjOBJ_BODY, self._camera_spec.body_name
        )
        if cam_body_id < 0:
            logger.error(
                f"SplatCamera: camera mount body '{self._camera_spec.body_name}' "
                f"not in MJCF; module will publish nothing"
            )
            return
        self._cam_body_id = cam_body_id

        self._backend = make_backend(splat, self._camera_spec)

        # Static intrinsics — built once, republished on a slow timer.
        spec = self._camera_spec
        self._cam_info_msg = CameraInfo(
            frame_id=self._frame_id,
            height=spec.height,
            width=spec.width,
            distortion_model="plumb_bob",
            D=[0.0, 0.0, 0.0, 0.0, 0.0],
            K=[
                spec.focal_pixels(),
                0.0,
                spec.cx(),
                0.0,
                spec.focal_pixels(),
                spec.cy(),
                0.0,
                0.0,
                1.0,
            ],
            R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            P=[
                spec.focal_pixels(),
                0.0,
                spec.cx(),
                0.0,
                0.0,
                spec.focal_pixels(),
                spec.cy(),
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
            ],
        )

        try:
            unsub = self.joint_state.subscribe(self._on_joint_state)
            self.register_disposable(Disposable(unsub))
        except Exception as e:
            logger.warning(f"SplatCamera: joint_state subscribe failed: {e}")

        try:
            unsub = self.odom.subscribe(self._on_odom)
            self.register_disposable(Disposable(unsub))
        except Exception as e:
            logger.warning(f"SplatCamera: odom subscribe failed: {e}")

        try:
            unsub = self.entity_states.subscribe(self._on_entity_states)
            self.register_disposable(Disposable(unsub))
        except Exception as e:
            logger.debug(f"SplatCamera: entity_states not wired ({e}); no entity overlay")

        if self._composite_robot:
            self._robot_overlay_geoms = _arm_hull_geoms(self._robot)
            logger.info(
                "SplatCamera: compositing %d robot overlay hulls",
                len(self._robot_overlay_geoms),
            )

        self._render_thread = threading.Thread(
            target=self._render_loop, name="splat-camera-render", daemon=True
        )
        self._render_thread.start()
        self._info_thread = threading.Thread(
            target=self._info_loop, name="splat-camera-info", daemon=True
        )
        self._info_thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        for t in (self._render_thread, self._info_thread):
            if t and t.is_alive():
                t.join(timeout=2.0)
        super().stop()

    # Painter-sorted compositing of overlay geometry onto the splat render:
    #  - MJCF scene rigging (manip_*, scene_editor_* bodies), legacy path
    #  - robot arm/hand convex hulls at FK poses (the splat doesn't
    #    contain the robot, so without this the operator can't see their
    #    own arms in sim)
    #  - primitive-shaped entities at live poses from entity_states
    #    (mesh entities are skipped — scanned furniture is visually part
    #    of the splat already)
    # Triangles are depth-sorted far-to-near across all sources, which
    # gives correct overlay self-occlusion. There is still no depth test
    # against the *splat* — overlays always win. Acceptable while overlay
    # objects sit in front of the camera with no splat geometry between
    # them and the lens; for anything else, composite with gsplat's depth
    # output instead.
    def _composite_scene_meshes(
        self,
        rgb: np.ndarray,
        cam_pos: np.ndarray,
        cam_wxyz: np.ndarray,
    ) -> np.ndarray:
        if self._robot is None:
            return rgb
        try:
            import cv2  # type: ignore[import-untyped]
        except Exception:
            return rgb

        # World→camera transform.  cam_wxyz / cam_pos are camera-in-world;
        # invert to get world→camera.
        cw, cx, cy, cz = cam_wxyz
        R_wc = _wxyz_to_rotmat_args(cw, cx, cy, cz)
        R_cw = R_wc.T  # world→camera rotation
        t_cw = -R_cw @ cam_pos

        spec = self._camera_spec
        fx = fy = spec.focal_pixels()
        cx_p, cy_p = spec.cx(), spec.cy()

        body_name_to_id = {n: i for i, n in enumerate(self._robot.body_names)}

        # Gather (depth, uv triangle, color) across all overlay sources.
        tri_depths: list[np.ndarray] = []
        tri_uvs: list[np.ndarray] = []
        tri_colors: list[tuple[int, int, int]] = []
        tri_counts: list[int] = []

        def project_geom(
            vertices: np.ndarray,
            faces: np.ndarray,
            world_pos: np.ndarray,
            world_wxyz: np.ndarray,
            local_pos: np.ndarray,
            local_wxyz: np.ndarray,
            rgba: tuple[float, ...],
        ) -> None:
            R_wb = _wxyz_to_rotmat_args(*world_wxyz)
            R_bg = _wxyz_to_rotmat_args(*local_wxyz)
            v_b = (R_bg @ vertices.astype(np.float64).T).T + local_pos
            v_w = (R_wb @ v_b.T).T + world_pos
            v_c = (R_cw @ v_w.T).T + t_cw  # (V, 3) camera frame

            in_front = v_c[:, 2] > 1e-3
            if not in_front.any():
                return
            zs = np.where(in_front, v_c[:, 2], 1.0)
            u = fx * (v_c[:, 0] / zs) + cx_p
            v = fy * (v_c[:, 1] / zs) + cy_p
            uv = np.stack([u, v], axis=1)

            face_ok = in_front[faces].all(axis=1)  # (F,)
            if not face_ok.any():
                return
            ok_faces = faces[face_ok]
            tri_depths.append(v_c[ok_faces][:, :, 2].mean(axis=1))
            tri_uvs.append(uv[ok_faces].astype(np.int32))
            tri_colors.append((int(rgba[0] * 255), int(rgba[1] * 255), int(rgba[2] * 255)))
            tri_counts.append(len(ok_faces))

        # 1. Legacy MJCF scene rigging + 2. robot arm hulls — FK-driven.
        scene_geoms = [
            g
            for g in self._robot.geoms
            if g.body_name.startswith("manip_") or g.body_name.startswith("scene_editor_")
        ]
        for geom in (*scene_geoms, *self._robot_overlay_geoms):
            body_id = body_name_to_id.get(geom.body_name)
            if body_id is None:
                continue
            project_geom(
                geom.vertices,
                geom.faces,
                self._robot.data.xpos[body_id],
                self._robot.data.xquat[body_id],
                geom.local_pos,
                geom.local_wxyz,
                geom.rgba,
            )

        # 3. Primitive entities at live poses.
        with self._state_lock:
            batch = self._latest_entity_batch
        if batch is not None:
            identity_pos = np.zeros(3)
            identity_wxyz = np.array([1.0, 0.0, 0.0, 0.0])
            for descriptor, pose in batch.entries:
                prim = _primitive_mesh(descriptor.shape_hint, descriptor.extents)
                if prim is None:
                    continue
                vertices, faces = prim
                world_pos = np.array([pose.position.x, pose.position.y, pose.position.z])
                world_wxyz = np.array(
                    [
                        pose.orientation.w,
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                    ]
                )
                rgba = descriptor.rgba if descriptor.rgba is not None else (0.62, 0.62, 0.68, 1.0)
                project_geom(
                    vertices, faces, world_pos, world_wxyz, identity_pos, identity_wxyz, rgba
                )

        if not tri_depths:
            return rgb

        depths = np.concatenate(tri_depths)
        order = np.argsort(-depths)  # far to near
        colors = np.repeat(np.arange(len(tri_colors)), np.asarray(tri_counts))  # geom index per tri
        uvs = np.concatenate(tri_uvs)

        out = rgb.copy()
        for idx in order:
            r, g, b = tri_colors[colors[idx]]
            cv2.fillConvexPoly(out, uvs[idx], (r, g, b))
        return out

    def _on_entity_states(self, batch: EntityStateBatch) -> None:
        with self._state_lock:
            self._latest_entity_batch = batch

    def _on_joint_state(self, msg: JointState) -> None:
        names = list(msg.name)
        positions = list(msg.position)
        if not names or len(names) != len(positions):
            return
        with self._state_lock:
            for n, q in zip(names, positions, strict=False):
                self._latest_joints[dimos_joint_to_mjcf(n)] = float(q)

    def _on_odom(self, msg: PoseStamped) -> None:
        with self._state_lock:
            self._latest_base_pos = np.array(
                [msg.position.x, msg.position.y, msg.position.z],
                dtype=np.float64,
            )
            # PoseStamped quat is xyzw; renderer + viser use wxyz.
            self._latest_base_wxyz = np.array(
                [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z],
                dtype=np.float64,
            )

    def _info_loop(self) -> None:
        while not self._stop_event.is_set():
            if self._cam_info_msg is not None:
                self._cam_info_msg.ts = time.time()
                try:
                    self.camera_info.publish(self._cam_info_msg)
                except Exception as e:
                    logger.debug(f"SplatCamera: camera_info publish failed: {e}")
            self._stop_event.wait(self._info_dt)

    def _render_loop(self) -> None:
        assert self._robot is not None
        if self._backend is None or self._cam_body_id is None:
            return
        next_tick = time.monotonic()
        while not self._stop_event.is_set():
            with self._state_lock:
                joints = dict(self._latest_joints)
                base_pos = None if self._latest_base_pos is None else self._latest_base_pos.copy()
                base_wxyz = (
                    None if self._latest_base_wxyz is None else self._latest_base_wxyz.copy()
                )

            try:
                apply_state(
                    self._robot,
                    base_pos=base_pos,
                    base_wxyz=base_wxyz,
                    joint_positions=joints,
                )
                body_pos = self._robot.data.xpos[self._cam_body_id]
                body_wxyz = self._robot.data.xquat[self._cam_body_id]
                cam_pos, cam_wxyz = world_pose(body_pos, body_wxyz, self._camera_spec)
                rgb = self._backend.render(cam_pos, cam_wxyz)
                rgb = self._composite_scene_meshes(rgb, cam_pos, cam_wxyz)
                ts = time.time()
                # Publish tf for the actual rendering pose so consumers
                # of the published image can do
                # ``tf.get(image.frame_id, world_frame, ts)`` and get
                # the *correct* world->optical transform — even when
                # ``camera_spec`` differs from the MJCF camera pose
                # MujocoSimModule's tf publish refers to (e.g. when
                # DIMOS_CAMERA_FORWARD=1 is set).
                self.tf.publish(
                    Transform(
                        translation=Vector3(
                            float(cam_pos[0]), float(cam_pos[1]), float(cam_pos[2])
                        ),
                        # ``world_pose`` returns wxyz; Quaternion stores xyzw.
                        rotation=Quaternion(
                            float(cam_wxyz[1]),
                            float(cam_wxyz[2]),
                            float(cam_wxyz[3]),
                            float(cam_wxyz[0]),
                        ),
                        frame_id="world",
                        child_frame_id=self._frame_id,
                        ts=ts,
                    )
                )
                self.color_image.publish(
                    Image(
                        ts=ts,
                        frame_id=self._frame_id,
                        format=ImageFormat.RGB,
                        data=rgb,
                    )
                )
            except Exception as e:
                logger.debug(f"SplatCamera render tick failed: {e}")

            next_tick += self._render_dt
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.monotonic()


class WorkspaceSplatCameraModule(SplatCameraModule):
    """Same behaviour as ``SplatCameraModule``; a distinct class lets the
    module coordinator keep two splat cameras in the same blueprint (it
    keys deployed modules by class, so two of the same class get silently
    collapsed into one). Used for the down-pitched workspace view."""


splat_camera = SplatCameraModule.blueprint

__all__ = [
    "GsplatBackend",
    "MacosBackend",
    "MlxBackend",
    "SplatCameraBackend",
    "SplatCameraModule",
    "WorkspaceSplatCameraModule",
    "make_backend",
    "splat_camera",
]
