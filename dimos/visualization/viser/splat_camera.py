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

from pathlib import Path as FilePath
import sys
import threading
import time
from typing import Any, Protocol

import mujoco
import numpy as np
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.sensor_msgs import CameraInfo, Image, JointState
from dimos.msgs.sensor_msgs.Image import ImageFormat
from dimos.utils.logging_config import setup_logger
from dimos.visualization.viser.camera import CameraSpec, g1_d435_default, world_pose
from dimos.visualization.viser.robot_meshes import (
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

    Linux + gsplat installed -> GsplatBackend.  Anything else -> Mac
    stub backend (which is fine on Linux too if you haven't installed
    the splat extra; you just won't get real images).
    """
    if sys.platform == "darwin":
        logger.info("SplatCamera: macOS detected, using stub backend")
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
# Module
# =============================================================================


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

    color_image: Out[Image]
    camera_info: Out[CameraInfo]
    joint_state: In[JointState]
    odom: In[PoseStamped]

    def __init__(
        self,
        splat_path: str | FilePath,
        mjcf_path: str | FilePath,
        *,
        alignment_yaml: str | FilePath | None = None,
        camera_spec: CameraSpec | None = None,
        render_hz: float = 10.0,
        info_hz: float = 1.0,
        frame_id: str = "camera_optical",
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

        self._state_lock = threading.Lock()
        self._latest_joints: dict[str, float] = {}
        self._latest_base_pos: np.ndarray | None = None
        self._latest_base_wxyz: np.ndarray | None = None

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

        from dimos.simulation.mujoco.model import get_assets

        self._robot = load_robot_meshes(self._mjcf_path, assets=get_assets())

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
            self._disposables.add(Disposable(unsub))
        except Exception as e:
            logger.warning(f"SplatCamera: joint_state subscribe failed: {e}")

        try:
            unsub = self.odom.subscribe(self._on_odom)
            self._disposables.add(Disposable(unsub))
        except Exception as e:
            logger.warning(f"SplatCamera: odom subscribe failed: {e}")

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
                self.color_image.publish(
                    Image(
                        ts=time.time(),
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


splat_camera = SplatCameraModule.blueprint

__all__ = [
    "GsplatBackend",
    "MacosBackend",
    "SplatCameraBackend",
    "SplatCameraModule",
    "make_backend",
    "splat_camera",
]
