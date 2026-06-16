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

"""Mesh-backed simulated RGBD camera.

Ray-casts a scene mesh from the robot's head-camera pose to produce a real
camera feed — **color and depth** — so downstream perception
(``ObjectTracking`` / ``TemporalMemory`` / object registration) sees the loaded
environment instead of a black placeholder. The depth is published in the same
``ImageFormat.DEPTH`` (float metres, registered to the color frame) wire format
``MujocoSimModule`` uses, so a perception module is identical whether MuJoCo or
this mesh camera rendered the frame.

Color quality is barycentric vertex-color interpolation (fine for geometry/
spatial perception; a textured-render path is a future upgrade for detectors).

Pipeline per tick:
  1. FK the robot off ``/coordinator/joint_state`` + ``/odom`` to find
     the camera body (same code path ``SplatCameraModule`` uses).
  2. Generate one pinhole ray per pixel in world frame.
  3. ``open3d.t.geometry.RaycastingScene.cast_rays`` returns triangle
     IDs + barycentric uvs at hit points.
  4. Per-vertex colors lerp'd by barycentric → ``(H, W, 3)`` uint8.
  5. Publish as ``sensor_msgs.Image`` (and ``camera_info`` on a slow
     timer, mirroring ``SplatCameraModule`` so the wire format is
     identical to the gsplat path).

Pure CPU (Open3D's BVH).  At 320x180 = 57600 rays it averages well
under 20 ms per frame on M-series, so the default 10 Hz target is
comfortable.
"""

from __future__ import annotations

from pathlib import Path as FilePath
import threading
import time
from typing import Any

import mujoco
import numpy as np
from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.simulation.scene_assets.mesh_scene import (
    SceneMeshAlignment,
    load_scene_mesh,
    make_raycasting_scene,
)
from dimos.utils.logging_config import setup_logger
from dimos.visualization.viser.camera import CameraSpec, g1_d435_default, world_pose
from dimos.visualization.viser.robot_meshes import (
    apply_state,
    dimos_joint_to_mjcf,
    load_robot_meshes,
)

logger = setup_logger()


class MeshCameraConfig(ModuleConfig):
    """Configuration for ``MeshCameraModule``."""

    scene_path: str = ""
    """Path to a ``.usdz`` / ``.glb`` / ``.obj`` / ``.ply`` / ``.stl``
    scene mesh.  Empty disables the publisher."""

    mjcf_path: str = ""
    """MJCF used for camera-body forward kinematics.  Should match the
    one ``MujocoSimModule`` simulates against."""

    scene_scale: float = 1.0
    scene_translation: tuple[float, float, float] = (0.0, 0.0, 0.0)
    scene_rotation_zyx_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)
    scene_y_up: bool = True

    render_hz: float = 10.0
    info_hz: float = 1.0
    frame_id: str = "camera_optical"

    sky_color: tuple[int, int, int] = (135, 175, 215)
    """RGB 0..255 for ray-misses (i.e. pixels whose rays don't hit any
    triangle).  Soft sky-blue by default."""


class MeshCameraModule(Module):
    """Ray-cast a scene mesh per-pixel to publish a real camera feed."""

    config: MeshCameraConfig = Field(default_factory=MeshCameraConfig)

    color_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraInfo]
    depth_camera_info: Out[CameraInfo]
    joint_state: In[JointState]
    odom: In[PoseStamped]

    def __init__(
        self,
        *,
        camera_spec: CameraSpec | None = None,
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)
        self._camera_spec = camera_spec if camera_spec is not None else g1_d435_default()

        self._state_lock = threading.Lock()
        self._latest_joints: dict[str, float] = {}
        self._latest_base_pos: np.ndarray | None = None
        self._latest_base_wxyz: np.ndarray | None = None

        self._scene = None  # open3d.t.geometry.RaycastingScene
        self._triangles: np.ndarray | None = None  # (N_tri, 3) int32
        self._vertex_colors: np.ndarray | None = None  # (N_vtx, 3) float32 in [0, 1]
        self._cam_body_id: int | None = None
        self._cam_info_msg: CameraInfo | None = None
        self._render_thread: threading.Thread | None = None
        self._info_thread: threading.Thread | None = None
        self._stop_event = threading.Event()

    @rpc
    def start(self) -> None:
        super().start()

        cfg = self.config
        if not cfg.scene_path or not cfg.mjcf_path:
            logger.info("MeshCameraModule: scene_path or mjcf_path empty, publisher disabled")
            return

        # Load the scene mesh + build a BVH for ray-casting; same loader
        # path as the viser module, so per-vertex colors come from USD
        # ``displayColor`` / material ``diffuseColor`` extraction.
        path = FilePath(cfg.scene_path).expanduser()
        alignment = SceneMeshAlignment(
            scale=cfg.scene_scale,
            rotation_zyx_deg=cfg.scene_rotation_zyx_deg,
            translation=cfg.scene_translation,
            y_up=cfg.scene_y_up,
        )
        logger.info(f"MeshCameraModule: loading scene mesh {path}")
        mesh = load_scene_mesh(path, alignment=alignment)
        self._scene = make_raycasting_scene(mesh)
        self._triangles = np.asarray(mesh.triangles, dtype=np.int32)
        if mesh.has_vertex_colors():
            self._vertex_colors = np.asarray(mesh.vertex_colors, dtype=np.float32)
        else:
            n_vtx = len(mesh.vertices)
            self._vertex_colors = np.full((n_vtx, 3), 0.7, dtype=np.float32)

        # Robot meshes — for FK to extract camera body pose each tick.
        # Inline asset walk: avoids importing splat_camera (and through
        # it the heavy gsplat / splat-apple machinery) just to reuse one
        # helper, and avoids dimos.simulation.mujoco.model.get_assets
        # which pulls mujoco_playground → warp → torch.
        from dimos.utils.data import get_data

        data_dir = FilePath(str(get_data("mujoco_sim")))
        person_dir = FilePath(str(get_data("person")))
        assets: dict[str, bytes] = {}
        for root, pattern in [
            (data_dir, "*.xml"),
            (data_dir, "*.obj"),
            (data_dir / "scene_office1" / "textures", "*.png"),
            (data_dir / "scene_office1" / "office_split", "*.obj"),
            (person_dir, "*.obj"),
            (person_dir, "*.png"),
        ]:
            if root.exists():
                for f in root.glob(pattern):
                    if f.is_file():
                        assets[f.name] = f.read_bytes()
        # Bundled menagerie (G1 / Go1 meshes) via importlib so we don't
        # exec ``mujoco_playground.__init__``.
        import importlib.util

        spec = importlib.util.find_spec("mujoco_playground")
        if spec is not None and spec.submodule_search_locations:
            menagerie = (
                FilePath(next(iter(spec.submodule_search_locations)))
                / "external_deps"
                / "mujoco_menagerie"
            )
            for sub in ("unitree_go1/assets", "unitree_g1/assets"):
                root = menagerie / sub
                if root.exists():
                    for f in root.iterdir():
                        if f.is_file():
                            assets[f.name] = f.read_bytes()
        self._robot = load_robot_meshes(FilePath(cfg.mjcf_path), assets=assets)

        cam_body_id = mujoco.mj_name2id(
            self._robot.model, mujoco.mjtObj.mjOBJ_BODY, self._camera_spec.body_name
        )
        if cam_body_id < 0:
            logger.error(
                f"MeshCameraModule: camera mount body '{self._camera_spec.body_name}' "
                f"not in MJCF; module will publish nothing"
            )
            return
        self._cam_body_id = cam_body_id

        self._cam_info_msg = self._build_camera_info()
        self._ray_dirs_cam = self._build_ray_directions_cam_frame()

        try:
            unsub = self.joint_state.subscribe(self._on_joint_state)
            self.register_disposable(Disposable(unsub))
        except Exception as e:
            logger.warning(f"MeshCameraModule: joint_state subscribe failed: {e}")

        try:
            unsub = self.odom.subscribe(self._on_odom)
            self.register_disposable(Disposable(unsub))
        except Exception as e:
            logger.warning(f"MeshCameraModule: odom subscribe failed: {e}")

        self._render_thread = threading.Thread(
            target=self._render_loop, name="mesh-camera-render", daemon=True
        )
        self._render_thread.start()
        self._info_thread = threading.Thread(
            target=self._info_loop, name="mesh-camera-info", daemon=True
        )
        self._info_thread.start()

        spec = self._camera_spec
        logger.info(
            f"MeshCameraModule publishing /splat/color_image at {cfg.render_hz} Hz "
            f"({spec.width}x{spec.height} @ vfov={spec.vfov_deg}°)"
        )

    @rpc
    def stop(self) -> None:
        if hasattr(self, "_stop_event"):
            self._stop_event.set()
            for t in (self._render_thread, self._info_thread):
                if t and t.is_alive():
                    t.join(timeout=2.0)
        super().stop()

    # ----- helpers -----

    def _build_camera_info(self) -> CameraInfo:
        spec = self._camera_spec
        f = spec.focal_pixels()
        cx, cy = spec.cx(), spec.cy()
        return CameraInfo(
            frame_id=self.config.frame_id,
            height=spec.height,
            width=spec.width,
            distortion_model="plumb_bob",
            D=[0.0, 0.0, 0.0, 0.0, 0.0],
            K=[f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0],
            R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            P=[f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
        )

    def _build_ray_directions_cam_frame(self) -> np.ndarray:
        """Per-pixel ray directions in *camera* frame (+x right, +y down, +z fwd).

        Returns a contiguous ``(H*W, 3)`` float32 array, normalized.
        """
        spec = self._camera_spec
        f = spec.focal_pixels()
        cx, cy = spec.cx(), spec.cy()
        ys, xs = np.meshgrid(
            np.arange(spec.height, dtype=np.float32),
            np.arange(spec.width, dtype=np.float32),
            indexing="ij",
        )
        x = (xs + 0.5 - cx) / f
        y = (ys + 0.5 - cy) / f
        z = np.ones_like(x, dtype=np.float32)
        d = np.stack([x, y, z], axis=-1).reshape(-1, 3)
        d /= np.linalg.norm(d, axis=1, keepdims=True)
        return d.astype(np.float32)

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
                [msg.position.x, msg.position.y, msg.position.z], dtype=np.float64
            )
            self._latest_base_wxyz = np.array(
                [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z],
                dtype=np.float64,
            )

    def _info_loop(self) -> None:
        period = 1.0 / self.config.info_hz if self.config.info_hz > 0 else 1.0
        while not self._stop_event.is_set():
            if self._cam_info_msg is not None:
                self._cam_info_msg.ts = time.time()
                try:
                    # Depth is registered to the color frame, so both infos are
                    # the same intrinsics — mirrors MujocoSimModule's RGBD pair.
                    self.camera_info.publish(self._cam_info_msg)
                    self.depth_camera_info.publish(self._cam_info_msg)
                except Exception as e:
                    logger.debug(f"MeshCameraModule: camera_info publish failed: {e}")
            self._stop_event.wait(period)

    def _render_loop(self) -> None:
        import open3d.core as o3c

        spec = self._camera_spec
        period = 1.0 / self.config.render_hz if self.config.render_hz > 0 else 0.1
        sky = np.array(self.config.sky_color, dtype=np.uint8)

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
                    self._robot, base_pos=base_pos, base_wxyz=base_wxyz, joint_positions=joints
                )
                body_pos = self._robot.data.xpos[self._cam_body_id]
                body_wxyz = self._robot.data.xquat[self._cam_body_id]
                cam_pos, cam_wxyz = world_pose(body_pos, body_wxyz, spec)

                # Camera-to-world rotation from wxyz quaternion.
                R = np.zeros(9, dtype=np.float64)
                mujoco.mju_quat2Mat(R, np.asarray(cam_wxyz, dtype=np.float64))
                R_world_cam = R.reshape(3, 3).astype(np.float32)

                dirs_world = (R_world_cam @ self._ray_dirs_cam.T).T  # (H*W, 3)
                origins = np.broadcast_to(np.asarray(cam_pos, dtype=np.float32), dirs_world.shape)
                rays = np.concatenate([origins, dirs_world], axis=-1)
                hit = self._scene.cast_rays(o3c.Tensor(rays, dtype=o3c.Dtype.Float32))

                t_hit = hit["t_hit"].numpy()
                tri_id = hit["primitive_ids"].numpy().astype(np.int64)
                uv = hit["primitive_uvs"].numpy()  # (N, 2) barycentric

                hit_mask = np.isfinite(t_hit)
                # Index safely: clamp triangle id for missed rays so
                # gathering doesn't OOB; we overwrite those pixels with
                # ``sky`` afterwards.
                tri_id_safe = np.where(hit_mask, tri_id, 0)
                tris = self._triangles[tri_id_safe]  # (N, 3)
                c0 = self._vertex_colors[tris[:, 0]]
                c1 = self._vertex_colors[tris[:, 1]]
                c2 = self._vertex_colors[tris[:, 2]]
                u = uv[:, 0:1].astype(np.float32)
                v = uv[:, 1:2].astype(np.float32)
                w = 1.0 - u - v
                colors_f = w * c0 + u * c1 + v * c2  # (N, 3) in [0, 1]
                colors_u8 = np.clip(colors_f * 255.0, 0.0, 255.0).astype(np.uint8)
                colors_u8[~hit_mask] = sky

                rgb = colors_u8.reshape(spec.height, spec.width, 3)
                self.color_image.publish(
                    Image(
                        ts=time.time(),
                        frame_id=self.config.frame_id,
                        format=ImageFormat.RGB,
                        data=rgb,
                    )
                )

                # Depth: project the ray hit distance onto the optical axis
                # (Z = t_hit · dir_z, dir normalized in camera frame), float
                # metres with misses = 0 — the same DEPTH wire format
                # MujocoSimModule publishes, so perception treats sim cameras
                # identically whichever backend rendered them.
                depth_m = (t_hit * self._ray_dirs_cam[:, 2]).astype(np.float32)
                depth_m[~hit_mask] = 0.0
                self.depth_image.publish(
                    Image(
                        ts=time.time(),
                        frame_id=self.config.frame_id,
                        format=ImageFormat.DEPTH,
                        data=depth_m.reshape(spec.height, spec.width),
                    )
                )
            except Exception as e:
                logger.debug(f"MeshCameraModule render tick failed: {e}")

            next_tick += period
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.monotonic()


mesh_camera = MeshCameraModule.blueprint

__all__ = ["MeshCameraConfig", "MeshCameraModule", "mesh_camera"]
