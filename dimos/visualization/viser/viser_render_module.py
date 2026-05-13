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

"""Viser-based 3D viewer module for dimos.

Streams a Gaussian splat scene + the robot (MJCF meshes, FK from
``/coordinator/joint_state`` + ``/odom``) into a browser at
http://localhost:<port>/.

This is render-only — the viewer subscribes to existing LCM topics and
does not feed back into the control path.  Teleop continues to come
from the existing command-center dashboard.
"""

from __future__ import annotations

from pathlib import Path as FilePath
import threading
import time
from typing import Any

import mujoco
import numpy as np
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Path import Path as PathMsg
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger
from dimos.visualization.viser.camera import CameraSpec, world_pose
from dimos.visualization.viser.robot_meshes import (
    RobotMeshes,
    apply_state,
    dimos_joint_to_mjcf,
    load_robot_meshes,
)
from dimos.visualization.viser.splat import SplatAlignment, load_splat

logger = setup_logger()


def _compose_scene_mesh_wxyz(
    *, y_up: bool, rotation_zyx_deg: tuple[float, float, float]
) -> tuple[float, float, float, float]:
    """Build the viser wxyz quaternion that applies (y_up swap then zyx euler) —
    same convention as SceneMeshAlignment, just expressed as a parent-frame
    transform so we don't have to bake it into vertices."""
    R = np.eye(3, dtype=np.float64)
    if y_up:
        R = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], dtype=np.float64)
    rz, ry, rx = (np.deg2rad(a) for a in rotation_zyx_deg)
    cz, sz = np.cos(rz), np.sin(rz)
    cy, sy = np.cos(ry), np.sin(ry)
    cx, sx = np.cos(rx), np.sin(rx)
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=np.float64)
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=np.float64)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=np.float64)
    R = Rz @ Ry @ Rx @ R
    import mujoco

    out = np.zeros(4, dtype=np.float64)
    mujoco.mju_mat2Quat(out, R.flatten())
    return (float(out[0]), float(out[1]), float(out[2]), float(out[3]))


class ViserRenderModule(Module):
    """Viser viewer that overlays the live robot on a Gaussian splat.

    Inputs:
        joint_state: per-joint q values from the coordinator.
        odom: base pose from the sim (or future real-hw) adapter.
    """

    joint_state: In[JointState]
    odom: In[PoseStamped]
    path: In[PathMsg]
    # Optional pointcloud overlay.  Named distinctly (not `lidar`) so the
    # global transport map doesn't collide with VoxelGridMapper's `lidar`
    # In port — the coordinator keys transports by (port_name, type) and
    # the last-registered module wins, so a name clash silently overrides
    # whichever transport was registered first.  Blueprints typically
    # wire this to /global_map (accumulated voxel cloud) for a persistent
    # obstacle-memory overlay; /lidar (per-scan, transient) also works
    # if the latest sweep is what you want.
    pointcloud_overlay: In[PointCloud2]
    clicked_point: Out[PointStamped]
    # Interactive pointing-target publisher. Wired up by the "Set point
    # goal" button: armed by the user pressing the button, the next
    # scene click ray-casts against the scene mesh (if loaded) or
    # intersects the eye-height plane, and the resulting world-frame
    # point is published here. G1ManipulationModule subscribes and runs
    # point_at on each value. Separate stream from clicked_point so a
    # human's pointing click doesn't accidentally retarget the navigator.
    point_goal: Out[PointStamped]

    def __init__(
        self,
        splat_path: str | FilePath | None,
        mjcf_path: str | FilePath,
        *,
        port: int = 8082,
        alignment_yaml: str | FilePath | None = None,
        render_hz: float = 30.0,
        camera_spec: CameraSpec | None = None,
        mjcf_meshdir: str | FilePath | None = None,
        scene_mesh_path: str | FilePath | None = None,
        scene_mesh_scale: float = 1.0,
        scene_mesh_translation: tuple[float, float, float] = (0.0, 0.0, 0.0),
        scene_mesh_rotation_zyx_deg: tuple[float, float, float] = (0.0, 0.0, 0.0),
        scene_mesh_y_up: bool = True,
        pointcloud_point_size: float = 0.04,
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)
        # Empty / None splat_path means "no splat in the viewer" — useful when
        # the world is provided as a mesh instead (DIMOS_SCENE_MESH_PATH).
        self._splat_path = FilePath(splat_path) if splat_path else None
        self._mjcf_path = FilePath(mjcf_path)
        self._mjcf_meshdir = FilePath(mjcf_meshdir) if mjcf_meshdir else None
        self._alignment_yaml = FilePath(alignment_yaml) if alignment_yaml else None
        self._port = port
        self._render_dt = 1.0 / float(render_hz)
        self._camera_spec = camera_spec
        self._scene_mesh_path = FilePath(scene_mesh_path) if scene_mesh_path else None
        self._scene_mesh_scale = scene_mesh_scale
        self._scene_mesh_translation = scene_mesh_translation
        self._scene_mesh_rotation_zyx_deg = scene_mesh_rotation_zyx_deg
        self._scene_mesh_y_up = scene_mesh_y_up
        self._pointcloud_point_size = pointcloud_point_size

        # viser handles for view-mode toggle
        self._splat_handle: Any = None
        self._scene_mesh_handle: Any = None
        # Open3D RaycastingScene built from the scene-mesh geometry. Used
        # by the "Set point goal" button to resolve a 2D click into an
        # exact 3D surface hit. None when no scene mesh is configured.
        self._raycast_scene: Any = None

        # Mutable shared state — written from In subscribers, read from
        # the render loop.  Plain dict + lock; values are lightweight.
        self._state_lock = threading.Lock()
        self._latest_joints: dict[str, float] = {}
        self._latest_base_pos: np.ndarray | None = None
        self._latest_base_wxyz: np.ndarray | None = None

        self._server: Any = None  # viser.ViserServer
        self._body_frames: dict[int, Any] = {}  # body_id -> viser frame handle
        self._camera_body_id: int | None = None
        self._camera_frustum: Any = None  # viser frustum handle
        self._robot: RobotMeshes | None = None
        self._render_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._path_handle: Any = None
        # Layer-visibility state.  Three independent toggles in the
        # viser GUI panel — Splat / Mesh / Lidar — each gating one
        # backdrop so any subset can be shown.  The current and
        # previous designs (a Splat-vs-Mesh dropdown plus separate
        # checkboxes) were redundant because they overlapped on mesh
        # visibility and forced exclusivity the user didn't want.
        # GaussianSplatHandle in viser 1.0.26 advertises `.visible` but
        # the splat shader pipeline (still labeled "work-in-progress"
        # in the docstring) ignores it — flipping the property does
        # nothing in the browser.  Keep a copy of the loaded splat so
        # the toggle can re-add the handle on demand instead.
        self._splat_visible: bool = True
        self._splat_checkbox: Any = None
        self._splat_data: Any = None  # cached load_splat() result
        self._scene_mesh_visible: bool = True
        self._scene_mesh_checkbox: Any = None
        # Lidar overlay handle is replaced (not appended) on every
        # incoming /global_map message so we don't accumulate
        # cloud-on-cloud.  `_lidar_visible` gates the upload itself
        # so toggling off costs nothing.
        self._lidar_handle: Any = None
        self._lidar_visible: bool = True
        self._lidar_checkbox: Any = None

    @rpc
    def start(self) -> None:
        super().start()

        import viser

        alignment = (
            SplatAlignment.from_yaml(self._alignment_yaml)
            if self._alignment_yaml and self._alignment_yaml.exists()
            else SplatAlignment()
        )

        if self._splat_path is not None:
            logger.info(f"Viser: loading splat from {self._splat_path}")
            splat = load_splat(self._splat_path, alignment=alignment)
            logger.info(f"Viser: loaded {len(splat.centers)} Gaussians")
        else:
            splat = None
            logger.info("Viser: splat disabled (no splat_path provided)")

        logger.info(f"Viser: loading robot meshes from {self._mjcf_path}")
        self._robot = load_robot_meshes(self._mjcf_path, meshdir=self._mjcf_meshdir)
        logger.info(
            f"Viser: {len(self._robot.geoms)} visual meshes across "
            f"{len(self._robot.body_names)} bodies"
        )

        self._server = viser.ViserServer(host="0.0.0.0", port=self._port)
        # Strip the floating control panel down to just a collapse button —
        # the viewer is render-only, no GUI controls live in the panel, and
        # viser exposes no API to hide the panel entirely.
        self._server.gui.set_panel_label(None)
        self._server.gui.configure_theme(
            control_layout="collapsible",
            show_logo=False,
            show_share_button=False,
            dark_mode=True,
        )
        logger.info(f"Viser viewer: http://localhost:{self._port}/")

        if splat is not None:
            self._splat_data = splat
            self._splat_handle = self._server.scene.add_gaussian_splats(
                "/splat",
                centers=splat.centers,
                covariances=splat.covariances,
                rgbs=splat.rgbs,
                opacities=splat.opacities,
            )

        # Optional scene mesh (.usdz / .glb / etc.) — drawn in the same
        # world frame as the robot.  ``MeshCameraModule`` ray-casts the
        # same mesh to feed the head-camera RGB topic.
        if self._scene_mesh_path is not None and self._scene_mesh_path.exists():
            try:
                self._add_scene_mesh()
            except Exception as e:
                logger.warning(f"Viser: scene mesh load failed: {e}")

        # Three independent layer toggles: Splat / Mesh / Lidar.
        # Each checkbox only appears when the corresponding backdrop
        # actually exists in this run (no point in a "Show splat" toggle
        # when no splat was loaded).  Combined, they cover every subset
        # — splat-only, mesh-only, lidar-only, splat+mesh, splat+lidar,
        # mesh+lidar, all three.
        if self._splat_handle is not None:
            self._splat_checkbox = self._server.gui.add_checkbox(
                "Show splat", initial_value=self._splat_visible
            )

            @self._splat_checkbox.on_update
            def _on_splat_toggle(_: Any) -> None:
                visible = bool(self._splat_checkbox.value)
                self._splat_visible = visible
                # `.visible = False` is silently ignored on
                # GaussianSplatHandle in viser 1.0.26, so add/remove
                # the handle outright.  Re-add costs ~one frame from
                # the cached splat data.
                if visible:
                    if self._splat_handle is None and self._splat_data is not None:
                        d = self._splat_data
                        self._splat_handle = self._server.scene.add_gaussian_splats(
                            "/splat",
                            centers=d.centers,
                            covariances=d.covariances,
                            rgbs=d.rgbs,
                            opacities=d.opacities,
                        )
                else:
                    if self._splat_handle is not None:
                        try:
                            self._splat_handle.remove()
                        except Exception as e:
                            logger.debug(f"Viser splat remove failed: {e}")
                        self._splat_handle = None

        if self._scene_mesh_handle is not None:
            self._scene_mesh_checkbox = self._server.gui.add_checkbox(
                "Show mesh", initial_value=self._scene_mesh_visible
            )

            @self._scene_mesh_checkbox.on_update
            def _on_scene_mesh_toggle(_: Any) -> None:
                self._scene_mesh_visible = bool(self._scene_mesh_checkbox.value)
                if self._scene_mesh_handle is not None:
                    self._scene_mesh_handle.visible = self._scene_mesh_visible

        # Lidar map overlay toggle is unconditional — when no publisher is
        # connected the cloud stays empty, but having the checkbox
        # always present makes the overlay discoverable.
        self._lidar_checkbox = self._server.gui.add_checkbox(
            "Show lidar map", initial_value=self._lidar_visible
        )

        @self._lidar_checkbox.on_update
        def _on_lidar_toggle(_: Any) -> None:
            self._lidar_visible = bool(self._lidar_checkbox.value)
            if self._lidar_handle is not None:
                self._lidar_handle.visible = self._lidar_visible

        # One frame per body; meshes are added as children so they
        # follow when the body frame moves.
        for body_id, body_name in enumerate(self._robot.body_names):
            self._body_frames[body_id] = self._server.scene.add_frame(
                f"/robot/{body_name}",
                show_axes=False,
            )
        for i, geom in enumerate(self._robot.geoms):
            color_rgb = (
                int(geom.rgba[0] * 255),
                int(geom.rgba[1] * 255),
                int(geom.rgba[2] * 255),
            )
            self._server.scene.add_mesh_simple(
                f"/robot/{geom.body_name}/geom_{i}",
                vertices=geom.vertices,
                faces=geom.faces,
                color=color_rgb,
                opacity=float(geom.rgba[3]) if geom.rgba[3] > 0 else 1.0,
                position=tuple(geom.local_pos),
                wxyz=tuple(geom.local_wxyz),
            )

        # Camera frustum overlay — shows where a robot-mounted RGB sensor
        # would look from.  Stays None if the configured mount body
        # isn't in this MJCF (e.g. swap to a robot without head_link).
        if self._camera_spec is not None:
            cam_body_id = mujoco.mj_name2id(
                self._robot.model, mujoco.mjtObj.mjOBJ_BODY, self._camera_spec.body_name
            )
            if cam_body_id < 0:
                logger.warning(
                    f"Viser: camera mount body '{self._camera_spec.body_name}' not in MJCF; "
                    "frustum overlay disabled"
                )
            else:
                self._camera_body_id = cam_body_id
                self._camera_frustum = self._server.scene.add_camera_frustum(
                    "/robot/_camera_frustum",
                    fov=float(np.radians(self._camera_spec.vfov_deg)),
                    aspect=float(self._camera_spec.aspect),
                    scale=float(self._camera_spec.frustum_scale),
                    color=self._camera_spec.frustum_color,
                )

        # Click-to-navigate. We arm a one-shot scene click callback when
        # the user presses "Set nav goal", because viser disables camera
        # orbit while the click callback is registered (App.tsx:514) — so
        # leaving it always-on would break LMB orbit globally.
        nav_goal_button = self._server.gui.add_button("Set nav goal")

        @nav_goal_button.on_click
        def _arm_nav_goal_click(_event: Any) -> None:
            nav_goal_button.disabled = True
            nav_goal_button.label = "Click on floor..."

            @self._server.scene.on_pointer_event(event_type="click")
            def _on_floor_click(event: Any) -> None:
                try:
                    self._handle_floor_click(event)
                finally:
                    self._server.scene.remove_pointer_callback()

            @self._server.scene.on_pointer_callback_removed
            def _rearm_button() -> None:
                nav_goal_button.disabled = False
                nav_goal_button.label = "Set nav goal"

        # Click-to-point. Same one-shot-callback pattern; the click ray
        # is intersected with the scene-mesh raycaster if one is loaded,
        # else with an eye-height (z=1.0 m) horizontal plane so pointing
        # works even when no scene mesh is configured. Published on
        # /point_goal; G1ManipulationModule subscribes and runs point_at.
        point_goal_button = self._server.gui.add_button("Set point goal")

        @point_goal_button.on_click
        def _arm_point_goal_click(_event: Any) -> None:
            point_goal_button.disabled = True
            point_goal_button.label = "Click target..."

            @self._server.scene.on_pointer_event(event_type="click")
            def _on_point_click(event: Any) -> None:
                try:
                    self._handle_point_goal_click(event)
                finally:
                    self._server.scene.remove_pointer_callback()

            @self._server.scene.on_pointer_callback_removed
            def _rearm_point_button() -> None:
                point_goal_button.disabled = False
                point_goal_button.label = "Set point goal"

        try:
            unsub = self.path.subscribe(self._on_path)
            self.register_disposable(Disposable(unsub))
        except Exception as e:
            logger.warning(f"Viser: path subscribe failed: {e}")

        try:
            unsub = self.pointcloud_overlay.subscribe(self._on_lidar)
            self.register_disposable(Disposable(unsub))
        except Exception as e:
            logger.warning(f"Viser: lidar subscribe failed: {e}")

        try:
            unsub = self.joint_state.subscribe(self._on_joint_state)
            self.register_disposable(Disposable(unsub))
        except Exception as e:
            logger.warning(f"Viser: joint_state subscribe failed: {e}")

        try:
            unsub = self.odom.subscribe(self._on_odom)
            self.register_disposable(Disposable(unsub))
        except Exception as e:
            logger.warning(f"Viser: odom subscribe failed: {e}")

        self._render_thread = threading.Thread(
            target=self._render_loop, name="viser-render", daemon=True
        )
        self._render_thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._render_thread and self._render_thread.is_alive():
            self._render_thread.join(timeout=2.0)
        if self._server is not None:
            try:
                self._server.stop()
            except Exception:
                pass
        super().stop()

    def _add_scene_mesh(self) -> None:
        """Add the configured scene mesh to viser.

        For ``.glb``/``.gltf`` we hand the file's bytes straight to
        ``server.scene.add_glb()`` — the browser renders the PBR materials
        natively. This is critical: going through ``load_scene_mesh()`` calls
        ``trimesh.load(force="mesh")`` which decompresses every embedded
        texture to sample per-vertex colors, allocating ~10 GB peak for a
        scene with many 4K PBR textures (e.g. the office mesh has 321 textures
        totaling ~895 MP). For unknown extensions (USD, OBJ, etc.) we fall back
        to the geometry path — those generally don't have the embedded-texture
        problem.
        """
        assert self._scene_mesh_path is not None
        path = self._scene_mesh_path
        suffix = path.suffix.lower()

        # Alignment for viser: handed to add_glb as scale + wxyz + position so
        # we don't touch the geometry at all on this path.
        wxyz = _compose_scene_mesh_wxyz(
            y_up=self._scene_mesh_y_up,
            rotation_zyx_deg=self._scene_mesh_rotation_zyx_deg,
        )
        position = tuple(float(x) for x in self._scene_mesh_translation)
        scale = float(self._scene_mesh_scale)

        # Always build a server-side raycaster from the geometry so the
        # "Set point goal" button can resolve clicks to surface hits.
        # Cheap now that GLBs are geometry-only (textures stripped at
        # asset-prep time) — load_scene_mesh peaks at ~1 GB even for the
        # 1.4M-vert office mesh.
        from dimos.mapping.mesh_scene import (
            SceneMeshAlignment,
            load_scene_mesh,
            make_raycasting_scene,
        )

        mesh_alignment = SceneMeshAlignment(
            scale=self._scene_mesh_scale,
            rotation_zyx_deg=self._scene_mesh_rotation_zyx_deg,
            translation=self._scene_mesh_translation,
            y_up=self._scene_mesh_y_up,
        )

        if suffix in {".glb", ".gltf"}:
            logger.info(f"Viser: loading scene mesh {path} (GLB native path)")
            with open(path, "rb") as f:
                glb_bytes = f.read()
            self._scene_mesh_handle = self._server.scene.add_glb(
                "/scene_mesh",
                glb_data=glb_bytes,
                scale=scale,
                wxyz=wxyz,
                position=position,
            )
            logger.info(f"Viser: scene mesh added ({len(glb_bytes) / 1e6:.1f} MB GLB)")
            # Build raycaster from geometry separately (browser already
            # has the bytes for display; we need o3d structures here).
            try:
                scene_mesh = load_scene_mesh(path, alignment=mesh_alignment)
                self._raycast_scene = make_raycasting_scene(scene_mesh)
                logger.info("Viser: scene-mesh raycaster ready for click-to-point")
            except Exception as e:
                logger.warning(
                    f"Viser: raycaster build failed (point-goal will use plane fallback): {e}"
                )
            return

        # Non-GLB path: USD, OBJ, PLY — geometry-only, no texture decode blowup.
        logger.info(f"Viser: loading scene mesh {path}")
        scene_mesh = load_scene_mesh(path, alignment=mesh_alignment)
        vertices = np.asarray(scene_mesh.vertices, dtype=np.float32)
        faces = np.asarray(scene_mesh.triangles, dtype=np.int32)
        self._scene_mesh_handle = self._server.scene.add_mesh_simple(
            "/scene_mesh",
            vertices=vertices,
            faces=faces,
            color=(180, 180, 180),
            opacity=1.0,
        )
        logger.info(f"Viser: scene mesh added ({len(vertices)} verts, {len(faces)} tris)")
        try:
            self._raycast_scene = make_raycasting_scene(scene_mesh)
            logger.info("Viser: scene-mesh raycaster ready for click-to-point")
        except Exception as e:
            logger.warning(f"Viser: raycaster build failed: {e}")

    def _handle_floor_click(self, event: Any) -> None:
        """Project the click ray onto the z=0 floor and publish a goal."""
        ray_origin = event.ray_origin
        ray_direction = event.ray_direction
        if ray_origin is None or ray_direction is None:
            return

        ox, oy, oz = ray_origin
        dx, dy, dz = ray_direction
        if abs(dz) < 1e-6:
            logger.info("Viser nav-goal: click ray is parallel to floor, ignoring")
            return
        t = -oz / dz
        if t <= 0:
            logger.info("Viser nav-goal: click is above the horizon, ignoring")
            return
        x = ox + t * dx
        y = oy + t * dy

        marker_color = (0, 200, 255)
        try:
            self._server.scene.add_icosphere(
                "/nav_goal_marker",
                radius=0.08,
                position=(float(x), float(y), 0.05),
                color=marker_color,
            )
        except Exception as e:
            logger.debug(f"Viser nav-goal marker failed: {e}")

        point = PointStamped(x=float(x), y=float(y), z=0.0, ts=time.time(), frame_id="map")
        self.clicked_point.publish(point)
        logger.info(f"Viser nav-goal: published clicked_point=({x:.3f}, {y:.3f})")

    def _handle_point_goal_click(self, event: Any) -> None:
        """Click → 3D world point → publish to /point_goal.

        Resolution order:
          1. Cast against the loaded scene-mesh raycaster (exact surface hit).
          2. Fall back to intersecting an eye-height z=1.0 m plane — picking
             a floor target makes no sense for pointing, and eye height is a
             reasonable default for "the user clicked on empty space".
        """
        ray_origin = event.ray_origin
        ray_direction = event.ray_direction
        if ray_origin is None or ray_direction is None:
            return

        ox, oy, oz = (float(v) for v in ray_origin)
        dx, dy, dz = (float(v) for v in ray_direction)

        hit_xyz: tuple[float, float, float] | None = None

        # 1. Scene-mesh ray-cast if we have one.
        if self._raycast_scene is not None:
            import open3d.core as o3c

            rays = o3c.Tensor(
                np.array([[ox, oy, oz, dx, dy, dz]], dtype=np.float32),
                dtype=o3c.Dtype.Float32,
            )
            t_hit = float(self._raycast_scene.cast_rays(rays)["t_hit"].numpy()[0])
            if np.isfinite(t_hit) and t_hit > 0:
                hit_xyz = (ox + t_hit * dx, oy + t_hit * dy, oz + t_hit * dz)

        # 2. Fallback: intersect z = 1.0 m plane.
        if hit_xyz is None:
            if abs(dz) < 1e-6:
                logger.info("Viser point-goal: ray parallel to eye plane, ignoring")
                return
            t = (1.0 - oz) / dz
            if t <= 0:
                logger.info("Viser point-goal: target behind camera, ignoring")
                return
            hit_xyz = (ox + t * dx, oy + t * dy, 1.0)

        x, y, z = hit_xyz
        try:
            self._server.scene.add_icosphere(
                "/point_goal_marker",
                radius=0.06,
                position=(float(x), float(y), float(z)),
                color=(255, 80, 200),  # magenta — distinct from nav-goal cyan
            )
        except Exception as e:
            logger.debug(f"Viser point-goal marker failed: {e}")

        point = PointStamped(x=float(x), y=float(y), z=float(z), ts=time.time(), frame_id="map")
        self.point_goal.publish(point)
        logger.info(f"Viser point-goal: published point_goal=({x:.3f}, {y:.3f}, {z:.3f})")

    def _on_path(self, msg: PathMsg) -> None:
        """Draw the planner's path as a polyline floating above the floor."""
        poses = msg.poses
        if len(poses) < 2:
            handle = self._path_handle
            if handle is not None:
                try:
                    handle.remove()
                except Exception:
                    pass
                self._path_handle = None
            return

        path_height = 0.10  # lift above floor so it doesn't z-fight with the splat
        pts = np.array(
            [[p.position.x, p.position.y, path_height] for p in poses],
            dtype=np.float32,
        )
        # add_line_segments wants (N, 2, 3): start/end of each segment.
        segments = np.stack([pts[:-1], pts[1:]], axis=1)

        try:
            self._path_handle = self._server.scene.add_line_segments(
                "/nav_path",
                points=segments,
                colors=(255, 30, 30),
                line_width=4.0,
            )
        except Exception as e:
            logger.debug(f"Viser nav-path render failed: {e}")

    def _on_lidar(self, msg: PointCloud2) -> None:
        """Replace the lidar overlay in viser with the latest pointcloud.

        The publisher hands us an ``open3d`` PointCloud whose points are
        already in the world frame (this is what ``VoxelGridMapper``
        consumes too — see its docstring).  We pass the (N, 3) array to
        viser's ``add_point_cloud``; the previous handle is overwritten
        in-place so we don't accumulate cloud-on-cloud across frames.
        """
        if not self._lidar_visible or self._server is None:
            return
        try:
            pcd = msg.pointcloud
            pts = np.asarray(pcd.points, dtype=np.float32)
            if pts.size == 0:
                return
            # Per-point colors via height-mapped turbo colormap — same
            # gradient + same z-normalization formula rerun's pointcloud
            # path uses (PointCloud2.to_rerun) so the two viewers look
            # identical when both are running.
            from dimos.msgs.sensor_msgs.PointCloud2 import _get_colormap_lut

            lut = _get_colormap_lut("turbo")  # (256, 3) uint8, lru-cached
            z = pts[:, 2]
            z_min, z_max = float(z.min()), float(z.max())
            class_ids = ((z - z_min) / (z_max - z_min + 1e-8) * 255).astype(np.uint8)
            colors = lut[class_ids]  # (N, 3) uint8
            self._lidar_handle = self._server.scene.add_point_cloud(
                "/lidar_overlay",
                points=pts,
                colors=colors,
                point_size=self._pointcloud_point_size,
            )
            self._lidar_handle.visible = self._lidar_visible
        except Exception as e:
            logger.debug(f"Viser lidar overlay update failed: {e}")

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
            # PoseStamped quaternion is (x, y, z, w); MuJoCo / Viser want (w, x, y, z).
            self._latest_base_wxyz = np.array(
                [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z],
                dtype=np.float64,
            )

    def _render_loop(self) -> None:
        assert self._robot is not None
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
                self._update_camera_frustum()
                xpos = self._robot.data.xpos
                xquat = self._robot.data.xquat
                for body_id, frame in self._body_frames.items():
                    frame.position = tuple(float(x) for x in xpos[body_id])
                    frame.wxyz = tuple(float(x) for x in xquat[body_id])
            except Exception as e:
                logger.debug(f"Viser render tick failed: {e}")

            next_tick += self._render_dt
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.monotonic()

    def _update_camera_frustum(self) -> None:
        """Place the camera frustum at the current pose of its mount body."""
        if self._camera_frustum is None or self._camera_body_id is None:
            return
        if self._camera_spec is None:
            return
        assert self._robot is not None
        body_pos = self._robot.data.xpos[self._camera_body_id]
        body_wxyz = self._robot.data.xquat[self._camera_body_id]
        cam_pos, cam_wxyz = world_pose(body_pos, body_wxyz, self._camera_spec)
        self._camera_frustum.position = tuple(float(x) for x in cam_pos)
        self._camera_frustum.wxyz = tuple(float(x) for x in cam_wxyz)


viser_render = ViserRenderModule.blueprint


__all__ = ["ViserRenderModule", "viser_render"]
