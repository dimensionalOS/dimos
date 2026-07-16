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

"""MuJoCo simulation engine implementation."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import math
from pathlib import Path
import threading
import time
from typing import TYPE_CHECKING, cast

import mujoco
import mujoco.viewer as viewer  # type: ignore[import-untyped]
import numpy as np
from numpy.typing import NDArray

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.simulation.engines.base import SimulationEngine
from dimos.simulation.engines.robot_sim_binding import (
    RobotSimBinding,
    RobotSimSpec,
    resolve_robot_sim_binding,
)
from dimos.simulation.utils.xml_parser import JointMapping, build_joint_mappings
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.JointState import JointState

logger = setup_logger()

# Step hook signature: called with the engine instance inside the sim thread.
StepHook = Callable[["MujocoEngine"], None]

_MJJNT_FREE = int(mujoco.mjtJoint.mjJNT_FREE)  # type: ignore[attr-defined]
_RESET_WAIT_TIMEOUT_S = 5.0


def _camera_name_candidates(camera_name: str) -> tuple[str, ...]:
    stripped = camera_name.strip("/")
    candidates = [camera_name]
    if stripped != camera_name:
        candidates.append(stripped)
    elif stripped:
        candidates.append(f"/{stripped}")
    return tuple(dict.fromkeys(candidates))


def _camera_ray_directions(width: int, height: int, fovy_degrees: float) -> NDArray[np.float64]:
    """Return normalized camera-frame ray directions for MuJoCo cameras.

    MuJoCo/OpenGL cameras look along local -Z, with +X right and +Y up.
    """
    fovy = math.radians(fovy_degrees)
    focal = height / (2.0 * math.tan(fovy / 2.0))
    cx = width / 2.0
    cy = height / 2.0

    ys, xs = np.mgrid[0:height, 0:width]
    x = (xs + 0.5 - cx) / focal
    y = -(ys + 0.5 - cy) / focal
    z = -np.ones_like(x)
    directions = np.stack((x, y, z), axis=-1).reshape(-1, 3).astype(np.float64)
    norms = np.linalg.norm(directions, axis=1, keepdims=True)
    return cast("NDArray[np.float64]", directions / norms)


@dataclass
class CameraConfig:
    name: str
    width: int = 640
    height: int = 480
    fps: float = 15.0
    max_geom: int | None = 10000
    geom_groups: tuple[int, ...] | None = None


@dataclass
class CameraFrame:
    rgb: NDArray[np.uint8]
    depth: NDArray[np.float32]
    cam_pos: NDArray[np.float64]
    cam_mat: NDArray[np.float64]
    fovy: float
    timestamp: float


@dataclass
class RaycastLidarConfig:
    name: str
    width: int = 64
    height: int = 32
    fps: float = 1.0
    min_range: float = 0.2
    max_range: float = 3.0
    max_height: float = 1.2
    geom_groups: tuple[int, ...] | None = None
    robot_exclusion_radius: float = 0.0


@dataclass
class RaycastLidarFrame:
    points: NDArray[np.float32]
    timestamp: float


@dataclass
class _CameraRendererState:
    cfg: CameraConfig
    cam_id: int
    rgb_renderer: mujoco.Renderer
    depth_renderer: mujoco.Renderer
    scene_option: mujoco.MjvOption | None
    interval: float
    last_render_time: float = 0.0


@dataclass
class _RaycastLidarState:
    cfg: RaycastLidarConfig
    cam_id: int
    ray_directions_camera: NDArray[np.float64]
    geomgroup: NDArray[np.uint8]
    interval: float
    last_cast_time: float = 0.0


class MujocoEngine(SimulationEngine):
    """
    MuJoCo simulation engine.

    - starts MuJoCo simulation engine
    - loads robot/environment into simulation
    - applies control commands
    """

    def __init__(
        self,
        config_path: Path,
        headless: bool,
        cameras: list[CameraConfig] | None = None,
        raycast_lidars: list[RaycastLidarConfig] | None = None,
        on_before_step: StepHook | None = None,
        on_after_step: StepHook | None = None,
        assets: dict[str, bytes] | None = None,
        model: mujoco.MjModel | None = None,
        robot_sim_spec: RobotSimSpec | None = None,
        spawn_xy: tuple[float, float] | None = None,
        spawn_z: float | None = None,
        spawn_yaw: float | None = None,
        reset_joint_positions: list[float] | None = None,
    ) -> None:
        super().__init__(config_path=config_path, headless=headless)
        self._on_before_step: StepHook | None = on_before_step
        self._on_after_step: StepHook | None = on_after_step
        self._spawn_xy = spawn_xy
        self._spawn_z = spawn_z
        self._spawn_yaw = spawn_yaw
        self._reset_joint_positions = reset_joint_positions

        model_path = self._resolve_model_path(config_path)
        binary_model = model_path.suffix.lower() == ".mjb"
        if model is not None:
            if assets is not None:
                raise ValueError("MujocoEngine cannot use injected assets with a precompiled model")
            self._model = model
        elif assets is not None:
            # MJCFs that reference meshes by bare filename (e.g. menagerie
            # G1) need the mesh bytes injected by name; from_xml_path can't
            # find them on disk.
            if binary_model:
                raise ValueError("MujocoEngine cannot inject XML assets into a binary MJB model")
            with open(model_path) as f:
                xml_str = f.read()
            self._model = mujoco.MjModel.from_xml_string(xml_str, assets=assets)
        elif binary_model:
            self._model = mujoco.MjModel.from_binary_path(str(model_path))  # type: ignore[attr-defined]
        else:
            self._model = mujoco.MjModel.from_xml_path(str(model_path))
        self._xml_path = model_path

        self._data = mujoco.MjData(self._model)
        mapping_xml_path = None if model is not None or binary_model else self._xml_path
        self._joint_mappings = build_joint_mappings(mapping_xml_path, self._model)
        self._robot_binding: RobotSimBinding | None = None
        if robot_sim_spec is not None:
            self._robot_binding = resolve_robot_sim_binding(
                self._model, robot_sim_spec, self._joint_mappings
            )
            self._joint_mappings = list(self._robot_binding.joint_mappings)
        self._joint_names = [mapping.name for mapping in self._joint_mappings]
        self._num_joints = len(self._joint_names)
        self._root_qpos_adr = self._robot_binding.root_qpos_adr if self._robot_binding else None
        self._root_qvel_adr = self._robot_binding.root_qvel_adr if self._robot_binding else None
        if self._root_qpos_adr is None:
            self._root_qpos_adr, self._root_qvel_adr = self._find_first_freejoint_adrs()
        timestep = float(self._model.opt.timestep)
        self._control_frequency = 1.0 / timestep if timestep > 0.0 else 100.0

        self._connected = False
        self._lock = threading.Lock()
        self._reset_requested = False
        self._reset_done_events: list[threading.Event] = []
        self._stop_event = threading.Event()
        self._sim_thread: threading.Thread | None = None

        self._joint_positions = [0.0] * self._num_joints
        self._joint_velocities = [0.0] * self._num_joints
        self._joint_efforts = [0.0] * self._num_joints

        self._joint_position_targets = [0.0] * self._num_joints
        self._joint_velocity_targets = [0.0] * self._num_joints
        self._joint_effort_targets = [0.0] * self._num_joints
        self._command_mode = "position"
        self._apply_spawn_pose_unlocked()
        self._apply_reset_joint_positions_unlocked()
        for i, mapping in enumerate(self._joint_mappings):
            current_pos = self._current_position(mapping)
            self._joint_position_targets[i] = current_pos
            self._joint_positions[i] = current_pos

        # Camera rendering state (renderers created in sim thread)
        self._camera_configs = cameras or []
        self._camera_frames: dict[str, CameraFrame] = {}
        self._camera_lock = threading.Lock()
        self._raycast_lidar_configs = raycast_lidars or []
        self._raycast_lidar_frames: dict[str, RaycastLidarFrame] = {}
        self._raycast_lidar_lock = threading.Lock()

    def set_step_hooks(
        self,
        before: StepHook | None = None,
        after: StepHook | None = None,
    ) -> None:
        """Install pre/post step hooks after construction.

        Use when the hooks depend on engine state (joint count, gripper
        index) that isn't known until the model is loaded.
        """
        self._on_before_step = before
        self._on_after_step = after

    def _resolve_model_path(self, config_path: Path) -> Path:
        if config_path is None:
            raise ValueError("config_path is required for MuJoCo simulation loading")
        resolved = config_path.expanduser()
        model_path = resolved / "scene.xml" if resolved.is_dir() else resolved
        if not model_path.exists():
            raise FileNotFoundError(f"MuJoCo model not found: {model_path}")
        return model_path

    def _find_first_freejoint_adrs(self) -> tuple[int | None, int | None]:
        if self._model.njnt > 0 and int(self._model.jnt_type[0]) == _MJJNT_FREE:
            return int(self._model.jnt_qposadr[0]), int(self._model.jnt_dofadr[0])
        return None, None

    def _current_position(self, mapping: JointMapping) -> float:
        if mapping.joint_id is not None and mapping.qpos_adr is not None:
            return float(self._data.qpos[mapping.qpos_adr])
        if mapping.tendon_qpos_adrs:
            return float(
                sum(self._data.qpos[adr] for adr in mapping.tendon_qpos_adrs)
                / len(mapping.tendon_qpos_adrs)
            )
        if mapping.actuator_id is not None:
            return float(self._data.actuator_length[mapping.actuator_id])
        return 0.0

    def _apply_control(self) -> None:
        with self._lock:
            if self._command_mode == "effort":
                targets = list(self._joint_effort_targets)
            elif self._command_mode == "velocity":
                targets = list(self._joint_velocity_targets)
            elif self._command_mode == "position":
                targets = list(self._joint_position_targets)
            for i, mapping in enumerate(self._joint_mappings):
                if mapping.actuator_id is None:
                    continue
                if i < len(targets):
                    self._data.ctrl[mapping.actuator_id] = targets[i]

    def _update_joint_state(self) -> None:
        with self._lock:
            for i, mapping in enumerate(self._joint_mappings):
                if mapping.joint_id is not None:
                    if mapping.qpos_adr is not None:
                        self._joint_positions[i] = float(self._data.qpos[mapping.qpos_adr])
                    if mapping.dof_adr is not None:
                        self._joint_velocities[i] = float(self._data.qvel[mapping.dof_adr])
                        self._joint_efforts[i] = float(self._data.qfrc_actuator[mapping.dof_adr])
                    continue

                if mapping.tendon_qpos_adrs:
                    pos_sum = sum(self._data.qpos[adr] for adr in mapping.tendon_qpos_adrs)
                    count = len(mapping.tendon_qpos_adrs)
                    self._joint_positions[i] = float(pos_sum / count)
                    if mapping.tendon_dof_adrs:
                        vel_sum = sum(self._data.qvel[adr] for adr in mapping.tendon_dof_adrs)
                        self._joint_velocities[i] = float(vel_sum / len(mapping.tendon_dof_adrs))
                    else:
                        self._joint_velocities[i] = 0.0
                elif mapping.actuator_id is not None:
                    self._joint_positions[i] = float(
                        self._data.actuator_length[mapping.actuator_id]
                    )
                    self._joint_velocities[i] = 0.0

                if mapping.actuator_id is not None:
                    self._joint_efforts[i] = float(self._data.actuator_force[mapping.actuator_id])

    def connect(self) -> bool:
        try:
            logger.info("connect()", cls=self.__class__.__name__)
            with self._lock:
                self._connected = True
                self._stop_event.clear()

            if self._sim_thread is None or not self._sim_thread.is_alive():
                self._sim_thread = threading.Thread(
                    target=self._sim_loop,
                    name=f"{self.__class__.__name__}Sim",
                    daemon=True,
                )
                self._sim_thread.start()
            return True
        except Exception as e:
            logger.error("connect() failed", cls=self.__class__.__name__, error=str(e))
            return False

    def disconnect(self) -> bool:
        try:
            logger.info("disconnect()", cls=self.__class__.__name__)
            with self._lock:
                self._connected = False
            self._stop_event.set()
            if self._sim_thread and self._sim_thread.is_alive():
                self._sim_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._sim_thread = None
            return True
        except Exception as e:
            logger.error("disconnect() failed", cls=self.__class__.__name__, error=str(e))
            return False

    def _init_cameras(self) -> dict[str, _CameraRendererState]:
        """Create renderers for all configured cameras"""
        # Grow the offscreen framebuffer to fit the largest camera — MJCF
        # <visual><global offwidth/offheight> defaults to 640x480 and
        # mujoco.Renderer refuses to render larger than the framebuffer.
        if self._camera_configs:
            self._model.vis.global_.offwidth = max(
                self._model.vis.global_.offwidth, max(c.width for c in self._camera_configs)
            )
            self._model.vis.global_.offheight = max(
                self._model.vis.global_.offheight, max(c.height for c in self._camera_configs)
            )
        cam_renderers: dict[str, _CameraRendererState] = {}
        for cfg in self._camera_configs:
            cam_id = self._camera_id(cfg.name)
            if cam_id < 0:
                logger.warning("Camera not found in MJCF, skipping", camera_name=cfg.name)
                continue
            max_geom = cfg.max_geom
            if max_geom is None:
                max_geom = max(10000, int(self._model.ngeom) + 1000)
            rgb_renderer = mujoco.Renderer(
                self._model,
                height=cfg.height,
                width=cfg.width,
                max_geom=max_geom,
            )  # type: ignore[call-arg]
            depth_renderer = mujoco.Renderer(
                self._model,
                height=cfg.height,
                width=cfg.width,
                max_geom=max_geom,
            )  # type: ignore[call-arg]
            depth_renderer.enable_depth_rendering()
            scene_option = None
            if cfg.geom_groups is not None:
                scene_option = mujoco.MjvOption()
                geomgroup = scene_option.geomgroup  # type: ignore[attr-defined]
                geomgroup[:] = 0
                for group in cfg.geom_groups:
                    if 0 <= group < len(geomgroup):
                        geomgroup[group] = 1
            interval = 1.0 / cfg.fps if cfg.fps > 0 else float("inf")
            cam_renderers[cfg.name] = _CameraRendererState(
                cfg=cfg,
                cam_id=cam_id,
                rgb_renderer=rgb_renderer,
                depth_renderer=depth_renderer,
                scene_option=scene_option,
                interval=interval,
            )
        return cam_renderers

    def _init_raycast_lidars(self) -> dict[str, _RaycastLidarState]:
        lidar_states: dict[str, _RaycastLidarState] = {}
        for cfg in self._raycast_lidar_configs:
            cam_id = self._camera_id(cfg.name)
            if cam_id < 0:
                logger.warning(
                    "Raycast lidar camera not found in MJCF, skipping",
                    camera_name=cfg.name,
                )
                continue

            geomgroup = np.zeros(6, dtype=np.uint8)
            if cfg.geom_groups is None:
                geomgroup[:] = 1
            else:
                for group in cfg.geom_groups:
                    if 0 <= group < len(geomgroup):
                        geomgroup[group] = 1

            lidar_states[cfg.name] = _RaycastLidarState(
                cfg=cfg,
                cam_id=cam_id,
                ray_directions_camera=_camera_ray_directions(
                    cfg.width,
                    cfg.height,
                    float(self._model.cam_fovy[cam_id]),
                ),
                geomgroup=geomgroup,
                interval=1.0 / cfg.fps if cfg.fps > 0 else float("inf"),
            )
        return lidar_states

    def _render_cameras(self, now: float, cam_renderers: dict[str, _CameraRendererState]) -> None:
        """Render all due cameras and store frames. Must be called from sim thread."""
        for state in cam_renderers.values():
            if now - state.last_render_time < state.interval:
                continue
            state.last_render_time = now

            state.rgb_renderer.update_scene(
                self._data, camera=state.cam_id, scene_option=state.scene_option
            )
            rgb = state.rgb_renderer.render().copy()

            state.depth_renderer.update_scene(
                self._data, camera=state.cam_id, scene_option=state.scene_option
            )
            depth = state.depth_renderer.render().copy()

            frame = CameraFrame(
                rgb=rgb,
                depth=depth.astype(np.float32),
                cam_pos=self._data.cam_xpos[state.cam_id].copy(),
                cam_mat=self._data.cam_xmat[state.cam_id].copy(),
                fovy=float(self._model.cam_fovy[state.cam_id]),
                timestamp=now,
            )
            with self._camera_lock:
                self._camera_frames[state.cfg.name] = frame

    def _raycast_lidars(
        self,
        now: float,
        lidar_states: dict[str, _RaycastLidarState],
    ) -> None:
        """Raycast lidar frames from MuJoCo cameras. Must be called from sim thread."""
        bodyexclude = self._robot_binding.root_body_id if self._robot_binding is not None else -1
        for state in lidar_states.values():
            if now - state.last_cast_time < state.interval:
                continue
            state.last_cast_time = now

            origin = self._data.cam_xpos[state.cam_id].copy()
            camera_mat = self._data.cam_xmat[state.cam_id].reshape(3, 3).copy()
            directions_world = state.ray_directions_camera @ camera_mat.T
            n_rays = directions_world.shape[0]
            geom_ids = np.full(n_rays, -1, dtype=np.int32)
            distances = np.full(n_rays, -1.0, dtype=np.float64)
            mujoco.mj_multiRay(  # type: ignore[attr-defined]
                self._model,
                self._data,
                origin,
                directions_world.ravel(),
                state.geomgroup,
                1,
                bodyexclude,
                geom_ids,
                distances,
                None,
                n_rays,
                state.cfg.max_range,
            )
            valid = (distances >= state.cfg.min_range) & (distances <= state.cfg.max_range)
            valid &= np.abs(state.ray_directions_camera[:, 1] * distances) <= state.cfg.max_height
            if np.any(valid):
                points_world = origin + directions_world[valid] * distances[valid, None]
                if state.cfg.robot_exclusion_radius > 0.0 and self._root_qpos_adr is not None:
                    root_xy = self._data.qpos[self._root_qpos_adr : self._root_qpos_adr + 2]
                    keep = (
                        np.linalg.norm(points_world[:, :2] - root_xy, axis=1)
                        >= state.cfg.robot_exclusion_radius
                    )
                    points_world = points_world[keep]
                arr = points_world.astype(np.float32)
            else:
                arr = np.empty((0, 3), dtype=np.float32)
            with self._raycast_lidar_lock:
                self._raycast_lidar_frames[state.cfg.name] = RaycastLidarFrame(
                    points=arr,
                    timestamp=now,
                )

    @staticmethod
    def _close_cam_renderers(cam_renderers: dict[str, _CameraRendererState]) -> None:
        for state in cam_renderers.values():
            state.rgb_renderer.close()
            state.depth_renderer.close()

    def _reset_unlocked(self) -> None:
        if self._model.nkey > 0:
            mujoco.mj_resetDataKeyframe(self._model, self._data, 0)
        else:
            mujoco.mj_resetData(self._model, self._data)  # type: ignore[attr-defined]
        self._apply_spawn_pose_unlocked()
        self._apply_reset_joint_positions_unlocked()
        for i, mapping in enumerate(self._joint_mappings):
            self._joint_position_targets[i] = self._current_position(mapping)
        self._command_mode = "position"

        root_pose = self.get_root_pose_unlocked()
        if root_pose is not None:
            position, _ = root_pose
            logger.info(
                "MuJoCo reset applied",
                x=float(position[0]),
                y=float(position[1]),
                z=float(position[2]),
            )

    def _apply_spawn_pose_unlocked(self) -> None:
        qpos_adr = self._root_qpos_adr
        if qpos_adr is None:
            mujoco.mj_forward(self._model, self._data)
            return

        qpos = self._data.qpos
        if self._spawn_xy is not None:
            qpos[qpos_adr] = float(self._spawn_xy[0])
            qpos[qpos_adr + 1] = float(self._spawn_xy[1])
        if self._spawn_z is not None:
            qpos[qpos_adr + 2] = float(self._spawn_z)
        if self._spawn_yaw is not None:
            qpos[qpos_adr + 3 : qpos_adr + 7] = [
                math.cos(float(self._spawn_yaw) * 0.5),
                0.0,
                0.0,
                math.sin(float(self._spawn_yaw) * 0.5),
            ]

        qvel_adr = self._root_qvel_adr
        if qvel_adr is not None:
            self._data.qvel[qvel_adr : qvel_adr + 6] = 0.0
        mujoco.mj_forward(self._model, self._data)

    def _apply_reset_joint_positions_unlocked(self) -> None:
        if self._reset_joint_positions is None:
            return
        for index, position in enumerate(self._reset_joint_positions[: self._num_joints]):
            mapping = self._joint_mappings[index]
            if mapping.qpos_adr is not None:
                self._data.qpos[mapping.qpos_adr] = float(position)
            if mapping.dof_adr is not None:
                self._data.qvel[mapping.dof_adr] = 0.0
            if mapping.actuator_id is not None:
                self._data.ctrl[mapping.actuator_id] = float(position)
        mujoco.mj_forward(self._model, self._data)

    def _sim_loop(self) -> None:
        logger.info("sim loop started", cls=self.__class__.__name__)
        dt = 1.0 / self._control_frequency

        # Camera renderers: created once in the sim thread
        cam_renderers = self._init_cameras()
        lidar_states = self._init_raycast_lidars()

        def _step_once(sync_viewer: bool) -> None:
            loop_start = time.time()
            reset_done_events: list[threading.Event] = []
            with self._lock:
                if self._reset_requested:
                    self._reset_requested = False
                    self._reset_unlocked()
                    reset_done_events = self._reset_done_events
                    self._reset_done_events = []
            for reset_done_event in reset_done_events:
                reset_done_event.set()
            if self._on_before_step is not None:
                try:
                    self._on_before_step(self)
                except Exception as exc:
                    logger.error("on_before_step failed", error=str(exc))
            self._apply_control()
            mujoco.mj_step(self._model, self._data)
            if sync_viewer:
                m_viewer.sync()
            self._update_joint_state()
            if self._on_after_step is not None:
                try:
                    self._on_after_step(self)
                except Exception as exc:
                    logger.error("on_after_step failed", error=str(exc))
            self._render_cameras(loop_start, cam_renderers)
            self._raycast_lidars(loop_start, lidar_states)

            elapsed = time.time() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        if self._headless:
            while not self._stop_event.is_set():
                _step_once(sync_viewer=False)
        else:
            with viewer.launch_passive(
                self._model, self._data, show_left_ui=False, show_right_ui=False
            ) as m_viewer:
                while m_viewer.is_running() and not self._stop_event.is_set():
                    _step_once(sync_viewer=True)

        self._close_cam_renderers(cam_renderers)
        logger.info("sim loop stopped", cls=self.__class__.__name__)

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._connected

    @property
    def num_joints(self) -> int:
        return self._num_joints

    @property
    def joint_names(self) -> list[str]:
        return list(self._joint_names)

    @property
    def robot_binding(self) -> RobotSimBinding | None:
        return self._robot_binding

    @property
    def has_root_freejoint(self) -> bool:
        return self._root_qpos_adr is not None

    @property
    def root_qpos_adr(self) -> int | None:
        return self._root_qpos_adr

    @property
    def root_qvel_adr(self) -> int | None:
        return self._root_qvel_adr

    @property
    def model(self) -> mujoco.MjModel:
        return self._model

    @property
    def data(self) -> mujoco.MjData:
        """Live MjData. In-process consumers (sensors, PD hooks) read it
        directly; physics integration in the sim thread mutates it under
        ``self._lock`` so reads inside the same MujocoEngine instance are
        coherent without extra locking."""
        return self._data

    @property
    def joint_positions(self) -> list[float]:
        with self._lock:
            return list(self._joint_positions)

    @property
    def joint_velocities(self) -> list[float]:
        with self._lock:
            return list(self._joint_velocities)

    @property
    def joint_efforts(self) -> list[float]:
        with self._lock:
            return list(self._joint_efforts)

    @property
    def control_frequency(self) -> float:
        return self._control_frequency

    def read_joint_positions(self) -> list[float]:
        return self.joint_positions

    def read_joint_velocities(self) -> list[float]:
        return self.joint_velocities

    def read_joint_efforts(self) -> list[float]:
        return self.joint_efforts

    def write_joint_command(self, command: JointState) -> None:
        if command.position:
            self._command_mode = "position"
            self._set_position_targets(command.position)
            return
        if command.velocity:
            self._command_mode = "velocity"
            self._set_velocity_targets(command.velocity)
            return
        if command.effort:
            self._command_mode = "effort"
            self._set_effort_targets(command.effort)
            return

    def _set_position_targets(self, positions: list[float]) -> None:
        if len(positions) > self._num_joints:
            raise ValueError(
                f"Position command has {len(positions)} joints, expected at most {self._num_joints}"
            )
        with self._lock:
            for i in range(len(positions)):
                self._joint_position_targets[i] = float(positions[i])

    def _set_velocity_targets(self, velocities: list[float]) -> None:
        if len(velocities) > self._num_joints:
            raise ValueError(
                f"Velocity command has {len(velocities)} joints, expected at most {self._num_joints}"
            )
        with self._lock:
            for i in range(len(velocities)):
                self._joint_velocity_targets[i] = float(velocities[i])

    def _set_effort_targets(self, efforts: list[float]) -> None:
        if len(efforts) > self._num_joints:
            raise ValueError(
                f"Effort command has {len(efforts)} joints, expected at most {self._num_joints}"
            )
        with self._lock:
            for i in range(len(efforts)):
                self._joint_effort_targets[i] = float(efforts[i])

    def set_position_target(self, index: int, value: float) -> None:
        with self._lock:
            self._joint_position_targets[index] = float(value)

    def get_position_target(self, index: int) -> float:
        with self._lock:
            return float(self._joint_position_targets[index])

    def hold_current_position(self) -> None:
        with self._lock:
            self._command_mode = "position"
            for i, mapping in enumerate(self._joint_mappings):
                self._joint_position_targets[i] = self._current_position(mapping)

    def reset(self) -> None:
        with self._lock:
            self._reset_unlocked()

    def request_reset(
        self,
        *,
        wait: bool = False,
        timeout: float = _RESET_WAIT_TIMEOUT_S,
    ) -> bool:
        done_event = threading.Event() if wait else None
        with self._lock:
            if done_event is not None:
                self._reset_done_events.append(done_event)
            self._reset_requested = True
        if done_event is None:
            return True
        return done_event.wait(timeout)

    def request_reset_to(
        self,
        *,
        spawn_xy: tuple[float, float],
        spawn_z: float | None = None,
        spawn_yaw: float | None = None,
        wait: bool = False,
        timeout: float = _RESET_WAIT_TIMEOUT_S,
    ) -> bool:
        done_event = threading.Event() if wait else None
        with self._lock:
            self._spawn_xy = spawn_xy
            if spawn_z is not None:
                self._spawn_z = spawn_z
            if spawn_yaw is not None:
                self._spawn_yaw = spawn_yaw
            if done_event is not None:
                self._reset_done_events.append(done_event)
            self._reset_requested = True
        if done_event is None:
            return True
        return done_event.wait(timeout)

    def ground_height_at(
        self,
        x: float,
        y: float,
        *,
        ray_start_z: float = 10.0,
    ) -> float | None:
        """Raycast likely scene support geoms at ``x, y``.

        Scene collision geoms are emitted in group 3 and legacy floors are
        commonly group 2. Restricting the ray to those groups avoids hitting
        the robot itself when respawning near its current pose.
        """
        with self._lock:
            origin = np.array([float(x), float(y), float(ray_start_z)], dtype=np.float64)
            direction = np.array([0.0, 0.0, -1.0], dtype=np.float64)
            geom_id = np.array([-1], dtype=np.int32)
            geomgroup = np.array([0, 0, 1, 1, 0, 0], dtype=np.uint8)
            distance = mujoco.mj_ray(  # type: ignore[attr-defined]
                self._model,
                self._data,
                origin,
                direction,
                geomgroup,
                1,
                -1,
                geom_id,
            )
            if distance < 0.0:
                return None
            return float(ray_start_z - distance)

    def get_root_pose(self) -> tuple[NDArray[np.float64], NDArray[np.float64]] | None:
        with self._lock:
            return self.get_root_pose_unlocked()

    def get_root_pose_unlocked(self) -> tuple[NDArray[np.float64], NDArray[np.float64]] | None:
        qpos_adr = self._root_qpos_adr
        if qpos_adr is None:
            return None
        position = self._data.qpos[qpos_adr : qpos_adr + 3].copy()
        qw, qx, qy, qz = self._data.qpos[qpos_adr + 3 : qpos_adr + 7].copy()
        return position, np.array([qx, qy, qz, qw], dtype=np.float64)

    def get_actuator_ctrl_range(self, joint_index: int) -> tuple[float, float] | None:
        mapping = self._joint_mappings[joint_index]
        if mapping.actuator_id is None:
            return None
        lo = float(self._model.actuator_ctrlrange[mapping.actuator_id, 0])
        hi = float(self._model.actuator_ctrlrange[mapping.actuator_id, 1])
        return (lo, hi)

    def get_joint_range(self, joint_index: int) -> tuple[float, float] | None:
        mapping = self._joint_mappings[joint_index]
        if mapping.tendon_qpos_adrs:
            first_adr = mapping.tendon_qpos_adrs[0]
            for jid in range(self._model.njnt):
                if self._model.jnt_qposadr[jid] == first_adr:
                    return (
                        float(self._model.jnt_range[jid, 0]),
                        float(self._model.jnt_range[jid, 1]),
                    )
        if mapping.joint_id is not None:
            return (
                float(self._model.jnt_range[mapping.joint_id, 0]),
                float(self._model.jnt_range[mapping.joint_id, 1]),
            )
        return None

    def read_camera(self, camera_name: str) -> CameraFrame | None:
        """Read the latest rendered frame for a camera (thread-safe).

        Returns None if the camera hasn't rendered yet or doesn't exist.
        """
        with self._camera_lock:
            return self._camera_frames.get(camera_name)

    def read_raycast_lidar(self, camera_name: str) -> RaycastLidarFrame | None:
        """Read the latest raycast lidar frame for a camera (thread-safe)."""
        with self._raycast_lidar_lock:
            return self._raycast_lidar_frames.get(camera_name)

    def get_camera_fovy(self, camera_name: str) -> float | None:
        """Get vertical field of view for a named camera, in degrees."""
        cam_id = self._camera_id(camera_name)
        if cam_id < 0:
            return None
        return float(self._model.cam_fovy[cam_id])

    def _camera_id(self, camera_name: str) -> int:
        for candidate in _camera_name_candidates(camera_name):
            cam_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_CAMERA, candidate)
            if cam_id >= 0:
                return int(cam_id)
        return -1
