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

"""Unified MuJoCo simulation Module.

Owns a single ``MujocoEngine`` and publishes:
- camera streams (Out ports), replacing ``MujocoCamera``
- joint state via shared memory, consumed by ``ShmMujocoAdapter`` inside
  ``ControlCoordinator``

This avoids the prior pattern of sharing engines via a global in-process
registry, which was fragile when ``WorkerManager`` places the adapter and
the camera in different worker processes.
"""

from __future__ import annotations

import math
from pathlib import Path
import threading
import time
from typing import Any

import mujoco
import numpy as np
from numpy.typing import NDArray
from pydantic import Field
import reactivex as rx
from scipy.spatial.transform import Rotation as R

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.hardware.sensors.camera.spec import DepthCameraConfig, DepthCameraHardware
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.simulation.engines.mujoco_engine import (
    CameraConfig,
    CameraFrame,
    MujocoEngine,
    RaycastLidarConfig,
)
from dimos.simulation.engines.mujoco_shm import (
    CMD_MODE_PD_TAU,
    ManipShmWriter,
    shm_key_from_path,
)
from dimos.simulation.engines.robot_sim_binding import RobotSimSpec
from dimos.simulation.mujoco.constants import LIDAR_RESOLUTION, MAX_HEIGHT, MAX_RANGE, MIN_RANGE
from dimos.spec import perception
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _find_sensor_slice(model: mujoco.MjModel, *names: str, dim: int = 3) -> slice | None:
    """Return the first matching MJCF sensor's slice into sensordata, or None."""
    for n in names:
        sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, n)  # type: ignore[attr-defined]
        if sid >= 0:
            adr = int(model.sensor_adr[sid])
            return slice(adr, adr + dim)
    return None


_RX180 = R.from_euler("x", 180, degrees=True)


def _default_identity_transform() -> Transform:
    return Transform(
        translation=Vector3(0.0, 0.0, 0.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
    )


def _imu_from_mujoco_wxyz(
    quaternion: tuple[float, float, float, float],
    gyroscope: tuple[float, float, float],
    accelerometer: tuple[float, float, float],
    *,
    frame_id: str,
    ts: float,
) -> Imu:
    w, x, y, z = quaternion
    return Imu(
        orientation=Quaternion(x, y, z, w),
        angular_velocity=Vector3(*gyroscope),
        linear_acceleration=Vector3(*accelerometer),
        frame_id=frame_id,
        ts=ts,
    )


class _WholeBodySimHooks:
    """Per-step bridge between MuJoCo actuators and whole-body SHM."""

    def __init__(
        self,
        shm: ManipShmWriter,
        dof: int,
        *,
        gripper_idx: int | None = None,
        gripper_ctrl_range: tuple[float, float] = (0.0, 1.0),
        gripper_joint_range: tuple[float, float] = (0.0, 1.0),
    ) -> None:
        self._shm = shm
        self._dof = dof
        self._gripper_idx = gripper_idx
        self._gripper_ctrl_range = gripper_ctrl_range
        self._gripper_joint_range = gripper_joint_range
        self._latest_pd_pos_target: NDArray[np.float64] | None = None
        self._latest_pd_kp: NDArray[np.float64] | None = None
        self._latest_pd_kd: NDArray[np.float64] | None = None
        self._latest_pd_tau: NDArray[np.float64] | None = None

    def pre_step(self, engine: MujocoEngine) -> None:
        shm = self._shm
        dof = self._dof

        pos_cmd = shm.read_position_command(dof)
        if pos_cmd is not None:
            if shm.read_command_mode() == CMD_MODE_PD_TAU:
                self._latest_pd_pos_target = pos_cmd
            else:
                engine.write_joint_command(JointState(position=pos_cmd.tolist()))

        vel_cmd = shm.read_velocity_command(dof)
        if vel_cmd is not None:
            engine.write_joint_command(JointState(velocity=vel_cmd.tolist()))

        kp_cmd = shm.read_kp_command(dof)
        if kp_cmd is not None:
            self._latest_pd_kp = kp_cmd
        kd_cmd = shm.read_kd_command(dof)
        if kd_cmd is not None:
            self._latest_pd_kd = kd_cmd
        tau_cmd = shm.read_tau_command(dof)
        if tau_cmd is not None:
            self._latest_pd_tau = tau_cmd

        if (
            self._latest_pd_pos_target is not None
            and self._latest_pd_kp is not None
            and self._latest_pd_kd is not None
        ):
            q = np.asarray(engine.joint_positions[:dof], dtype=np.float64)
            dq = np.asarray(engine.joint_velocities[:dof], dtype=np.float64)
            tau_ff = self._latest_pd_tau if self._latest_pd_tau is not None else np.zeros(dof)
            tau = (
                self._latest_pd_kp * (self._latest_pd_pos_target - q)
                + self._latest_pd_kd * (-dq)
                + tau_ff
            )
            engine.write_joint_command(JointState(effort=tau.tolist()))

        if self._gripper_idx is not None:
            gripper_cmd = shm.read_gripper_command()
            if gripper_cmd is not None:
                engine.set_position_target(
                    self._gripper_idx, self._gripper_joint_to_ctrl(gripper_cmd)
                )

    def post_step(self, engine: MujocoEngine) -> None:
        shm = self._shm
        shm.write_joint_state(
            positions=engine.joint_positions,
            velocities=engine.joint_velocities,
            efforts=engine.joint_efforts,
        )
        if self._gripper_idx is not None:
            positions = engine.joint_positions
            if self._gripper_idx < len(positions):
                shm.write_gripper_state(positions[self._gripper_idx])

    def clear_latched_commands(self) -> None:
        self._latest_pd_pos_target = None
        self._latest_pd_kp = None
        self._latest_pd_kd = None
        self._latest_pd_tau = None

    def _gripper_joint_to_ctrl(self, joint_position: float) -> float:
        jlo, jhi = self._gripper_joint_range
        clo, chi = self._gripper_ctrl_range
        clamped = max(jlo, min(jhi, joint_position))
        if jhi == jlo:
            return clo
        t = (clamped - jlo) / (jhi - jlo)
        return chi - t * (chi - clo)


class MujocoSimModuleConfig(ModuleConfig, DepthCameraConfig):
    """Configuration for the unified MuJoCo simulation module.

    Use ``address`` for the legacy path: one MJCF/MJB that already contains
    scene and robot. Use ``robot_mjcf`` plus optional ``scene_xml`` for scene
    packages: the module composes the scene wrapper, robot MJCF, and package
    entities at startup.
    """

    address: str | Path = ""
    scene_xml: str | Path | None = None
    robot_mjcf: str | Path | None = None
    robot_meshdir: str | Path | None = None
    robot_id: str = ""
    scene_entities: list[dict[str, Any]] = Field(default_factory=list)
    spawn_xy: tuple[float, float] | None = None
    spawn_z: float | None = None
    spawn_yaw: float | None = None
    reset_joint_positions: list[float] | None = None
    headless: bool = False
    dof: int = 7

    # Camera config (matches former MujocoCameraConfig).
    camera_name: str = "wrist_camera"
    width: int = 640
    height: int = 480
    fps: int = 15
    base_frame_id: str = "link7"
    base_transform: Transform | None = Field(default_factory=_default_identity_transform)
    align_depth_to_color: bool = True
    enable_color: bool = True
    enable_depth: bool = True
    enable_pointcloud: bool = False
    pointcloud_fps: float = 5.0
    camera_info_fps: float = 1.0
    # Optional MuJoCo-native lidar: cast rays from one or more named cameras
    # and publish world-frame PointCloud2 points on ``pointcloud``.
    enable_mujoco_lidar: bool = False
    mujoco_lidar_camera_names: list[str] = Field(default_factory=list)
    mujoco_lidar_voxel_size: float = LIDAR_RESOLUTION
    mujoco_lidar_geom_groups: list[int] = Field(default_factory=lambda: [0, 1, 2, 3, 4, 5])
    mujoco_lidar_raycast_width: int = 64
    mujoco_lidar_raycast_height: int = 32
    mujoco_lidar_min_range: float = MIN_RANGE
    mujoco_lidar_max_range: float = MAX_RANGE
    mujoco_lidar_max_height: float = MAX_HEIGHT
    mujoco_lidar_robot_exclusion_radius: float = 0.0
    # Inject menagerie/dimos-bundled mesh bytes (via
    # dimos.simulation.mujoco.model.get_assets) into MjModel.from_xml_string.
    # MJCFs that reference meshes by bare filename (G1 GR00T, Go2) need this;
    # self-contained MJCFs with on-disk meshes (xarm scene.xml) don't.
    inject_legacy_assets: bool = False
    robot_sim_spec: RobotSimSpec | None = None
    # MJCF sensor names used to publish IMU. The module probes these in
    # order and uses the first that exists in the model; if none match
    # IMU publishing stays silent. Default list covers the common
    # humanoid pelvis-mounted naming conventions (menagerie + dimos
    # bundled MJCFs); pass robot-specific names for other platforms.
    imu_gyro_sensor_names: list[str] = Field(
        default_factory=lambda: [
            "imu-pelvis-angular-velocity",
            "imu-torso-angular-velocity",
            "gyro_pelvis",
            "imu_gyro",
        ]
    )
    imu_accel_sensor_names: list[str] = Field(
        default_factory=lambda: [
            "imu-pelvis-linear-acceleration",
            "imu-torso-linear-acceleration",
            "accelerometer_pelvis",
            "imu_accel",
        ]
    )


class MujocoSimModule(
    DepthCameraHardware,
    Module,
    perception.DepthCamera,
):
    """Single Module that owns a MujocoEngine, publishes camera streams, and
    exposes joint state/commands to a ``ShmMujocoAdapter`` via shared memory.

    The adapter attaches to the same SHM buffers using the MJCF path as the
    discovery key - no RPC, no globals. From ControlCoordinator's perspective
    the adapter is an ordinary ``ManipulatorAdapter``; SHM is its transport.
    """

    config: MujocoSimModuleConfig
    color_image: Out[Image]
    depth_image: Out[Image]
    pointcloud: Out[PointCloud2]
    camera_info: Out[CameraInfo]
    depth_camera_info: Out[CameraInfo]
    imu: Out[Imu]
    # Floating-base pose for robots whose MJCF has a free joint at the
    # root. Published every step; consumers like the viser viewer use
    # this to translate the robot in world space.
    odom: Out[PoseStamped]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._engine: MujocoEngine | None = None
        self._shm: ManipShmWriter | None = None
        self._sim_hooks: _WholeBodySimHooks | None = None
        self._gripper_idx: int | None = None
        self._gripper_ctrl_range: tuple[float, float] = (0.0, 1.0)
        self._gripper_joint_range: tuple[float, float] = (0.0, 1.0)
        self._stop_event = threading.Event()
        self._publish_thread: threading.Thread | None = None
        self._camera_info_base: CameraInfo | None = None
        self._shm_ready_signaled = False

        # IMU sensor slices into MjData.sensordata, resolved once at start.
        # None if the MJCF has no recognized IMU sensors (e.g. arm-only sims).
        self._imu_quat_slice: slice | None = None
        self._imu_gyro_slice: slice | None = None
        self._imu_accel_slice: slice | None = None
        # Quaternion is read from the floating-base qpos when the model
        # has a free joint at the robot root; None otherwise.
        self._imu_base_qpos_slice: slice | None = None
        self._root_base_qpos_adr: int | None = None
        self._root_spawn_clearance_z: float | None = None

    @property
    def _camera_link(self) -> str:
        return f"{self.config.camera_name}_link"

    @property
    def _color_frame(self) -> str:
        return f"{self.config.camera_name}_color_frame"

    @property
    def _color_optical_frame(self) -> str:
        return f"{self.config.camera_name}_color_optical_frame"

    @property
    def _depth_frame(self) -> str:
        return f"{self.config.camera_name}_depth_frame"

    @property
    def _depth_optical_frame(self) -> str:
        return f"{self.config.camera_name}_depth_optical_frame"

    @rpc
    def get_color_camera_info(self) -> CameraInfo | None:
        if self._camera_info_base is None:
            return None
        return self._camera_info_base.with_ts(time.time())

    @rpc
    def get_depth_camera_info(self) -> CameraInfo | None:
        if self._camera_info_base is None:
            return None
        return self._camera_info_base.with_ts(time.time())

    @rpc
    def get_depth_scale(self) -> float:
        return 1.0

    @rpc
    def start(self) -> None:
        if not self.config.address and not self.config.robot_mjcf:
            raise RuntimeError(
                "MujocoSimModule: either config.address (legacy MJCF path) "
                "or config.robot_mjcf (composed scene path) is required"
            )

        # SHM key - adapters derive the same key from the robot/control MJCF path.
        shm_key_source = self.config.robot_mjcf or self.config.address
        shm_key = shm_key_from_path(shm_key_source)
        self._shm = ManipShmWriter(shm_key)
        self._shm_ready_signaled = False

        # Build engine with SHM hooks installed.
        engine_assets: dict[str, bytes] | None = None
        if self.config.inject_legacy_assets:
            # Lazy import: get_assets pulls in mujoco_playground (heavy,
            # optional) and is only needed when injecting bundled meshes.
            from dimos.simulation.mujoco.model import get_assets

            engine_assets = get_assets()
        # Compose rendered cameras separately from raycast lidar. Each
        # rendered camera blocks the sim thread, so MuJoCo lidar does not
        # register depth cameras here.
        cameras_by_name: dict[str, CameraConfig] = {}
        raycast_lidars: list[RaycastLidarConfig] = []

        def add_camera(
            name: str,
            *,
            geom_groups: list[int] | None = None,
            max_geom: int | None = 10000,
        ) -> None:
            if not name:
                return
            groups = tuple(geom_groups) if geom_groups is not None else None
            existing = cameras_by_name.get(name)
            if existing is not None:
                if groups is not None:
                    existing.geom_groups = groups
                    existing.max_geom = max_geom
                return
            cameras_by_name[name] = CameraConfig(
                name=name,
                width=self.config.width,
                height=self.config.height,
                fps=float(self.config.fps),
                max_geom=max_geom,
                geom_groups=groups,
            )

        primary_needed = (
            self.config.enable_color
            or self.config.enable_depth
            or (self.config.enable_pointcloud and not self.config.enable_mujoco_lidar)
        )
        if primary_needed:
            add_camera(self.config.camera_name)

        if self.config.enable_pointcloud and self.config.enable_mujoco_lidar:
            for camera_name in self._mujoco_lidar_camera_names():
                raycast_lidars.append(
                    RaycastLidarConfig(
                        name=camera_name,
                        width=self.config.mujoco_lidar_raycast_width,
                        height=self.config.mujoco_lidar_raycast_height,
                        fps=self.config.pointcloud_fps,
                        min_range=self.config.mujoco_lidar_min_range,
                        max_range=self.config.mujoco_lidar_max_range,
                        max_height=self.config.mujoco_lidar_max_height,
                        geom_groups=tuple(self.config.mujoco_lidar_geom_groups),
                        robot_exclusion_radius=self.config.mujoco_lidar_robot_exclusion_radius,
                    )
                )

        cameras = list(cameras_by_name.values())

        # Hooks are installed via set_step_hooks() after gripper detection
        # below, since they depend on the resolved gripper index.
        engine_kwargs: dict[str, Any] = dict(
            headless=self.config.headless,
            cameras=cameras,
            raycast_lidars=raycast_lidars,
            robot_sim_spec=self.config.robot_sim_spec,
            reset_joint_positions=self.config.reset_joint_positions,
        )
        if self.config.robot_mjcf is not None:
            engine_kwargs["config_path"] = Path(self.config.robot_mjcf)
            engine_kwargs["model"] = self._compose_model()
        else:
            engine_kwargs["config_path"] = Path(self.config.address)
            engine_kwargs["assets"] = engine_assets
            engine_kwargs["spawn_xy"] = self.config.spawn_xy
            engine_kwargs["spawn_z"] = self.config.spawn_z
            engine_kwargs["spawn_yaw"] = self.config.spawn_yaw
        self._engine = MujocoEngine(**engine_kwargs)

        # Detect gripper (extra joint beyond dof).
        dof = self.config.dof
        joint_names = list(self._engine.joint_names)
        if len(joint_names) > dof:
            ctrl_range = self._engine.get_actuator_ctrl_range(dof)
            joint_range = self._engine.get_joint_range(dof)
            if ctrl_range is None or joint_range is None:
                raise ValueError(f"Gripper joint at index {dof} missing ctrl/joint range in MJCF")
            self._gripper_idx = dof
            self._gripper_ctrl_range = ctrl_range
            self._gripper_joint_range = joint_range
            logger.info(
                "MujocoSimModule: gripper detected",
                idx=dof,
                ctrl_range=ctrl_range,
                joint_range=joint_range,
            )

        # Resolve IMU/root state once. RobotSimSpec wins when provided:
        # it scopes sensors and floating base to the policy robot rather
        # than assuming global model order.
        binding = self._engine.robot_binding
        if binding is not None:
            self._imu_quat_slice = binding.imu_quat_slice
            self._imu_gyro_slice = binding.imu_gyro_slice
            self._imu_accel_slice = binding.imu_accel_slice
            self._root_base_qpos_adr = binding.root_qpos_adr
        else:
            self._imu_quat_slice = None
            self._imu_gyro_slice = _find_sensor_slice(
                self._engine.model, *self.config.imu_gyro_sensor_names, dim=3
            )
            self._imu_accel_slice = _find_sensor_slice(
                self._engine.model, *self.config.imu_accel_sensor_names, dim=3
            )
            self._root_base_qpos_adr = self._engine.root_qpos_adr

        if self._root_base_qpos_adr is not None:
            self._imu_base_qpos_slice = slice(
                self._root_base_qpos_adr + 3, self._root_base_qpos_adr + 7
            )
        else:
            self._imu_base_qpos_slice = None
        self._root_spawn_clearance_z = self._compute_root_spawn_clearance_z()

        # Wire SHM bridge hooks.
        self._sim_hooks = _WholeBodySimHooks(
            self._shm,
            dof=dof,
            gripper_idx=self._gripper_idx,
            gripper_ctrl_range=self._gripper_ctrl_range,
            gripper_joint_range=self._gripper_joint_range,
        )
        self._engine.set_step_hooks(
            before=self._sim_hooks.pre_step,
            after=self._publish_shm_and_lcm,
        )

        # Start physics (sim thread spawned inside engine.connect()).
        if not self._engine.connect():
            raise RuntimeError("MujocoSimModule: engine.connect() failed")

        # Camera intrinsics.
        self._build_camera_info()

        self._stop_event.clear()
        self._publish_thread = threading.Thread(
            target=self._publish_loop, daemon=True, name="MujocoSimPublish"
        )
        self._publish_thread.start()

        # Periodic camera_info publishing.
        interval_sec = 1.0 / self.config.camera_info_fps
        self.register_disposable(
            rx.interval(interval_sec).subscribe(
                on_next=lambda _: self._publish_camera_info(),
                on_error=lambda e: logger.error("CameraInfo publish error", error=str(e)),
            )
        )

        # Optional pointcloud generation. Default mode preserves the legacy
        # RGB-D camera pointcloud; MuJoCo lidar mode publishes native raycast
        # hits in world coordinates.
        if self.config.enable_pointcloud and (
            self.config.enable_depth or self.config.enable_mujoco_lidar
        ):
            pc_interval = 1.0 / self.config.pointcloud_fps
            self.register_disposable(
                rx.interval(pc_interval).subscribe(
                    on_next=lambda _: self._generate_pointcloud(),
                    on_error=lambda e: logger.error("Pointcloud error", error=str(e)),
                )
            )

        logger.info(
            "MujocoSimModule started",
            address=self.config.address,
            robot_mjcf=self.config.robot_mjcf,
            scene_xml=self.config.scene_xml,
            dof=dof,
            camera=self.config.camera_name,
            shm_key=shm_key,
        )

    def _compose_model(self) -> mujoco.MjModel:
        """Compose optional scene package MJCF + robot MJCF + scene-package entities."""
        from dimos.simulation.mujoco.scene_package_entity_composer import (
            add_scene_package_entities_to_spec,
            find_scene_package_entity_spawn_penetrators,
        )

        if self.config.robot_mjcf is None:
            raise RuntimeError("MujocoSimModule: robot_mjcf is required for composition")

        def build_spec(*, force_static: frozenset[str] = frozenset()) -> mujoco.MjSpec:
            if self.config.scene_xml is not None:
                spec_scene = mujoco.MjSpec.from_file(str(self.config.scene_xml))
            else:
                spec_scene = mujoco.MjSpec()

            spec_robot = mujoco.MjSpec.from_file(str(self.config.robot_mjcf))
            if self.config.robot_meshdir is not None:
                spec_robot.meshdir = str(self.config.robot_meshdir)

            # Keep the robot controller timing stable when attached to a scene
            # package whose wrapper may have different default options.
            spec_scene.option.timestep = spec_robot.option.timestep

            spawn_xy = self.config.spawn_xy or (0.0, 0.0)
            spawn_z = self.config.spawn_z if self.config.spawn_z is not None else 0.0
            frame_kwargs: dict[str, Any] = {
                "pos": [float(spawn_xy[0]), float(spawn_xy[1]), float(spawn_z)],
            }
            if self.config.spawn_yaw is not None:
                yaw = float(self.config.spawn_yaw)
                frame_kwargs["quat"] = [math.cos(yaw * 0.5), 0.0, 0.0, math.sin(yaw * 0.5)]
            frame = spec_scene.worldbody.add_frame(
                **frame_kwargs,
            )
            prefix = f"{self.config.robot_id}-" if self.config.robot_id else None
            spec_scene.attach(spec_robot, prefix=prefix, frame=frame)

            if self.config.scene_entities:
                add_scene_package_entities_to_spec(
                    spec_scene, self.config.scene_entities, force_static=force_static
                )
            return spec_scene

        spec_scene = build_spec()
        model = spec_scene.compile()
        if self.config.scene_entities:
            penetrators = find_scene_package_entity_spawn_penetrators(model)
            if penetrators:
                logger.warning(
                    "MujocoSimModule: scene-package entities spawn in deep contact; welding static",
                    count=len(penetrators),
                    samples=sorted(penetrators)[:20],
                )
                model = build_spec(force_static=penetrators).compile()
        return model

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._publish_thread and self._publish_thread.is_alive():
            self._publish_thread.join(timeout=2.0)
        self._publish_thread = None

        errors: list[tuple[str, BaseException]] = []
        if self._engine is not None:
            try:
                self._engine.disconnect()
                self._engine = None
            except Exception as exc:
                logger.error("engine.disconnect() failed", error=str(exc))
                errors.append(("engine.disconnect", exc))
        if self._shm is not None:
            try:
                self._shm.signal_stop()
                self._shm.cleanup()
                self._shm = None
            except Exception as exc:
                logger.error("SHM cleanup failed", error=str(exc))
                errors.append(("shm.cleanup", exc))

        self._sim_hooks = None
        self._camera_info_base = None
        super().stop()

        if errors:
            op, err = errors[0]
            raise RuntimeError(f"MujocoSimModule.stop() failed during {op}: {err}") from err

    @rpc
    def reset(self) -> bool:
        """Reset the running simulation to its current configured spawn pose."""
        engine = self._engine
        if engine is None:
            return False
        if self._sim_hooks is not None:
            self._sim_hooks.clear_latched_commands()
        applied = engine.request_reset(wait=True)
        logger.info("MujocoSimModule: reset requested", applied=applied)
        return applied

    @rpc
    def respawn_at(
        self,
        x: float,
        y: float,
        z: float | None = None,
        yaw: float | None = None,
    ) -> bool:
        engine = self._engine
        if engine is None:
            return False
        if self._sim_hooks is not None:
            self._sim_hooks.clear_latched_commands()

        ground_z = None
        spawn_z = None if z is None else float(z)
        if spawn_z is None:
            ground_z = engine.ground_height_at(float(x), float(y))
            if ground_z is not None and self._root_spawn_clearance_z is not None:
                spawn_z = ground_z + self._root_spawn_clearance_z

        applied = engine.request_reset_to(
            spawn_xy=(float(x), float(y)),
            spawn_z=spawn_z,
            spawn_yaw=None if yaw is None else float(yaw),
            wait=True,
        )
        logger.info(
            "MujocoSimModule: respawn_at requested",
            x=x,
            y=y,
            z=z,
            ground_z=ground_z,
            root_clearance_z=self._root_spawn_clearance_z,
            spawn_z=spawn_z,
            yaw=yaw,
            applied=applied,
        )
        return applied

    def _compute_root_spawn_clearance_z(self) -> float | None:
        engine = self._engine
        qpos_adr = self._root_base_qpos_adr
        if engine is None or qpos_adr is None:
            return None

        qpos = engine.data.qpos
        root_x = float(qpos[qpos_adr])
        root_y = float(qpos[qpos_adr + 1])
        root_z = float(qpos[qpos_adr + 2])
        ground_z = engine.ground_height_at(root_x, root_y)
        if ground_z is not None:
            return root_z - ground_z
        if self.config.spawn_z is not None:
            return root_z - float(self.config.spawn_z)
        return root_z

    def _publish_shm_and_lcm(self, engine: MujocoEngine) -> None:
        """Post-step hook: SHM writes + LCM publishes.

        This stays in the module so odom/IMU continue to flow through normal
        typed ports while the whole-body adapter consumes joint state via SHM.
        """
        if self._sim_hooks is not None:
            self._sim_hooks.post_step(engine)
        shm = self._shm
        if shm is None:
            return

        # Odom - when the MJCF has a free-joint root, publish base pose
        # every step.  Without this, downstream consumers (viser viewer,
        # nav stack) only see joint articulation, not base translation
        # through the world.
        data = engine.data  # in-process: same MjData the sim thread mutates
        if self._root_base_qpos_adr is not None:
            base_pos = data.qpos[self._root_base_qpos_adr : self._root_base_qpos_adr + 3]
            base_quat = data.qpos[
                self._root_base_qpos_adr + 3 : self._root_base_qpos_adr + 7
            ]  # (w, x, y, z) per MuJoCo convention
            self.odom.publish(
                PoseStamped(
                    ts=time.time(),
                    frame_id="world",
                    position=Vector3(float(base_pos[0]), float(base_pos[1]), float(base_pos[2])),
                    orientation=Quaternion(
                        float(base_quat[1]),
                        float(base_quat[2]),
                        float(base_quat[3]),
                        float(base_quat[0]),
                    ),  # PoseStamped uses x,y,z,w
                )
            )

        # IMU - only if MJCF declared the sensors.
        if (
            self._imu_quat_slice is None
            and self._imu_gyro_slice is None
            and self._imu_accel_slice is None
            and self._imu_base_qpos_slice is None
        ):
            if not self._shm_ready_signaled:
                shm.signal_ready(num_joints=len(engine.joint_names))
                self._shm_ready_signaled = True
            return

        if self._imu_quat_slice is not None:
            q = data.sensordata[self._imu_quat_slice]
            quat = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
        elif self._imu_base_qpos_slice is not None:
            q = data.qpos[self._imu_base_qpos_slice]
            quat = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
        else:
            quat = (1.0, 0.0, 0.0, 0.0)
        if self._imu_gyro_slice is not None:
            g = data.sensordata[self._imu_gyro_slice]
            gyro = (float(g[0]), float(g[1]), float(g[2]))
        else:
            gyro = (0.0, 0.0, 0.0)
        if self._imu_accel_slice is not None:
            a = data.sensordata[self._imu_accel_slice]
            accel = (float(a[0]), float(a[1]), float(a[2]))
        else:
            accel = (0.0, 0.0, 0.0)
        shm.write_imu(quaternion=quat, gyroscope=gyro, accelerometer=accel)
        # Also publish on the stream port for downstream consumers.
        # MuJoCo reports quaternions as (w,x,y,z); Imu/Quaternion stores (x,y,z,w).
        self.imu.publish(
            _imu_from_mujoco_wxyz(quat, gyro, accel, frame_id="pelvis", ts=time.time())
        )

        if not self._shm_ready_signaled:
            shm.signal_ready(num_joints=len(engine.joint_names))
            self._shm_ready_signaled = True

    def _build_camera_info(self) -> None:
        if self._engine is None:
            return
        fovy_deg = self._engine.get_camera_fovy(self.config.camera_name)
        if fovy_deg is None:
            logger.error("Camera not found in MJCF", camera_name=self.config.camera_name)
            return
        h = self.config.height
        w = self.config.width
        fovy_rad = math.radians(fovy_deg)
        fy = h / (2.0 * math.tan(fovy_rad / 2.0))
        fx = fy  # square pixels
        self._camera_info_base = CameraInfo.from_intrinsics(
            fx=fx,
            fy=fy,
            cx=w / 2.0,
            cy=h / 2.0,
            width=w,
            height=h,
            frame_id=self._color_optical_frame,
        )

    def _publish_loop(self) -> None:
        """Poll engine for rendered frames and publish at configured FPS."""
        engine = self._engine
        if engine is None:
            return

        interval = 1.0 / self.config.fps
        last_timestamp = 0.0
        published_count = 0

        # Wait for engine to actually be connected (sim thread may take a tick).
        deadline = time.monotonic() + 30.0
        while not self._stop_event.is_set() and not engine.connected:
            if time.monotonic() > deadline:
                logger.error("MujocoSimModule: timed out waiting for engine to connect")
                return
            self._stop_event.wait(timeout=0.1)

        if self._stop_event.is_set():
            return

        while not self._stop_event.is_set():
            try:
                frame = engine.read_camera(self.config.camera_name)
            except RuntimeError as exc:
                logger.error(
                    "MuJoCo render failed; stopping publish loop",
                    camera_name=self.config.camera_name,
                    error=str(exc),
                    exc_info=True,
                )
                return

            if frame is None or frame.timestamp <= last_timestamp:
                self._stop_event.wait(timeout=interval * 0.5)
                continue
            last_timestamp = frame.timestamp
            ts = time.time()

            if self.config.enable_color:
                color_img = Image(
                    data=frame.rgb,
                    format=ImageFormat.RGB,
                    frame_id=self._color_optical_frame,
                    ts=ts,
                )
                self.color_image.publish(color_img)

            if self.config.enable_depth:
                depth_img = Image(
                    data=frame.depth,
                    format=ImageFormat.DEPTH,
                    frame_id=self._color_optical_frame,
                    ts=ts,
                )
                self.depth_image.publish(depth_img)

            self._publish_tf(ts, frame)

            published_count += 1
            if published_count == 1:
                logger.info(
                    "MujocoSimModule first frame published",
                    rgb_shape=frame.rgb.shape,
                    depth_shape=frame.depth.shape,
                )

            elapsed = time.time() - ts
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _publish_camera_info(self) -> None:
        base = self._camera_info_base
        if base is None:
            return
        ts = time.time()
        info = CameraInfo(
            height=base.height,
            width=base.width,
            distortion_model=base.distortion_model,
            D=base.D,
            K=base.K,
            P=base.P,
            frame_id=base.frame_id,
            ts=ts,
        )
        self.camera_info.publish(info)
        self.depth_camera_info.publish(info)

    def _publish_tf(self, ts: float, frame: CameraFrame | None) -> None:
        if frame is None:
            return
        mj_rot = R.from_matrix(frame.cam_mat.reshape(3, 3))
        optical_rot = mj_rot * _RX180
        q = optical_rot.as_quat()  # xyzw
        pos = Vector3(
            float(frame.cam_pos[0]),
            float(frame.cam_pos[1]),
            float(frame.cam_pos[2]),
        )
        rot = Quaternion(float(q[0]), float(q[1]), float(q[2]), float(q[3]))
        self.tf.publish(
            Transform(
                translation=pos,
                rotation=rot,
                frame_id="world",
                child_frame_id=self._color_optical_frame,
                ts=ts,
            ),
            Transform(
                translation=pos,
                rotation=rot,
                frame_id="world",
                child_frame_id=self._depth_optical_frame,
                ts=ts,
            ),
            Transform(
                translation=pos,
                rotation=rot,
                frame_id="world",
                child_frame_id=self._camera_link,
                ts=ts,
            ),
        )

    def _generate_pointcloud(self) -> None:
        if self._engine is None:
            return
        if self.config.enable_mujoco_lidar:
            self._generate_mujoco_lidar_pointcloud()
            return
        # Back-project the primary camera's depth image.
        if self._camera_info_base is None:
            return
        frame = self._engine.read_camera(self.config.camera_name)
        if frame is None:
            return
        try:
            color_img = Image(
                data=frame.rgb,
                format=ImageFormat.RGB,
                frame_id=self._color_optical_frame,
                ts=frame.timestamp,
            )
            depth_img = Image(
                data=frame.depth,
                format=ImageFormat.DEPTH,
                frame_id=self._color_optical_frame,
                ts=frame.timestamp,
            )
            pcd = PointCloud2.from_rgbd(
                color_image=color_img,
                depth_image=depth_img,
                camera_info=self._camera_info_base,
                depth_scale=1.0,
            )
            pcd = pcd.voxel_downsample(0.005)
            self.pointcloud.publish(pcd)
        except Exception as exc:
            logger.error("Pointcloud generation error", error=str(exc))

    def _mujoco_lidar_camera_names(self) -> list[str]:
        names = self.config.mujoco_lidar_camera_names
        return list(names) if names else [self.config.camera_name]

    def _generate_mujoco_lidar_pointcloud(self) -> None:
        engine = self._engine
        if engine is None:
            return

        all_points: list[NDArray[Any]] = []
        latest_ts = 0.0
        for camera_name in self._mujoco_lidar_camera_names():
            frame = engine.read_raycast_lidar(camera_name)
            if frame is None:
                continue
            if frame.points.size > 0:
                all_points.append(frame.points)
                latest_ts = max(latest_ts, frame.timestamp)

        if not all_points:
            return

        try:
            pcd = PointCloud2.from_numpy(
                np.vstack(all_points),
                frame_id="world",
                timestamp=latest_ts or time.time(),
            )
            pcd = pcd.voxel_downsample(self.config.mujoco_lidar_voxel_size)
            self.pointcloud.publish(pcd)
        except Exception as exc:
            logger.error("MuJoCo lidar pointcloud generation error", error=str(exc))
