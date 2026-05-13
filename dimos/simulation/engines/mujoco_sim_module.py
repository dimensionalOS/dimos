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
import open3d as o3d
from pydantic import Field
import reactivex as rx
from reactivex.disposable import Disposable
from scipy.spatial.transform import Rotation as R

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.hardware.sensors.camera.spec import DepthCameraConfig, DepthCameraHardware
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.simulation.engines.mujoco_engine import (
    CameraConfig,
    CameraFrame,
    MujocoEngine,
)
from dimos.simulation.engines.mujoco_shm import (
    ManipShmWriter,
    shm_key_from_path,
)
from dimos.spec import perception
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_RX180 = R.from_euler("x", 180, degrees=True)


def _default_identity_transform() -> Transform:
    return Transform(
        translation=Vector3(0.0, 0.0, 0.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
    )


class MujocoSimModuleConfig(ModuleConfig, DepthCameraConfig):
    """Configuration for the unified MuJoCo simulation module."""

    address: str = ""
    meshdir: str | None = None
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
    lidar_camera_names: list[str] = Field(default_factory=list)
    lidar_camera_width: int = 640
    lidar_camera_height: int = 360
    lidar_voxel_size: float = 0.05
    enable_kinematic_base_control: bool = False


class MujocoSimModule(
    DepthCameraHardware,
    Module,
    perception.DepthCamera,
):
    """Single Module that owns a MujocoEngine, publishes camera streams, and
    exposes joint state/commands to a ``ShmMujocoAdapter`` via shared memory.

    The adapter attaches to the same SHM buffers using the MJCF path as the
    discovery key — no RPC, no globals. From ControlCoordinator's perspective
    the adapter is an ordinary ``ManipulatorAdapter``; SHM is its transport.
    """

    config: MujocoSimModuleConfig
    color_image: Out[Image]
    depth_image: Out[Image]
    pointcloud: Out[PointCloud2]
    camera_info: Out[CameraInfo]
    depth_camera_info: Out[CameraInfo]
    joint_state: Out[JointState]
    odom: Out[PoseStamped]
    cmd_vel: In[Twist]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._engine: MujocoEngine | None = None
        self._shm: ManipShmWriter | None = None
        self._gripper_idx: int | None = None
        self._gripper_ctrl_range: tuple[float, float] = (0.0, 1.0)
        self._gripper_joint_range: tuple[float, float] = (0.0, 1.0)
        self._stop_event = threading.Event()
        self._publish_thread: threading.Thread | None = None
        self._camera_info_base: CameraInfo | None = None
        self._cmd_vel_lock = threading.Lock()
        self._cmd_vel = Twist.zero()
        self._last_cmd_vel_time = 0.0
        self._kinematic_base_z: float | None = None

    @property
    def _camera_enabled(self) -> bool:
        return self.config.enable_color or self.config.enable_depth or self.config.enable_pointcloud

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
        if not self.config.address:
            raise RuntimeError("MujocoSimModule: config.address (MJCF path) is required")

        # SHM key — adapter derives the same key from the same MJCF path.
        shm_key = shm_key_from_path(self.config.address)
        self._shm = ManipShmWriter(shm_key)

        camera_configs: list[CameraConfig] = []
        primary_needed = (
            self.config.enable_color
            or self.config.enable_depth
            or (self.config.enable_pointcloud and not self.config.lidar_camera_names)
        )
        if primary_needed:
            camera_configs.append(
                CameraConfig(
                    name=self.config.camera_name,
                    width=self.config.width,
                    height=self.config.height,
                    fps=float(self.config.fps),
                )
            )

        lidar_scene_option = mujoco.MjvOption()
        lidar_scene_option.geomgroup[0] = 0
        lidar_scene_option.geomgroup[1] = 0
        lidar_scene_option.geomgroup[2] = 1
        lidar_scene_option.geomgroup[3] = 1
        lidar_scene_option.geomgroup[4] = 0
        lidar_scene_option.geomgroup[5] = 0
        for lidar_name in self.config.lidar_camera_names:
            if lidar_name == self.config.camera_name and primary_needed:
                continue
            camera_configs.append(
                CameraConfig(
                    name=lidar_name,
                    width=self.config.lidar_camera_width,
                    height=self.config.lidar_camera_height,
                    fps=float(self.config.pointcloud_fps),
                    scene_option=lidar_scene_option,
                )
            )

        # Build engine with SHM hooks installed. Camera configs are optional:
        # physics and SHM state exchange must keep working in headless/no-render
        # deployments.
        self._engine = MujocoEngine(
            config_path=Path(self.config.address),
            headless=self.config.headless,
            cameras=camera_configs,
            meshdir=self.config.meshdir,
            on_before_step=self._apply_shm_commands,
            on_after_step=self._publish_shm_state,
        )

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

        # Start physics (sim thread spawned inside engine.connect()).
        if not self._engine.connect():
            raise RuntimeError("MujocoSimModule: engine.connect() failed")

        self._shm.signal_ready(num_joints=len(joint_names))

        self._stop_event.clear()

        if self.config.enable_kinematic_base_control:
            if not self._engine.has_root_freejoint:
                logger.warning("Kinematic base control requested, but MJCF has no freejoint root")
            root_pose = self._engine.get_root_pose()
            self._kinematic_base_z = None if root_pose is None else float(root_pose[0][2])
            self.register_disposable(Disposable(self.cmd_vel.subscribe(self._on_cmd_vel)))

        if primary_needed:
            # Camera intrinsics.
            self._build_camera_info()

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

        # Optional pointcloud generation.
        if self.config.enable_pointcloud and (primary_needed or self.config.lidar_camera_names):
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
            dof=dof,
            camera=self.config.camera_name,
            camera_enabled=self._camera_enabled,
            shm_key=shm_key,
        )

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

        self._camera_info_base = None
        super().stop()

        if errors:
            op, err = errors[0]
            raise RuntimeError(f"MujocoSimModule.stop() failed during {op}: {err}") from err

    def _apply_shm_commands(self, engine: MujocoEngine) -> None:
        """Pre-step hook: pull command targets from SHM into the engine."""
        shm = self._shm
        if shm is None:
            return
        dof = self.config.dof

        pos_cmd = shm.read_position_command(dof)
        if pos_cmd is not None:
            engine.write_joint_command(JointState(position=pos_cmd.tolist()))

        vel_cmd = shm.read_velocity_command(dof)
        if vel_cmd is not None:
            engine.write_joint_command(JointState(velocity=vel_cmd.tolist()))

        if self._gripper_idx is not None:
            gripper_cmd = shm.read_gripper_command()
            if gripper_cmd is not None:
                ctrl_value = self._gripper_joint_to_ctrl(gripper_cmd)
                engine.set_position_target(self._gripper_idx, ctrl_value)

    def _on_cmd_vel(self, msg: Twist) -> None:
        with self._cmd_vel_lock:
            self._cmd_vel = Twist(msg)
            self._last_cmd_vel_time = time.monotonic()

    def _apply_cmd_vel(self, engine: MujocoEngine) -> None:
        if not self.config.enable_kinematic_base_control:
            return
        with self._cmd_vel_lock:
            cmd = Twist(self._cmd_vel)
            age = time.monotonic() - self._last_cmd_vel_time
        if age > 0.5:
            cmd = Twist.zero()
        engine.apply_root_twist(
            cmd.linear.x,
            cmd.linear.y,
            cmd.angular.z,
            fixed_z=self._kinematic_base_z,
        )

    def _publish_shm_state(self, engine: MujocoEngine) -> None:
        """Post-step hook: publish joint state to SHM."""
        self._apply_cmd_vel(engine)

        shm = self._shm
        if shm is None:
            return
        shm.write_joint_state(
            positions=engine.joint_positions,
            velocities=engine.joint_velocities,
            efforts=engine.joint_efforts,
        )
        self.joint_state.publish(
            JointState(
                frame_id="mujoco",
                name=engine.joint_names,
                position=engine.joint_positions,
                velocity=engine.joint_velocities,
                effort=engine.joint_efforts,
            )
        )
        root_pose = engine.get_root_pose()
        if root_pose is not None:
            position, quat_xyzw = root_pose
            self.odom.publish(
                PoseStamped(
                    ts=time.time(),
                    frame_id="world",
                    position=Vector3(position),
                    orientation=Quaternion(quat_xyzw),
                )
            )
        if self._gripper_idx is not None:
            positions = engine.joint_positions
            if self._gripper_idx < len(positions):
                shm.write_gripper_state(positions[self._gripper_idx])

    def _gripper_joint_to_ctrl(self, joint_position: float) -> float:
        """Map joint-space gripper position to actuator control value."""
        jlo, jhi = self._gripper_joint_range
        clo, chi = self._gripper_ctrl_range
        clamped = max(jlo, min(jhi, joint_position))
        if jhi == jlo:
            return clo
        t = (clamped - jlo) / (jhi - jlo)
        return chi - t * (chi - clo)

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
        if self.config.lidar_camera_names:
            self._generate_lidar_pointcloud()
            return
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

    def _generate_lidar_pointcloud(self) -> None:
        if self._engine is None:
            return
        try:
            from dimos.simulation.mujoco.depth_camera import depth_image_to_point_cloud

            all_points: list[np.ndarray] = []
            latest_ts = 0.0
            for camera_name in self.config.lidar_camera_names:
                frame = self._engine.read_camera(camera_name)
                if frame is None:
                    continue
                points = depth_image_to_point_cloud(
                    frame.depth,
                    frame.cam_pos,
                    frame.cam_mat.reshape(3, 3),
                    fov_degrees=frame.fovy,
                )
                if points.size:
                    all_points.append(points)
                latest_ts = max(latest_ts, frame.timestamp)
            if not all_points:
                return
            cloud = o3d.geometry.PointCloud()
            cloud.points = o3d.utility.Vector3dVector(np.vstack(all_points))
            cloud = cloud.voxel_down_sample(self.config.lidar_voxel_size)
            self.pointcloud.publish(
                PointCloud2(pointcloud=cloud, ts=latest_ts or time.time(), frame_id="world")
            )
        except Exception as exc:
            logger.error("Multi-camera lidar fusion error", error=str(exc))


__all__ = ["MujocoSimModule", "MujocoSimModuleConfig"]
