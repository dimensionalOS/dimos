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
import shutil
import subprocess
import sys
import threading
import time
from typing import Any, Literal

import mujoco
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


def _find_sensor_slice(model: mujoco.MjModel, *names: str, dim: int = 3) -> slice | None:
    """Return the first matching MJCF sensor's slice into sensordata, or None."""
    for n in names:
        sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, n)
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


class MujocoSimModuleConfig(ModuleConfig, DepthCameraConfig):
    """Configuration for the unified MuJoCo simulation module."""

    address: str | Path = ""
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
    # Inject menagerie/dimos-bundled mesh bytes (via
    # dimos.simulation.mujoco.model.get_assets) into MjModel.from_xml_string.
    # MJCFs that reference meshes by bare filename (G1 GR00T, Go2) need this;
    # self-contained MJCFs with on-disk meshes (xarm scene.xml) don't.
    inject_legacy_assets: bool = False
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
    # Engine execution mode:
    #   "thread"     — engine runs on a dimos worker thread inside this
    #                  Module (current default). Cameras supported.
    #   "subprocess" — Module spawns ``python -m
    #                  dimos.simulation.engines.mujoco_engine`` and proxies
    #                  to it via SHM. Needed when a passive viewer is wanted
    #                  (mujoco.viewer.launch_passive requires main thread,
    #                  which a dimos worker can't provide on macOS).
    #                  Cameras + image-stream Out ports are not supported
    #                  in this mode — set enable_color/depth/pointcloud=False.
    engine_mode: Literal["thread", "subprocess"] = "thread"


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
    imu: Out[Imu]
    # Floating-base pose (qpos[0:7]) for robots whose MJCF has a free
    # joint at the root.  Published every step; consumers like the viser
    # viewer use this to translate the robot in world space.
    odom: Out[PoseStamped]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._engine: MujocoEngine | None = None
        self._shm: ManipShmWriter | None = None
        self._sim_hooks: Any | None = None  # WholeBodySimHooks; thread mode only
        self._engine_proc: subprocess.Popen[Any] | None = None  # subprocess mode only
        self._gripper_idx: int | None = None
        self._gripper_ctrl_range: tuple[float, float] = (0.0, 1.0)
        self._gripper_joint_range: tuple[float, float] = (0.0, 1.0)
        self._stop_event = threading.Event()
        self._publish_thread: threading.Thread | None = None
        self._camera_info_base: CameraInfo | None = None

        # IMU sensor slices into MjData.sensordata, resolved once at start.
        # None if the MJCF has no recognized IMU sensors (e.g. arm-only sims).
        self._imu_gyro_slice: slice | None = None
        self._imu_accel_slice: slice | None = None
        # Quaternion is read from the floating-base qpos[3:7] when the model
        # has a free joint at the root; None otherwise.
        self._imu_base_qpos_slice: slice | None = None

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
        # Both engine_mode paths use it.
        shm_key = shm_key_from_path(self.config.address)

        if self.config.engine_mode == "subprocess":
            self._start_subprocess(shm_key)
            return

        # Thread mode is the default engine path.
        self._shm = ManipShmWriter(shm_key)

        # Build engine with SHM hooks installed.
        engine_assets: dict[str, bytes] | None = None
        if self.config.inject_legacy_assets:
            from dimos.simulation.mujoco.model import get_assets

            engine_assets = get_assets()
        # Compose the camera list.  Each registered camera blocks the
        # sim thread inside _step_once (mujoco_engine._render_cameras
        # does update_scene + GPU render synchronously between physics
        # steps — typically 5-30 ms per camera), so registering a camera
        # nobody consumes burns the 500 Hz tick deadline for nothing.
        # Skip the primary camera entirely when none of color / depth /
        # pointcloud is enabled.
        cameras: list[CameraConfig] = []
        primary_needed = (
            self.config.enable_color or self.config.enable_depth or self.config.enable_pointcloud
        )
        if primary_needed:
            cameras.append(
                CameraConfig(
                    name=self.config.camera_name,
                    width=self.config.width,
                    height=self.config.height,
                    fps=float(self.config.fps),
                )
            )

        self._engine = MujocoEngine(
            config_path=Path(self.config.address),
            headless=self.config.headless,
            cameras=cameras,
            on_before_step=None,  # set after gripper detection below
            on_after_step=None,
            assets=engine_assets,
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

        # Resolve IMU sensors once. Names come from config so robot-
        # specific blueprints (G1, H1, Optimus, …) can override; manipulator
        # MJCFs typically have neither — we leave the slices as None and
        # skip IMU publishing for those.
        self._imu_gyro_slice = _find_sensor_slice(
            self._engine.model, *self.config.imu_gyro_sensor_names, dim=3
        )
        self._imu_accel_slice = _find_sensor_slice(
            self._engine.model, *self.config.imu_accel_sensor_names, dim=3
        )
        # Floating-base orientation is qpos[3:7] (w,x,y,z) when the root
        # joint is a free joint.  Detect by checking jnt_type[0].
        if self._engine.model.njnt > 0 and int(self._engine.model.jnt_type[0]) == int(
            mujoco.mjtJoint.mjJNT_FREE
        ):
            self._imu_base_qpos_slice = slice(3, 7)
        else:
            self._imu_base_qpos_slice = None

        # Wire SHM bridge hooks (shared with subprocess mode).
        from dimos.simulation.engines.wholebody_sim_hooks import WholeBodySimHooks

        self._sim_hooks = WholeBodySimHooks(
            self._shm,
            dof=dof,
            gripper_idx=self._gripper_idx,
            gripper_ctrl_range=self._gripper_ctrl_range,
            gripper_joint_range=self._gripper_joint_range,
        )
        self._engine._on_before_step = self._sim_hooks.pre_step  # type: ignore[attr-defined]
        self._engine._on_after_step = self._publish_shm_and_lcm  # type: ignore[attr-defined]

        # Start physics (sim thread spawned inside engine.connect()).
        if not self._engine.connect():
            raise RuntimeError("MujocoSimModule: engine.connect() failed")

        self._shm.signal_ready(num_joints=len(joint_names))

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

        # Optional pointcloud generation: back-projects primary camera depth.
        if self.config.enable_pointcloud and self.config.enable_depth:
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
            shm_key=shm_key,
        )

    def _start_subprocess(self, shm_key: str) -> None:
        """Spawn ``mujoco_engine.engine_main`` as a child process.

        macOS needs ``mjpython`` for ``viewer.launch_passive`` (it bridges
        the Cocoa main-thread requirement); Linux runs fine under the
        regular Python interpreter. Cameras + image-stream outputs are
        not supported in this mode — frame transport would need an SHM
        ring buffer that doesn't exist yet — so we refuse early to give
        a clear error.
        """
        if self.config.enable_color or self.config.enable_depth or self.config.enable_pointcloud:
            raise RuntimeError(
                "MujocoSimModule(engine_mode='subprocess') does not support cameras "
                "(no cross-process frame buffer yet). Set enable_color / enable_depth / "
                "enable_pointcloud to False or use engine_mode='thread'."
            )
        if sys.platform == "darwin":
            interp = shutil.which("mjpython") or shutil.which("python")
        else:
            interp = sys.executable
        if interp is None:
            raise RuntimeError(
                "MujocoSimModule(engine_mode='subprocess'): no mjpython/python on PATH"
            )

        cmd = [
            interp,
            "-m",
            "dimos.simulation.engines.mujoco_engine",
            str(self.config.address),
            shm_key,
            str(self.config.dof),
        ]
        if not self.config.headless:
            cmd.append("--view")
        if not self.config.inject_legacy_assets:
            cmd.append("--no-asset-inject")

        self._engine_proc = subprocess.Popen(cmd)
        time.sleep(0.2)
        returncode = self._engine_proc.poll()
        if returncode is not None:
            self._engine_proc = None
            raise RuntimeError(
                "MujocoSimModule engine subprocess exited immediately "
                f"(returncode={returncode}, address={self.config.address})"
            )
        logger.info(
            "MujocoSimModule spawned engine subprocess",
            pid=self._engine_proc.pid,
            interp=interp,
            address=self.config.address,
            shm_key=shm_key,
        )
        # The dimos-side adapter polls SHM readiness; start() only catches
        # immediate spawn failures here.

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._publish_thread and self._publish_thread.is_alive():
            self._publish_thread.join(timeout=2.0)
        self._publish_thread = None

        errors: list[tuple[str, BaseException]] = []
        # Thread-mode engine teardown.
        if self._engine is not None:
            try:
                self._engine.disconnect()
                self._engine = None
            except Exception as exc:
                logger.error("engine.disconnect() failed", error=str(exc))
                errors.append(("engine.disconnect", exc))
        # Subprocess-mode teardown — SIGTERM, escalate to SIGKILL after
        # a grace period if the subprocess doesn't exit (SHM cleanup
        # happens inside engine_main's finally block).
        if self._engine_proc is not None and self._engine_proc.poll() is None:
            try:
                self._engine_proc.terminate()
                self._engine_proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                logger.warning(
                    f"engine subprocess pid={self._engine_proc.pid} didn't exit in 3 s; SIGKILL"
                )
                self._engine_proc.kill()
            except Exception as exc:
                logger.error("engine subprocess terminate raised", error=str(exc))
                errors.append(("engine_proc.terminate", exc))
            finally:
                self._engine_proc = None
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

    def _publish_shm_and_lcm(self, engine: MujocoEngine) -> None:
        """Post-step hook: SHM writes (via WholeBodySimHooks) + LCM publishes.

        Subprocess-mode engines run the SHM half in their own process; the
        LCM half there too (see ``mujoco_engine.engine_main``). This thread-
        mode version stays here so existing in-Module behaviour is identical.
        """
        if self._sim_hooks is not None:
            self._sim_hooks.post_step(engine)
        shm = self._shm
        if shm is None:
            return

        # Odom — when the MJCF has a free-joint root, publish base pose
        # from qpos[0:7] every step.  Without this, downstream consumers
        # (viser viewer, nav stack) only see joint articulation, not
        # base translation through the world.
        data = engine.data  # in-process: same MjData the sim thread mutates
        if self._imu_base_qpos_slice is not None:
            base_pos = data.qpos[0:3]
            base_quat = data.qpos[3:7]  # (w, x, y, z) per MuJoCo convention
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

        # IMU — only if MJCF declared the sensors.
        if (
            self._imu_gyro_slice is None
            and self._imu_accel_slice is None
            and self._imu_base_qpos_slice is None
        ):
            return
        if self._imu_base_qpos_slice is not None:
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
        self.imu.publish(
            Imu(
                ts=time.time(),
                frame_id="pelvis",
                orientation=Quaternion(quat[1], quat[2], quat[3], quat[0]),
                angular_velocity=Vector3(gyro[0], gyro[1], gyro[2]),
                linear_acceleration=Vector3(accel[0], accel[1], accel[2]),
            )
        )

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


__all__ = ["MujocoSimModule", "MujocoSimModuleConfig"]
