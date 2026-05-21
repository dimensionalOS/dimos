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
from pathlib import Path
import signal
import threading
import time
from typing import TYPE_CHECKING

import mujoco
import mujoco.viewer as viewer  # type: ignore[import-untyped]
import numpy as np
from numpy.typing import NDArray

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.simulation.engines.base import SimulationEngine
from dimos.simulation.engines.mujoco_shm import ManipShmWriter
from dimos.simulation.engines.wholebody_sim_hooks import WholeBodySimHooks
from dimos.simulation.utils.xml_parser import JointMapping, build_joint_mappings
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.JointState import JointState

logger = setup_logger()

# Step hook signature: called with the engine instance inside the sim thread.
StepHook = Callable[["MujocoEngine"], None]


@dataclass
class CameraConfig:
    name: str
    width: int = 640
    height: int = 480
    fps: float = 15.0


@dataclass
class CameraFrame:
    rgb: NDArray[np.uint8]
    depth: NDArray[np.float32]
    cam_pos: NDArray[np.float64]
    cam_mat: NDArray[np.float64]
    fovy: float
    timestamp: float


@dataclass
class _CameraRendererState:
    cfg: CameraConfig
    cam_id: int
    rgb_renderer: mujoco.Renderer
    depth_renderer: mujoco.Renderer
    interval: float
    last_render_time: float = 0.0


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
        on_before_step: StepHook | None = None,
        on_after_step: StepHook | None = None,
        assets: dict[str, bytes] | None = None,
    ) -> None:
        super().__init__(config_path=config_path, headless=headless)
        self._on_before_step: StepHook | None = on_before_step
        self._on_after_step: StepHook | None = on_after_step

        xml_path = self._resolve_xml_path(config_path)
        if assets is not None:
            # MJCFs that reference meshes by bare filename (e.g. menagerie
            # G1) need the mesh bytes injected by name; from_xml_path can't
            # find them on disk.
            with open(xml_path) as f:
                xml_str = f.read()
            self._model = mujoco.MjModel.from_xml_string(xml_str, assets=assets)
        else:
            self._model = mujoco.MjModel.from_xml_path(str(xml_path))
        self._xml_path = xml_path

        self._data = mujoco.MjData(self._model)
        self._joint_mappings = build_joint_mappings(self._xml_path, self._model)
        self._joint_names = [mapping.name for mapping in self._joint_mappings]
        self._num_joints = len(self._joint_names)
        timestep = float(self._model.opt.timestep)
        self._control_frequency = 1.0 / timestep if timestep > 0.0 else 100.0

        self._connected = False
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._sim_thread: threading.Thread | None = None

        self._joint_positions = [0.0] * self._num_joints
        self._joint_velocities = [0.0] * self._num_joints
        self._joint_efforts = [0.0] * self._num_joints

        self._joint_position_targets = [0.0] * self._num_joints
        self._joint_velocity_targets = [0.0] * self._num_joints
        self._joint_effort_targets = [0.0] * self._num_joints
        self._command_mode = "position"
        for i, mapping in enumerate(self._joint_mappings):
            current_pos = self._current_position(mapping)
            self._joint_position_targets[i] = current_pos
            self._joint_positions[i] = current_pos

        # Camera rendering state (renderers created in sim thread)
        self._camera_configs = cameras or []
        self._camera_frames: dict[str, CameraFrame] = {}
        self._camera_lock = threading.Lock()

    def _resolve_xml_path(self, config_path: Path) -> Path:
        if config_path is None:
            raise ValueError("config_path is required for MuJoCo simulation loading")
        resolved = config_path.expanduser()
        xml_path = resolved / "scene.xml" if resolved.is_dir() else resolved
        if not xml_path.exists():
            raise FileNotFoundError(f"MuJoCo XML not found: {xml_path}")
        return xml_path

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

    def run_blocking(self, on_started: Callable[[], None] | None = None) -> None:
        """Run the simulation loop on the current thread until stopped.

        This is used by the subprocess entry point so ``mujoco.viewer`` runs
        on that process's main thread. The normal Module path still uses
        ``connect()``, which starts the loop on a worker thread.
        """
        logger.info("run_blocking()", cls=self.__class__.__name__)
        with self._lock:
            self._connected = True
            self._stop_event.clear()
        try:
            self._sim_loop(on_started=on_started)
        finally:
            with self._lock:
                self._connected = False

    def request_stop(self) -> None:
        """Ask the sim loop to stop without joining a thread."""
        with self._lock:
            self._connected = False
        self._stop_event.set()

    def disconnect(self) -> bool:
        try:
            logger.info("disconnect()", cls=self.__class__.__name__)
            self.request_stop()
            if self._sim_thread and self._sim_thread.is_alive():
                self._sim_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._sim_thread = None
            return True
        except Exception as e:
            logger.error("disconnect() failed", cls=self.__class__.__name__, error=str(e))
            return False

    def _init_cameras(self) -> dict[str, _CameraRendererState]:
        """Create renderers for all configured cameras"""
        cam_renderers: dict[str, _CameraRendererState] = {}
        for cfg in self._camera_configs:
            cam_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_CAMERA, cfg.name)
            if cam_id < 0:
                logger.warning("Camera not found in MJCF, skipping", camera_name=cfg.name)
                continue
            rgb_renderer = mujoco.Renderer(self._model, height=cfg.height, width=cfg.width)
            depth_renderer = mujoco.Renderer(self._model, height=cfg.height, width=cfg.width)
            depth_renderer.enable_depth_rendering()
            interval = 1.0 / cfg.fps if cfg.fps > 0 else float("inf")
            cam_renderers[cfg.name] = _CameraRendererState(
                cfg=cfg,
                cam_id=cam_id,
                rgb_renderer=rgb_renderer,
                depth_renderer=depth_renderer,
                interval=interval,
            )
        return cam_renderers

    def _render_cameras(self, now: float, cam_renderers: dict[str, _CameraRendererState]) -> None:
        """Render all due cameras and store frames. Must be called from sim thread."""
        for state in cam_renderers.values():
            if now - state.last_render_time < state.interval:
                continue
            state.last_render_time = now

            state.rgb_renderer.update_scene(self._data, camera=state.cam_id)
            rgb = state.rgb_renderer.render().copy()

            state.depth_renderer.update_scene(self._data, camera=state.cam_id)
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

    @staticmethod
    def _close_cam_renderers(cam_renderers: dict[str, _CameraRendererState]) -> None:
        for state in cam_renderers.values():
            state.rgb_renderer.close()
            state.depth_renderer.close()

    def _sim_loop(self, on_started: Callable[[], None] | None = None) -> None:
        logger.info("sim loop started", cls=self.__class__.__name__)
        dt = 1.0 / self._control_frequency

        # Camera renderers: created once in the sim thread
        cam_renderers = self._init_cameras()

        def _step_once(sync_viewer: bool) -> None:
            loop_start = time.time()
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

            elapsed = time.time() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        if self._headless:
            if on_started is not None:
                on_started()
            while not self._stop_event.is_set():
                _step_once(sync_viewer=False)
        else:
            with viewer.launch_passive(
                self._model, self._data, show_left_ui=False, show_right_ui=False
            ) as m_viewer:
                if on_started is not None:
                    on_started()
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

    def get_camera_fovy(self, camera_name: str) -> float | None:
        """Get vertical field of view for a named camera, in degrees."""
        cam_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
        if cam_id < 0:
            return None
        return float(self._model.cam_fovy[cam_id])


def engine_main(
    mjcf_path: str,
    shm_key: str,
    dof: int,
    *,
    headless: bool = True,
    inject_legacy_assets: bool = True,
    odom_topic: str = "/odom",
    imu_topic: str = "/imu",
    imu_gyro_sensor_names: tuple[str, ...] = (
        "imu-pelvis-angular-velocity",
        "imu-torso-angular-velocity",
        "gyro_pelvis",
        "imu_gyro",
    ),
    imu_accel_sensor_names: tuple[str, ...] = (
        "imu-pelvis-linear-acceleration",
        "imu-torso-linear-acceleration",
        "accelerometer_pelvis",
        "imu_accel",
    ),
) -> None:
    """Standalone whole-body sim entry point.

    Runs an in-process MuJoCo engine + the whole-body SHM bridge + (optionally)
    a passive viewer, all on the main thread. ``MujocoSimModule`` spawns this
    as a subprocess when ``engine_mode='subprocess'`` is set — that way MuJoCo
    can render with ``viewer.launch_passive`` on macOS (which requires main
    thread) while dimos workers remain free to be daemonic.

    SHM layout matches ``ManipShmWriter`` so the same
    ``WholeBodyAdapter`` (sim_mujoco_g1) reads commands + writes states from
    the dimos side. The /odom + /imu LCM publishes mirror what
    ``MujocoSimModule`` does in thread mode.
    """
    # SHM writer that mirrors the in-process module's layout — the
    # dimos-side adapter reads the same buffers either way.
    shm = ManipShmWriter(shm_key)

    # Engine + asset injection (mirrors MujocoSimModule.start()).
    assets: dict[str, bytes] | None = None
    if inject_legacy_assets:
        try:
            from dimos.simulation.mujoco.model import get_assets

            assets = get_assets()
        except Exception as e:  # pragma: no cover - bare MJCFs don't need this
            logger.warning(f"engine_main: asset injection skipped: {e}")
    eng = MujocoEngine(
        config_path=Path(mjcf_path),
        headless=headless,
        cameras=[],
        assets=assets,
    )

    # Resolve IMU sensors + base-qpos slice once.
    imu_gyro_slice = _find_sensor_slice_inline(eng.model, imu_gyro_sensor_names)
    imu_accel_slice = _find_sensor_slice_inline(eng.model, imu_accel_sensor_names)
    has_freejoint = bool(
        eng.model.njnt > 0 and int(eng.model.jnt_type[0]) == int(mujoco.mjtJoint.mjJNT_FREE)
    )

    # SHM bridge — runs in the engine's sim loop.
    hooks = WholeBodySimHooks(shm, dof=dof)

    # LCM publishers (started lazily; .start() spawns the LCM thread).
    odom_tx: LCMTransport[PoseStamped] = LCMTransport(odom_topic, PoseStamped)
    odom_tx.start()
    imu_tx: LCMTransport[Imu] = LCMTransport(imu_topic, Imu)
    imu_tx.start()

    def _on_after_step(engine: MujocoEngine) -> None:
        """Composite post-step: SHM writes + LCM publishes."""
        hooks.post_step(engine)

        data = engine.data
        ts = time.time()

        # Base pose (qpos[0:7]) → /odom
        if has_freejoint:
            pos = data.qpos[0:3]
            quat = data.qpos[3:7]  # (w, x, y, z) MuJoCo convention
            odom_tx.publish(
                PoseStamped(
                    ts=ts,
                    frame_id="world",
                    position=Vector3(float(pos[0]), float(pos[1]), float(pos[2])),
                    orientation=Quaternion(
                        float(quat[1]), float(quat[2]), float(quat[3]), float(quat[0])
                    ),
                )
            )

        # IMU sensors → SHM + /imu
        if imu_gyro_slice is None and imu_accel_slice is None and not has_freejoint:
            return
        quat_tup = (
            (
                float(data.qpos[3]),
                float(data.qpos[4]),
                float(data.qpos[5]),
                float(data.qpos[6]),
            )
            if has_freejoint
            else (1.0, 0.0, 0.0, 0.0)
        )
        if imu_gyro_slice is not None:
            gyro_vals = data.sensordata[imu_gyro_slice]
            gyro_tup = (float(gyro_vals[0]), float(gyro_vals[1]), float(gyro_vals[2]))
        else:
            gyro_tup = (0.0, 0.0, 0.0)
        if imu_accel_slice is not None:
            accel_vals = data.sensordata[imu_accel_slice]
            accel_tup = (float(accel_vals[0]), float(accel_vals[1]), float(accel_vals[2]))
        else:
            accel_tup = (0.0, 0.0, 0.0)
        shm.write_imu(quaternion=quat_tup, gyroscope=gyro_tup, accelerometer=accel_tup)
        imu_tx.publish(
            Imu(
                ts=ts,
                frame_id="pelvis",
                orientation=Quaternion(quat_tup[1], quat_tup[2], quat_tup[3], quat_tup[0]),
                angular_velocity=Vector3(*gyro_tup),
                linear_acceleration=Vector3(*accel_tup),
            )
        )

    eng._on_before_step = hooks.pre_step  # type: ignore[attr-defined]
    eng._on_after_step = _on_after_step  # type: ignore[attr-defined]

    def _handle_sig(signum: int, frame: object) -> None:
        logger.info(f"engine_main: signal {signum} received, stopping")
        eng.request_stop()

    signal.signal(signal.SIGINT, _handle_sig)
    signal.signal(signal.SIGTERM, _handle_sig)

    def _mark_ready() -> None:
        shm.signal_ready(num_joints=eng.num_joints)
        logger.info(
            "engine_main: ready",
            mjcf=mjcf_path,
            shm_key=shm_key,
            dof=dof,
            headless=headless,
        )

    try:
        eng.run_blocking(on_started=_mark_ready)
    finally:
        try:
            eng.request_stop()
        except Exception as e:
            logger.warning(f"engine_main: request_stop raised: {e}")
        try:
            shm.signal_stop()
            shm.cleanup()
        except Exception as e:
            logger.warning(f"engine_main: shm cleanup raised: {e}")
        odom_tx.stop()
        imu_tx.stop()


def _find_sensor_slice_inline(
    model: mujoco.MjModel, names: tuple[str, ...], dim: int = 3
) -> slice | None:
    for n in names:
        sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, n)
        if sid >= 0:
            adr = int(model.sensor_adr[sid])
            return slice(adr, adr + dim)
    return None


__all__ = [
    "CameraConfig",
    "CameraFrame",
    "MujocoEngine",
    "StepHook",
    "engine_main",
]


if __name__ == "__main__":
    import argparse

    p = argparse.ArgumentParser(
        description="Standalone MuJoCo whole-body sim subprocess.",
        prog="python -m dimos.simulation.engines.mujoco_engine",
    )
    p.add_argument("mjcf", help="Path to MJCF XML")
    p.add_argument("shm_key", help="SHM key (matches the dimos-side adapter)")
    p.add_argument("dof", type=int, help="Number of motor DOFs")
    p.add_argument("--view", action="store_true", help="Launch passive viewer")
    p.add_argument("--no-asset-inject", action="store_true", help="Skip menagerie asset injection")
    p.add_argument("--odom-topic", default="/odom")
    p.add_argument("--imu-topic", default="/imu")
    args = p.parse_args()

    engine_main(
        mjcf_path=args.mjcf,
        shm_key=args.shm_key,
        dof=args.dof,
        headless=not args.view,
        inject_legacy_assets=not args.no_asset_inject,
        odom_topic=args.odom_topic,
        imu_topic=args.imu_topic,
    )
