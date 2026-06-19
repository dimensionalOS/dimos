# Copyright 2026 Dimensional Inc.
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

"""MuJoCo-backed `WholeBodyAdapter` for the Go2 quadruped.

Loads the vendored MJCF at `data/go2_mjlab/xmls/scene_go2.xml`, runs the
physics step in a background thread at the MJCF's native step rate
(0.005 s -> 200 Hz), and exposes the standard `WholeBodyAdapter` interface
so the same coordinator/task stack works against sim or a future DDS adapter.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import threading
import time

import numpy as np

from dimos.hardware.whole_body.spec import (
    IMUState,
    MotorCommand,
    MotorState,
    WholeBodyAdapter,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


_DEFAULT_MJCF: Path = (
    Path(__file__).resolve().parents[3] / "data" / "go2_mjlab" / "xmls" / "scene_go2.xml"
)


@dataclass
class MujocoGo2Config:
    mjcf_path: Path = _DEFAULT_MJCF
    step_period: float = 0.005  # 200Hz matches training sim
    keyframe_name: str = "lie"
    render: bool = False


# Wire order (matches Unitree LowCmd_.motor_cmd[0..11]). Short names; connect()
# appends '_joint' for MJCF joint lookups.
GO2_ACTUATOR_ORDER: tuple[str, ...] = (
    "FR_hip",
    "FR_thigh",
    "FR_calf",
    "FL_hip",
    "FL_thigh",
    "FL_calf",
    "RR_hip",
    "RR_thigh",
    "RR_calf",
    "RL_hip",
    "RL_thigh",
    "RL_calf",
)


class MujocoGo2WholeBody(WholeBodyAdapter):
    """Sim adapter exposing the Go2 as a `WholeBodyAdapter`.

    Threading: integration runs on a daemon thread; reads/writes are guarded
    by a single lock. The PD loop (kp/kd/q/dq/tau -> ctrl) is applied inside
    the step thread using the most recent command.
    """

    def __init__(self, config: MujocoGo2Config | None = None) -> None:
        self.config = config or MujocoGo2Config()
        self._mj_model = None
        self._mj_data = None
        self._actuator_ids: list[int] = []
        self._qpos_ids: list[int] = []  # qpos index per actuator joint
        self._qvel_ids: list[int] = []
        self._imu_sensor_ids: dict[str, int] = {}
        self._latest_cmd: list[MotorCommand] = [MotorCommand() for _ in GO2_ACTUATOR_ORDER]
        self._has_states = False
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._step_thread: threading.Thread | None = None
        self._viewer_thread: threading.Thread | None = None

    # --- WholeBodyAdapter Protocol -------------------------------------------

    def connect(self) -> bool:
        import mujoco

        path = Path(self.config.mjcf_path)
        if not path.exists():
            logger.error(f"MJCF not found at {path}")
            return False

        self._mj_model = mujoco.MjModel.from_xml_path(str(path))
        self._mj_data = mujoco.MjData(self._mj_model)

        for short_name in GO2_ACTUATOR_ORDER:
            act_id = mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_ACTUATOR, short_name)
            if act_id < 0:
                logger.error(f"Actuator {short_name!r} not found in MJCF")
                return False
            jnt_name = f"{short_name}_joint"
            jnt_id = mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_JOINT, jnt_name)
            if jnt_id < 0:
                logger.error(f"Joint {jnt_name!r} not found in MJCF")
                return False
            self._actuator_ids.append(act_id)
            self._qpos_ids.append(int(self._mj_model.jnt_qposadr[jnt_id]))
            self._qvel_ids.append(int(self._mj_model.jnt_dofadr[jnt_id]))

        for s in ("imu_quat", "imu_gyro", "imu_acc"):
            sid = mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_SENSOR, s)
            if sid < 0:
                logger.error(f"Sensor {s!r} not found")
                return False
            self._imu_sensor_ids[s] = sid

        mujoco.mj_resetData(self._mj_model, self._mj_data)
        key_id = mujoco.mj_name2id(
            self._mj_model, mujoco.mjtObj.mjOBJ_KEY, self.config.keyframe_name
        )
        if key_id >= 0:
            n = int(self._mj_model.key_qpos.shape[1])
            # Robot freejoint (7) + 12 leg DOFs; rest of qpos is scene objects.
            robot_qpos_len = min(19, n)
            self._mj_data.qpos[:robot_qpos_len] = self._mj_model.key_qpos[key_id, :robot_qpos_len]
            # Seed PD with the keyframe's targets so the step loop holds spawn
            # pose from tick 0; caller's first command overrides kp/kd.
            if (
                self._mj_model.key_ctrl is not None
                and self._mj_model.key_ctrl.shape[1] >= self._mj_model.nu
            ):
                for i, act_id in enumerate(self._actuator_ids):
                    self._latest_cmd[i] = MotorCommand(
                        q=float(self._mj_model.key_ctrl[key_id, act_id]),
                        dq=0.0,
                        kp=50.0,
                        kd=3.5,
                        tau=0.0,
                    )
        else:
            logger.warning(f"Keyframe {self.config.keyframe_name!r} missing - using default qpos")

        mujoco.mj_forward(self._mj_model, self._mj_data)
        self._has_states = True
        logger.info(
            f"Spawn pose: base_z={float(self._mj_data.qpos[2]):.3f} "
            f"joints={[round(float(self._mj_data.qpos[7 + i]), 2) for i in range(12)]}"
        )

        self._stop_event.clear()
        self._step_thread = threading.Thread(
            target=self._step_loop, name="mujoco-go2-step", daemon=True
        )
        self._step_thread.start()

        if self.config.render:
            self._viewer_thread = threading.Thread(
                target=self._viewer_loop, name="mujoco-go2-viewer", daemon=True
            )
            self._viewer_thread.start()

        logger.info(f"MujocoGo2WholeBody connected ({path.name})")
        return True

    def disconnect(self) -> None:
        self._stop_event.set()
        if self._step_thread is not None and self._step_thread.is_alive():
            self._step_thread.join(timeout=1.0)
            self._step_thread = None
        if self._viewer_thread is not None and self._viewer_thread.is_alive():
            self._viewer_thread.join(timeout=1.0)
            self._viewer_thread = None
        self._mj_model = None
        self._mj_data = None
        self._has_states = False
        logger.info("MujocoGo2WholeBody disconnected")

    def is_connected(self) -> bool:
        return (
            self._mj_data is not None
            and self._step_thread is not None
            and self._step_thread.is_alive()
        )

    def read_motor_states(self) -> list[MotorState]:
        with self._lock:
            if self._mj_data is None:
                return [MotorState() for _ in GO2_ACTUATOR_ORDER]
            qpos = self._mj_data.qpos
            qvel = self._mj_data.qvel
            qfrc = self._mj_data.qfrc_actuator
            return [
                MotorState(
                    q=float(qpos[self._qpos_ids[i]]),
                    dq=float(qvel[self._qvel_ids[i]]),
                    tau=float(qfrc[self._qvel_ids[i]]),
                )
                for i in range(len(GO2_ACTUATOR_ORDER))
            ]

    def has_motor_states(self) -> bool:
        return self._has_states

    def read_imu(self) -> IMUState:
        import mujoco

        with self._lock:
            if self._mj_data is None:
                return IMUState()
            sdata = self._mj_data.sensordata
            adr = self._mj_model.sensor_adr
            quat = tuple(
                float(x)
                for x in sdata[
                    adr[self._imu_sensor_ids["imu_quat"]] : adr[self._imu_sensor_ids["imu_quat"]]
                    + 4
                ]
            )
            gyro = tuple(
                float(x)
                for x in sdata[
                    adr[self._imu_sensor_ids["imu_gyro"]] : adr[self._imu_sensor_ids["imu_gyro"]]
                    + 3
                ]
            )
            acc = tuple(
                float(x)
                for x in sdata[
                    adr[self._imu_sensor_ids["imu_acc"]] : adr[self._imu_sensor_ids["imu_acc"]] + 3
                ]
            )
            rpy = np.zeros(3)
            mujoco.mju_quat2Vel(
                rpy, np.array(quat, dtype=np.float64), 1.0
            )  # quat -> rotvec, not RPY; fall through
        return IMUState(
            quaternion=quat,  # (w, x, y, z)
            gyroscope=gyro,
            accelerometer=acc,
            rpy=(float(rpy[0]), float(rpy[1]), float(rpy[2])),
        )

    def write_motor_commands(self, commands: list[MotorCommand]) -> bool:
        if len(commands) != len(GO2_ACTUATOR_ORDER):
            return False
        with self._lock:
            self._latest_cmd = list(commands)
        return True

    # --- Internals -----------------------------------------------------------

    def _step_loop(self) -> None:
        import mujoco

        period = float(self.config.step_period)
        next_tick = time.perf_counter()

        while not self._stop_event.is_set():
            with self._lock:
                if self._mj_data is None:
                    break
                # tau = kp*(q_des - q) + kd*(dq_des - dq) + tau_ff.
                from dimos.hardware.whole_body.spec import POS_STOP, VEL_STOP

                for i, cmd in enumerate(self._latest_cmd):
                    if cmd.q == POS_STOP and cmd.dq == VEL_STOP and cmd.tau == 0.0:
                        self._mj_data.ctrl[self._actuator_ids[i]] = 0.0
                        continue
                    q = self._mj_data.qpos[self._qpos_ids[i]]
                    dq = self._mj_data.qvel[self._qvel_ids[i]]
                    q_des = cmd.q if cmd.q != POS_STOP else q
                    dq_des = cmd.dq if cmd.dq != VEL_STOP else 0.0
                    tau = cmd.kp * (q_des - q) + cmd.kd * (dq_des - dq) + cmd.tau
                    self._mj_data.ctrl[self._actuator_ids[i]] = tau

                mujoco.mj_step(self._mj_model, self._mj_data)

            next_tick += period
            sleep_for = next_tick - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.perf_counter()

    def _viewer_loop(self) -> None:
        from mujoco import viewer

        with viewer.launch_passive(self._mj_model, self._mj_data) as v:
            while not self._stop_event.is_set() and v.is_running():
                with self._lock:
                    v.sync()
                time.sleep(1.0 / 60.0)


__all__ = ["GO2_ACTUATOR_ORDER", "MujocoGo2Config", "MujocoGo2WholeBody"]
