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

"""G1 wholebody Module.

Owns the G1 low-level DDS connection directly.

Stream interface:
  - motor_states:  Out[JointState]         29 motors, q/dq/tau in position/velocity/effort
  - imu:           Out[Imu]                quaternion + gyro + accel
  - motor_command: In[MotorCommandArray]   29-DOF q/dq/kp/kd/tau

Hardware protocol:
  - DDS topics: rt/lowstate (subscribe) + rt/lowcmd (publish)
  - IDL: unitree_hg (G1 specific; Go2 uses unitree_go)
  - mode_machine: read from first LowState, echoed back in every LowCmd
  - CRC: computed on every LowCmd via unitree_sdk2py.utils.crc.CRC
  - Sport-mode release: gated by release_sport_mode config (set False for sim/mock)

mode_machine never appears on the published JointState — it stays internal.

Motor ordering (29 joints):
  0-5:   left leg  (hip pitch/roll/yaw, knee, ankle pitch/roll)
  6-11:  right leg
  12-14: waist (yaw, roll, pitch — roll/pitch may be invalid on some variants)
  15-21: left arm  (shoulder pitch/roll/yaw, elbow, wrist roll/pitch/yaw)
  22-28: right arm
"""

from __future__ import annotations

import threading
from threading import Thread
import time
from typing import Any

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.components import make_humanoid_joints
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.hardware.whole_body.spec import POS_STOP, VEL_STOP
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_NUM_MOTORS = 29
_NUM_MOTOR_SLOTS = 35  # G1 hg LowCmd has 35 slots; only 29 are used
_MODE_MACHINE_WAIT_S = 10.0

# Joint names sourced from the canonical helper. Order matches the motor index
# convention above. Single-source-of-truth so any coordinator-side adapter built
# from make_humanoid_joints("g1") agrees on the wire-level name → motor-index mapping.
G1_JOINT_NAMES: list[str] = make_humanoid_joints("g1")
assert len(G1_JOINT_NAMES) == _NUM_MOTORS


class G1WholeBodyConnectionConfig(ModuleConfig):
    network_interface: str = Field(default="")
    release_sport_mode: bool = True
    publish_rate_hz: float = 500.0
    frame_id: str = "g1_pelvis"


class G1WholeBodyConnection(Module):
    """G1 humanoid Module — owns the DDS connection in its own worker."""

    config: G1WholeBodyConnectionConfig

    motor_command: In[MotorCommandArray]
    motor_states: Out[JointState]
    imu: Out[Imu]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

        # DDS / SDK objects — populated in start(), torn down in stop().
        self._publisher: Any = None
        self._subscriber: Any = None
        self._low_cmd: Any = None
        self._low_state: Any = None
        self._crc: Any = None

        # mode_machine must be read from first LowState and echoed back in every LowCmd.
        self._mode_machine: int | None = None

        # Guards _low_cmd, _low_state, _mode_machine across the DDS callback thread,
        # the publish loop thread, and the motor_command (LCM) callback thread.
        self._lock = threading.Lock()

        self._stop_event = threading.Event()
        self._publish_thread: Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()

        # Lazy SDK imports so the Module module file imports cleanly outside the
        # nix env / [unitree-dds] extra. Connect path:
        #   1. ChannelFactoryInitialize(0[, nic])
        #   2. publisher rt/lowcmd, subscriber rt/lowstate
        #   3. LowCmd buffer initialised with safe defaults (POS_STOP, kp=0)
        #   4. CRC computer
        #   5. (optional) MotionSwitcher release sport mode
        #   6. wait up to _MODE_MACHINE_WAIT_S for first LowState
        from unitree_sdk2py.core.channel import (
            ChannelFactoryInitialize,
            ChannelPublisher,
            ChannelSubscriber,
        )
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC

        nic = self.config.network_interface
        logger.info(f"Initializing DDS (G1 wholebody) interface={nic!r}...")
        try:
            if nic:
                ChannelFactoryInitialize(0, nic)
            else:
                ChannelFactoryInitialize(0)
        except Exception as e:
            # Idempotent: if the factory was already initialised in this process
            # (e.g. by a sibling DDS publisher in the same worker), continue.
            logger.debug(f"ChannelFactoryInitialize raised (likely already init'd): {e}")

        self._publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self._publisher.Init()

        self._subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self._subscriber.Init(self._on_low_state, 10)

        # LowCmd safe defaults — POS_STOP/VEL_STOP sentinels with zero gains so a
        # robot doesn't twitch on the first publish if commands haven't started.
        self._low_cmd = unitree_hg_msg_dds__LowCmd_()
        self._low_cmd.mode_pr = 0  # PR (pitch/roll) mode
        for i in range(_NUM_MOTOR_SLOTS):
            self._low_cmd.motor_cmd[i].mode = 0x01  # enable
            self._low_cmd.motor_cmd[i].q = POS_STOP
            self._low_cmd.motor_cmd[i].kp = 0
            self._low_cmd.motor_cmd[i].dq = VEL_STOP
            self._low_cmd.motor_cmd[i].kd = 0
            self._low_cmd.motor_cmd[i].tau = 0

        self._crc = CRC()

        if self.config.release_sport_mode:
            logger.info("Releasing sport mode...")
            self._release_sport_mode()
        else:
            logger.info("Skipping sport mode release (release_sport_mode=False)")

        logger.info("Waiting for first LowState to capture mode_machine...")
        deadline = time.time() + _MODE_MACHINE_WAIT_S
        while self._mode_machine is None and time.time() < deadline:
            time.sleep(0.1)
        if self._mode_machine is None:
            raise RuntimeError(
                f"Timed out after {_MODE_MACHINE_WAIT_S:.1f}s waiting for "
                f"first LowState — mode_machine never captured"
            )

        logger.info(f"G1WholeBodyConnection connected (mode_machine={self._mode_machine})")

        self.register_disposable(Disposable(self.motor_command.subscribe(self._on_motor_command)))

        self._publish_thread = Thread(
            target=self._publish_loop, name="g1-wholebody-pump", daemon=True
        )
        self._publish_thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._publish_thread is not None and self._publish_thread.is_alive():
            self._publish_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._publish_thread = None

        # Close DDS endpoints explicitly before dropping refs — relying on GC
        # leaves the cyclonedds participant in a state where in-flight callbacks
        # race with cleanup and segfault on process exit. Verified against
        # unitree_sdk2py/core/channel.py (Close exists at lines 267 + 288);
        # mirrors the Go2 SDK adapter's pattern in
        # dimos/hardware/drive_trains/unitree_go2/adapter.py.
        if self._subscriber is not None:
            try:
                self._subscriber.Close()
            except (OSError, RuntimeError) as e:
                logger.warning(f"ChannelSubscriber Close raised: {e}")
        if self._publisher is not None:
            try:
                self._publisher.Close()
            except (OSError, RuntimeError) as e:
                logger.warning(f"ChannelPublisher Close raised: {e}")

        self._publisher = None
        self._subscriber = None
        self._low_cmd = None
        self._low_state = None
        self._mode_machine = None
        self._crc = None

        logger.info("G1WholeBodyConnection disconnected")
        super().stop()

    def _publish_loop(self) -> None:
        period = 1.0 / float(self.config.publish_rate_hz)
        next_tick = time.perf_counter()
        frame_id = self.config.frame_id

        # Identity quaternion + zeros while LowState hasn't arrived (start() blocks
        # for it, but the publish loop may also see _low_state cleared during stop()).
        zero_quat = (1.0, 0.0, 0.0, 0.0)
        zero_vec3 = (0.0, 0.0, 0.0)

        while not self._stop_event.is_set():
            with self._lock:
                ls = self._low_state
                if ls is None:
                    positions: list[float] = [0.0] * _NUM_MOTORS
                    velocities: list[float] = [0.0] * _NUM_MOTORS
                    efforts: list[float] = [0.0] * _NUM_MOTORS
                    quat = zero_quat
                    gyro = zero_vec3
                    accel = zero_vec3
                else:
                    positions = [ls.motor_state[i].q for i in range(_NUM_MOTORS)]
                    velocities = [ls.motor_state[i].dq for i in range(_NUM_MOTORS)]
                    efforts = [ls.motor_state[i].tau_est for i in range(_NUM_MOTORS)]
                    quat = tuple(ls.imu_state.quaternion)
                    gyro = tuple(ls.imu_state.gyroscope)
                    accel = tuple(ls.imu_state.accelerometer)

            now = time.time()
            self.motor_states.publish(
                JointState(
                    ts=now,
                    frame_id=frame_id,
                    name=G1_JOINT_NAMES,
                    position=positions,
                    velocity=velocities,
                    effort=efforts,
                )
            )

            # Unitree IMU quaternion is (w, x, y, z); dimos Quaternion is (x, y, z, w).
            self.imu.publish(
                Imu(
                    ts=now,
                    frame_id=frame_id,
                    orientation=Quaternion(quat[1], quat[2], quat[3], quat[0]),
                    angular_velocity=Vector3(gyro[0], gyro[1], gyro[2]),
                    linear_acceleration=Vector3(accel[0], accel[1], accel[2]),
                )
            )

            next_tick += period
            sleep_for = next_tick - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.perf_counter()

    def _on_motor_command(self, msg: MotorCommandArray) -> None:
        if msg.num_joints != _NUM_MOTORS:
            logger.warning(f"Expected {_NUM_MOTORS} motor commands, got {msg.num_joints}; ignoring")
            return

        with self._lock:
            if (
                self._low_cmd is None
                or self._crc is None
                or self._publisher is None
                or self._mode_machine is None
            ):
                # Pre-start or post-stop — drop silently.
                return

            # Echo mode_machine from the latest LowState — required by G1 firmware.
            self._low_cmd.mode_machine = self._mode_machine

            for i in range(_NUM_MOTORS):
                self._low_cmd.motor_cmd[i].q = msg.q[i]
                self._low_cmd.motor_cmd[i].dq = msg.dq[i]
                self._low_cmd.motor_cmd[i].kp = msg.kp[i]
                self._low_cmd.motor_cmd[i].kd = msg.kd[i]
                self._low_cmd.motor_cmd[i].tau = msg.tau[i]

            self._low_cmd.crc = self._crc.Crc(self._low_cmd)
            self._publisher.Write(self._low_cmd)

    def _on_low_state(self, msg: Any) -> None:
        """rt/lowstate callback — captures mode_machine and the latest snapshot."""
        with self._lock:
            self._low_state = msg
            if self._mode_machine is None:
                self._mode_machine = msg.mode_machine

    def _release_sport_mode(self) -> None:
        """Loop ReleaseMode until MotionSwitcher reports no active controller."""
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
            MotionSwitcherClient,
        )

        msc = MotionSwitcherClient()
        msc.SetTimeout(5.0)
        msc.Init()

        # MotionSwitcher.CheckMode() returns (status, dict) while a sport
        # mode is active and (status, None) once nothing is active. Use a
        # null-tolerant check so the loop exits cleanly after the release
        # rather than crashing on `None["name"]`.
        _status, result = msc.CheckMode()
        while result and result.get("name"):
            msc.ReleaseMode()
            _status, result = msc.CheckMode()
            time.sleep(1)

        logger.info("Sport mode released — low-level control active")


__all__ = ["G1_JOINT_NAMES", "G1WholeBodyConnection", "G1WholeBodyConnectionConfig"]
