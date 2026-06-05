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

"""Go2 low-level DDS Module: rt/lowstate (sub) + rt/lowcmd (pub), unitree_go IDL.

Streams: motor_states (Out[JointState]), imu (Out[Imu]),
motor_command (In[MotorCommandArray]). 12 motors, ordering from
make_quadruped_joints("go2") (FR → FL → RR → RL, each hip → thigh → calf).

Quadruped IDL (unitree_go)
  - 12 motors / 20 LowCmd_ slots
  - No `mode_machine` field (humanoid-only) — not echoed
  - No `mode_pr` field (humanoid-only) — not set
  - `head[0..1] = 0xFE, 0xEF` + `level_flag = 0xFF` defaults required by
    the Go2 motor-controller firmware
  - MotionSwitcher release: skip if already released, otherwise loop
    until empty

Operator responsibility: put the robot in a safe pose (sat or laid down)
BEFORE starting any task that publishes targets. Module start itself does
not command motors, but a first target published against a standing
robot at high kp will lurch — the safe pattern is sync-then-arm.
"""

from __future__ import annotations

import threading
from threading import Thread
import time
from typing import TYPE_CHECKING, Any

from pydantic import Field
from reactivex.disposable import Disposable

if TYPE_CHECKING:
    from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.utils.crc import CRC

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.components import make_quadruped_joints
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

_NUM_MOTORS = 12
_NUM_MOTOR_SLOTS = 20  # Go2 LowCmd has 20 slots; only 12 are used

# PMSM motor enable byte (Unitree convention).
_MOTOR_MODE_ENABLE: int = 0x01
_MOTOR_MODE_DISABLE: int = 0x00

_FIRST_LOWSTATE_TIMEOUT_S: float = 3.0

# MotionSwitcher release loop bounds
_MSC_RPC_TIMEOUT_S: float = 5.0
_SPORT_RELEASE_DEADLINE_S: float = 30.0
_SPORT_RELEASE_POLL_S: float = 1.0

_dds_initialized: bool = False

GO2_JOINT_NAMES: list[str] = make_quadruped_joints("go2")
assert len(GO2_JOINT_NAMES) == _NUM_MOTORS


class Go2WholeBodyConnectionConfig(ModuleConfig):
    network_interface: str = Field(default="")
    release_sport_mode: bool = True
    publish_rate_hz: float = 500.0
    frame_id: str = "go2_base"


class Go2WholeBodyConnection(Module):
    """Go2 quadruped Module — owns the DDS connection in its own worker."""

    config: Go2WholeBodyConnectionConfig

    motor_command: In[MotorCommandArray]
    motor_states: Out[JointState]
    imu: Out[Imu]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

        self._publisher: ChannelPublisher | None = None
        self._subscriber: ChannelSubscriber | None = None
        self._low_cmd: LowCmd_ | None = None
        self._low_state: LowState_ | None = None
        self._crc: CRC | None = None
        # Guards _low_cmd / _low_state across DDS, publish, and motor_command threads.
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._publish_thread: Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()

        # Lazy SDK imports — file must import cleanly outside the [unitree-dds] extra.
        from unitree_sdk2py.core.channel import (
            ChannelFactoryInitialize,
            ChannelPublisher,
            ChannelSubscriber,
        )
        from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC

        global _dds_initialized
        nic = self.config.network_interface
        if not _dds_initialized:
            logger.info(f"Initializing DDS (Go2 wholebody) interface={nic!r}...")
            if nic:
                ChannelFactoryInitialize(0, nic)
            else:
                ChannelFactoryInitialize(0)
            _dds_initialized = True
        else:
            logger.info(
                f"DDS already initialized in this process; reusing existing domain (interface={nic!r} ignored)"
            )

        self._publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self._publisher.Init()

        # Passive subscriber — Read() per tick from the publish loop. Callback
        # mode is unreliable under cyclonedds on macOS.
        self._subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self._subscriber.Init(None, 0)

        # POS_STOP/VEL_STOP + zero gains so the robot can't twitch pre-command.
        # head/level_flag are Go2 firmware-required magic bytes; gpio left 0.
        self._low_cmd = unitree_go_msg_dds__LowCmd_()
        self._low_cmd.head[0] = 0xFE
        self._low_cmd.head[1] = 0xEF
        self._low_cmd.level_flag = 0xFF
        self._low_cmd.gpio = 0
        self._reset_motor_cmd_slots(_MOTOR_MODE_ENABLE)

        self._crc = CRC()

        if self.config.release_sport_mode:
            logger.info("Releasing sport mode...")
            self._release_sport_mode()
        else:
            logger.info("Skipping sport mode release (release_sport_mode=False)")

        # Drain LowState until we have a first frame — proves DDS is alive
        # before we hand out command authority on the motor rail.
        logger.info("Waiting for first LowState...")
        deadline = time.time() + _FIRST_LOWSTATE_TIMEOUT_S
        while time.time() < deadline:
            self._drain_low_state()
            with self._lock:
                if self._low_state is not None:
                    break
            time.sleep(0.05)
        else:
            raise RuntimeError(
                f"Timed out after {_FIRST_LOWSTATE_TIMEOUT_S:.1f}s waiting "
                f"for first LowState — robot offline or wrong DDS domain?"
            )

        logger.info("Go2WholeBodyConnection connected")

        self.register_disposable(Disposable(self.motor_command.subscribe(self._on_motor_command)))

        self._publish_thread = Thread(
            target=self._publish_loop, name="go2-wholebody-pump", daemon=True
        )
        self._publish_thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._publish_thread is not None and self._publish_thread.is_alive():
            self._publish_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._publish_thread = None

        # Final safe-stop lowcmd: disable every motor (mode=0x00, kp=kd=0,
        # tau=0). Without this, the motors freeze stiffly at whatever the
        # last commanded pose was. Best-effort: any failure is logged, not
        # raised, so cleanup still drains the DDS endpoints.
        if self._publisher is not None and self._low_cmd is not None and self._crc is not None:
            try:
                with self._lock:
                    self._reset_motor_cmd_slots(_MOTOR_MODE_DISABLE)
                    self._low_cmd.crc = self._crc.Crc(self._low_cmd)
                    self._publisher.Write(self._low_cmd)
                logger.info("Sent safe-stop lowcmd (motors disabled)")
            except (OSError, RuntimeError, AttributeError) as e:
                logger.warning(f"Safe-stop lowcmd failed: {e}")

        # Close DDS endpoints explicitly — GC-based cleanup races with
        # in-flight callbacks and segfaults on process exit.
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
        self._crc = None

        logger.info("Go2WholeBodyConnection disconnected")
        super().stop()

    def _publish_loop(self) -> None:
        period = 1.0 / float(self.config.publish_rate_hz)
        next_tick = time.perf_counter()
        frame_id = self.config.frame_id

        while not self._stop_event.is_set():
            self._drain_low_state()
            sample = self._snapshot_motor_imu()
            if sample is not None:
                positions, velocities, efforts, quat, gyro, accel = sample
                self._publish_motor_state_and_imu(
                    now=time.time(),
                    frame_id=frame_id,
                    positions=positions,
                    velocities=velocities,
                    efforts=efforts,
                    quat=quat,
                    gyro=gyro,
                    accel=accel,
                )

            next_tick += period
            sleep_for = next_tick - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.perf_counter()

    def _drain_low_state(self) -> None:
        sub = self._subscriber
        if sub is None:
            return
        fresh = sub.Read()
        if fresh is None:
            return
        with self._lock:
            self._low_state = fresh

    def _snapshot_motor_imu(
        self,
    ) -> (
        tuple[
            list[float],
            list[float],
            list[float],
            tuple[float, float, float, float],
            tuple[float, float, float],
            tuple[float, float, float],
        ]
        | None
    ):
        with self._lock:
            ls = self._low_state
            if ls is None:
                return None
            return (
                [ls.motor_state[i].q for i in range(_NUM_MOTORS)],
                [ls.motor_state[i].dq for i in range(_NUM_MOTORS)],
                [ls.motor_state[i].tau_est for i in range(_NUM_MOTORS)],
                tuple(ls.imu_state.quaternion),
                tuple(ls.imu_state.gyroscope),
                tuple(ls.imu_state.accelerometer),
            )

    def _publish_motor_state_and_imu(
        self,
        now: float,
        frame_id: str,
        positions: list[float],
        velocities: list[float],
        efforts: list[float],
        quat: tuple[float, float, float, float],
        gyro: tuple[float, float, float],
        accel: tuple[float, float, float],
    ) -> None:
        self.motor_states.publish(
            JointState(
                ts=now,
                frame_id=frame_id,
                name=GO2_JOINT_NAMES,
                position=positions,
                velocity=velocities,
                effort=efforts,
            )
        )
        # Unitree quat is (w,x,y,z); dimos Quaternion is (x,y,z,w).
        self.imu.publish(
            Imu(
                ts=now,
                frame_id=frame_id,
                orientation=Quaternion(quat[1], quat[2], quat[3], quat[0]),
                angular_velocity=Vector3(gyro[0], gyro[1], gyro[2]),
                linear_acceleration=Vector3(accel[0], accel[1], accel[2]),
            )
        )

    def _on_motor_command(self, msg: MotorCommandArray) -> None:
        if msg.num_joints != _NUM_MOTORS:
            logger.warning(f"Expected {_NUM_MOTORS} motor commands, got {msg.num_joints}; ignoring")
            return

        with self._lock:
            if self._low_cmd is None or self._crc is None or self._publisher is None:
                # Pre-start or post-stop — drop silently.
                return

            for i in range(_NUM_MOTORS):
                self._low_cmd.motor_cmd[i].mode = _MOTOR_MODE_ENABLE
                self._low_cmd.motor_cmd[i].q = msg.q[i]
                self._low_cmd.motor_cmd[i].dq = msg.dq[i]
                self._low_cmd.motor_cmd[i].kp = msg.kp[i]
                self._low_cmd.motor_cmd[i].kd = msg.kd[i]
                self._low_cmd.motor_cmd[i].tau = msg.tau[i]

            self._low_cmd.crc = self._crc.Crc(self._low_cmd)
            self._publisher.Write(self._low_cmd)

    def _reset_motor_cmd_slots(self, mode: int) -> None:
        """Reset all 20 motor_cmd slots in self._low_cmd to a safe baseline.

        Sets every slot to ``mode`` with POS_STOP/VEL_STOP sentinels and zero
        gains/torque. Caller is responsible for holding ``self._lock`` and for
        ensuring ``self._low_cmd is not None``.
        """
        assert self._low_cmd is not None
        for i in range(_NUM_MOTOR_SLOTS):
            self._low_cmd.motor_cmd[i].mode = mode
            self._low_cmd.motor_cmd[i].q = POS_STOP
            self._low_cmd.motor_cmd[i].dq = VEL_STOP
            self._low_cmd.motor_cmd[i].kp = 0
            self._low_cmd.motor_cmd[i].kd = 0
            self._low_cmd.motor_cmd[i].tau = 0

    def _release_sport_mode(self) -> None:
        """Loop ReleaseMode until MotionSwitcher reports no active controller.

        Bails early if the first CheckMode reports nothing active. Calling
        ReleaseMode anyway opens a window where motor controllers are
        mid-handoff while we're already publishing rt/lowcmd, which has
        been observed to cause mechanical noise from the gearboxes.
        """
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
            MotionSwitcherClient,
        )

        msc = MotionSwitcherClient()
        msc.SetTimeout(_MSC_RPC_TIMEOUT_S)
        msc.Init()

        _status, result = msc.CheckMode()
        if not result or not result.get("name"):
            logger.info("Sport mode already released — skipping ReleaseMode")
            return

        deadline = time.time() + _SPORT_RELEASE_DEADLINE_S
        while result and result.get("name"):
            if time.time() > deadline:
                raise RuntimeError(
                    f"MotionSwitcher.ReleaseMode did not clear active controller "
                    f"{result.get('name')!r} within {_SPORT_RELEASE_DEADLINE_S:.0f}s "
                )
            msc.ReleaseMode()
            _status, result = msc.CheckMode()
            time.sleep(_SPORT_RELEASE_POLL_S)

        logger.info("Sport mode released — low-level control active")


__all__ = [
    "GO2_JOINT_NAMES",
    "Go2WholeBodyConnection",
    "Go2WholeBodyConnectionConfig",
]
