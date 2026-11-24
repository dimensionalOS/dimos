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

"""G1 low-level DDS Module: rt/lowstate (sub) + rt/lowcmd (pub), unitree_hg IDL.

Streams: joint_state (Out[JointState]), imu (Out[Imu]),
position_command (In[JointState]). 29 motors, ordering from
make_humanoid_joints("g1") (left leg -> right leg -> waist -> left arm -> right arm).
"""

from __future__ import annotations

from dataclasses import dataclass
import threading
from threading import Thread
import time
from typing import TYPE_CHECKING, Annotated, Any

from pydantic import Field
from reactivex.disposable import Disposable

if TYPE_CHECKING:
    from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.utils.crc import CRC

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.connection.connection_module import ConnectionConfig, ConnectionModule
from dimos.control.contract.convert import joint_state_to_values
from dimos.control.contract.keys import Interface
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.hardware.whole_body.spec import POS_STOP, VEL_STOP
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel
from dimos.robot.unitree.g1.config import G1_JOINT_MODEL, G1_JOINT_NAMES
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_NUM_MOTORS = 29
_NUM_MOTOR_SLOTS = 35  # G1 hg LowCmd has 35 slots; only 29 are used
# Default mode_machine for the 29-DOF G1 gear variant. Users can override
# G1WholeBodyConnectionConfig.mode_machine for other firmware/hardware
# variants.
_MODE_MACHINE_G1: int = 5

# Joint names sourced from the canonical helper. Order matches the motor index
# convention above. The model and command encoder use this same name -> motor-index mapping.
assert len(G1_JOINT_NAMES) == _NUM_MOTORS


def _imu_from_unitree_wxyz(
    quaternion: tuple[float, float, float, float],
    gyroscope: tuple[float, float, float],
    accelerometer: tuple[float, float, float],
    *,
    frame_id: str,
    ts: float,
) -> Imu:
    _w, x, y, z = quaternion
    return Imu(
        orientation=Quaternion(x, y, z, _w),
        angular_velocity=Vector3(*gyroscope),
        linear_acceleration=Vector3(*accelerometer),
        frame_id=frame_id,
        ts=ts,
    )


_Gain = Annotated[float, Field(ge=0.0, allow_inf_nan=False)]


class G1WholeBodyConnectionConfig(ConnectionConfig):
    source: str = "g1"
    model: RobotModel | None = G1_JOINT_MODEL
    joint_commands: dict[str, tuple[Interface, ...]] = {
        name: (Interface.POSITION,) for name in G1_JOINT_NAMES
    }
    kp: tuple[_Gain, ...] = Field(default=(40.0,) * _NUM_MOTORS, min_length=29, max_length=29)
    kd: tuple[_Gain, ...] = Field(default=(3.0,) * _NUM_MOTORS, min_length=29, max_length=29)
    feedback_timeout_s: float = Field(default=0.2, gt=0.0, allow_inf_nan=False)
    network_interface: str = Field(default="")
    release_sport_mode: bool = True
    publish_rate_hz: float = 500.0
    frame_id: str = "g1_pelvis"
    mode_machine: int = _MODE_MACHINE_G1
    # Stiffness soft-start on taking low-level control: the first commands go
    # out with full damping (kd) but kp and tau scaled in over this window,
    # mirroring the remote's damp -> lock-stand feel. Full commanded stiffness
    # in the very first frame slams the robot from wherever it hangs to the
    # commanded pose. 0 disables.
    soft_start_seconds: float = Field(default=3.0, ge=0.0, allow_inf_nan=False)


@dataclass(frozen=True)
class G1LowStateSnapshot:
    """Motor and IMU data copied from one G1 LowState frame.

    Attributes:
        positions: Motor positions for the 29 controllable G1 motors.
        velocities: Motor velocities for the 29 controllable G1 motors.
        efforts: Estimated motor torques for the 29 controllable G1 motors.
        quaternion: IMU orientation in Unitree/MuJoCo w,x,y,z order.
        gyroscope: IMU angular velocity in rad/s.
        accelerometer: IMU linear acceleration in m/s^2.
    """

    positions: list[float]
    velocities: list[float]
    efforts: list[float]
    quaternion: tuple[float, float, float, float]
    gyroscope: tuple[float, float, float]
    accelerometer: tuple[float, float, float]


class G1WholeBodyConnection(ConnectionModule):
    """G1 humanoid Module - owns the DDS connection in its own worker."""

    config: G1WholeBodyConnectionConfig

    position_command: In[JointState]
    joint_state: Out[JointState]
    imu: Out[Imu]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

        self._publisher: ChannelPublisher | None = None
        self._subscriber: ChannelSubscriber | None = None
        self._low_cmd: LowCmd_ | None = None
        self._low_state: LowState_ | None = None
        self._state_received_at = float("-inf")
        self._crc: CRC | None = None
        # mode_machine: resolved from config at start(). We log a one-shot
        # warning if the first LowState disagrees, which is the early signal
        # that this robot variant needs a different configured value.
        self._mode_machine: int | None = None
        self._mode_machine_verified: bool = False
        # Guards _low_cmd / _low_state / _mode_machine across DDS, publish, and LCM threads.
        self._dds_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._publish_thread: Thread | None = None
        # Soft-start clock, armed by the first motor command after start().
        self._soft_start_t0: float | None = None
        self._soft_start_done = False

    @rpc
    def start(self) -> None:
        super().start()

        self.register_disposable(
            Disposable(
                self.position_command.subscribe(
                    lambda msg: self.command(joint_state_to_values(msg, (Interface.POSITION,)))
                )
            )
        )

    def connect(self) -> None:
        self._stop_event.clear()
        self._state_received_at = float("-inf")
        self._mode_machine_verified = False
        # Lazy SDK imports - file must import cleanly outside the [unitree-dds] extra.
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
            # Idempotent - already initialised by a sibling participant is fine.
            logger.debug(f"ChannelFactoryInitialize raised (likely already init'd): {e}")

        self._publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self._publisher.Init()

        # Passive subscriber - Read() per tick from the publish loop.  The
        # callback variant (Init(self._on_low_state, 10)) doesn't fire
        # reliably under cyclonedds on macOS, which used to leave us
        # blocked here forever waiting for a first LowState.
        self._subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self._subscriber.Init(None, 0)

        # POS_STOP/VEL_STOP + zero gains so the robot can't twitch pre-command.
        self._low_cmd = unitree_hg_msg_dds__LowCmd_()
        self._low_cmd.mode_pr = 0  # PR (pitch/roll) mode
        # mode_machine is a static value for a given robot variant.
        self._mode_machine = int(self.config.mode_machine)
        self._low_cmd.mode_machine = self._mode_machine
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

        logger.info("G1WholeBodyConnection connected", mode_machine=self._mode_machine)

        # Fresh soft-start every time control is (re)acquired.
        self._soft_start_t0 = None
        self._soft_start_done = False

        self._publish_thread = Thread(
            target=self._publish_loop, name="g1-wholebody-pump", daemon=True
        )
        self._publish_thread.start()

    @rpc
    def stop(self) -> None:
        super().stop()
        self._stop_event.set()
        if self._publish_thread is not None and self._publish_thread.is_alive():
            self._publish_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._publish_thread = None

        # Final safe-stop lowcmd: disable every motor (mode=0x00, kp=kd=0,
        # tau=0).  Without this, the motors freeze stiffly at whatever
        # the last commanded pose was and the next ``dimos run`` opens
        # against a robot that's actively fighting its own controllers
        # - observed as horrible mechanical noise during sport-mode
        # release.  Best-effort: any failure is logged, not raised, so
        # cleanup still drains the DDS endpoints.
        if self._publisher is not None and self._low_cmd is not None and self._crc is not None:
            sent_safe_stop = False
            with self._dds_lock:
                for i in range(_NUM_MOTOR_SLOTS):
                    self._low_cmd.motor_cmd[i].mode = 0x00  # disable
                    self._low_cmd.motor_cmd[i].q = POS_STOP
                    self._low_cmd.motor_cmd[i].dq = VEL_STOP
                    self._low_cmd.motor_cmd[i].kp = 0
                    self._low_cmd.motor_cmd[i].kd = 0
                    self._low_cmd.motor_cmd[i].tau = 0
                self._low_cmd.crc = self._crc.Crc(self._low_cmd)
                try:
                    self._publisher.Write(self._low_cmd)
                    sent_safe_stop = True
                except (OSError, RuntimeError) as e:
                    logger.warning("Safe-stop lowcmd failed", error=str(e))
            if sent_safe_stop:
                logger.info("Sent safe-stop lowcmd (motors disabled)")

        # Close DDS endpoints explicitly - GC-based cleanup races with in-flight
        # callbacks and segfaults on process exit (mirrors the Go2 adapter).
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

    def _drain_low_state(self) -> bool:
        """Pull the freshest LowState frame off the subscriber and stash it."""
        sub = self._subscriber
        if sub is None:
            return False
        fresh = sub.Read()
        if fresh is None:
            return False
        with self._dds_lock:
            self._low_state = fresh
            self._state_received_at = time.monotonic()
        self._verify_mode_machine_once(fresh)
        return True

    def _verify_mode_machine_once(self, sample: LowState_) -> None:
        """Adopt the firmware-reported mode_machine if it differs from config.

        Commands with a wrong mode_machine are silently rejected, and the
        reported value varies per boot on some G1 units, so the configured
        value is only a default until the first LowState tells us the truth.
        """
        if self._mode_machine_verified:
            return
        self._mode_machine_verified = True
        actual = int(sample.mode_machine)
        if actual != self._mode_machine:
            logger.warning(
                "mode_machine mismatch; adopting the firmware-reported value "
                "(commands with the configured one would be silently rejected).",
                configured=self._mode_machine,
                reported=actual,
            )
            with self._dds_lock:
                self._mode_machine = actual
                if self._low_cmd is not None:
                    self._low_cmd.mode_machine = actual

    def _snapshot_motor_imu(self) -> G1LowStateSnapshot | None:
        """Return the latest real motor/IMU sample, or None before first LowState."""
        with self._dds_lock:
            ls = self._low_state
            if ls is None:
                return None
            return G1LowStateSnapshot(
                positions=[float(ls.motor_state[i].q) for i in range(_NUM_MOTORS)],
                velocities=[float(ls.motor_state[i].dq) for i in range(_NUM_MOTORS)],
                efforts=[float(ls.motor_state[i].tau_est) for i in range(_NUM_MOTORS)],
                quaternion=(
                    float(ls.imu_state.quaternion[0]),
                    float(ls.imu_state.quaternion[1]),
                    float(ls.imu_state.quaternion[2]),
                    float(ls.imu_state.quaternion[3]),
                ),
                gyroscope=(
                    float(ls.imu_state.gyroscope[0]),
                    float(ls.imu_state.gyroscope[1]),
                    float(ls.imu_state.gyroscope[2]),
                ),
                accelerometer=(
                    float(ls.imu_state.accelerometer[0]),
                    float(ls.imu_state.accelerometer[1]),
                    float(ls.imu_state.accelerometer[2]),
                ),
            )

    def _publish_motor_state_and_imu(
        self, now: float, frame_id: str, sample: G1LowStateSnapshot
    ) -> None:
        self.joint_state.publish(
            JointState(
                ts=now,
                frame_id=frame_id,
                name=G1_JOINT_NAMES,
                position=sample.positions,
                velocity=sample.velocities,
                effort=sample.efforts,
            )
        )
        # Unitree reports quaternions as (w,x,y,z); Imu/Quaternion stores (x,y,z,w).
        self.imu.publish(
            _imu_from_unitree_wxyz(
                sample.quaternion,
                sample.gyroscope,
                sample.accelerometer,
                frame_id=frame_id,
                ts=now,
            )
        )

    def _publish_loop(self) -> None:
        period = 1.0 / float(self.config.publish_rate_hz)
        next_tick = time.perf_counter()
        frame_id = self.config.frame_id

        while not self._stop_event.is_set():
            fresh = self._drain_low_state()
            sample = self._snapshot_motor_imu() if fresh else None
            if sample is not None:
                self._publish_motor_state_and_imu(now=time.time(), frame_id=frame_id, sample=sample)

            next_tick += period
            sleep_for = next_tick - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.perf_counter()

    def _soft_start_scale(self, now: float) -> float:
        """Stiffness scale in [0, 1] for this command frame. Caller holds the lock."""
        duration = self.config.soft_start_seconds
        if duration <= 0.0:
            return 1.0
        if self._soft_start_t0 is None:
            self._soft_start_t0 = now
            logger.info(
                "Soft-start: full damping now, stiffness ramping in",
                duration_seconds=duration,
            )
        scale = min(1.0, (now - self._soft_start_t0) / duration)
        if scale >= 1.0 and not self._soft_start_done:
            self._soft_start_done = True
            logger.info("Soft-start complete - full commanded stiffness")
        return scale

    def write(self, values: dict[str, float]) -> None:
        keys = [f"{name}/position" for name in G1_JOINT_NAMES]
        if not all(key in values for key in keys):
            raise ValueError("G1 requires a complete 29-joint position command")
        with self._dds_lock:
            if (
                self._low_state is None
                or time.monotonic() - self._state_received_at > self.config.feedback_timeout_s
            ):
                raise ValueError("G1 requires fresh measured low-state before commanding motors")
            if (
                self._low_cmd is None
                or self._crc is None
                or self._publisher is None
                or self._mode_machine is None
            ):
                raise RuntimeError("G1 DDS is not connected")
            self._low_cmd.mode_machine = self._mode_machine
            # Full damping from the first frame; stiffness fades in. The public
            # interface is position only, so desired velocity and feedforward
            # torque are zero, not a second motor-command protocol.
            scale = self._soft_start_scale(time.perf_counter())
            for i, key in enumerate(keys):
                motor = self._low_cmd.motor_cmd[i]
                motor.q, motor.dq = values[key], 0.0
                motor.kp, motor.kd = self.config.kp[i] * scale, self.config.kd[i]
                motor.tau = 0.0
            self._low_cmd.crc = self._crc.Crc(self._low_cmd)
            self._publisher.Write(self._low_cmd)

    def halt(self) -> None:
        """Request firmware damping, not torque disable; never replay a target."""
        with self._dds_lock:
            self._soft_start_t0 = None
            self._soft_start_done = False
            if self._low_cmd is None or self._crc is None or self._publisher is None:
                return
            for i in range(_NUM_MOTORS):
                motor = self._low_cmd.motor_cmd[i]
                motor.q, motor.dq = POS_STOP, 0.0
                motor.kp, motor.kd, motor.tau = 0.0, self.config.kd[i], 0.0
            self._low_cmd.crc = self._crc.Crc(self._low_cmd)
            self._publisher.Write(self._low_cmd)

    def _release_sport_mode(self) -> None:
        """Loop ReleaseMode until MotionSwitcher reports no active controller.

        Bails early if the first CheckMode reports nothing active.  That
        matters for back-to-back ``dimos run`` invocations: the first run
        already released sport mode, so on a clean second start there's
        nothing to release.  Calling ReleaseMode anyway opens a window
        where motor controllers are mid-handoff while we're already
        publishing rt/lowcmd, which has been observed to cause horrible
        mechanical noise from the gearboxes.
        """
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
            MotionSwitcherClient,
        )

        msc = MotionSwitcherClient()
        msc.SetTimeout(5.0)
        msc.Init()

        # CheckMode returns (status, None) - or (status, {"name": ""}) on
        # some firmwares - once nothing is active.  Treat both as "already
        # released" and return without poking ReleaseMode.
        _status, result = msc.CheckMode()
        if not result or not result.get("name"):
            logger.info("Sport mode already released - skipping ReleaseMode")
            return

        while result and result.get("name"):
            msc.ReleaseMode()
            _status, result = msc.CheckMode()
            time.sleep(1)

        logger.info("Sport mode released - low-level control active")
