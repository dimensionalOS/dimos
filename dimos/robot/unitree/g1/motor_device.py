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

"""G1 DDS device: motor commands and coherent motor/IMU feedback."""

from __future__ import annotations

from dataclasses import dataclass
import threading
from threading import Thread
import time
from typing import TYPE_CHECKING

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.components import make_humanoid_joints
from dimos.control.connection.device import Feedback
from dimos.control.contract.convert import motor_command_from_values
from dimos.hardware.whole_body.spec import POS_STOP, VEL_STOP
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.utils.crc import CRC

    from dimos.robot.unitree.g1.connection import G1ConnectionConfig

logger = setup_logger()

_NUM_MOTORS = 29
_NUM_MOTOR_SLOTS = 35  # G1 hg LowCmd has 35 slots; only 29 are used.
G1_JOINT_NAMES: list[str] = make_humanoid_joints("g1")
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


class G1MotorDevice:
    """Unitree DDS motor IO owned by a retained connection module."""

    def __init__(self, config: G1ConnectionConfig, feedback: Feedback) -> None:
        self._config = config
        self._feedback = feedback
        self._joint_names = make_humanoid_joints(config.source)

        self._publisher: ChannelPublisher | None = None
        self._subscriber: ChannelSubscriber | None = None
        self._low_cmd: LowCmd_ | None = None
        self._low_state: LowState_ | None = None
        self._crc: CRC | None = None
        # mode_machine: resolved from config at start(). We log a one-shot
        # warning if the first LowState disagrees, which is the early signal
        # that this robot variant needs a different configured value.
        self._mode_machine: int | None = None
        self._mode_machine_verified: bool = False
        # Guards _low_cmd / _low_state / _mode_machine across DDS, publish, and LCM threads.
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._publish_thread: Thread | None = None
        # Soft-start clock, armed by the first motor command after start().
        self._soft_start_t0: float | None = None
        self._soft_start_done = False

    def start(self) -> None:
        # Lazy SDK imports - file must import cleanly outside the [unitree-dds] extra.
        from unitree_sdk2py.core.channel import (
            ChannelFactoryInitialize,
            ChannelPublisher,
            ChannelSubscriber,
        )
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC

        nic = self._config.network_interface
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
        self._mode_machine = int(self._config.mode_machine)
        self._low_cmd.mode_machine = self._mode_machine
        for i in range(_NUM_MOTOR_SLOTS):
            self._low_cmd.motor_cmd[i].mode = 0x01  # enable
            self._low_cmd.motor_cmd[i].q = POS_STOP
            self._low_cmd.motor_cmd[i].kp = 0
            self._low_cmd.motor_cmd[i].dq = VEL_STOP
            self._low_cmd.motor_cmd[i].kd = 0
            self._low_cmd.motor_cmd[i].tau = 0

        self._crc = CRC()

        if self._config.release_sport_mode:
            logger.info("Releasing sport mode...")
            self._release_sport_mode()
        else:
            logger.info("Skipping sport mode release (release_sport_mode=False)")

        logger.info("G1 motor device connected", mode_machine=self._mode_machine)

        self._stop_event.clear()
        self._mode_machine_verified = False

        # Fresh soft-start every time control is (re)acquired.
        self._soft_start_t0 = None
        self._soft_start_done = False

        self._publish_thread = Thread(
            target=self._publish_loop, name="g1-wholebody-pump", daemon=True
        )
        self._publish_thread.start()

    def stop(self) -> None:
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
            with self._lock:
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

        logger.info("G1 motor device disconnected")

    def _drain_low_state(self) -> None:
        """Pull the freshest LowState frame off the subscriber and stash it."""
        sub = self._subscriber
        if sub is None:
            return
        fresh = sub.Read()
        if fresh is None:
            return
        with self._lock:
            self._low_state = fresh
        self._verify_mode_machine_once(fresh)

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
            with self._lock:
                self._mode_machine = actual
                if self._low_cmd is not None:
                    self._low_cmd.mode_machine = actual

    def _snapshot_motor_imu(self) -> G1LowStateSnapshot | None:
        """Return the latest real motor/IMU sample, or None before first LowState."""
        with self._lock:
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
        self._feedback(
            "motor_states",
            JointState(
                ts=now,
                frame_id=frame_id,
                name=self._joint_names,
                position=sample.positions,
                velocity=sample.velocities,
                effort=sample.efforts,
            ),
        )
        # Unitree reports quaternions as (w,x,y,z); Imu/Quaternion stores (x,y,z,w).
        self._feedback(
            "imu",
            _imu_from_unitree_wxyz(
                sample.quaternion,
                sample.gyroscope,
                sample.accelerometer,
                frame_id=frame_id,
                ts=now,
            ),
        )

    def _publish_loop(self) -> None:
        period = 1.0 / float(self._config.publish_rate_hz)
        next_tick = time.perf_counter()
        frame_id = self._config.frame_id

        while not self._stop_event.is_set():
            self._drain_low_state()
            sample = self._snapshot_motor_imu()
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
        duration = self._config.soft_start_seconds
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
        """Send a checked complete PD command in the G1's fixed motor order."""
        msg = motor_command_from_values(values, self._joint_names)

        with self._lock:
            if (
                self._low_cmd is None
                or self._crc is None
                or self._publisher is None
                or self._mode_machine is None
            ):
                # Pre-start or post-stop - drop silently.
                return

            # G1 firmware requires mode_machine on every LowCmd frame.
            self._low_cmd.mode_machine = self._mode_machine

            # Damping-first bring-up: kd applies in full from the first frame
            # (that is Unitree's own damp mode), while kp and tau fade in so
            # taking control never step-changes the stiffness.
            scale = self._soft_start_scale(time.perf_counter())

            for i in range(_NUM_MOTORS):
                self._low_cmd.motor_cmd[i].q = msg.q[i]
                self._low_cmd.motor_cmd[i].dq = msg.dq[i]
                self._low_cmd.motor_cmd[i].kp = msg.kp[i] * scale
                self._low_cmd.motor_cmd[i].kd = msg.kd[i]
                self._low_cmd.motor_cmd[i].tau = msg.tau[i] * scale

            self._low_cmd.crc = self._crc.Crc(self._low_cmd)
            self._publisher.Write(self._low_cmd)

    def halt(self) -> None:
        """Immediately send damping, leaving DDS open for a later command."""
        with self._lock:
            if self._publisher is None or self._low_cmd is None or self._crc is None:
                return
            for motor in self._low_cmd.motor_cmd[:_NUM_MOTORS]:
                motor.q = 0.0  # kp is zero, so the position has no stiffness effect.
                motor.dq = 0.0
                motor.kp = 0.0
                motor.kd = self._config.halt_damping
                motor.tau = 0.0
            self._low_cmd.crc = self._crc.Crc(self._low_cmd)
            self._publisher.Write(self._low_cmd)
            self._soft_start_t0 = None
            self._soft_start_done = False

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
