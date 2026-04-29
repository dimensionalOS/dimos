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

"""Unitree G1 low-level adapter -- direct 29-DOF joint control over DDS.

Uses ``rt/lowcmd`` / ``rt/lowstate`` DDS topics for motor-level
position/velocity/torque control, bypassing the high-level LocoClient.

Important: The G1 must first exit sport mode (via MotionSwitcherClient)
before low-level commands are accepted.

Motor ordering (29 joints):
  0-5:   Left leg  (HipPitch, HipRoll, HipYaw, Knee, AnklePitch, AnkleRoll)
  6-11:  Right leg
  12:    WaistYaw
  13-14: WaistRoll, WaistPitch (may be invalid on some variants)
  15-21: Left arm  (ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow,
                    WristRoll, WristPitch, WristYaw)
  22-28: Right arm

G1-specific protocol details:
  - Uses ``unitree_hg`` IDL types (not ``unitree_go`` like the Go2)
  - LowCmd has ``mode_pr`` and ``mode_machine`` fields instead of head/level_flag
  - ``mode_machine`` must be read from LowState and echoed back in every LowCmd
  - Motor array has 35 slots (only 29 are used)
"""

from __future__ import annotations

import logging
import threading
import time
from typing import TYPE_CHECKING

from dimos.hardware.whole_body.spec import (
    POS_STOP,
    VEL_STOP,
    IMUState,
    MotorCommand,
    MotorState,
)

if TYPE_CHECKING:
    from dimos.hardware.whole_body.registry import WholeBodyAdapterRegistry

logger = logging.getLogger(__name__)

_NUM_MOTORS = 29
_NUM_MOTOR_SLOTS = 35


class UnitreeG1LowLevelAdapter:
    """WholeBodyAdapter implementation for Unitree G1 -- low-level DDS.

    The coordinator's tick loop drives the publish cadence.  Each call to
    ``write_motor_commands()`` updates the ``LowCmd_`` buffer, computes
    CRC, and publishes immediately -- no background thread needed.

    Args:
        network_interface: DDS network interface name (e.g., "eth0"). Empty string
            means let CycloneDDS pick the default interface — matches the Go2
            adapter's convention. The cyclonedds Python binding requires a string;
            int values like 0 trigger AttributeError on participant init.
        release_sport_mode: If True (default), call MotionSwitcher to exit sport mode
            during connect(). Set False for mock/sim backends that don't run MotionSwitcher.
    """

    def __init__(
        self,
        network_interface: str = "",
        release_sport_mode: bool = True,
        **_: object,
    ) -> None:
        self._network_interface = network_interface
        self._release_sport_mode_flag = release_sport_mode

        self._connected = False
        self._lock = threading.Lock()

        # SDK objects (lazy-imported on connect)
        self._low_cmd = None
        self._publisher = None
        self._subscriber = None
        self._crc = None

        # Latest feedback
        self._low_state = None

        # mode_machine must be read from first LowState and echoed back
        self._mode_machine: int | None = None

    # =========================================================================
    # Connection
    # =========================================================================

    def connect(self) -> bool:
        """Connect to G1 and release sport mode for low-level control."""
        try:
            from unitree_sdk2py.core.channel import (
                ChannelFactoryInitialize,
                ChannelPublisher,
                ChannelSubscriber,
            )
            from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
            from unitree_sdk2py.utils.crc import CRC

            # 1. Initialise DDS transport. Pass NIC only if non-empty — passing
            # an int or empty arg trips cyclonedds' participant init.
            logger.info(f"Initializing DDS (G1 low-level) interface={self._network_interface!r}...")
            try:
                if self._network_interface:
                    ChannelFactoryInitialize(0, self._network_interface)
                else:
                    ChannelFactoryInitialize(0)
            except Exception as e:
                logger.debug(f"ChannelFactoryInitialize raised (likely already init'd): {e}")

            # 2. Create publisher / subscriber
            self._publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
            self._publisher.Init()

            self._subscriber = ChannelSubscriber("rt/lowstate", LowState_)
            self._subscriber.Init(self._on_low_state, 10)

            # 3. Initialise LowCmd with safe defaults
            self._low_cmd = unitree_hg_msg_dds__LowCmd_()
            self._low_cmd.mode_pr = 0  # PR mode (Pitch/Roll)
            for i in range(_NUM_MOTOR_SLOTS):
                self._low_cmd.motor_cmd[i].mode = 0x01  # Enable
                self._low_cmd.motor_cmd[i].q = POS_STOP
                self._low_cmd.motor_cmd[i].kp = 0
                self._low_cmd.motor_cmd[i].dq = VEL_STOP
                self._low_cmd.motor_cmd[i].kd = 0
                self._low_cmd.motor_cmd[i].tau = 0

            self._crc = CRC()

            # 4. Release sport mode so low-level commands are accepted
            if self._release_sport_mode_flag:
                logger.info("Releasing sport mode...")
                self._release_sport_mode()
            else:
                logger.info("Skipping sport mode release (release_sport_mode=False)")

            # 5. Wait for first LowState to get mode_machine
            logger.info("Waiting for first LowState to capture mode_machine...")
            deadline = time.time() + 10.0
            while self._mode_machine is None and time.time() < deadline:
                time.sleep(0.1)

            if self._mode_machine is None:
                logger.error("Timed out waiting for LowState — mode_machine not captured")
                self._connected = False
                return False

            self._connected = True
            logger.info(f"G1 low-level adapter connected (mode_machine={self._mode_machine})")
            return True

        except Exception as e:
            logger.error(f"Failed to connect G1 low-level adapter: {e}")
            self._connected = False
            return False

    def disconnect(self) -> None:
        """Disconnect from the robot."""
        self._connected = False
        self._publisher = None
        self._subscriber = None
        self._low_cmd = None
        self._low_state = None
        self._mode_machine = None
        logger.info("G1 low-level adapter disconnected")

    def is_connected(self) -> bool:
        return self._connected

    # =========================================================================
    # State Reading
    # =========================================================================

    def read_motor_states(self) -> list[MotorState]:
        """Read motor states for all 29 joints."""
        with self._lock:
            if self._low_state is None:
                return [MotorState()] * _NUM_MOTORS
            return [
                MotorState(
                    q=self._low_state.motor_state[i].q,
                    dq=self._low_state.motor_state[i].dq,
                    tau=self._low_state.motor_state[i].tau_est,
                )
                for i in range(_NUM_MOTORS)
            ]

    def read_imu(self) -> IMUState:
        """Read IMU state."""
        with self._lock:
            if self._low_state is None:
                return IMUState()
            imu = self._low_state.imu_state
            return IMUState(
                quaternion=tuple(imu.quaternion),
                gyroscope=tuple(imu.gyroscope),
                accelerometer=tuple(imu.accelerometer),
                rpy=tuple(imu.rpy),
            )

    # =========================================================================
    # Control
    # =========================================================================

    def write_motor_commands(self, commands: list[MotorCommand]) -> bool:
        """Update command buffer, compute CRC, and publish immediately.

        Called by the coordinator tick loop on every tick -- no background
        thread needed.
        """
        if len(commands) != _NUM_MOTORS:
            logger.error(f"Expected {_NUM_MOTORS} commands, got {len(commands)}")
            return False

        with self._lock:
            if self._low_cmd is None or self._crc is None or self._publisher is None:
                return False
            if self._mode_machine is None:
                return False

            # Echo mode_machine from latest LowState
            self._low_cmd.mode_machine = self._mode_machine

            for i, cmd in enumerate(commands):
                self._low_cmd.motor_cmd[i].q = cmd.q
                self._low_cmd.motor_cmd[i].dq = cmd.dq
                self._low_cmd.motor_cmd[i].kp = cmd.kp
                self._low_cmd.motor_cmd[i].kd = cmd.kd
                self._low_cmd.motor_cmd[i].tau = cmd.tau
            self._low_cmd.crc = self._crc.Crc(self._low_cmd)
            self._publisher.Write(self._low_cmd)
        return True

    # =========================================================================
    # Internal
    # =========================================================================

    def _on_low_state(self, msg: object) -> None:
        """DDS callback for rt/lowstate."""
        with self._lock:
            self._low_state = msg
            # Capture mode_machine from first LowState
            if self._mode_machine is None:
                self._mode_machine = msg.mode_machine  # type: ignore[attr-defined]

    def _release_sport_mode(self) -> None:
        """Exit sport mode so that low-level commands are accepted.

        Loops ReleaseMode until CheckMode returns empty.
        """
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
            MotionSwitcherClient,
        )

        msc = MotionSwitcherClient()
        msc.SetTimeout(5.0)
        msc.Init()

        _status, result = msc.CheckMode()
        while result["name"]:
            msc.ReleaseMode()
            _status, result = msc.CheckMode()
            time.sleep(1)

        logger.info("Sport mode released -- low-level control active")


def register(registry: WholeBodyAdapterRegistry) -> None:
    """Register this adapter with the whole-body registry."""
    registry.register("unitree_g1", UnitreeG1LowLevelAdapter)


__all__ = ["UnitreeG1LowLevelAdapter"]
