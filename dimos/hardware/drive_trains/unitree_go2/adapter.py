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

"""Unitree Go2 adapter — wraps Unitree SDK2 for quadruped base control.

The Go2 is a quadruped robot with 3 DOF velocity control: [vx, vy, wz].
This adapter uses the Unitree SDK2 Python bindings to communicate via DDS.

Important initialization sequence:
  1. ChannelFactoryInitialize(0) - Initialize DDS
  2. SportClient.StandUp() - Stand the robot up
  3. SportClient.FreeWalk() - Activate locomotion mode
  4. SportClient.Move(vx, vy, vyaw) - Send velocity commands
"""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import TYPE_CHECKING, Any

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.hardware.drive_trains.registry import TwistBaseAdapterRegistry

logger = setup_logger()


@dataclass
class _Session:
    """Active connection state for a Go2."""

    client: Any  # SportClient
    lock: threading.Lock
    state_sub: Any = None  # ChannelSubscriber
    latest_state: Any = None  # SportModeState_
    enabled: bool = False
    locomotion_ready: bool = False


class UnitreeGo2TwistAdapter:
    """TwistBaseAdapter implementation for Unitree Go2 quadruped.

    Communicates with Go2 via Unitree SDK2 Python over DDS.
    Expects 3 DOF: [vx, vy, wz] where:
      - vx: forward/backward velocity (m/s)
      - vy: left/right lateral velocity (m/s)
      - wz: yaw rotation velocity (rad/s)

    Args:
        dof: Number of velocity DOFs (must be 3 for Go2)
        network_interface: DDS network interface ID (default: 0)
    """

    def __init__(self, dof: int = 3, network_interface: int = 0, **_: object) -> None:
        if dof != 3:
            raise ValueError(f"Go2 only supports 3 DOF (vx, vy, wz), got {dof}")

        self._network_interface = network_interface  # domain ID for DDS
        self._session: _Session | None = None

    def _get_session(self) -> _Session:
        """Return active session or raise if not connected."""
        if self._session is None:
            raise RuntimeError("Go2 not connected")
        return self._session

    # =========================================================================
    # Connection
    # =========================================================================

    def connect(self) -> bool:
        """Connect to Go2 and initialize locomotion mode."""
        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
            from unitree_sdk2py.go2.sport.sport_client import SportClient
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

            # Initialize DDS
            logger.info(f"Initializing DDS with network interface {self._network_interface}...")
            ChannelFactoryInitialize(self._network_interface)

            # Create sport client
            logger.info("Connecting to Go2 SportClient...")
            client = SportClient()
            client.SetTimeout(5.0)
            client.Init()

            # Create session — callback closes over it for state updates
            session = _Session(client=client, lock=threading.Lock())

            def state_callback(msg: SportModeState_) -> None:
                with session.lock:
                    session.latest_state = msg

            state_sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
            state_sub.Init(state_callback, 10)
            session.state_sub = state_sub

            self._session = session
            logger.info("Connected to Go2")

            # Stand up and activate locomotion
            if not self._initialize_locomotion():
                logger.error("Failed to initialize locomotion mode")
                self.disconnect()
                return False

            return True

        except Exception as e:
            logger.error(f"Failed to connect to Go2: {e}")
            self._session = None
            return False

    def disconnect(self) -> None:
        """Disconnect and safely shut down the robot."""
        session = self._session
        if session is not None:
            try:
                session.client.StopMove()
                time.sleep(0.5)

                logger.info("Standing down Go2...")
                session.client.StandDown()
                time.sleep(2)
            except Exception as e:
                logger.error(f"Error during disconnect: {e}")

        self._session = None

    def is_connected(self) -> bool:
        """Check if connected to Go2."""
        return self._session is not None

    # =========================================================================
    # Info
    # =========================================================================

    def get_dof(self) -> int:
        """Go2 is always 3 DOF (vx, vy, wz)."""
        return 3

    # =========================================================================
    # State Reading
    # =========================================================================

    def read_velocities(self) -> list[float]:
        """Read actual velocities from SportModeState as [vx, vy, wz]."""
        session = self._get_session()
        with session.lock:
            if session.latest_state is None:
                return [0.0, 0.0, 0.0]
            try:
                state = session.latest_state
                return [
                    float(state.velocity[0]),  # vx
                    float(state.velocity[1]),  # vy
                    float(state.imu_state.gyroscope[2]),  # wz (yaw rate)
                ]
            except Exception as e:
                logger.warning(f"Error reading Go2 velocities: {e}")
                return [0.0, 0.0, 0.0]

    def read_odometry(self) -> list[float] | None:
        """Read odometry from Go2 as [x, y, theta].

        Returns position from SportModeState which provides:
          - position[0]: x (meters)
          - position[1]: y (meters)
          - imu_state.rpy[2]: yaw (radians)
        """
        session = self._get_session()
        with session.lock:
            if session.latest_state is None:
                return None

            try:
                state = session.latest_state
                return [
                    float(state.position[0]),
                    float(state.position[1]),
                    float(state.imu_state.rpy[2]),  # yaw
                ]
            except Exception as e:
                logger.error(f"Error reading Go2 odometry: {e}")
                return None

    # =========================================================================
    # Control
    # =========================================================================

    def write_velocities(self, velocities: list[float]) -> bool:
        """Send velocity command to Go2.

        Args:
            velocities: [vx, vy, wz] in standard frame (m/s, m/s, rad/s)
        """
        if len(velocities) != 3:
            return False

        session = self._get_session()

        if not session.enabled:
            logger.warning("Go2 not enabled, ignoring velocity command")
            return False

        if not session.locomotion_ready:
            logger.warning("Go2 locomotion not ready, ignoring velocity command")
            return False

        vx, vy, wz = velocities
        return self._send_velocity(vx, vy, wz)

    def write_stop(self) -> bool:
        """Stop all motion."""
        session = self._get_session()
        try:
            session.client.StopMove()
            return True
        except Exception as e:
            logger.error(f"Error stopping Go2: {e}")
            return False

    # =========================================================================
    # Enable/Disable
    # =========================================================================

    def write_enable(self, enable: bool) -> bool:
        """Enable/disable the platform.

        When enabling, ensures the robot is stood up and locomotion is ready.
        When disabling, stops motion but keeps standing.
        """
        session = self._get_session()

        if enable:
            if not session.locomotion_ready:
                logger.info("Locomotion not ready, initializing...")
                if not self._initialize_locomotion():
                    logger.error("Failed to initialize locomotion")
                    return False

            session.enabled = True
            logger.info("Go2 enabled")
            return True
        else:
            # Disable: stop motion but keep standing
            self.write_stop()
            session.enabled = False
            logger.info("Go2 disabled")
            return True

    def read_enabled(self) -> bool:
        """Check if platform is enabled."""
        return self._session is not None and self._session.enabled

    # =========================================================================
    # Internal
    # =========================================================================

    def _initialize_locomotion(self) -> bool:
        """Initialize locomotion mode: StandUp + FreeWalk.

        This is the critical sequence discovered during testing:
        1. StandUp() - Robot stands but is in "locked" state
        2. Wait for standup to complete
        3. FreeWalk() - Activates locomotion controller
        4. Now Move() commands will work

        Returns:
            True if successful, False otherwise
        """
        session = self._get_session()

        try:
            logger.info("Standing up Go2...")
            ret = session.client.StandUp()
            if ret != 0:
                logger.error(f"StandUp() failed with code {ret}")
                return False
            time.sleep(3)  # Wait for standup to complete

            logger.info("Activating FreeWalk locomotion mode...")
            ret = session.client.FreeWalk()
            if ret != 0:
                logger.error(f"FreeWalk() failed with code {ret}")
                return False
            time.sleep(2)  # Wait for locomotion mode to activate

            session.locomotion_ready = True
            logger.info("Go2 locomotion ready")
            return True

        except Exception as e:
            logger.error(f"Error initializing locomotion: {e}")
            return False

    def _send_velocity(self, vx: float, vy: float, wz: float) -> bool:
        """Send raw velocity to Go2 via SportClient.Move().

        Args:
            vx: forward/backward velocity (m/s)
            vy: left/right lateral velocity (m/s)
            wz: yaw rotation velocity (rad/s)
        """
        session = self._get_session()
        try:
            with session.lock:
                ret = session.client.Move(vx, vy, wz)

            if ret != 0:
                logger.warning(f"Move() returned error code {ret}")
                return False

            return True

        except Exception as e:
            logger.error(f"Error sending Go2 velocity: {e}")
            return False


def register(registry: TwistBaseAdapterRegistry) -> None:
    """Register this adapter with the registry."""
    registry.register("unitree_go2", UnitreeGo2TwistAdapter)


__all__ = ["UnitreeGo2TwistAdapter"]
