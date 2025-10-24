# Copyright 2025 Dimensional Inc.
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

"""
State Query Component for XArmDriver.

Provides RPC methods for querying robot state including:
- Joint state
- Robot state
- Cartesian position
- Firmware version
"""

from typing import List, Tuple, Optional
from dimos.core import rpc
from dimos.msgs.sensor_msgs import JointState, RobotState
from dimos.utils.logging_config import setup_logger

logger = setup_logger(__file__)


class StateQueryComponent:
    """
    Component providing state query RPC methods for XArmDriver.

    This component assumes the parent class has:
    - self.arm: XArmAPI instance
    - self.config: XArmDriverConfig instance
    - self._joint_state_lock: threading.Lock
    - self._joint_states_: Optional[JointState]
    - self._robot_state_: Optional[RobotState]
    """

    @rpc
    def get_joint_state(self) -> Optional[JointState]:
        """
        Get the current joint state (RPC method).

        Returns:
            Current JointState or None
        """
        with self._joint_state_lock:
            return self._joint_states_

    @rpc
    def get_robot_state(self) -> Optional[RobotState]:
        """
        Get the current robot state (RPC method).

        Returns:
            Current RobotState or None
        """
        with self._joint_state_lock:
            return self._robot_state_

    @rpc
    def get_position(self) -> Tuple[int, Optional[List[float]]]:
        """
        Get TCP position [x, y, z, roll, pitch, yaw].

        Returns:
            Tuple of (code, position)
        """
        try:
            code, position = self.arm.get_position(is_radian=self.config.is_radian)
            return (code, list(position) if code == 0 else None)
        except Exception as e:
            logger.error(f"get_position failed: {e}")
            return (-1, None)

    @rpc
    def get_version(self) -> Tuple[int, Optional[str]]:
        """Get firmware version."""
        try:
            code, version = self.arm.get_version()
            return (code, version if code == 0 else None)
        except Exception as e:
            return (-1, None)
