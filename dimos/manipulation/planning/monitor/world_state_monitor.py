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

"""
World State Monitor

Monitors joint state updates and syncs them to a WorldSpec instance.
This is the WorldSpec-based replacement for StateMonitor.

Example:
    monitor = WorldStateMonitor(world, lock, robot_id, joint_names)
    monitor.start()
    monitor.on_joint_state(joint_state_msg)  # Called by subscriber
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

import numpy as np

from dimos.msgs.sensor_msgs import JointState
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable
    import threading

    from numpy.typing import NDArray

    from dimos.manipulation.planning.spec import WorldSpec

logger = setup_logger()


class WorldStateMonitor:
    """Monitors joint state updates and syncs them to WorldSpec.

    This class subscribes to joint state messages and calls
    world.sync_from_joint_state() to keep the world's live context
    synchronized with the real robot state.

    ## Thread Safety

    All state updates are protected by the provided lock. The on_joint_state
    callback can be called from any thread.

    ## Comparison with StateMonitor

    - StateMonitor: Works with PlanningScene ABC
    - WorldStateMonitor: Works with WorldSpec Protocol
    """

    def __init__(
        self,
        world: WorldSpec,
        lock: threading.RLock,
        robot_id: str,
        joint_names: list[str],
        joint_name_mapping: dict[str, str] | None = None,
        timeout: float = 1.0,
    ):
        """Create a world state monitor.

        Args:
            world: WorldSpec instance to sync state to
            lock: Shared lock for thread-safe access
            robot_id: ID of the robot to monitor
            joint_names: Ordered list of joint names for this robot (URDF names)
            joint_name_mapping: Maps coordinator joint names to URDF joint names.
                Example: {"left_joint1": "joint1"} means messages with "left_joint1"
                will be mapped to URDF "joint1". If None, names must match exactly.
            timeout: Timeout for waiting for initial state (seconds)
        """
        self._world = world
        self._lock = lock
        self._robot_id = robot_id
        self._joint_names = joint_names
        self._timeout = timeout

        # Joint name mapping: coordinator name -> URDF name
        self._joint_name_mapping = joint_name_mapping or {}
        # Build reverse mapping: URDF name -> coordinator name
        self._reverse_mapping = {v: k for k, v in self._joint_name_mapping.items()}

        # Latest state
        self._latest_positions: NDArray[np.float64] | None = None
        self._latest_velocities: NDArray[np.float64] | None = None
        self._last_update_time: float | None = None

        # Running state
        self._running = False

        # Callbacks: (robot_id, joint_state) called on each state update
        self._state_callbacks: list[Callable[[str, JointState], None]] = []

    def start(self) -> None:
        """Start the state monitor."""
        self._running = True
        logger.info(f"World state monitor started for robot '{self._robot_id}'")

    def stop(self) -> None:
        """Stop the state monitor."""
        self._running = False
        logger.info(f"World state monitor stopped for robot '{self._robot_id}'")

    def is_running(self) -> bool:
        """Check if monitor is running."""
        return self._running

    @property
    def robot_id(self) -> str:
        """Get the robot ID being monitored."""
        return self._robot_id

    def on_joint_state(self, msg: JointState) -> None:
        """Handle incoming joint state message.

        This is called by the subscriber when a new JointState message arrives.
        It extracts joint positions and syncs them to the world.

        Args:
            msg: JointState message with joint names and positions
        """
        try:
            if not self._running:
                return

            # Extract positions for our robot's joints
            positions = self._extract_positions(msg)
            if positions is None:
                logger.debug(
                    "[WorldStateMonitor] Failed to extract positions - joint names mismatch"
                )
                logger.debug(f"  Expected joints: {self._joint_names}")
                logger.debug(f"  Received joints: {msg.name}")
                return  # Not all joints present in message

            velocities = self._extract_velocities(msg)

            # Track message count for debugging
            self._msg_count = getattr(self, "_msg_count", 0) + 1

            with self._lock:
                current_time = time.time()

                # Store latest state FIRST - this ensures planning always has
                # current positions even if sync_from_joint_state fails
                # (e.g., after dynamically adding obstacles)
                self._latest_positions = positions
                self._latest_velocities = velocities
                self._last_update_time = current_time

                # Sync to world's live context (for visualization)
                try:
                    # Create JointState for world sync (API uses JointState)
                    joint_state = JointState(
                        name=self._joint_names,
                        position=positions.tolist(),
                    )
                    self._world.sync_from_joint_state(self._robot_id, joint_state)
                except Exception as e:
                    logger.error(f"Failed to sync joint state to live context: {e}")

                # Call registered callbacks
                for callback in self._state_callbacks:
                    try:
                        callback(self._robot_id, joint_state)
                    except Exception as e:
                        logger.error(f"State callback error: {e}")

        except Exception as e:
            logger.error(f"[WorldStateMonitor] Unexpected exception in on_joint_state: {e}")
            import traceback

            logger.error(traceback.format_exc())

    def _extract_positions(self, msg: JointState) -> NDArray[np.float64] | None:
        """Extract positions for our joints from JointState message.

        Handles joint name translation from coordinator namespace to URDF namespace.
        If joint_name_mapping is set, message names are looked up via the reverse mapping.

        Args:
            msg: JointState message (may use coordinator joint names)

        Returns:
            Array of joint positions or None if any joint is missing
        """
        # Build name->index map from message (coordinator names)
        name_to_idx = {name: i for i, name in enumerate(msg.name)}

        positions = []
        for urdf_joint_name in self._joint_names:
            # Try direct match first (when no mapping or names already match)
            if urdf_joint_name in name_to_idx:
                idx = name_to_idx[urdf_joint_name]
            else:
                # Try reverse mapping: URDF name -> coordinator name -> msg index
                orch_name = self._reverse_mapping.get(urdf_joint_name)
                if orch_name is None or orch_name not in name_to_idx:
                    return None  # Missing joint
                idx = name_to_idx[orch_name]

            if idx >= len(msg.position):
                return None  # Position not available
            positions.append(msg.position[idx])

        return np.array(positions, dtype=np.float64)

    def _extract_velocities(self, msg: JointState) -> NDArray[np.float64] | None:
        """Extract velocities for our joints.

        Uses same name translation as _extract_positions.
        """
        if not msg.velocity or len(msg.velocity) == 0:
            return None

        name_to_idx = {name: i for i, name in enumerate(msg.name)}

        velocities = []
        for urdf_joint_name in self._joint_names:
            # Try direct match first
            if urdf_joint_name in name_to_idx:
                idx = name_to_idx[urdf_joint_name]
            else:
                # Try reverse mapping
                orch_name = self._reverse_mapping.get(urdf_joint_name)
                if orch_name is None or orch_name not in name_to_idx:
                    return None
                idx = name_to_idx[orch_name]

            if idx >= len(msg.velocity):
                return None
            velocities.append(msg.velocity[idx])

        return np.array(velocities, dtype=np.float64)

    def get_current_positions(self) -> NDArray[np.float64] | None:
        """Get current joint positions (thread-safe).

        Returns:
            Current positions or None if not yet received
        """
        with self._lock:
            return self._latest_positions.copy() if self._latest_positions is not None else None

    def get_current_velocities(self) -> NDArray[np.float64] | None:
        """Get current joint velocities (thread-safe).

        Returns:
            Current velocities or None if not available
        """
        with self._lock:
            return self._latest_velocities.copy() if self._latest_velocities is not None else None

    def wait_for_state(self, timeout: float | None = None) -> bool:
        """Wait until a state is received.

        Args:
            timeout: Maximum time to wait (uses default if None)

        Returns:
            True if state was received, False if timeout
        """
        timeout = timeout if timeout is not None else self._timeout
        start_time = time.time()

        while time.time() - start_time < timeout:
            with self._lock:
                if self._latest_positions is not None:
                    return True
            time.sleep(0.01)

        return False

    def get_state_age(self) -> float | None:
        """Get age of the latest state in seconds.

        Returns:
            Age in seconds or None if no state received
        """
        with self._lock:
            if self._last_update_time is None:
                return None
            return time.time() - self._last_update_time

    def is_state_stale(self, max_age: float = 1.0) -> bool:
        """Check if state is stale (older than max_age).

        Args:
            max_age: Maximum acceptable age in seconds

        Returns:
            True if state is stale or not received
        """
        age = self.get_state_age()
        if age is None:
            return True
        return age > max_age

    def add_state_callback(
        self,
        callback: Callable[[str, JointState], None],
    ) -> None:
        """Add callback for state updates.

        Args:
            callback: Function called with (robot_id, joint_state) on each update
        """
        self._state_callbacks.append(callback)

    def remove_state_callback(
        self,
        callback: Callable[[str, JointState], None],
    ) -> None:
        """Remove a state callback."""
        if callback in self._state_callbacks:
            self._state_callbacks.remove(callback)
