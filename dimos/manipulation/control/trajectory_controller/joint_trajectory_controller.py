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
Joint Trajectory Controller

A simple joint-space trajectory executor. Does NOT:
- Use Cartesian space
- Compute error
- Apply PID
- Call IK

Just samples a trajectory at time t and sends joint positions to the driver.

Behavior:
- execute_trajectory(): Preempts any active trajectory, starts new one immediately
- cancel(): Stops at current position
- reset(): Required to recover from FAULT state
"""

from dataclasses import dataclass
import threading
import time
from typing import Any

from dimos.core import In, Module, Out, rpc
from dimos.core.module import ModuleConfig
from dimos.msgs.sensor_msgs import JointCommand, JointState, RobotState
from dimos.msgs.trajectory_msgs import JointTrajectory, TrajectoryState, TrajectoryStatus
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass
class JointTrajectoryControllerConfig(ModuleConfig):
    """Configuration for joint trajectory controller."""

    control_frequency: float = 100.0  # Hz - trajectory execution rate


class JointTrajectoryController(Module):
    """
    Joint-space trajectory executor.

    Executes joint trajectories at 100Hz by sampling and forwarding
    joint positions to the arm driver. Uses ROS action-server-like
    state machine for execution control.

    State Machine:
        IDLE ──execute()──► EXECUTING ──done──► COMPLETED
          ▲                     │                    │
          │                  cancel()             reset()
          │                     ▼                    │
          └─────reset()───── ABORTED ◄──────────────┘
                                │
                             error
                                ▼
                              FAULT ──reset()──► IDLE
    """

    default_config = JointTrajectoryControllerConfig
    config: JointTrajectoryControllerConfig  # Type hint for proper attribute access

    # Input topics
    joint_state: In[JointState] = None  # type: ignore[assignment]  # Feedback from arm driver
    robot_state: In[RobotState] = None  # type: ignore[assignment]  # Robot status from arm driver
    trajectory: In[JointTrajectory] = None  # type: ignore[assignment]  # Trajectory to execute (topic-based)

    # Output topics
    joint_position_command: Out[JointCommand] = None  # type: ignore[assignment]  # To arm driver

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)

        # State machine
        self._state = TrajectoryState.IDLE
        self._lock = threading.Lock()

        # Active trajectory
        self._trajectory: JointTrajectory | None = None
        self._start_time: float = 0.0

        # Latest feedback
        self._latest_joint_state: JointState | None = None
        self._latest_robot_state: RobotState | None = None

        # Error tracking
        self._error_message: str = ""

        # Execution thread
        self._exec_thread: threading.Thread | None = None
        self._stop_event = threading.Event()

        logger.info(f"JointTrajectoryController initialized at {self.config.control_frequency}Hz")

    @rpc
    def start(self) -> None:
        """Start the trajectory controller."""
        super().start()

        # Subscribe to feedback topics
        try:
            if self.joint_state.connection is not None or self.joint_state._transport is not None:
                self.joint_state.subscribe(self._on_joint_state)
                logger.info("Subscribed to joint_state")
        except Exception as e:
            logger.warning(f"Failed to subscribe to joint_state: {e}")

        try:
            if self.robot_state.connection is not None or self.robot_state._transport is not None:
                self.robot_state.subscribe(self._on_robot_state)
                logger.info("Subscribed to robot_state")
        except Exception as e:
            logger.warning(f"Failed to subscribe to robot_state: {e}")

        # Subscribe to trajectory topic
        try:
            if self.trajectory.connection is not None or self.trajectory._transport is not None:
                self.trajectory.subscribe(self._on_trajectory)
                logger.info("Subscribed to trajectory topic")
        except Exception:
            logger.debug("trajectory topic not connected (expected - can use RPC instead)")

        # Start execution thread
        self._stop_event.clear()
        self._exec_thread = threading.Thread(
            target=self._execution_loop, daemon=True, name="trajectory_exec_thread"
        )
        self._exec_thread.start()

        logger.info("JointTrajectoryController started")

    @rpc
    def stop(self) -> None:
        """Stop the trajectory controller."""
        logger.info("Stopping JointTrajectoryController...")

        self._stop_event.set()

        if self._exec_thread and self._exec_thread.is_alive():
            self._exec_thread.join(timeout=2.0)

        super().stop()
        logger.info("JointTrajectoryController stopped")

    # =========================================================================
    # RPC Methods - Action-server-like interface
    # =========================================================================

    @rpc
    def execute_trajectory(self, trajectory: JointTrajectory) -> bool:
        """
        Set and start executing a new trajectory immediately.
        If currently executing, preempts and starts new trajectory.

        Args:
            trajectory: JointTrajectory to execute

        Returns:
            True if accepted, False if in FAULT state or trajectory invalid
        """
        with self._lock:
            # Cannot execute if in FAULT state
            if self._state == TrajectoryState.FAULT:
                logger.warning(
                    "Cannot execute trajectory: controller in FAULT state (call reset())"
                )
                return False

            # Validate trajectory
            if trajectory is None or trajectory.duration <= 0:
                logger.warning("Invalid trajectory: None or zero duration")
                return False

            if not trajectory.points:
                logger.warning("Invalid trajectory: no points")
                return False

            # Preempt any active trajectory
            if self._state == TrajectoryState.EXECUTING:
                logger.info("Preempting active trajectory")

            # Start new trajectory
            self._trajectory = trajectory
            self._start_time = time.time()
            self._state = TrajectoryState.EXECUTING
            self._error_message = ""

            logger.info(
                f"Executing trajectory: {len(trajectory.points)} points, "
                f"duration={trajectory.duration:.3f}s"
            )
            return True

    @rpc
    def cancel(self) -> bool:
        """
        Cancel the currently executing trajectory.
        Robot stops at current position.

        Returns:
            True if cancelled, False if no active trajectory
        """
        with self._lock:
            if self._state != TrajectoryState.EXECUTING:
                logger.debug("No active trajectory to cancel")
                return False

            self._state = TrajectoryState.ABORTED
            logger.info("Trajectory cancelled")
            return True

    @rpc
    def reset(self) -> bool:
        """
        Reset from FAULT, COMPLETED, or ABORTED state back to IDLE.
        Required before executing new trajectories after a fault.

        Returns:
            True if reset successful, False if currently EXECUTING
        """
        with self._lock:
            if self._state == TrajectoryState.EXECUTING:
                logger.warning("Cannot reset while executing (call cancel() first)")
                return False

            self._state = TrajectoryState.IDLE
            self._trajectory = None
            self._error_message = ""
            logger.info("Controller reset to IDLE")
            return True

    @rpc
    def get_status(self) -> TrajectoryStatus:
        """
        Get the current status of the trajectory execution.

        Returns:
            TrajectoryStatus with state, progress, and error info
        """
        with self._lock:
            time_elapsed = 0.0
            time_remaining = 0.0
            progress = 0.0

            if self._trajectory is not None and self._state == TrajectoryState.EXECUTING:
                time_elapsed = time.time() - self._start_time
                time_remaining = max(0.0, self._trajectory.duration - time_elapsed)
                progress = (
                    min(1.0, time_elapsed / self._trajectory.duration)
                    if self._trajectory.duration > 0
                    else 1.0
                )

            return TrajectoryStatus(
                state=self._state,
                progress=progress,
                time_elapsed=time_elapsed,
                time_remaining=time_remaining,
                error=self._error_message,
            )

    # =========================================================================
    # Callbacks
    # =========================================================================

    def _on_joint_state(self, msg: JointState) -> None:
        """Callback for joint state feedback."""
        self._latest_joint_state = msg

    def _on_robot_state(self, msg: RobotState) -> None:
        """Callback for robot state feedback."""
        self._latest_robot_state = msg

    def _on_trajectory(self, msg: JointTrajectory) -> None:
        """Callback when trajectory is received via topic."""
        logger.info(
            f"Received trajectory via topic: {len(msg.points)} points, duration={msg.duration:.3f}s"
        )
        self.execute_trajectory(msg)

    # =========================================================================
    # Execution Loop
    # =========================================================================

    def _execution_loop(self) -> None:
        """
        Main execution loop running at control_frequency Hz.

        When EXECUTING:
        1. Compute elapsed time
        2. Sample trajectory at t
        3. Publish joint command
        4. Check if done
        """
        period = 1.0 / self.config.control_frequency
        logger.info(f"Execution loop started at {self.config.control_frequency}Hz")

        while not self._stop_event.is_set():
            try:
                with self._lock:
                    # Only process if executing
                    if self._state != TrajectoryState.EXECUTING:
                        # Release lock and sleep
                        pass
                    else:
                        # Compute elapsed time
                        t = time.time() - self._start_time

                        # Check if trajectory complete
                        if self._trajectory is None:
                            self._state = TrajectoryState.FAULT
                            logger.error("Trajectory is None during execution")
                        elif t >= self._trajectory.duration:
                            self._state = TrajectoryState.COMPLETED
                            logger.info(
                                f"Trajectory completed: duration={self._trajectory.duration:.3f}s"
                            )
                        else:
                            # Sample trajectory
                            q_ref, _qd_ref = self._trajectory.sample(t)

                            # Create and publish command (outside lock would be better but simpler here)
                            cmd = JointCommand(positions=q_ref, timestamp=time.time())

                            # Publish - must release lock first for thread safety
                            trajectory_active = True

                if trajectory_active if "trajectory_active" in dir() else False:
                    try:
                        self.joint_position_command.publish(cmd)
                    except Exception as e:
                        logger.error(f"Failed to publish joint command: {e}")
                        with self._lock:
                            self._state = TrajectoryState.FAULT
                            self._error_message = f"Publish failed: {e}"

                # Reset flag
                trajectory_active = False

                # Maintain loop frequency
                time.sleep(period)

            except Exception as e:
                logger.error(f"Error in execution loop: {e}")
                with self._lock:
                    if self._state == TrajectoryState.EXECUTING:
                        self._state = TrajectoryState.FAULT
                        self._error_message = str(e)
                time.sleep(period)

        logger.info("Execution loop stopped")


# Expose blueprint for declarative composition
joint_trajectory_controller = JointTrajectoryController.blueprint
