#!/usr/bin/env python3

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
Navigator module for coordinating global and local planning.
"""

from collections.abc import Callable
import threading
import time

from dimos_lcm.std_msgs import Bool, String  # type: ignore[import-untyped]
from reactivex.disposable import Disposable

from dimos.core import In, Module, Out, rpc
from dimos.core.rpc_client import RpcCall
from dimos.mapping.occupancy.gradient import gradient
from dimos.mapping.occupancy.inflation import simple_inflate
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.nav_msgs import OccupancyGrid
from dimos.navigation.base import NavigationInterface, NavigationState
from dimos.navigation.bt_navigator.goal_validator import find_safe_goal
from dimos.navigation.bt_navigator.recovery_server import RecoveryServer
from dimos.protocol.tf import TF
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import apply_transform

logger = setup_logger()


class BehaviorTreeNavigator(Module, NavigationInterface):
    """
    Navigator module for coordinating navigation tasks.

    Manages the state machine for navigation, coordinates between global
    and local planners, and monitors goal completion.

    Inputs:
        - odom: Current robot odometry

    Outputs:
        - goal: Goal pose for global planner
    """

    # LCM inputs
    odom: In[PoseStamped]
    goal_request: In[PoseStamped]  # Input for receiving goal requests
    global_costmap: In[OccupancyGrid]

    # LCM outputs
    target: Out[PoseStamped]
    goal_reached: Out[Bool]
    navigation_state: Out[String]

    def __init__(  # type: ignore[no-untyped-def]
        self,
        publishing_frequency: float = 1.0,
        reset_local_planner: Callable[[], None] | None = None,
        check_goal_reached: Callable[[], bool] | None = None,
        **kwargs,
    ) -> None:
        """Initialize the Navigator.

        Args:
            publishing_frequency: Frequency to publish goals to global planner (Hz)
            goal_tolerance: Distance threshold to consider goal reached (meters)
        """
        super().__init__(**kwargs)

        # Parameters
        self.publishing_frequency = publishing_frequency
        self.publishing_period = 1.0 / publishing_frequency

        # State machine
        self.state = NavigationState.IDLE
        self.state_lock = threading.Lock()

        # Current goal
        self.current_goal: PoseStamped | None = None
        self.original_goal: PoseStamped | None = None
        self.goal_lock = threading.Lock()

        # Goal reached state
        self._goal_reached = False

        # Latest data
        self.latest_odom: PoseStamped | None = None
        self.latest_costmap: OccupancyGrid | None = None

        # Control thread
        self.control_thread: threading.Thread | None = None
        self.stop_event = threading.Event()

        # TF listener
        self.tf = TF()

        # Local planner
        self.reset_local_planner = reset_local_planner
        self.check_goal_reached = check_goal_reached

        # Recovery server for stuck detection
        self.recovery_server = RecoveryServer(stuck_duration=5.0)

        logger.info("Navigator initialized with stuck detection")

    @rpc
    def set_HolonomicLocalPlanner_reset(self, callable: RpcCall) -> None:
        self.reset_local_planner = callable
        self.reset_local_planner.set_rpc(self.rpc)  # type: ignore[arg-type]

    @rpc
    def set_HolonomicLocalPlanner_is_goal_reached(self, callable: RpcCall) -> None:
        self.check_goal_reached = callable
        self.check_goal_reached.set_rpc(self.rpc)  # type: ignore[arg-type]

    @rpc
    def start(self) -> None:
        super().start()

        # Subscribe to inputs
        unsub = self.odom.subscribe(self._on_odom)
        self._disposables.add(Disposable(unsub))

        unsub = self.goal_request.subscribe(self._on_goal_request)
        self._disposables.add(Disposable(unsub))

        unsub = self.global_costmap.subscribe(self._on_costmap)
        self._disposables.add(Disposable(unsub))

        # Start control thread
        self.stop_event.clear()
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()

        logger.info("Navigator started")

    @rpc
    def stop(self) -> None:
        """Clean up resources including stopping the control thread."""

        self.stop_navigation()

        self.stop_event.set()
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=2.0)

        super().stop()

    @rpc
    def cancel_goal(self) -> bool:
        """
        Cancel the current navigation goal.

        Returns:
            True if goal was cancelled, False if no goal was active
        """
        self.stop_navigation()
        return True

    @rpc
    def set_goal(self, goal: PoseStamped) -> bool:
        """
        Set a new navigation goal.

        Args:
            goal: Target pose to navigate to

        Returns:
            non-blocking: True if goal was accepted, False otherwise
            blocking: True if goal was reached, False otherwise
        """
        transformed_goal = self._transform_goal_to_odom_frame(goal)
        if not transformed_goal:
            logger.error("Failed to transform goal to odometry frame")
            return False

        with self.goal_lock:
            self.current_goal = transformed_goal
            self.original_goal = transformed_goal

        self._goal_reached = False

        with self.state_lock:
            self.state = NavigationState.FOLLOWING_PATH

        return True

    @rpc
    def get_state(self) -> NavigationState:
        """Get the current state of the navigator."""
        return self.state

    def _on_odom(self, msg: PoseStamped) -> None:
        """Handle incoming odometry messages."""
        self.latest_odom = msg

        if self.state == NavigationState.FOLLOWING_PATH:
            self.recovery_server.update_odom(msg)

    def _on_goal_request(self, msg: PoseStamped) -> None:
        """Handle incoming goal requests."""
        self.set_goal(msg)

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        """Handle incoming costmap messages."""
        self.latest_costmap = msg

    def _transform_goal_to_odom_frame(self, goal: PoseStamped) -> PoseStamped | None:
        """Transform goal pose to the odometry frame."""
        if not goal.frame_id:
            return goal

        if not self.latest_odom:
            logger.error("No odometry data available to transform goal")
            return None

        odom_frame = self.latest_odom.frame_id
        if goal.frame_id == odom_frame:
            return goal

        try:
            transform = None
            max_retries = 3

            for attempt in range(max_retries):
                transform = self.tf.get(
                    parent_frame=odom_frame,
                    child_frame=goal.frame_id,
                )

                if transform:
                    break

                if attempt < max_retries - 1:
                    logger.warning(
                        f"Transform attempt {attempt + 1}/{max_retries} failed, retrying..."
                    )
                    time.sleep(1.0)
                else:
                    logger.error(
                        f"Could not find transform from '{goal.frame_id}' to '{odom_frame}' after {max_retries} attempts"
                    )
                    return None

            pose = apply_transform(goal, transform)  # type: ignore[arg-type]
            transformed_goal = PoseStamped(
                position=pose.position,
                orientation=pose.orientation,
                frame_id=odom_frame,
                ts=goal.ts,
            )
            return transformed_goal

        except Exception as e:
            logger.error(f"Failed to transform goal: {e}")
            return None

    def _control_loop(self) -> None:
        """Main control loop running in separate thread."""
        while not self.stop_event.is_set():
            with self.state_lock:
                current_state = self.state
                self.navigation_state.publish(String(data=current_state.value))

            if current_state == NavigationState.FOLLOWING_PATH:
                with self.goal_lock:
                    goal = self.current_goal
                    original_goal = self.original_goal

                if goal is not None and self.latest_costmap is not None:
                    # Check if robot is stuck
                    if self.recovery_server.check_stuck():
                        logger.warning("Robot is stuck! Cancelling goal and resetting.")
                        self.cancel_goal()
                        continue

                    costmap = gradient(simple_inflate(self.latest_costmap, 0.1), max_distance=1.0)

                    # Find safe goal position
                    safe_goal_pos = find_safe_goal(
                        costmap,
                        original_goal.position,  # type: ignore[union-attr]
                        algorithm="bfs",
                        cost_threshold=60,
                        min_clearance=0.25,
                        max_search_distance=5.0,
                    )

                    # Create new goal with safe position
                    if safe_goal_pos:
                        safe_goal = PoseStamped(
                            position=safe_goal_pos,
                            orientation=goal.orientation,
                            frame_id=goal.frame_id,
                            ts=goal.ts,
                        )
                        self.target.publish(safe_goal)
                        self.current_goal = safe_goal
                    else:
                        logger.warning("Could not find safe goal position, cancelling goal")
                        self.cancel_goal()

                    # Check if goal is reached
                    if self.check_goal_reached():  # type: ignore[misc]
                        reached_msg = Bool()
                        reached_msg.data = True
                        self.goal_reached.publish(reached_msg)
                        self.stop_navigation()
                        self._goal_reached = True
                        logger.info("Goal reached, resetting local planner")

            elif current_state == NavigationState.RECOVERY:
                with self.state_lock:
                    self.state = NavigationState.IDLE

            time.sleep(self.publishing_period)

    @rpc
    def is_goal_reached(self) -> bool:
        """Check if the current goal has been reached.

        Returns:
            True if goal was reached, False otherwise
        """
        return self._goal_reached

    def stop_navigation(self) -> None:
        """Stop navigation and return to IDLE state."""
        with self.goal_lock:
            self.current_goal = None

        self._goal_reached = False

        with self.state_lock:
            self.state = NavigationState.IDLE

        self.reset_local_planner()  # type: ignore[misc]
        self.recovery_server.reset()  # Reset recovery server when stopping

        logger.info("Navigator stopped")


behavior_tree_navigator = BehaviorTreeNavigator.blueprint

__all__ = ["BehaviorTreeNavigator", "behavior_tree_navigator"]
