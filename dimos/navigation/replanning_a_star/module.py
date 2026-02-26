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

import math
import os
import traceback
from typing import Callable

from dimos_lcm.std_msgs import Bool, String
from reactivex.disposable import Disposable

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import ControlCoordinator
from dimos.core import In, Module, Out, rpc
from dimos.core.global_config import GlobalConfig, global_config
from dimos.hardware.manipulators.registry import adapter_registry
from dimos.msgs.geometry_msgs import PoseStamped, Twist
from dimos.msgs.nav_msgs import OccupancyGrid, Path
from dimos.navigation.base import NavigationInterface, NavigationState
from dimos.navigation.replanning_a_star.global_planner import GlobalPlanner
from dimos.navigation.replanning_a_star.path_follower_task import (
    PathFollowerTask,
    PathFollowerTaskConfig,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

class ReplanningAStarPlanner(Module, NavigationInterface):
    odom: In[PoseStamped]  # TODO: Use TF.
    global_costmap: In[OccupancyGrid]
    goal_request: In[PoseStamped]
    target: In[PoseStamped]

    goal_reached: Out[Bool]
    navigation_state: Out[String]  # TODO: set it
    cmd_vel: Out[Twist]
    path: Out[Path]
    navigation_costmap: Out[OccupancyGrid]

    _planner: GlobalPlanner
    _global_config: GlobalConfig

    def __init__(self, cfg: GlobalConfig = global_config) -> None:
        super().__init__()
        self._global_config = cfg
        self._planner = GlobalPlanner(self._global_config)

        self._coordinator: ControlCoordinator | None = None
        self._path_follower_task: PathFollowerTask | None = None
        self._local_planner_cmd_vel_disposable: Disposable | None = None

    @rpc
    def start(self) -> None:
        super().start()

        self._disposables.add(Disposable(self.odom.subscribe(self._planner.handle_odom)))
        self._disposables.add(
            Disposable(self.global_costmap.subscribe(self._planner.handle_global_costmap))
        )
        self._disposables.add(
            Disposable(self.goal_request.subscribe(self._planner.handle_goal_request))
        )
        self._disposables.add(Disposable(self.target.subscribe(self._planner.handle_goal_request)))

        self._disposables.add(self._planner.path.subscribe(self.path.publish))

        if self._path_follower_task is None:
            self._local_planner_cmd_vel_disposable = self._planner.cmd_vel.subscribe(self.cmd_vel.publish)
            self._disposables.add(self._local_planner_cmd_vel_disposable)
            logger.warning("PathFollowerTask not set - using LocalPlanner fallback")
        else:
            logger.info("PathFollowerTask active - LocalPlanner cmd_vel disabled")

        self._disposables.add(self._planner.goal_reached.subscribe(self.goal_reached.publish))

        if "DEBUG_NAVIGATION" in os.environ:
            self._disposables.add(
                self._planner.navigation_costmap.subscribe(self.navigation_costmap.publish)
            )

        self._planner.start()

    @rpc
    def stop(self) -> None:
        self.cancel_goal()
        self._planner.stop()

        super().stop()

    @rpc
    def set_goal(self, goal: PoseStamped) -> bool:
        self._planner.handle_goal_request(goal)
        return True

    @rpc
    def get_state(self) -> NavigationState:
        return self._planner.get_state()

    @rpc
    def is_goal_reached(self) -> bool:
        return self._planner.is_goal_reached()

    @rpc
    def cancel_goal(self) -> bool:
        self._planner.cancel_goal()
        return True

    @rpc
    def set_coordinator(self, coordinator: ControlCoordinator) -> bool:
        """Set the ControlCoordinator for path following.
    
        Args:
            coordinator: ControlCoordinator instance
            
        Returns:
            True if setup successful
        """
        try:
            self._coordinator = coordinator

            # Create path follower task
            task_config = PathFollowerTaskConfig(
                joint_names=["base_vx", "base_vy", "base_wz"],
                priority=10,
                max_linear_speed=2.2,
                goal_tolerance=0.2,                      # match GlobalPlanner._goal_tolerance
                orientation_tolerance=math.radians(15),  # match _rotation_tolerance
            )
            self._path_follower_task = PathFollowerTask(
                name="path_follower",
                config=task_config,
                global_config=self._global_config,
            )
            
            # Add task to coordinator
            if not coordinator.add_task(self._path_follower_task):
                logger.error("Failed to add PathFollowerTask to coordinator")
                return False

            # Tell the GlobalPlanner to use the coordinator-backed PathFollowerTask.
            # NOTE: The actual task instance that runs lives inside the ControlCoordinator
            # process; we refer to it by name via coordinator.task_invoke().
            self._planner.set_path_follower_task(coordinator, "path_follower")
            

            # Disable LocalPlanner cmd_vel if it was subscribed
            if self._local_planner_cmd_vel_disposable is not None:
                self._local_planner_cmd_vel_disposable.dispose()
                self._local_planner_cmd_vel_disposable = None
                logger.info("Disabled LocalPlanner cmd_vel - PathFollowerTask is now active")
            
            logger.info("PathFollowerTask added to ControlCoordinator and connected to GlobalPlanner")
            return True
        except Exception as e:
            logger.error(f"Failed to set coordinator: {e}")
            logger.error(traceback.format_exc())
            return False

    @rpc
    def setup_base_hardware_with_callback(
        self,
        coordinator: ControlCoordinator,
        twist_callback: Callable[[Twist], None] | None,
    ) -> bool:
        """Set up base hardware adapter with provided callback.
        
        Args:
            coordinator: ControlCoordinator instance
            twist_callback: Callback function that receives Twist commands
                        (should call GO2Connection.move() or a ROS publisher)
            
        Returns:
            True if setup successful
        """
        try:
            # Create adapter
            adapter = adapter_registry.create(
                "base_twist",
                twist_callback=twist_callback,
                dof=3,
            )
            
            # Create hardware component
            component = HardwareComponent(
                hardware_id="base",
                hardware_type=HardwareType.MANIPULATOR,
                joints=["base_vx", "base_vy", "base_wz"],
            )
            
            # Add hardware to coordinator
            if not coordinator.add_hardware(adapter, component):
                logger.error("Failed to add base hardware to coordinator")
                return False
            
            logger.info("Base hardware adapter set up successfully")
            return True
            
        except Exception as e:
            logger.error(f"Failed to set up base hardware: {e}")
            return False


replanning_a_star_planner = ReplanningAStarPlanner.blueprint

__all__ = ["ReplanningAStarPlanner", "replanning_a_star_planner"]
