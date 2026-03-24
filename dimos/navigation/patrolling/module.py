# Copyright 2026 Dimensional Inc.
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


import threading

from dimos_lcm.std_msgs import Bool
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.navigation.patrolling.create_patrol_router import create_patrol_router
from dimos.navigation.patrolling.routers.patrol_router import PatrolRouter
from dimos.navigation.replanning_a_star.module_spec import ReplanningAStarPlannerSpec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class PatrollingModule(Module):
    odom: In[PoseStamped]
    global_costmap: In[OccupancyGrid]
    goal_reached: In[Bool]
    goal_request: Out[PoseStamped]

    _global_config: GlobalConfig
    _router: PatrolRouter
    _planner_spec: ReplanningAStarPlannerSpec

    _clearance_multiplier = 0.5

    def __init__(self, g: GlobalConfig = global_config) -> None:
        super().__init__()
        self._global_config = g
        clearance_radius_m = self._global_config.robot_width * self._clearance_multiplier
        self._router = create_patrol_router("coverage", clearance_radius_m)

        self._patrol_lock = threading.RLock()
        self._patrol_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._goal_reached_event = threading.Event()
        self._goal_or_stop_event = threading.Event()
        self._latest_pose: PoseStamped | None = None

    @rpc
    def start(self) -> None:
        super().start()

        self._disposables.add(Disposable(self.odom.subscribe(self._on_odom)))
        self._disposables.add(
            Disposable(self.global_costmap.subscribe(self._router.handle_occupancy_grid))
        )
        self._disposables.add(Disposable(self.goal_reached.subscribe(self._on_goal_reached)))

    @rpc
    def stop(self) -> None:
        self._stop_patrolling()
        super().stop()

    @skill
    def start_patrol(self) -> str:
        """Start patrolling the known area. The robot will continuously pick patrol goals from the router and navigate to them until `stop_patrol` is called."""
        self._router.reset()

        with self._patrol_lock:
            if self._patrol_thread is not None and self._patrol_thread.is_alive():
                return "Patrol is already running. Use `stop_patrol` to stop."
            self._planner_spec.set_replanning_enabled(False)
            self._planner_spec.set_safe_goal_clearance(
                self._global_config.robot_rotation_diameter / 2 + 0.2
            )
            self._stop_event.clear()
            self._patrol_thread = threading.Thread(
                target=self._patrol_loop, daemon=True, name=self.__class__.__name__
            )
            self._patrol_thread.start()
        return "Patrol started. Use `stop_patrol` to stop."

    @rpc
    def is_patrolling(self) -> bool:
        with self._patrol_lock:
            return self._patrol_thread is not None and self._patrol_thread.is_alive()

    @skill
    def stop_patrol(self) -> str:
        """Stop the ongoing patrol."""
        self._stop_patrolling()
        return "Patrol stopped."

    def _on_odom(self, msg: PoseStamped) -> None:
        self._latest_pose = msg
        self._router.handle_odom(msg)

    def _on_goal_reached(self, _msg: Bool) -> None:
        self._goal_reached_event.set()
        self._goal_or_stop_event.set()

    def _patrol_loop(self) -> None:
        while not self._stop_event.is_set():
            goal = self._router.next_goal()
            if goal is None:
                logger.info("No patrol goal available, retrying in 2s")
                if self._stop_event.wait(timeout=2.0):
                    break
                continue

            self._goal_reached_event.clear()
            self.goal_request.publish(goal)

            # Wait until goal is reached or stop is requested.
            self._goal_or_stop_event.wait()
            self._goal_or_stop_event.clear()

    def _stop_patrolling(self) -> None:
        self._stop_event.set()
        self._goal_or_stop_event.set()
        self._planner_spec.set_replanning_enabled(True)
        self._planner_spec.reset_safe_goal_clearance()

        # Publish current position as goal to cancel in-progress navigation.
        pose = self._latest_pose
        if pose is not None:
            self.goal_request.publish(pose)
        with self._patrol_lock:
            if self._patrol_thread is not None:
                self._patrol_thread.join()
                self._patrol_thread = None
