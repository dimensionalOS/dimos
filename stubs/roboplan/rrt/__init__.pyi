"""Partial stubs matching RoboPlan's bundled RRT pybind stubs."""

import roboplan.core as roboplan_core

__version__: str

class RRTOptions:
    def __init__(
        self,
        group_name: str = ...,
        max_nodes: int = ...,
        max_connection_distance: float = ...,
        collision_check_step_size: float = ...,
        collision_check_use_bisection: bool = ...,
        goal_biasing_probability: float = ...,
        max_planning_time: float = ...,
        rrt_connect: bool = ...,
        rrt_star: bool = ...,
        rewire_distance: float = ...,
        fast_return: bool = ...,
    ) -> None: ...
    @property
    def group_name(self) -> str: ...
    @group_name.setter
    def group_name(self, arg: str, /) -> None: ...
    @property
    def max_planning_time(self) -> float: ...
    @max_planning_time.setter
    def max_planning_time(self, arg: float, /) -> None: ...
    @property
    def collision_check_use_bisection(self) -> bool: ...
    @collision_check_use_bisection.setter
    def collision_check_use_bisection(self, arg: bool, /) -> None: ...

class RRT:
    def __init__(self, scene: roboplan_core.Scene, options: RRTOptions) -> None: ...
    def plan(
        self,
        start: roboplan_core.JointConfiguration,
        goal: roboplan_core.JointConfiguration,
    ) -> roboplan_core.JointPath | None: ...
