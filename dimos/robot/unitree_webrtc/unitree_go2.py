import functools
from dimos.robot.unitree_webrtc.type.map import Map
from dimos.robot.unitree_webrtc.connection import Connection
from dimos.robot.global_planner.planner import AstarPlanner
from dimos.utils.reactive import backpressure, getter_streaming
from dimos.types.vector import Vector
from dimos.types.position import Position


from go2_webrtc_driver.constants import VUI_COLOR


class Color(VUI_COLOR): ...


class UnitreeGo2(Connection):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.map = Map(pos=lambda: self.coords)

        self._odom_getter = getter_streaming(self.odom_stream())

        self.global_planner = AstarPlanner(
            set_local_nav=lambda path, stop_event, goal_theta: self.navigate_path_local(path),
            get_costmap=lambda: self.costmap,
            get_robot_pos=lambda: self.odom.coords.to_2d(),
        )

    def navigate_path_local(self, path, stop_event=None, goal_theta=None):
        print("NAV", path)

    @functools.lru_cache(maxsize=None)
    def map_stream(self):
        return backpressure(self.map.consume(self.lidar_stream()))

    @property
    def costmap(self):
        return self.map.costmap

    @property
    def coords(self) -> Vector:
        return self._odom_getter().coords

    @property
    def rot(self) -> Vector:
        return self._odom_getter().pos

    @property
    def odom(self) -> Position:
        return self._odom_getter()
