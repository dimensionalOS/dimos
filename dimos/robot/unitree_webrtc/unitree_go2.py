from dimos.robot.unitree_webrtc.type.map import Map
from dimos.robot.unitree_webrtc.connection import Connection
from dimos.robot.global_planner.planner import AstarPlanner
from dimos.utils.reactive import backpressure

from dimos.utils.reactive import getter_streaming

from go2_webrtc_driver.constants import VUI_COLOR


class Color(VUI_COLOR): ...


class UnitreeGo2(Connection):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.odom = getter_streaming(self.odom_stream())

        self.map = Map()
        self.map_stream = backpressure(self.map.consume(self.lidar_stream()))

        self.global_planner = AstarPlanner(
            set_local_nav=lambda: self.navigate_path_local,
            get_costmap=lambda: self.map.costmap,
            get_robot_pos=lambda: self.odom().pos,
        )

    @property
    def costmap(self):
        return self.map.costmap
