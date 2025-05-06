from dataclasses import dataclass
from dimos.types.path import Path
from dimos.types.vector import Vector
from dimos.robot.unitree_webrtc.type.map import Map
from dimos.robot.unitree_webrtc.connection import Connection
from dimos.robot.global_planner.planner import AstarPlanner

from go2_webrtc_driver.constants import VUI_COLOR


class Color(VUI_COLOR): ...


class UnitreeGo2(Connection):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.global_planner = AstarPlanner(
            set_local_nav=lambda: self.navigate_path_local,
            get_costmap=lambda: self.map.costmap,
            get_robot_pos=lambda: [0, 0, 0],
        )
