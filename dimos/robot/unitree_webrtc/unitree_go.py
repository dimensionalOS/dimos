from dataclasses import dataclass
from dimos.types.path import Path
from dimos.types.vector import Vector
from dimos.robot.unitree_webrtc.type.map import Map
from dimos.robot.unitree_webrtc.connection import Go2WebRTCConnection
from dimos.robot.global_planner.planner import AstarPlanner


class Mapper:
    def __init__(self):
        self.map = Map()

    def map_stream(self):
        return self.map.consume(self.lidar_stream())


class GlobalPlanner:
    def navigate(self, target: Vector):
        self.topic_latest("costmap")


class LocalPlanner:
    def navigate_local(self, path: Path): ...


@dataclass
class UnitreeGo2(Go2WebRTCConnection):
    def __post_init__(self):
        self.global_planner = AstarPlanner(
            set_local_nav=self.navigate_path_local,  # needs implementation
            get_costmap=lambda: self.map.costmap,
            get_robot_pos=lambda: [0, 0, 0],
        )  # self.ros_control.transform_euler_pos("base_link"),
