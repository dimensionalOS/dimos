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

import multiprocessing as mp
import os
from typing import Optional, Union

from dask.distributed import Client, LocalCluster
from go2_webrtc_driver.constants import VUI_COLOR

from dimos.robot.global_planner.planner import AstarPlanner
from dimos.robot.local_planner import navigate_path_local
from dimos.robot.unitree.unitree_skills import MyUnitreeSkills
from dimos.robot.unitree_webrtc.connection import WebRTCRobot
from dimos.robot.unitree_webrtc.type.map import Map
from dimos.skills.skills import AbstractSkill, SkillLibrary
from dimos.types.vector import Vector
from dimos.utils.reactive import getter_streaming


class Color(VUI_COLOR): ...


def init_dask_client():
    process_count = mp.cpu_count()
    cluster = LocalCluster(n_workers=process_count, threads_per_worker=1)
    client = Client(cluster)
    return client


class UnitreeGo2(WebRTCRobot):
    def __init__(
        self,
        ip: str,
        mode: str = "ai",
        skills: Optional[Union[MyUnitreeSkills, AbstractSkill]] = None,
        skill_library: SkillLibrary = None,
        output_dir: str = os.path.join(os.getcwd(), "assets", "output"),
    ):
        super().__init__(ip=ip, mode=mode)

        self.odom = getter_streaming(self.odom_stream())
        dask_client = init_dask_client()
        self.map = dask_client.submit(Map, actor=True).result()

        self.lidar_stream().subscribe(lambda frame: self.map.add_frame(frame).result())

        self.global_planner = AstarPlanner(
            set_local_nav=lambda path, stop_event=None, goal_theta=None: navigate_path_local(
                self, path, timeout=120.0, goal_theta=goal_theta, stop_event=stop_event
            ),
            get_costmap=lambda: self.map.costmap().result(),
            get_robot_pos=lambda: self.odom().pos,
        )

        # # Initialize skills
        # if skills is None:
        #     skills = MyUnitreeSkills(robot=self)

        # self.skill_library = skills if skills else SkillLibrary()

        # if self.skill_library is not None:
        #     for skill in self.skill_library:
        #         if isinstance(skill, AbstractRobotSkill):
        #             self.skill_library.create_instance(skill.__name__, robot=self)
        #     if isinstance(self.skill_library, MyUnitreeSkills):
        #         self.skill_library._robot = self
        #         self.skill_library.init()
        #         self.skill_library.initialize_skills()

        # # Camera stuff
        # self.camera_intrinsics = [819.553492, 820.646595, 625.284099, 336.808987]
        # self.camera_pitch = np.deg2rad(0)  # negative for downward pitch
        # self.camera_height = 0.44  # meters

        # os.makedirs(self.output_dir, exist_ok=True)

        # # Initialize visual servoing if enabled
        # if self.get_video_stream() is not None:
        #     self.person_tracker = PersonTrackingStream(
        #         camera_intrinsics=self.camera_intrinsics,
        #         camera_pitch=self.camera_pitch,
        #         camera_height=self.camera_height,
        #     )
        #     self.object_tracker = ObjectTrackingStream(
        #         camera_intrinsics=self.camera_intrinsics,
        #         camera_pitch=self.camera_pitch,
        #         camera_height=self.camera_height,
        #     )
        #     person_tracking_stream = self.person_tracker.create_stream(self.get_video_stream())
        #     object_tracking_stream = self.object_tracker.create_stream(self.get_video_stream())

        #     self.person_tracking_stream = person_tracking_stream
        #     self.object_tracking_stream = object_tracking_stream

        # Initialize the local planner and create BEV visualization stream
        # self.local_planner = VFHPurePursuitPlanner(
        #     robot=self,
        #     robot_width=0.36,  # Unitree Go2 width in meters
        #     robot_length=0.6,  # Unitree Go2 length in meters
        #     max_linear_vel=0.5,
        #     lookahead_distance=0.6,
        #     visualization_size=500,  # 500x500 pixel visualization
        # )

        # Create the visualization stream at 5Hz
        # self.local_planner_viz_stream = self.local_planner.create_stream(frequency_hz=5.0)

    def move(self, vector: Vector):
        super().move(vector)

    def get_skills(self) -> Optional[SkillLibrary]:
        return self.skill_library

    @property
    def costmap(self):
        return self.map.costmap
