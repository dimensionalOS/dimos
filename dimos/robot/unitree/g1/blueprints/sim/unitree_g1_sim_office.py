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


"""A simulated G1 in office 555 on MuJoCo, walked by GR00T, with a Mid-360 and a head camera on its mounts: drive it with cmd_vel."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.g1.body_sim import G1_ODOM_TF, G1BodySim
from dimos.robot.unitree.g1.walker import G1Walker
from dimos.sim2.camera import SimCamera
from dimos.sim2.livox import LivoxSim
from dimos.sim2.mujoco.sim import MujocoSim
from dimos.sim2.odom_tf import OdomTf
from dimos.sim2.sim import on_mount, sim
from dimos.simulation.scenes.office_555.room import Office555

unitree_g1_sim_office = autoconnect(
    sim({"mujoco": MujocoSim.blueprint()}, default="mujoco"),
    Office555.blueprint(),
    on_mount(
        autoconnect(
            G1BodySim.blueprint(attached_to=Office555.mounts.main.name),
            OdomTf.blueprint(**G1_ODOM_TF),
            G1Walker.blueprint(),
            # Its lidar and camera on the topics a viewer looks for them on, as a real G1's are.
            on_mount(
                LivoxSim.blueprint(attached_to=G1BodySim.mounts.lidar.name),
                G1BodySim.mounts.lidar,
                shared={"lidar"},
            ),
            on_mount(
                SimCamera.blueprint(attached_to=G1BodySim.mounts.camera.name),
                G1BodySim.mounts.camera,
                shared={"color_image", "camera_info"},
            ),
        ),
        Office555.mounts.main,
        # What drives one drives this, and its odometry is where a viewer looks for it.
        shared={"cmd_vel", "odom", "lidar", "color_image", "camera_info"},
    ),
).global_config(robot_model="unitree_g1")
