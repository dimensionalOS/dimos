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

"""A simulated Go2 in office 555 on MuJoCo, on the streams a real Go2's GO2Connection has: drive it from Controller."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.go2.webrtc_sim import Go2WebRtcSim
from dimos.sim2.mujoco.sim import MujocoSim
from dimos.sim2.odom_tf import OdomTf
from dimos.sim2.sim import on_mount, sim
from dimos.simulation.scenes.office_555.room import Office555

unitree_go2_sim_office = autoconnect(
    sim({"mujoco": MujocoSim.blueprint()}, default="mujoco"),
    Office555.blueprint(),
    on_mount(
        autoconnect(
            Go2WebRtcSim.blueprint(attached_to=Office555.mounts.main.name),
            OdomTf.blueprint(tf_frames_local={"odom": "odom", "body": "base_link"}),
        ),
        Office555.mounts.main,
        # Where a real Go2's GO2Connection has them, so what drives or draws one drives or draws this.
        shared={"cmd_vel", "odom", "lidar", "color_image", "camera_info"},
    ),
)
