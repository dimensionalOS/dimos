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


from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.robot.assets.model import UsdModel
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config
from dimos.robot.unitree.go2.sim2 import Go2Sim
from dimos.robot.usd_manager import UsdManager
from dimos.sim2.assets.logistics import LOGISTICS
from dimos.sim2.camera import SimCamera
from dimos.sim2.livox import LivoxSim
from dimos.sim2.sim import sim_module
from dimos.sim2.spec import SimulatedRobot
from dimos.simulation.go2_legged.robot import CONTROL_DT
from dimos.visualization.vis_module import vis_module

ROBOT = "go2"
SPOT = (0.0, 0.0, 0.35)
SCENE = "scene"
SCENE_MANAGER = f"{SCENE}/{UsdManager.name}"

_room = UsdModel.from_file(LOGISTICS).with_frames({ROBOT: ("logistics", SPOT, (0.0, 0.0, 0.0))})

_lidar = LivoxSim.blueprint(
    frame_id=Go2Sim.lidar_mount_frame, go2_occlusion=True, scene_manager=SCENE_MANAGER
)
_camera = SimCamera.blueprint(frame_id=Go2Sim.camera_mount_frame, scene_manager=SCENE_MANAGER)

_robot = UsdManager.blueprint(
    model=Go2Sim.usd,
    mounts={Go2Sim.lidar_mount_frame: _lidar, Go2Sim.camera_mount_frame: _camera},
).remappings([(UsdManager, "coordinator_joint_state", "joint_state")])

_scene = UsdManager.blueprint(
    model=_room,
    mounts={ROBOT: _robot},
    frame_id_prefix="",
    tf_renames={"logistics": "world"},
).namespace(SCENE, expose={"render_file_request", "file_info", "tf"})

unitree_go2_sim2_mid360 = autoconnect(
    sim_module(
        scene_manager=SCENE_MANAGER,
        robots=[SimulatedRobot(ROBOT, Go2Sim.usd, control_period=CONTROL_DT)],
    ),
    vis_module(viewer_backend=global_config.viewer, rerun_config=rerun_config),
    Go2Sim.blueprint(robot_id=ROBOT),
    _scene,
    _robot,
    _lidar,
    _camera,
).global_config(n_workers=8, robot_model="unitree_go2")
