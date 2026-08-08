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

"""Private xArm grasp-simulation stack used by its public agent blueprint."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.manipulation.grasping.grasp_gen_x import GraspGenXModule
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.manipulation.visualization.rerun import picknplace_rerun_config
from dimos.perception.sim_object_scene import SimObjectScene
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.xarm.config import (
    XARM_GRASP_SIM_PATH,
    make_xarm7_sim_hardware,
    make_xarm7_sim_module_kwargs,
    make_xarm7_sim_robot_config,
)
from dimos.robot.manipulators.xarm.grasp_config import make_xarm_graspgenx_config
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.utils.data import LfsPath
from dimos.visualization.vis_module import vis_module

_TABLE = {"name": "table", "center": (0.47, 0.0, 0.065), "size": (0.38, 0.60, 0.13)}
_GRASPGENX = make_xarm_graspgenx_config()
_MESH_DIR = LfsPath("xarm_grasp_sim") / "assets" / "manip"
_OBJECTS = {
    name: str(_MESH_DIR / f"{name}.obj")
    for name in ("bottle", "box", "can", "cup", "marker", "tape")
}
_hardware = make_xarm7_sim_hardware(XARM_GRASP_SIM_PATH)

_xarm_grasp_sim = autoconnect(
    PickAndPlaceModule.blueprint(
        robots=[make_xarm7_sim_robot_config()],
        planning_timeout=10.0,
        visualization={"backend": "viser"},
        static_box_obstacles=[_TABLE],
        max_grasp_candidates_to_check=30,
        use_mesh_obstacles=True,
        grasp_verification={
            "open_position": 0.85,
            "closed_position": 0.0,
            "held_threshold": 0.02,
        },
    ),
    MujocoSimModule.blueprint(**make_xarm7_sim_module_kwargs(XARM_GRASP_SIM_PATH)),
    SimObjectScene.blueprint(objects=_OBJECTS),
    GraspGenXModule.blueprint(
        **_GRASPGENX.model_dump(exclude={"rpc_transport", "tf_transport", "g"})
    ),
    coordinator(hardware=[_hardware], tasks=[trajectory_task(_hardware)]),
    vis_module(global_config.viewer, rerun_config=picknplace_rerun_config()),
)
