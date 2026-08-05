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

"""Simulation xArm perception manipulation blueprints."""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.grasping.grasp_gen_x import GraspGenXModule
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.perception.sim_object_scene import SimObjectScene
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.xarm.config import (
    XARM7_SIM_PATH,
    XARM_GRASP_SIM_PATH,
    make_xarm7_sim_hardware,
    make_xarm7_sim_module_kwargs,
    make_xarm7_sim_robot_config,
)
from dimos.robot.manipulators.xarm.grasp_config import make_xarm_graspgenx_config
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.utils.data import LfsPath
from dimos.visualization.rerun.bridge import RerunBridgeModule


def _xarm7_perception_sim(
    scene_path: object,
    static_box_obstacles: tuple = (),
    object_scene: object | None = None,
    pick_and_place_kwargs: dict[str, object] | None = None,
) -> object:
    hw = make_xarm7_sim_hardware(scene_path)
    return autoconnect(
        PickAndPlaceModule.blueprint(
            robots=[make_xarm7_sim_robot_config()],
            planning_timeout=10.0,
            visualization={"backend": "viser"},
            heuristic_grasp_fallback=True,
            static_box_obstacles=list(static_box_obstacles),
            **(pick_and_place_kwargs or {}),
        ),
        MujocoSimModule.blueprint(**make_xarm7_sim_module_kwargs(scene_path)),
        object_scene or ObjectSceneRegistrationModule.blueprint(target_frame="world"),
        coordinator(hardware=[hw], tasks=[trajectory_task(hw)]),
        RerunBridgeModule.blueprint(),
    )


xarm_perception_sim = _xarm7_perception_sim(XARM7_SIM_PATH)

# The room-and-objects scene with learned grasps: GraspGenX proposals feed
# pick's provider path, and the table matches data/xarm_grasp_sim/scene.xml so
# the planner always respects it.
_XARM_GRASP_TABLE = {"name": "table", "center": (0.47, 0.0, 0.065), "size": (0.38, 0.60, 0.13)}

# Ground-truth detections from sim state instead of the camera: perception is
# the weak link in this scene, and grasping is what we are testing.
_XARM_GRASPGENX = make_xarm_graspgenx_config()
_XARM_GRASP_MESH_DIR = LfsPath("xarm_grasp_sim") / "assets" / "manip"
_XARM_GRASP_OBJECTS = {
    name: str(_XARM_GRASP_MESH_DIR / f"{name}.obj")
    for name in ("bottle", "box", "can", "cup", "marker", "tape")
}

xarm_grasp_sim = autoconnect(
    _xarm7_perception_sim(
        XARM_GRASP_SIM_PATH,
        static_box_obstacles=(_XARM_GRASP_TABLE,),
        object_scene=SimObjectScene.blueprint(objects=_XARM_GRASP_OBJECTS),
        pick_and_place_kwargs={
            "max_grasp_candidates_to_check": 30,
            "grasp_viz_gripper": _XARM_GRASPGENX.gripper,
            "grasp_viz_frame_to_tcp": _XARM_GRASPGENX.grasp_frame_to_tcp,
            "use_mesh_obstacles": True,
        },
    ),
    GraspGenXModule.blueprint(
        **_XARM_GRASPGENX.model_dump(exclude={"rpc_transport", "tf_transport", "g"})
    ),
)
