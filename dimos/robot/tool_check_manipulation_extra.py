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

"""Check runtime extras without pytest or development dependencies.

Run each check after an exact sync, e.g.:
    uv sync --locked --no-default-groups --extra planning
    .venv/bin/python -m dimos.robot.tool_check_manipulation_extra planning

Robot assets may be downloaded. No physical hardware or paid APIs are used.
Imports are resolved per check because narrower extras omit later capabilities.
"""

import argparse
from importlib import import_module
from importlib.util import find_spec
from pathlib import Path

import numpy as np

from dimos.control.tasks.pose_target_ik import PinkPoseTargetSolver, PoseTargetIKTaskConfig
from dimos.core.coordination.blueprints import Blueprint
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.get_all_blueprints import get_blueprint_by_name
from dimos.robot.manipulators.xarm.config import make_xarm6_model_config


def check_control() -> None:
    for module in (
        "dimos.hardware.manipulators.xarm.adapter",
        "dimos.hardware.manipulators.piper.adapter",
        "dimos.hardware.whole_body.damiao.adapter",
        "dimos.hardware.manipulators.galaxea_a1z.gs_usb_bus",
        "portal",
        "pygame",
    ):
        import_module(module)
    assert isinstance(get_blueprint_by_name("coordinator-basic"), Blueprint)
    assert isinstance(get_blueprint_by_name("coordinator-xarm7"), Blueprint)
    model = make_xarm6_model_config()
    state = JointState(name=model.joint_names, position=[0.0] * len(model.joint_names))
    solver = PinkPoseTargetSolver(
        PoseTargetIKTaskConfig(
            joint_names=tuple(model.joint_names), robot_model=model, target_frames=("link_tcp",)
        )
    )
    poses = solver.frame_poses(state, ["link_tcp"])
    command = solver.step(poses, state, dt=0.01)
    assert command is not None
    np.testing.assert_allclose(command.position, state.position, atol=1e-6)
    print("control: hardware imports, coordinator blueprints, and Pink QP passed", flush=True)


def check_planning() -> None:
    world_module = import_module("dimos.manipulation.planning.world.roboplan_world")
    planner_module = import_module("dimos.manipulation.planning.planners.roboplan_planner")
    config_module = import_module("dimos.manipulation.planning.planners.roboplan_config")
    groups = import_module("dimos.manipulation.planning.groups.models")
    transforms = import_module("dimos.msgs.geometry_msgs.Transform")
    vectors = import_module("dimos.msgs.geometry_msgs.Vector3")
    model = make_xarm6_model_config()
    world = world_module.RoboPlanWorld()
    world.load_model(model)
    world.finalize()
    state = JointState(name=model.joint_names, position=[0.0] * len(model.joint_names))
    world.sync_from_joint_state(state)
    group = groups.PlanningGroup(
        id="manipulator",
        joint_names=tuple(model.joint_names),
        base_link=model.base_link,
        tip_link="link_tcp",
    )
    selection = groups.PlanningGroupSelection(
        groups=(group,), group_ids=(group.id,), joint_names=group.joint_names
    )
    planner = planner_module.RoboPlanPlanner(world, config_module.RoboPlanPlannerConfig())
    result = planner.plan_cartesian_path(
        world,
        selection,
        state,
        {
            group.id: (
                transforms.Transform.identity(),
                transforms.Transform(translation=vectors.Vector3(0.005, 0.0, 0.0)),
            )
        },
        config_module.RoboPlanCartesianPathConfig(toppra_blend_deviation=0.0),
    )
    assert result.status.name == "SUCCESS", result.message
    assert len(result.path) >= 2
    assert result.timestamps is not None
    assert len(result.timestamps) == len(result.path)
    assert np.all(np.diff(result.timestamps) > 0)
    import_module("dimos.manipulation.planning.world.drake_world")
    runtime_module = import_module("dimos.manipulation.visualization.viser.runtime")
    config = import_module("dimos.manipulation.visualization.viser.config")
    runtime = runtime_module.ViserRuntime(
        config.ViserVisualizationConfig(port=0, open_browser=False)
    )
    try:
        assert runtime.start() is not None
    finally:
        runtime.close()
    print(
        "planning: xArm Cartesian plan, TOPP-RA timing, Drake import, and Viser passed", flush=True
    )


def check_manipulation() -> None:
    registry = import_module("dimos.robot.all_blueprints").all_blueprints
    for name, target in registry.items():
        if target.startswith(("dimos.robot.manipulators.", "dimos.teleop.quest.blueprints:")):
            print(f"blueprint: {name}", flush=True)
            assert isinstance(get_blueprint_by_name(name), Blueprint)
    assert find_spec("unitree_webrtc_connect") is None, "Arm workflows pulled in Unitree SDK"
    for module in (
        "dimos.agents.mcp.mcp_client",
        "dimos.agents.mcp.mcp_server",
        "dimos.models.segmentation.edge_tam",
        "dimos.models.embedding.treid",
        "dimos.models.embedding.mobileclip",
        "dimos.simulation.engines.mujoco_sim_module",
        "sam2.sam2_image_predictor",
        "moondream",
        "onnxruntime",
    ):
        import_module(module)
    # Construct the real EdgeTAM config on CPU. This exercises Hydra's runtime
    # imports (including timm) without requiring GPU hardware or model weights.
    edge_tam = import_module("dimos.models.segmentation.edge_tam")
    assert edge_tam.__file__ is not None
    config_path = Path(edge_tam.__file__).parent / "configs" / "edgetam.yaml"
    config = import_module("omegaconf").OmegaConf.load(config_path)
    model = import_module("hydra.utils").instantiate(config.model)
    torch = import_module("torch")
    torch.set_num_threads(2)
    model.eval()
    with torch.inference_mode():
        encoded = model.image_encoder(torch.zeros(1, 3, 1024, 1024))
    assert torch.isfinite(encoded["vision_features"]).all()
    mujoco = import_module("mujoco")
    xarm = import_module("dimos.robot.manipulators.xarm.config")
    sim = mujoco.MjModel.from_xml_path(str(xarm.XARM7_SIM_PATH))
    data = mujoco.MjData(sim)
    for _ in range(10):
        mujoco.mj_step(sim, data)
    assert data.time > 0
    assert np.isfinite(data.qpos).all()
    print(
        "manipulation: blueprint imports, agent/MCP imports, EdgeTAM encoder, and MuJoCo steps passed",
        flush=True,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("extra", choices=("control", "planning", "manipulation"))
    extra = parser.parse_args().extra
    assert find_spec("pytest") is None, "Run without default development groups"
    if extra != "manipulation":
        assert find_spec("torch") is None, "Narrow extras pulled in perception dependencies"
    if extra == "control":
        assert find_spec("roboplan") is None, "Control pulled in planning dependencies"
    check_control()
    if extra in ("planning", "manipulation"):
        check_planning()
    if extra == "manipulation":
        check_manipulation()
