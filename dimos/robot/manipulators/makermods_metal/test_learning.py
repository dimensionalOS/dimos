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

"""Station composition checks; no adapters, cameras, or remote clients are started."""

from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.control.coordinator import ControlCoordinator
from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.eef_trajectory_task.trajectory import EefTrajectory, EefTrajectoryPoint
from dimos.control.tasks.trajectory_task.trajectory_task import TrajectoryExecutionStatus
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.imitation.policy.dreamlabs.backend import DreamlabsBackend
from dimos.imitation.policy.dreamlabs.kinematics import DreamlabsKinematics
from dimos.imitation.policy.module import PolicyModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.protocol.encode.cbor import decode, encode
from dimos.robot.manipulators.makermods_metal.learning import build_metal_learning

SETUPS = [
    (False, ("overhead",)),
    (True, ("wrist_l", "wrist_r")),
    (True, ("overhead", "wrist_l", "wrist_r")),
]


@pytest.mark.parametrize("dual, roles", SETUPS)
def test_collection_and_rollout_share_observation_and_joint_contract(dual, roles):
    collection = build_metal_learning(dual=dual, camera_roles=roles)
    rollout = build_metal_learning(dual=dual, collection=False, camera_roles=roles)
    schema = next(
        a.kwargs["recording_schema"]
        for a in collection.blueprints
        if "recording_schema" in a.kwargs
    )
    policy = next(a.kwargs for a in rollout.blueprints if issubclass(a.module, PolicyModule))
    camera_streams = {f.stream for f in schema.observation.values() if f.dtype == "video"}
    assert camera_streams == set(policy["image_mapping"])
    assert set(policy["image_mapping"].values()) == set(roles)
    assert schema.observation["observation.state"].names == policy["joint_names"]
    assert schema.action["action"].names == policy["joint_names"]
    assert schema.action["action"].stream == "coordinator_joint_command"
    assert schema.action["action"].source_kind == "joint_position_updates"
    assert policy["dreamlabs"].camera_roles == roles
    assert policy["dreamlabs"].mode == "eef"
    assert set(policy["dreamlabs"].arms) == ({"left", "right"} if dual else {"right"})


@pytest.mark.parametrize("dual, roles", SETUPS)
def test_rollout_routes_manual_takeover_and_uses_one_rig_model(dual, roles):
    blueprint = build_metal_learning(dual=dual, collection=False, camera_roles=roles)
    coordinator = next(a for a in blueprint.blueprints if issubclass(a.module, ControlCoordinator))
    planner = next(a for a in blueprint.blueprints if a.module is ManipulationModule)
    policy = next(a.kwargs for a in blueprint.blueprints if issubclass(a.module, PolicyModule))
    tasks = {task.name: task for task in coordinator.kwargs["tasks"]}
    eef = tasks["eef_trajectory"]
    assert eef.params["robot_model"] is planner.kwargs["model"]
    assert policy["dreamlabs"].model is planner.kwargs["model"].model
    assert eef.params["gripper_joint_names"] == [
        arm.gripper_joint for arm in policy["dreamlabs"].arms.values()
    ]
    assert tasks["joint_trajectory"].joint_names == policy["joint_names"]
    for side, arm in policy["dreamlabs"].arms.items():
        teleop = tasks[f"teleop_metal_{side}"]
        assert teleop.priority > eef.priority
        assert teleop.joint_names == arm.joint_names
        assert teleop.params["bindings"] == [{"hand": side, "target_frame": arm.target_frame}]
    assert {"teleop_buttons", "right_cartesian_command"} <= {
        stream.name for stream in coordinator.streams
    }


@pytest.mark.parametrize("dual, roles", SETUPS)
def test_station_camera_and_can_selectors_parse_without_loading_assets(
    dual, roles, tmp_path, mocker
):
    load = mocker.patch(
        "dimos.robot.assets.model.RobotModel.load", side_effect=AssertionError("must stay lazy")
    )
    blueprint = build_metal_learning(dual=dual, camera_roles=roles)
    args = [
        "--recorder.recording",
        str(tmp_path / "recording"),
        "--episodes.task",
        "pick up object",
    ]
    names = {
        "overhead": "OverheadCamera",
        "wrist_l": "LeftWristCamera",
        "wrist_r": "RightWristCamera",
    }
    for role in roles:
        args.extend([f"--{names[role]}.device", f"/dev/{role}"])
    selectors = (
        {"left_can_port": "left-bus", "right_can_port": "right-bus"}
        if dual
        else {"can_port": "single-bus"}
    )
    for key, value in selectors.items():
        args.extend([f"--ControlCoordinator.{key.replace('_', '-')}", value])
    parsed = BlueprintConfigParser(blueprint).parse(args, environ={})
    for role in roles:
        assert parsed.module_kwargs(names[role])["device"] == f"/dev/{role}"
    for key, value in selectors.items():
        assert parsed.module_kwargs("ControlCoordinator")[key] == value
    load.assert_not_called()


@pytest.mark.parametrize("dual, roles", SETUPS)
def test_standard_station_declares_model_tcp_without_loading_assets(dual, roles, mocker):
    load = mocker.patch("dimos.robot.assets.model.RobotModel.load")
    blueprint = build_metal_learning(dual=dual, collection=False, camera_roles=roles)
    policy = next(a.kwargs for a in blueprint.blueprints if issubclass(a.module, PolicyModule))
    planner = next(
        a.kwargs["model"] for a in blueprint.blueprints if a.module is ManipulationModule
    )
    for side, arm in policy["dreamlabs"].arms.items():
        assert arm.sdk_tcp_frame == arm.target_frame
        assert arm.target_frame == (f"{side}_grasp_frame" if dual else "grasp_frame")
        assert arm.target_frame in {group.tip_link for group in planner.planning_groups}
        assert arm.target_to_sdk_xyz is None
        assert arm.target_to_sdk_xyzw is None
    load.assert_not_called()


@pytest.mark.parametrize("dual, roles", SETUPS)
def test_model_tcp_mapping_survives_worker_cbor_round_trip(dual, roles):
    blueprint = build_metal_learning(dual=dual, collection=False, camera_roles=roles)
    config = next(
        a.kwargs["dreamlabs"] for a in blueprint.blueprints if issubclass(a.module, PolicyModule)
    )
    restored = decode(encode(config))
    assert restored.arms == config.arms
    assert encode(restored.model) == encode(config.model)
    assert all(arm.sdk_tcp_frame == arm.target_frame for arm in restored.arms.values())


@pytest.mark.self_hosted
@pytest.mark.parametrize("dual, roles", SETUPS)
def test_legacy_tool_override_becomes_a_link_shared_with_every_model_consumer(dual, roles):
    sides = ("left", "right") if dual else ("right",)
    transform = ((0.01, -0.02, 0.12), (0.0, 0.0, 2**-0.5, 2**-0.5))
    blueprint = build_metal_learning(
        dual=dual,
        collection=False,
        camera_roles=roles,
        tcp_transforms=dict.fromkeys(sides, transform),
    )
    config = next(
        a.kwargs["dreamlabs"] for a in blueprint.blueprints if issubclass(a.module, PolicyModule)
    )
    planner = next(
        a.kwargs["model"] for a in blueprint.blueprints if a.module is ManipulationModule
    )
    coordinator = next(a for a in blueprint.blueprints if issubclass(a.module, ControlCoordinator))
    eef = next(task for task in coordinator.kwargs["tasks"] if task.type == "eef_trajectory")
    assert eef.params["robot_model"] is planner
    assert planner.model is config.model
    fk = DreamlabsKinematics(config)
    positions = {name: 0.0 for arm in config.arms.values() for name in arm.joint_names}
    mount_from_sdk = np.eye(4)
    mount_from_sdk[:3, :3] = Rotation.from_quat(transform[1]).as_matrix()
    mount_from_sdk[:3, 3] = transform[0]
    for side, arm in config.arms.items():
        prefix = f"{side}_" if dual else ""
        expected = (
            np.linalg.inv(fk.root_pose(arm.base_frame, positions))
            @ fk.root_pose(f"{prefix}gripper_base", positions)
            @ mount_from_sdk
        )
        np.testing.assert_allclose(fk.sdk_pose(side, positions), expected, atol=1e-12)


@pytest.mark.self_hosted
@pytest.mark.parametrize("dual, roles", SETUPS)
def test_mock_station_instantiates_real_tasks_and_executes_both_output_paths(dual, roles, mocker):
    blueprint = build_metal_learning(dual=dual, collection=False, camera_roles=roles)
    atom = next(a for a in blueprint.blueprints if issubclass(a.module, ControlCoordinator))
    policy = next(a.kwargs for a in blueprint.blueprints if issubclass(a.module, PolicyModule))
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.start")
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.serve_module_rpc")
    coordinator = atom.module(**atom.kwargs, publish_joint_state=False)
    try:
        coordinator._setup_from_config()
        positions = {name: 0.0 for name in policy["joint_names"]}
        state = CoordinatorState(
            joints=JointStateSnapshot(joint_positions=positions), t_now=10.0, dt=0.01
        )
        coordinator._set_latest_state(state)
        chunk = JointTrajectory(
            joint_names=policy["joint_names"],
            points=[
                TrajectoryPoint(positions=list(positions.values())),
                TrajectoryPoint(positions=list(positions.values()), time_from_start=1.0),
            ],
        )
        result = coordinator.task_invoke("joint_trajectory", "execute", {"trajectory": chunk})
        assert result.status == TrajectoryExecutionStatus.ACCEPTED
        joint_output = coordinator._tasks["joint_trajectory"].compute(state)
        assert joint_output.positions == [0.0] * len(positions)
        assert coordinator.task_invoke("joint_trajectory", "cancel").safe
        fk = DreamlabsKinematics(policy["dreamlabs"])
        for side, arm in policy["dreamlabs"].arms.items():
            grasp = fk.model.get_joint(f"{side}_grasp_frame_joint" if dual else "grasp_frame_joint")
            assert grasp.origin_xyz == (0.0, 0.0, 0.08)
            sdk_pose = fk.sdk_pose(side, positions)
            target = fk.canonical_pose(side, sdk_pose)
            np.testing.assert_allclose(
                target.position.to_numpy(), fk.root_pose(arm.target_frame, positions)[:3, 3]
            )
        poses = {
            arm.target_frame: fk.to_pose(fk.root_pose(arm.target_frame, positions))
            for arm in policy["dreamlabs"].arms.values()
        }
        grippers = {arm.gripper_joint: 0.0 for arm in policy["dreamlabs"].arms.values()}
        eef_chunk = EefTrajectory(
            [
                EefTrajectoryPoint(time_from_start=0.0, poses=poses, grippers=grippers),
                EefTrajectoryPoint(time_from_start=1.0, poses=poses, grippers=grippers),
            ]
        )
        result = coordinator.task_invoke("eef_trajectory", "execute", {"trajectory": eef_chunk})
        assert result.status == TrajectoryExecutionStatus.ACCEPTED
        eef_output = coordinator._tasks["eef_trajectory"].compute(state)
        assert eef_output is not None
        assert eef_output.positions == pytest.approx([0.0] * len(positions), abs=0.01)
        assert coordinator.task_invoke("eef_trajectory", "cancel").safe
    finally:
        coordinator.stop()


@pytest.mark.parametrize("dual, roles", SETUPS)
def test_metal_never_accepts_yam_joint_output(dual, roles):
    blueprint = build_metal_learning(dual=dual, collection=False, camera_roles=roles)
    policy = next(a.kwargs for a in blueprint.blueprints if issubclass(a.module, PolicyModule))
    policy["dreamlabs"] = policy["dreamlabs"].model_copy(update={"mode": "joint"})
    config = SimpleNamespace(**policy, fps=None, execution_steps=None, policy_joint_names=None)
    with pytest.raises(ValueError, match="canonical dual YAM"):
        DreamlabsBackend(config)
