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

"""Native-boundary mocks for the radio-only SDK planning/dispatch contract."""

from contextlib import contextmanager
import copy
import hashlib
from threading import RLock
import time
from types import SimpleNamespace

import numpy as np
import pytest

from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_spec import (
    ExecutionResult,
    ExecutionStatus,
    PlanResult,
    PlanStatus,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.protocol.rpc.zenohrpc import ZenohRPC
from dimos.simulation.behavior.r1pro_model import simulation_model_config
from dimos.simulation.behavior.radio_bimanual import BimanualRadioManipulationModule
from dimos.simulation.behavior.radio_checkpoint import (
    RadioCheckpointScene,
    RadioEpisodeTerminalError,
    RadioGraspCheckpoint,
    radio_episode_terminal,
)
from dimos.simulation.behavior.radio_contact_evidence import radio_finger_contacts
from dimos.utils.transform_utils import pose_to_matrix

JOINT = "r1pro/right_arm_joint1"


def trajectory(end=0.01):
    return JointTrajectory(
        joint_names=[JOINT],
        points=[
            TrajectoryPoint(positions=[0.0], velocities=[0.0], time_from_start=0.0),
            TrajectoryPoint(positions=[end], velocities=[0.0], time_from_start=1.0),
        ],
    )


@pytest.fixture
def collision_scene(mocker, tmp_path):
    # Only native/world I/O is mocked. Exercise normal scene registration,
    # ownership, pair restoration and support-geometry interpolation.
    mesh = tmp_path / "part.obj"
    mesh.write_text("v -.01 -.01 0\nv .01 -.01 0\nv 0 .01 0\nf 1 2 3\n")
    geometry = {
        "model": "wxnicr",
        "provenance": "synthetic native-boundary fixture",
        "meshes": [{"path": str(mesh), "sha256": hashlib.sha256(mesh.read_bytes()).hexdigest()}]
        * 14,
        "table": {"pose": np.eye(4).tolist(), "extent": [2.0, 2.0, 0.1]},
    }
    geometry["table"]["pose"][2][3] = -0.05
    objects = {}
    world = mocker.Mock()
    native = mocker.Mock()
    world._lock = RLock()
    world._require_scene.return_value = native
    world.get_prepared_model.return_value = SimpleNamespace(
        config=SimpleNamespace(joint_names=[JOINT]),
        joint_space=SimpleNamespace(position_limits=lambda: (np.array([-1.0]), np.array([1.0]))),
    )
    world.get_obstacles.side_effect = lambda: list(objects.values())

    def add(obj):
        objects[obj.name] = obj
        return obj.name

    world.add_obstacle.side_effect = add
    world.update_obstacle_pose.side_effect = lambda name, pose: name in objects
    world.check_config_collision_free.return_value = True
    world.check_edge_collision_free.return_value = True

    @contextmanager
    def scratch():
        yield SimpleNamespace(q=None)

    world.scratch_context.side_effect = scratch
    world.set_joint_state.side_effect = lambda ctx, state: setattr(ctx, "q", state)

    def fk(ctx, link):
        mat = np.eye(4)
        if ctx.q is not None:
            mat[2, 3] = ctx.q.position[0] - 0.0015
        return mat

    world.get_link_pose.side_effect = fk
    scene = RadioCheckpointScene(world, geometry)
    request = {
        "phase": "departure",
        "signature": scene.signature,
        "radio_pose": np.eye(4).tolist(),
        "table_pose": scene.table.tolist(),
        "gripper_from_radio": np.eye(4).tolist(),
    }
    return scene, request, native, geometry


def test_support_departure_checks_effective_trajectory_and_restores_pairs(collision_scene):
    scene, request, native, _ = collision_scene
    result = scene.validate(
        JointState(name=[JOINT], position=[0.0]), trajectory(), [JOINT], request
    )
    assert result["geometry_signature"] == scene.signature
    assert result["support_departure"]["initial_height"] == pytest.approx(-0.0015)
    assert result["support_departure"]["final_height"] == pytest.approx(0.0085)
    calls = [c.args for c in native.setCollisions.call_args_list]
    assert ("radio_part_0", "table", False) in calls
    assert ("right_gripper_finger_link1", "radio_part_6", False) in calls
    assert calls[-3:] == [
        ("radio_part_0", "table", True),
        ("right_gripper_finger_link1", "radio_part_6", True),
        ("right_gripper_finger_link2", "radio_part_6", True),
    ]


@pytest.mark.parametrize("end", [-0.01, 0.001])
def test_downward_or_insufficient_departure_is_rejected(collision_scene, end):
    scene, request, _, _ = collision_scene
    with pytest.raises(RuntimeError, match="support contact"):
        scene.validate(JointState(name=[JOINT], position=[0.0]), trajectory(end), [JOINT], request)


def test_all_table_collisions_enabled_after_clearance(collision_scene):
    scene, request, native, _ = collision_scene
    request["phase"] = "lift"
    with scene.context(request):
        assert native.setCollisions.call_args_list[-3].args == ("radio_part_0", "table", True)


@pytest.mark.parametrize("left_error,accepted", [(0.0019, True), (0.0021, False)])
def test_reposition_keeps_strict_both_tcp_endpoints_and_table_pairs(
    collision_scene, left_error, accepted
):
    scene, request, native, _ = collision_scene
    right = np.eye(4)
    right[2, 3] = 0.0085
    left = right.copy()
    left[0, 3] = left_error
    request.update(
        phase="reposition", right_target_pose=right.tolist(), left_target_pose=left.tolist()
    )
    if accepted:
        result = scene.validate(
            JointState(name=[JOINT], position=[0]), trajectory(), [JOINT], request
        )
        assert result["endpoint_tcp_errors"]["right"]["position_m"] == pytest.approx(0)
        assert result["endpoint_tcp_errors"]["left"]["position_m"] == pytest.approx(left_error)
    else:
        with pytest.raises(RuntimeError, match="Reposition left endpoint"):
            scene.validate(JointState(name=[JOINT], position=[0]), trajectory(), [JOINT], request)
    assert not any(
        c.args == ("radio_part_0", "table", False) for c in native.setCollisions.call_args_list
    )


def test_reposition_uses_sdk_pose_targets_not_joint_commands(sdk_module, mocker):
    module, state = sdk_module
    right = PoseStamped(frame_id="world", position=[0, 0.1, 0.15])
    left = PoseStamped(frame_id="world", position=[0.05, 0.1, 0.15])
    request = {
        "phase": "reposition",
        "right_target_pose": pose_to_matrix(right).tolist(),
        "left_target_pose": pose_to_matrix(left).tolist(),
    }
    plan = SimpleNamespace(trajectory=trajectory(), message="SDK pose path")
    planning = mocker.patch.object(
        ManipulationModule,
        "plan_to_poses",
        return_value=PlanResult(PlanStatus.SUCCEEDED, plan=plan),
    )
    cartesian = mocker.patch.object(module, "generate_cartesian_plan")
    module.plan_radio_checkpoint(right, request, state, True, left)
    planning.assert_called_once_with(
        {"right_arm": right, "left_arm": left}, speed_scale=0.05, auxiliary_groups=["torso"]
    )
    module._radio_checkpoint.validate.assert_called_once_with(
        state, plan.trajectory, (JOINT,), request
    )
    cartesian.assert_not_called()


def test_scene_signature_or_checksum_change_rejected(collision_scene, mocker):
    scene, request, _, geometry = collision_scene
    request["signature"] = "different"
    with pytest.raises(ValueError, match="identity"):
        with scene.context(request):
            pytest.fail("must reject")
    geometry["meshes"][0]["sha256"] = "different"
    with pytest.raises(ValueError, match="checksum"):
        RadioCheckpointScene(mocker.Mock(), geometry)


@pytest.fixture
def client(mocker, collision_scene):
    scene, _, _, _ = collision_scene
    proxy = mocker.Mock()
    arm = mocker.Mock()
    arm.info = SimpleNamespace(id="right_arm", joint_names=(JOINT,))
    arm.rpc = proxy
    coordinator = mocker.Mock()
    coordinator.prepare_development_trajectory.side_effect = lambda t: t
    mocker.patch.object(scene, "validate", return_value={"effective_digest": "test"})
    state = mocker.Mock(return_value=JointState(name=[JOINT], position=[0.0]))
    observation = mocker.Mock(
        side_effect=lambda: {
            "observed_at_monotonic": time.monotonic(),
            "episode": "episode",
            "radio_pose": np.eye(4).tolist(),
            "table_pose": scene.table.tolist(),
        }
    )
    guard = mocker.Mock()
    runner = RadioGraspCheckpoint(arm, coordinator, scene, state, observation, guard, [])
    plan = SimpleNamespace(trajectory=trajectory(), plan_id="same-stored-id")
    proxy.plan_radio_checkpoint.return_value = PlanResult(PlanStatus.SUCCEEDED, plan=plan)
    proxy.execute.return_value = ExecutionResult(ExecutionStatus.ACCEPTED)
    proxy.wait_for_execution.return_value = ExecutionResult(ExecutionStatus.COMPLETED)
    target = PoseStamped(frame_id="world", position=[0, 0, -0.0015])
    return runner, proxy, coordinator, observation, target


def test_dispatch_uses_same_stored_plan_id_after_effective_validation(client):
    runner, proxy, coordinator, _, target = client
    runner.move(target, "pregrasp")
    assert proxy.execute.call_args.kwargs["plan_id"] == "same-stored-id"
    runner.scene.validate.assert_called_once()
    coordinator.authorize_development_trajectory.assert_called_once()
    assert runner.evidence[-1]["execution_status"] == "COMPLETED"
    assert "measured_arrival_fk" in runner.evidence[-1]
    assert runner.evidence[-1]["measured_start_joints"]
    assert runner.evidence[-1]["target_pose"] == pose_to_matrix(target).tolist()
    assert runner.evidence[-1]["planning_result"]["status"] == "SUCCEEDED"


def test_checkpoint_dispatch_hook_keeps_same_validated_id_and_default_execution_off(client, mocker):
    runner, proxy, _, _, target = client
    hook = mocker.Mock(return_value=ExecutionResult(ExecutionStatus.ACCEPTED))
    runner.move(target, "pregrasp", dispatch=hook, cancelled=lambda: False)
    assert hook.call_args.args[0] == "same-stored-id"
    assert hook.call_args.args[1] > 0
    proxy.execute.assert_not_called()


def test_cancelled_checkpoint_never_plans_or_dispatches(client):
    runner, proxy, _, _, target = client
    with pytest.raises(RuntimeError, match="Cancelled before checkpoint planning"):
        runner.move(target, "pregrasp", cancelled=lambda: True)
    proxy.plan_radio_checkpoint.assert_not_called()
    proxy.execute.assert_not_called()


@pytest.mark.parametrize(
    "point,normal,accepted",
    [
        ([0.01, 0, 0], [1, 0, 0], True),
        ([0, 0, 0], [1, 0, 0], False),
        ([0.01, 0, 0], [-1, 0, 0], False),
    ],
)
def test_sensor_contact_matches_physical_surface_without_marker_coordinate(
    collision_scene, monkeypatch, point, normal, accepted
):
    scene, _, _, _ = collision_scene
    vertices = np.array(
        [[x, y, z] for x in (-0.01, 0.01) for y in (-0.01, 0.01) for z in (-0.01, 0.01)]
    )
    monkeypatch.setattr(scene, "vertices", vertices)
    monkeypatch.setattr(scene, "part_vertices", [vertices])
    request = {
        "contact": {
            "source": "caller_sensor_intent",
            "surface_in_radio": point,
            "outward_normal_in_radio": normal,
            "pad_in_gripper": [0, 0, -0.078],
        }
    }
    if accepted:
        assert scene._contact_points(request)[0] == pytest.approx(point)
    else:
        with pytest.raises(ValueError, match="outward physical body surface"):
            scene._contact_points(request)


@pytest.fixture
def terminal_goal_status():
    return {
        "state": "finished",
        "error": None,
        "episode": {
            "id": "episode",
            "step": 42,
            "success": True,
            "terminated": True,
            "truncated": False,
            "info": {
                "done": {"success": True, "goal_status": {"satisfied": [0], "unsatisfied": []}}
            },
        },
    }


def test_terminal_goal_cancels_without_reading_stale_feedback_or_claiming_arrival(
    client, mocker, terminal_goal_status
):
    runner, proxy, _, observation, target = client
    running = {"state": "running", "episode": {"id": "episode"}}
    runner.episode_status = mocker.Mock(side_effect=[running, terminal_goal_status])
    proxy.cancel.return_value = ExecutionResult(ExecutionStatus.ABORTED)
    with pytest.raises(RadioEpisodeTerminalError) as caught:
        runner.move(target, "pregrasp")
    assert caught.value.outcome["kind"] == "TASK_GOAL_MET"
    assert caught.value.outcome["motion_arrived"] is False
    assert caught.value.outcome["stop_confirmed"] is True
    assert observation.call_count == 2
    proxy.cancel.assert_called_once()
    proxy.wait_for_execution.assert_not_called()
    assert "measured_arrival_fk" not in runner.evidence[-1]


@pytest.mark.parametrize(
    "mutation", ["wrong_episode", "runtime_fault", "truncated", "missing_goal", "still_running"]
)
def test_partial_or_wrong_terminal_evidence_never_proves_goal(terminal_goal_status, mutation):
    status = copy.deepcopy(terminal_goal_status)
    if mutation == "wrong_episode":
        status["episode"]["id"] = "other"
    elif mutation == "runtime_fault":
        status["state"] = "error"
        status["error"] = "simulator fault"
    elif mutation == "truncated":
        status["episode"]["truncated"] = True
    elif mutation == "missing_goal":
        status["episode"]["info"] = {}
    else:
        status["state"] = "running"
    outcome = radio_episode_terminal(status, "episode")
    assert outcome is None or outcome["kind"] == "RUNTIME_FAULT"


def test_unsuccessful_episode_end_is_separate_from_goal(terminal_goal_status):
    terminal_goal_status["episode"]["success"] = False
    terminal_goal_status["episode"]["info"]["done"] = {
        "success": False,
        "goal_status": {"satisfied": [], "unsatisfied": [0]},
    }
    assert radio_episode_terminal(terminal_goal_status, "episode")["kind"] == "EPISODE_ENDED"


def test_missing_feedback_alone_remains_fault(client, mocker):
    runner, proxy, _, observation, target = client
    fresh = observation()
    observation.side_effect = [
        fresh,
        fresh,
        RuntimeError("Development grasp truth is stale or episode changed"),
    ]
    runner.episode_status = mocker.Mock(
        return_value={"state": "running", "episode": {"id": "episode"}}
    )
    proxy.cancel.return_value = ExecutionResult(ExecutionStatus.ABORTED)
    with pytest.raises(RuntimeError, match="stale"):
        runner.move(target, "pregrasp")
    assert runner.evidence[-1]["outcome"] == "RUNTIME_FAULT"
    assert "episode_terminal" not in runner.evidence[-1]


def test_terminal_event_racing_feedback_loss_is_explicit_and_never_arrival(
    client, mocker, terminal_goal_status
):
    runner, proxy, _, observation, target = client
    fresh = observation()
    observation.side_effect = [
        fresh,
        fresh,
        RuntimeError("Development grasp truth is stale or episode changed"),
    ]
    running = {"state": "running", "episode": {"id": "episode"}}
    runner.episode_status = mocker.Mock(side_effect=[running, running, terminal_goal_status])
    proxy.cancel.return_value = ExecutionResult(ExecutionStatus.NO_EXECUTION)
    with pytest.raises(RadioEpisodeTerminalError) as caught:
        runner.move(target, "pregrasp")
    assert caught.value.outcome["kind"] == "TASK_GOAL_MET"
    assert runner.evidence[-1]["motion_arrived"] is False
    assert "measured_arrival_fk" not in runner.evidence[-1]


def test_missing_terminal_rpc_preserves_fault_without_guessing_success(client, mocker):
    runner, proxy, _, _, target = client
    running = {"state": "running", "episode": {"id": "episode"}}
    runner.episode_status = mocker.Mock(
        side_effect=[
            running,
            RuntimeError("status disconnected"),
            RuntimeError("status disconnected"),
        ]
    )
    proxy.cancel.return_value = ExecutionResult(ExecutionStatus.ABORTED)
    with pytest.raises(RuntimeError, match="status disconnected"):
        runner.move(target, "pregrasp")
    assert runner.evidence[-1]["outcome"] == "RUNTIME_FAULT"
    assert "episode_terminal" not in runner.evidence[-1]


def test_physical_fault_is_not_overridden_by_concurrent_goal(client, mocker, terminal_goal_status):
    runner, proxy, _, _, target = client
    running = {"state": "running", "episode": {"id": "episode"}}
    runner.episode_status = mocker.Mock(side_effect=[running, running, terminal_goal_status])
    runner.guard.side_effect = [None, None, RuntimeError("physical grasp drift")]
    proxy.cancel.return_value = ExecutionResult(ExecutionStatus.ABORTED)
    with pytest.raises(RuntimeError, match="physical grasp drift"):
        runner.move(target, "pregrasp")
    assert runner.evidence[-1]["outcome"] == "RUNTIME_FAULT"
    assert runner.evidence[-1]["episode_terminal"]["kind"] == "TASK_GOAL_MET"


def test_goal_with_unconfirmed_stop_remains_fault(client, mocker, terminal_goal_status):
    runner, proxy, _, _, target = client
    running = {"state": "running", "episode": {"id": "episode"}}
    runner.episode_status = mocker.Mock(side_effect=[running, terminal_goal_status])
    proxy.cancel.return_value = ExecutionResult(ExecutionStatus.TIMED_OUT)
    with pytest.raises(RadioEpisodeTerminalError) as caught:
        runner.move(target, "pregrasp")
    assert caught.value.outcome["kind"] == "RUNTIME_FAULT"
    assert caught.value.outcome["stop_confirmed"] is False


def test_normal_motion_cancel_is_not_goal_success(client):
    runner, proxy, _, _, target = client
    proxy.wait_for_execution.return_value = ExecutionResult(ExecutionStatus.ABORTED)
    proxy.cancel.return_value = ExecutionResult(ExecutionStatus.ABORTED)
    with pytest.raises(RuntimeError):
        runner.move(target, "pregrasp")
    assert runner.evidence[-1]["outcome"] == "MOTION_CANCELLED"


@pytest.mark.parametrize(
    "values,allowed",
    [
        ([0.1, 0.05, 0], True),
        ([0.1, 0.105, 0], False),
        ([0.1, 0, -0.01], False),
        ([0.1, 0.05, 0.01], False),
    ],
)
def test_placement_requires_monotone_vertical_arrival_with_bounded_support(
    collision_scene, values, allowed
):
    scene, request, _, _ = collision_scene
    request["phase"] = "place"
    path = JointTrajectory(
        joint_names=[JOINT],
        points=[TrajectoryPoint(positions=[q], time_from_start=i) for i, q in enumerate(values)],
    )
    state = JointState(name=[JOINT], position=[values[0]])
    if allowed:
        result = scene._support_placement(state, path, request)
        assert result["final_height"] == pytest.approx(-0.0015)
        assert result["support_pair_only"] == ["radio_part_0", "table"]
    else:
        with pytest.raises(RuntimeError, match="Placement"):
            scene._support_placement(state, path, request)


def test_tabletop_sequence_rejects_goal_before_intended_press(client, mocker, terminal_goal_status):
    runner, _, _, observation, _ = client
    runner.episode = "episode"
    runner.episode_status = mocker.Mock(return_value=terminal_goal_status)
    with pytest.raises(RadioEpisodeTerminalError) as caught:
        runner._observation("reorient")
    assert caught.value.outcome["kind"] == "RUNTIME_FAULT"
    assert "Unintended" in caught.value.outcome["reason"]
    observation.assert_not_called()


def test_effective_collision_failure_prevents_authorization_and_execution(client):
    runner, proxy, coordinator, _, target = client
    runner.scene.validate.side_effect = RuntimeError("effective collision")
    with pytest.raises(RuntimeError, match="effective collision"):
        runner.move(target, "pregrasp")
    coordinator.authorize_development_trajectory.assert_not_called()
    proxy.execute.assert_not_called()


def test_guard_failure_during_execution_cancels_without_opening_gripper(client):
    runner, proxy, _, _, target = client
    runner.guard.side_effect = [None, None, RuntimeError("lost contact")]
    with pytest.raises(RuntimeError, match="lost contact"):
        runner.move(target, "pregrasp")
    proxy.cancel.assert_called_once()
    proxy.set_gripper_position.assert_not_called()
    record = runner.evidence[-1]
    assert record["observation_guard_error"] == "lost contact"
    assert record["rejected_observation"]["episode"] == "episode"
    assert record["rejected_observation"]["radio_pose"] == np.eye(4).tolist()
    assert record["rejected_observation"]["radio_stage_displacement"]["translation_norm_m"] == 0


def test_changed_scene_prevents_dispatch(client):
    runner, proxy, coordinator, observation, target = client
    before = {
        "observed_at_monotonic": time.monotonic(),
        "episode": "episode",
        "radio_pose": np.eye(4).tolist(),
        "table_pose": runner.scene.table.tolist(),
    }
    after = {**before, "radio_pose": np.eye(4).tolist()}
    after["radio_pose"][0][3] = 0.01
    observation.side_effect = [before, after]
    with pytest.raises(RuntimeError, match="before dispatch"):
        runner.move(target, "pregrasp")
    coordinator.authorize_development_trajectory.assert_not_called()
    proxy.execute.assert_not_called()


@pytest.fixture
def sdk_module(mocker):
    mocker.patch.object(ZenohRPC, "start")
    mocker.patch.object(ZenohRPC, "serve_module_rpc")
    mocker.patch("dimos.core.module.get_loop", return_value=(None, None))
    module = BimanualRadioManipulationModule(
        model=simulation_model_config(), rpc_transport=ZenohRPC
    )
    module._world_monitor = mocker.Mock()
    module._radio_checkpoint = mocker.Mock()
    module._radio_checkpoint.context.side_effect = lambda request: __import__(
        "contextlib"
    ).nullcontext()
    state = JointState(name=[JOINT], position=[0.0])
    module._world_monitor.current_model_joint_state.return_value = state
    module._world_monitor.get_group_ee_pose.return_value = PoseStamped(
        frame_id="world", position=[0, 0, 0]
    )
    module._world_monitor.planning_groups.select.return_value = SimpleNamespace(
        joint_names=(JOINT,)
    )
    try:
        yield module, state
    finally:
        module.stop()


@pytest.mark.parametrize("auxiliary_torso", [False, True])
def test_sdk_stores_cartesian_plan_then_validates_same_geometry(
    sdk_module, mocker, auxiliary_torso
):
    module, state = sdk_module
    plan = SimpleNamespace(trajectory=trajectory(), message="stored SDK plan", plan_id="stored-id")
    generate = mocker.patch.object(module, "generate_cartesian_plan", return_value=plan)
    result = module.plan_radio_checkpoint(
        PoseStamped(frame_id="world", position=[0, 0, 0.01]),
        {"phase": "departure"},
        state,
        auxiliary_torso,
    )
    assert result.plan is plan
    assert generate.call_args.kwargs["check_collision"]
    assert generate.call_args.kwargs["auxiliary_groups"] == (["torso"] if auxiliary_torso else [])
    module._world_monitor.planning_groups.select.assert_called_with(
        ["right_arm", "torso"] if auxiliary_torso else ["right_arm"]
    )
    module._radio_checkpoint.context.assert_called_once_with({"phase": "departure"})
    module._radio_checkpoint.validate.assert_called_once_with(
        state, plan.trajectory, (JOINT,), {"phase": "departure"}
    )


def test_sdk_support_validation_failure_clears_pending_plan(sdk_module, mocker):
    module, state = sdk_module
    plan = SimpleNamespace(trajectory=trajectory(), message="stored", plan_id="id")
    mocker.patch.object(module, "generate_cartesian_plan", return_value=plan)
    clear = mocker.patch.object(module, "_clear_pending_plan")
    module._radio_checkpoint.validate.side_effect = RuntimeError("support violation")
    with pytest.raises(RuntimeError, match="support violation"):
        module.plan_radio_checkpoint(
            PoseStamped(frame_id="world", position=[0, 0, 0.01]), {"phase": "departure"}, state
        )
    clear.assert_called_once_with()


def test_sdk_rejects_changed_measured_start_before_planning(sdk_module, mocker):
    module, _ = sdk_module
    generate = mocker.patch.object(module, "generate_cartesian_plan")
    with pytest.raises(RuntimeError, match="fresh client measurement"):
        module.plan_radio_checkpoint(
            PoseStamped(frame_id="world", position=[0, 0, 0.01]),
            {"phase": "lift"},
            JointState(name=[JOINT], position=[0.01]),
        )
    generate.assert_not_called()


def test_contact_evidence_requires_exact_finger_link_names():
    result = radio_finger_contacts(
        [
            ("/World/robot/right_gripper_finger_link1", "/World/radio/base_link"),
            ("/World/robot/right_gripper_finger_link2_extra", "/World/radio/base_link"),
            ("/World/robot/right_gripper_link", "/World/radio/base_link"),
        ]
    )
    assert result["right_gripper_finger_link1"]
    assert not result["right_gripper_finger_link2"]
    assert not result["left_gripper_finger_link1"]


def test_measured_radio_slip_cancels_lift_and_records_unconfirmed_stop(client):
    runner, proxy, _, observation, target = client
    before = {
        "observed_at_monotonic": time.monotonic(),
        "episode": "episode",
        "radio_pose": np.eye(4).tolist(),
        "table_pose": runner.scene.table.tolist(),
    }
    slipped = {**before, "radio_pose": np.eye(4).tolist()}
    slipped["radio_pose"][0][3] = 0.01
    observation.side_effect = [before, before, slipped]
    proxy.cancel.return_value = ExecutionResult(ExecutionStatus.UNCERTAIN)
    with pytest.raises(RuntimeError, match="slipped"):
        runner.move(target, "departure")
    assert runner.evidence[-1]["cancel_stop_confirmed"] is False
    assert runner.evidence[-1]["timeline"][-1]["radio_stage_displacement"][
        "translation_norm_m"
    ] == pytest.approx(0.01)
    proxy.cancel.assert_called_once()
    proxy.set_gripper_position.assert_not_called()


def test_measured_joint_limit_violation_cancels_with_failed_sample_preserved(client):
    runner, proxy, _, _, target = client
    runner.full_state.side_effect = [
        JointState(name=[JOINT], position=[0.0]),
        JointState(name=[JOINT], position=[0.0]),
        JointState(name=[JOINT], position=[1.00001]),
    ]
    with pytest.raises(RuntimeError, match="actual position limits"):
        runner.move(target, "pregrasp")
    assert runner.evidence[-1]["timeline"][-1]["measured_joints"][JOINT] == 1.00001
    proxy.cancel.assert_called_once()
    proxy.set_gripper_position.assert_not_called()


def test_retention_pairs_radio_and_encoders_instead_of_newer_rpc_feedback(client):
    runner, proxy, _, observation, _ = client
    calls = 0
    newer = [0.0]

    def measured_observation():
        nonlocal calls
        calls += 1
        q = 0.0 if calls <= 2 else 0.01 if calls == 3 else 0.03
        newer[0] = 0.0 if calls <= 2 else 0.03
        radio = np.eye(4)
        radio[2, 3] = q
        return {
            "episode": "episode",
            "step": calls,
            "observed_at_monotonic": time.monotonic(),
            "radio_pose": radio.tolist(),
            "table_pose": runner.scene.table.tolist(),
            "measured_joints": {JOINT: q},
        }

    observation.side_effect = measured_observation
    runner.full_state.side_effect = lambda: JointState(name=[JOINT], position=[newer[0]])
    runner.state_from_observation = lambda value: JointState(
        name=[JOINT], position=[value["measured_joints"][JOINT]]
    )
    proxy.plan_radio_checkpoint.return_value.plan.trajectory = trajectory(0.03)
    runner.move(PoseStamped(frame_id="world", position=[0, 0, 0.0285]), "lift")
    record = runner.evidence[-1]
    assert record["execution_status"] == "COMPLETED"
    assert record["timeline"][0]["sdk_measured_fk"][2][3] == pytest.approx(0.0085)
    assert record["measured_end_joints"][JOINT] == 0.03
    proxy.cancel.assert_not_called()


def test_bad_effective_cached_start_is_rejected_even_when_collision_free(collision_scene):
    scene, request, _, _ = collision_scene
    command = trajectory()
    command.points[0].positions = [-0.1]
    with pytest.raises(RuntimeError, match="start exceeds"):
        scene.validate(JointState(name=[JOINT], position=[0.0]), command, [JOINT], request)


def test_dual_path_rejects_intermediate_holding_drift(collision_scene):
    scene, request, _, _ = collision_scene
    request.update(phase="press_approach", holding_pose=np.eye(4).tolist())
    request["holding_pose"][2][3] = -0.0015
    path = JointTrajectory(
        joint_names=[JOINT],
        points=[
            TrajectoryPoint(positions=[q], velocities=[0], time_from_start=i)
            for i, q in enumerate([0.0, 0.003, 0.0])
        ],
    )
    with pytest.raises(RuntimeError, match="holding TCP"):
        scene.validate(JointState(name=[JOINT], position=[0.0]), path, [JOINT], request)


@pytest.mark.parametrize(
    "clearance,lateral,valid", [(-0.001, 0.0, True), (-0.003, 0.0, False), (0.0, 0.02, False)]
)
def test_contact_corridor_bounds_depth_and_lateral_motion(
    collision_scene, mocker, clearance, lateral, valid
):
    scene, request, native, _ = collision_scene
    surface = np.array([0.0446848528, 0.0420822057, -0.0124619396])
    pad = np.array([0, 0, -0.07])
    left = np.eye(4)
    left[:3, 3] = surface + np.array([clearance, lateral, 0]) - pad
    mocker.patch.object(
        scene.world,
        "get_link_pose",
        side_effect=lambda ctx, link: np.eye(4) if link == "right_gripper_link" else left,
    )
    request.update(
        phase="press_contact",
        holding_pose=np.eye(4).tolist(),
        contact={"surface_in_radio": surface.tolist(), "pad_in_left_gripper": pad.tolist()},
    )
    state = JointState(name=[JOINT], position=[0])
    with scene.context(request):
        if valid:
            assert scene.holding_sample(state, request)[
                "signed_contact_clearance_m"
            ] == pytest.approx(clearance)
        else:
            with pytest.raises(RuntimeError, match="contact corridor"):
                scene.holding_sample(state, request)
    calls = [c.args for c in native.setCollisions.call_args_list]
    assert ("radio_part_0", "table", False) not in calls
    assert ("left_gripper_finger_link1", "radio_part_0", False) in calls
    assert ("left_gripper_finger_link1", "radio_part_0", True) in calls
    assert ("left_gripper_finger_link1", "radio_part_6", False) not in calls
    assert ("left_gripper_finger_link2", "radio_part_0", False) not in calls


def test_dual_sdk_plan_constrains_both_tcps_with_explicit_torso(sdk_module, mocker):
    module, state = sdk_module
    holding = module._world_monitor.get_group_ee_pose("right_arm")
    left = PoseStamped(frame_id="world", position=[0.01, 0, 0])
    request = {"phase": "press_approach", "holding_pose": pose_to_matrix(holding).tolist()}
    plan = SimpleNamespace(trajectory=trajectory(), message="dual", plan_id="dual-id")
    generate = mocker.patch.object(module, "generate_cartesian_plan", return_value=plan)
    result = module.plan_radio_checkpoint(holding, request, state, True, left)
    assert result.plan is plan
    targets = generate.call_args.args[0]
    assert set(targets) == {"right_arm", "left_arm"}
    assert targets["right_arm"][1] is holding
    assert targets["left_arm"][1] is left
    assert generate.call_args.kwargs["auxiliary_groups"] == ["torso"]
    assert generate.call_args.kwargs["check_collision"] is True
    module._world_monitor.planning_groups.select.assert_called_with(
        ["right_arm", "left_arm", "torso"]
    )


def test_dual_phase_requires_left_target_before_planning(client):
    runner, proxy, _, _, target = client
    with pytest.raises(ValueError, match="left target"):
        runner.move(target, "press_approach")
    proxy.plan_radio_checkpoint.assert_not_called()


def test_table_press_allows_only_calibrated_right_finger_two_contact(collision_scene):
    scene, request, native, _ = collision_scene
    request.update(
        phase="table_press",
        contact={
            "surface_in_radio": [0.0446848528, 0.0420822057, -0.0124619396],
            "pad_in_gripper": [0, 0, -0.078],
        },
    )
    with scene.context(request):
        native.setCollisions.assert_any_call("right_gripper_finger_link2", "radio_part_0", False)
        assert not any(
            call.args == ("right_gripper_finger_link1", "radio_part_0", False)
            for call in native.setCollisions.call_args_list
        )
    native.setCollisions.assert_any_call("right_gripper_finger_link2", "radio_part_0", True)


def test_unintended_toggle_preserves_first_rejected_observation(client):
    runner, _, _, observation, _ = client
    value = observation.side_effect()
    value["evaluator_toggle_region"] = {"finger_contact_steps": 1}
    observation.side_effect = None
    observation.return_value = value
    runner.evidence.append({"phase": "reorient", "before": value, "request": {}})
    with pytest.raises(RuntimeError, match="trigger activated"):
        runner._observation("reorient")
    assert (
        runner.evidence[-1]["rejected_observation"]["evaluator_toggle_region"][
            "finger_contact_steps"
        ]
        == 1
    )


def test_planning_time_reduces_dispatch_budget_and_prevents_partial_motion(client, mocker):
    runner, proxy, coordinator, _, target = client
    clock = [100.0]
    mocker.patch(
        "dimos.simulation.behavior.radio_checkpoint.time.monotonic", side_effect=lambda: clock[0]
    )
    prepared = trajectory()
    prepared.points[-1].time_from_start = 18.0

    def prepare(_):
        clock[0] += 3.0
        return prepared

    coordinator.prepare_development_trajectory.side_effect = prepare
    with pytest.raises(TimeoutError, match="exceeds remaining"):
        runner.move(target, "pregrasp", timeout=20)
    proxy.execute.assert_not_called()
    coordinator.authorize_development_trajectory.assert_not_called()
    assert runner.evidence[-1]["dispatch_budget"] == {
        "effective_duration_s": 18.0,
        "remaining_s": 17.0,
    }


def test_linear_checkpoint_resolves_start_in_planner_snapshot_and_preserves_goal(
    sdk_module, mocker
):
    module, state = sdk_module
    goal = PoseStamped(frame_id="world", position=[0, 0, 0.05])
    fresh_pose = PoseStamped(frame_id="world", position=[0.00001, 0, 0])
    world = module._world_monitor.world
    world.scratch_context.return_value = __import__("contextlib").nullcontext("context")
    world.get_group_ee_pose.return_value = fresh_pose
    world.get_joint_state.return_value = JointState(
        name=["r1pro/base_x", JOINT], position=[3.6, -0.001]
    )
    selection = SimpleNamespace(joint_names=(JOINT,))
    mocker.patch.object(
        ManipulationModule, "_resolve_group_plan_start", return_value=(selection, state)
    )
    plan = SimpleNamespace(trajectory=trajectory(), message="stored", plan_id="id")
    captured = {}

    def generate(targets, *args, **kwargs):
        module._resolve_group_plan_start(("right_arm", "torso"), 1)
        captured.update(targets)
        return plan

    mocker.patch.object(module, "generate_cartesian_plan", side_effect=generate)
    module.plan_radio_checkpoint(goal, {"phase": "lift"}, state, True)
    assert captured["right_arm"] == (fresh_pose, goal)
    ctx, full = world.set_joint_state.call_args.args
    assert ctx == "context"
    assert full.name == ["r1pro/base_x", JOINT]
    assert full.position == [3.6, 0.0]
    assert module._radio_start_targets.get() is None
