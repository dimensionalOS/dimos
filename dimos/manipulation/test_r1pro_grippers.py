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

"""Exercise real manipulation, task, tick routing and adapter with only ROS I/O mocked."""

from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from dimos.control.coordinator import ControlCoordinator
from dimos.control.tick_loop import TickLoop
from dimos.hardware.manipulators.r1pro_gripper import adapter as driver
from dimos.hardware.manipulators.r1pro_gripper.adapter import R1ProGripperAdapter
from dimos.hardware.manipulators.spec import ManipulatorAdapter
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.protocol.pubsub.impl.rospubsub import RawROSTopic
from dimos.robot.galaxea.r1pro.blueprints.manipulation.r1pro_manipulation import (
    _gripper_hardware,
    _manipulation_tasks,
)
from dimos.robot.galaxea.r1pro.config import make_r1pro_model_config


def feedback(opening):
    return SimpleNamespace(
        position=[opening], header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0))
    )


def status(code=0):
    return SimpleNamespace(
        errors=[
            SimpleNamespace(
                name="motor", error_code=code, error_description="fault" if code else ""
            )
        ]
    )


def refresh(adapter, opening):
    adapter._on_feedback(feedback(opening), RawROSTopic("unused", object))
    for segment in ("torso", f"arm_{adapter._side}"):
        adapter._on_status(status(), RawROSTopic(f"/hdas/feedback_status_{segment}", object))


@pytest.fixture
def framework(mocker, module_factory):
    clock = mocker.patch.object(driver.time, "monotonic", return_value=100.0)
    mocker.patch.object(driver, "_make_qos", return_value=None)
    mocker.patch.object(driver, "FeedbackStatus", MagicMock())
    mocker.patch.object(
        driver,
        "RosJointState",
        side_effect=lambda: SimpleNamespace(
            header=SimpleNamespace(stamp=None), name=[], position=[]
        ),
    )
    ros_nodes = []

    def make_ros(**kwargs):
        ros = MagicMock()
        ros._node.count_subscribers.return_value = 1
        ros._node.count_publishers.return_value = 1
        ros_nodes.append(ros)
        return ros

    mocker.patch.object(driver, "RawROS", side_effect=make_ros)
    coordinator = ControlCoordinator(publish_joint_state=False)
    mocker.patch.object(coordinator, "_sync_stream_subscriptions")
    adapters = {}
    try:
        for component in _gripper_hardware:
            side = component.adapter_kwargs["side"]
            adapter = R1ProGripperAdapter(side=side)
            assert isinstance(adapter, ManipulatorAdapter)
            assert adapter.connect() and adapter.activate()
            refresh(adapter, 100.0 if side == "left" else 50.0)
            coordinator.add_hardware(adapter, component)
            adapters[side] = adapter
        for cfg in _manipulation_tasks:
            if cfg.type == "gripper":
                task = coordinator._create_task_from_config(cfg)
                coordinator.add_task(task, task_type="gripper")
        loop = TickLoop(
            100,
            coordinator._hardware,
            coordinator._hardware_lock,
            coordinator._tasks,
            coordinator._task_lock,
            coordinator._joint_to_hardware,
        )
        module = module_factory(coordinator)
        module.config.model = make_r1pro_model_config()
        module._world_monitor = MagicMock()
        module._world_monitor.planning_groups = PlanningGroupRegistry(
            module.config.model.planning_groups
        )
        module._world_monitor.current_group_joint_state.return_value = JointState()
        module._world_monitor.get_group_ee_pose.return_value = None
        loop._tick()
        yield SimpleNamespace(
            module=module,
            coordinator=coordinator,
            loop=loop,
            adapters=adapters,
            ros=ros_nodes,
            clock=clock,
        )
    finally:
        coordinator.stop()


def test_lifecycle_and_idle_ticks_never_command_grippers(framework):
    framework.loop._tick()
    framework.coordinator.stop()
    for ros in framework.ros:
        ros._node.create_publisher.return_value.publish.assert_not_called()


@pytest.mark.parametrize("side", ["left", "right"])
def test_standard_api_reaches_only_selected_vendor_endpoint(framework, side):
    result = framework.module.set_gripper_position(0.9, f"{side}_arm")
    assert result.succeeded
    framework.loop._tick()
    own = framework.adapters[side]._publisher
    own.publish.assert_called_once()
    message = own.publish.call_args.args[0]
    assert message.position == [90.0]
    assert message.name == [f"{side}_gripper"]
    other = "right" if side == "left" else "left"
    framework.adapters[other]._publisher.publish.assert_not_called()
    framework.loop._tick()
    own.publish.assert_called_once()  # Hold ticks do not restart vendor interpolation.


def test_group_capabilities_and_readbacks_are_distinct(framework):
    capabilities = {g.id: g.has_gripper for g in framework.module.list_planning_groups()}
    assert capabilities == {"left_arm": True, "right_arm": True, "torso": False}
    snapshot = framework.module.get_state()
    assert snapshot.groups["left_arm"].gripper_position == 1.0
    assert snapshot.groups["right_arm"].gripper_position == 0.5
    assert snapshot.groups["torso"].gripper_position is None


@pytest.mark.parametrize("group", [None, "torso", "unknown"])
def test_ambiguous_or_non_gripper_groups_are_rejected(framework, group):
    assert not framework.module.set_gripper_position(0.4, group).succeeded
    framework.loop._tick()
    for adapter in framework.adapters.values():
        adapter._publisher.publish.assert_not_called()


@pytest.mark.parametrize("opening", [-0.1, 1.1, float("nan"), float("inf")])
def test_invalid_opening_never_reaches_vendor(framework, opening):
    assert not framework.module.set_gripper_position(opening, "left_arm").succeeded
    framework.loop._tick()
    framework.adapters["left"]._publisher.publish.assert_not_called()


def test_stale_feedback_clears_target_and_readback_without_replay(framework):
    assert framework.module.set_gripper_position(0.9, "left_arm").succeeded
    framework.loop._tick()
    publisher = framework.adapters["left"]._publisher
    publisher.publish.assert_called_once()
    framework.clock.return_value = 103.0
    framework.loop._tick()
    assert framework.module.get_state().groups["left_arm"].gripper_position is None
    refresh(framework.adapters["left"], 95.0)
    framework.loop._tick()
    assert framework.module.get_state().groups["left_arm"].gripper_position == 0.95
    publisher.publish.assert_called_once()
    assert framework.module.set_gripper_position(0.9, "left_arm").succeeded
    framework.loop._tick()
    assert publisher.publish.call_count == 2


@pytest.mark.parametrize("fault_segment", ["torso", "arm_left", "gripper_left"])
def test_fault_drops_pending_target_and_recovery_does_not_replay(framework, fault_segment):
    assert framework.module.set_gripper_position(0.9, "left_arm").succeeded
    adapter = framework.adapters["left"]
    adapter._on_status(status(42), RawROSTopic(f"/hdas/feedback_status_{fault_segment}", object))
    framework.loop._tick()
    assert framework.module.get_state().groups["left_arm"].gripper_position is None
    adapter._on_status(status(), RawROSTopic(f"/hdas/feedback_status_{fault_segment}", object))
    framework.loop._tick()
    adapter._publisher.publish.assert_not_called()


def test_estop_clear_requires_a_new_command(framework):
    assert framework.module.set_gripper_position(0.9, "left_arm").succeeded
    framework.coordinator.set_estop(True)
    framework.loop._tick()
    framework.coordinator.set_estop(False)
    framework.loop._tick()
    framework.adapters["left"]._publisher.publish.assert_not_called()
    assert framework.module.set_gripper_position(0.8, "left_arm").succeeded
    framework.loop._tick()
    framework.adapters["left"]._publisher.publish.assert_called_once()


def test_shutdown_clears_latched_task_target(framework):
    assert framework.module.set_gripper_position(0.9, "left_arm").succeeded
    framework.coordinator.stop()
    framework.coordinator.set_estop(False)
    framework.loop._tick()
    for ros in framework.ros:
        ros._node.create_publisher.return_value.publish.assert_not_called()


def test_unscoped_broadcast_cannot_move_both_grippers(framework):
    framework.coordinator._dispatch("gripper_command", Float32(data=0.5))
    framework.loop._tick()
    for adapter in framework.adapters.values():
        adapter._publisher.publish.assert_not_called()


@pytest.mark.parametrize("subscribers,publishers", [(0, 1), (2, 1), (1, 2)])
def test_driver_or_publisher_conflict_blocks_output(framework, subscribers, publishers):
    assert framework.module.set_gripper_position(0.9, "left_arm").succeeded
    node = framework.adapters["left"]._ros._node
    node.count_subscribers.return_value = subscribers
    node.count_publishers.return_value = publishers
    framework.loop._tick()
    framework.adapters["left"]._publisher.publish.assert_not_called()
    assert framework.module.get_state().groups["left_arm"].gripper_position is None


def test_raw_readback_and_unknown_dedicated_faults_are_preserved(framework):
    adapter = framework.adapters["left"]
    refresh(adapter, 101.25)
    framework.loop._tick()
    assert framework.module.get_state().groups["left_arm"].gripper_position == 1.0125
    assert adapter.get_diagnostics()["gripper_fault_status_available"] is False


def test_unsupported_vendor_operations_never_report_success(framework):
    adapter = framework.adapters["left"]
    assert adapter.write_stop() is False
    assert adapter.write_enable(True) is False
    assert adapter.write_clear_errors() is False
    assert adapter.write_joint_velocities([1.0]) is False
    adapter._publisher.publish.assert_not_called()


def test_conflicting_legacy_and_group_bindings_are_rejected():
    data = make_r1pro_model_config().model_dump()
    data["gripper_hardware_id"] = "arm"
    with pytest.raises(ValueError, match="not both"):
        RobotModelConfig(**data)


@pytest.mark.parametrize(
    "bindings", [{"torso": "hand"}, {"unknown": "hand"}, {"left_arm": "same", "right_arm": "same"}]
)
def test_invalid_group_bindings_fail_at_configuration_boundary(bindings):
    data = make_r1pro_model_config().model_dump()
    data["gripper_hardware_ids"] = bindings
    with pytest.raises(ValueError):
        RobotModelConfig(**data)


def test_legacy_scalar_binding_rejects_multiple_pose_groups():
    data = make_r1pro_model_config().model_dump()
    data["gripper_hardware_ids"] = {}
    data["gripper_hardware_id"] = "arm"
    with pytest.raises(ValueError, match="exactly one"):
        RobotModelConfig(**data)


def test_invalid_feedback_drops_target_and_requires_new_intent(framework):
    assert framework.module.set_gripper_position(0.9, "left_arm").succeeded
    bad = feedback(float("nan"))
    framework.adapters["left"]._on_feedback(bad, RawROSTopic("unused", object))
    framework.loop._tick()
    assert framework.module.get_state().groups["left_arm"].gripper_position is None
    refresh(framework.adapters["left"], 100.0)
    framework.loop._tick()
    framework.adapters["left"]._publisher.publish.assert_not_called()


def test_disconnect_then_reconnect_never_replays_target(framework):
    adapter = framework.adapters["left"]
    publisher = adapter._publisher
    assert framework.module.set_gripper_position(0.9, "left_arm").succeeded
    adapter.disconnect()
    framework.loop._tick()
    assert framework.module.get_state().groups["left_arm"].gripper_position is None
    assert adapter.connect() and adapter.activate()
    refresh(adapter, 100.0)
    framework.loop._tick()
    publisher.publish.assert_not_called()
    adapter._publisher.publish.assert_not_called()


def test_extra_gripper_feedback_never_enters_18_axis_planning_model(framework):
    names = framework.module.config.model.joint_names
    assert len(names) == 18
    framework.module._on_joint_state(
        JointState(
            name=[*names, "r1pro_left/gripper", "r1pro_right/gripper"],
            position=[0.0] * 18 + [90.0, 40.0],
        )
    )
    forwarded = framework.module._world_monitor.on_joint_state.call_args.args[0]
    assert forwarded.name == names
    assert forwarded.position == [0.0] * 18


@pytest.mark.parametrize("event", ["fault", "invalid_feedback", "reconnect"])
def test_transient_loss_between_ticks_still_discards_old_intent(framework, event):
    adapter = framework.adapters["left"]
    old_publisher = adapter._publisher
    assert framework.module.set_gripper_position(0.9, "left_arm").succeeded
    if event == "fault":
        topic = RawROSTopic("/hdas/feedback_status_arm_left", object)
        adapter._on_status(status(42), topic)
        adapter._on_status(status(), topic)
    elif event == "invalid_feedback":
        adapter._on_feedback(feedback(float("nan")), RawROSTopic("unused", object))
        refresh(adapter, 100.0)
    else:
        adapter.disconnect()
        assert adapter.connect() and adapter.activate()
        refresh(adapter, 100.0)
    # Even a read-only observer must not consume the invalidation barrier.
    assert adapter.get_diagnostics()["ready"] is False
    framework.loop._tick()
    framework.loop._tick()
    old_publisher.publish.assert_not_called()
    adapter._publisher.publish.assert_not_called()
    assert framework.module.set_gripper_position(0.8, "left_arm").succeeded
    framework.loop._tick()
    adapter._publisher.publish.assert_called_once()
