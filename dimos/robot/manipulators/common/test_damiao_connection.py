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

"""Real connection lifecycles with an in-memory motor adapter; no CAN traffic."""

import threading
import xml.etree.ElementTree as ET

import pytest

from dimos.control.connection_source import ConnectionSource
from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.control.tick_loop import TickLoop
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.stream import Transport
from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig
from dimos.hardware.whole_body.mock.adapter import MockWholeBodyAdapter
from dimos.hardware.whole_body.registry import whole_body_adapter_registry
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.protocol.encode.cbor import decode, encode
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.common import damiao_connection as common
from dimos.robot.manipulators.common.damiao_blueprint import DamiaoCoordinator
from dimos.robot.manipulators.common.damiao_connection import damiao_control_model
from dimos.robot.manipulators.makermods_metal.blueprints.connection import (
    makermods_metal_connection_control,
)
from dimos.robot.manipulators.makermods_metal.connection import MetalConnection
from dimos.robot.manipulators.openarm.blueprints.connection import openarm_connection_control
from dimos.robot.manipulators.openarm.connection import OpenArmConnection
from dimos.robot.manipulators.openyam.blueprints.connection import openyam_connection_control
from dimos.robot.manipulators.openyam.connection import OpenYamConnection
from dimos.utils.testing.waiting import wait_until

CASES = [
    (
        MetalConnection,
        "metal",
        [f"joint{i}" for i in range(1, 7)],
        ("gripper",),
        [100, 100, 100, 50, 20, 20, 0],
        [5, 5, 5, 5, 0.6, 0.6, 0],
    ),
    (
        OpenYamConnection,
        "yam",
        [f"yam_joint{i}" for i in range(1, 7)],
        ("gripper",),
        [80, 80, 80, 10, 10, 10, 0],
        [5, 5, 5, 1.5, 1.5, 1.5, 0],
    ),
    (
        OpenArmConnection,
        "openarm",
        [f"openarm_{side}_joint{i}" for side in ("left", "right") for i in range(1, 8)],
        ("left_gripper", "right_gripper"),
        [100, 100, 80, 80, 60, 60, 60] * 2 + [0, 0],
        [1.5, 1.5, 1, 1, 0.8, 0.8, 0.8] * 2 + [0, 0],
    ),
]


@pytest.fixture(autouse=True)
def no_network(mocker):
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.start")
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.serve_module_rpc")


@pytest.fixture(params=CASES, ids=["metal", "yam", "openarm"])
def topology(request):
    return request.param


@pytest.fixture
def arm_model(tmp_path, topology):
    _, _, joints, _, _, _ = topology
    root = ET.Element("robot", name="fixture")
    ET.SubElement(root, "link", name="base")
    parent = "base"
    for index, name in enumerate(joints):
        child = f"link{index}"
        ET.SubElement(root, "link", name=child)
        joint = ET.SubElement(root, "joint", name=name, type="revolute")
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=child)
        ET.SubElement(joint, "limit", lower="-2", upper="2", velocity="1", effort="10")
        parent = child
    path = tmp_path / "fixture.urdf"
    path.write_text(ET.tostring(root, encoding="unicode"))
    return RobotModel.from_file(path)


@pytest.fixture
def model(arm_model, topology, tmp_path, monkeypatch):
    monkeypatch.setattr(common, "DEFAULT_ROBOT_ASSET_CACHE_ROOT", tmp_path)
    return damiao_control_model(arm_model, topology[3])


@pytest.fixture
def rig(model, topology, mocker):
    cls, source, joints, grippers, _, _ = topology
    adapter = MockWholeBodyAdapter(
        dof=len(joints) + len(grippers),
        initial_positions=[0.2] * len(joints) + [0.4] * len(grippers),
    )
    factory = mocker.patch.object(whole_body_adapter_registry, "create", return_value=adapter)
    module = cls(model=model, source="arm", deadman_timeout_s=10.0)
    positions = mocker.Mock(spec=Transport)
    callbacks = []
    positions.subscribe.side_effect = lambda cb, *args, **kw: (
        callbacks.append(cb) or (lambda: callbacks.remove(cb))
    )
    states = mocker.Mock(spec=Transport)
    module.set_transport("position_command", positions)
    module.set_transport("joint_state", states)
    writes = mocker.spy(adapter, "write_motor_commands")
    deactivate = mocker.spy(adapter, "deactivate")
    disconnect = mocker.spy(adapter, "disconnect")
    made = (module, adapter, callbacks, states, writes, deactivate, disconnect, factory)
    try:
        module.start()
        wait_until(lambda: states.broadcast.called, timeout=2)
        yield made
    finally:
        module.stop()


def test_partial_targets_keep_other_targets_and_vendor_gains(rig, topology):
    module, _, callbacks, states, writes, _, _, _ = rig
    _, _, joints, grippers, kp, kd = topology
    source = module.config.source
    callbacks[0](
        JointState(
            name=[f"{source}/{joints[1]}", f"{source}/{grippers[0]}"],
            position=[-0.5, 0.8],
            velocity=[99, 99],
        )
    )
    commands = writes.call_args.args[0]
    expected = [0.2] * len(joints) + [0.4] * len(grippers)
    expected[1], expected[len(joints)] = -0.5, 0.8
    assert [c.q for c in commands] == expected
    assert [c.kp for c in commands] == kp
    assert [c.kd for c in commands] == kd
    assert [c.dq for c in commands] == [0.0] * len(expected)
    callbacks[0](JointState(name=[f"{source}/{joints[0]}"], position=[0.6]))
    assert writes.call_args.args[0][1].q == -0.5
    assert writes.call_args.args[0][len(joints)].q == 0.8
    wait_until(lambda: states.broadcast.call_args.args[1].position[0] == 0.6, timeout=2)
    measured = states.broadcast.call_args.args[1]
    assert measured.name == [f"{source}/{name}" for name in (*joints, *grippers)]
    assert measured.frame_id == "base"
    assert measured.velocity == [0.0] * len(expected)
    assert measured.effort == [0.0] * len(expected)
    assert module.status().last_error is None


@pytest.mark.parametrize("target", [3.0, float("nan"), float("inf")])
def test_invalid_command_is_rejected_as_a_whole(rig, topology, target):
    module, _, callbacks, _, writes, _, _, _ = rig
    source, joints = module.config.source, topology[2]
    callbacks[0](
        JointState(name=[f"{source}/{joints[0]}", f"{source}/{joints[1]}"], position=[0.7, target])
    )
    writes.assert_not_called()
    assert module.status().last_rejection is not None


def test_normalized_gripper_rejects_outside_range(rig, topology):
    module, _, callbacks, _, writes, _, _, _ = rig
    callbacks[0](JointState(name=[f"{module.config.source}/{topology[3][0]}"], position=[1.01]))
    writes.assert_not_called()
    assert module.status().last_rejection is not None


def test_deadman_holds_measured_position_and_next_command_recovers(rig, topology, mocker):
    module, adapter, callbacks, _, writes, _, _, _ = rig
    source, joints = module.config.source, topology[2]
    halted = threading.Event()
    halt = mocker.patch.object(module, "halt", side_effect=lambda: halted.set())
    callbacks[0](JointState(name=[f"{source}/{joints[0]}"], position=[0.6]))
    module._on_loop(lambda: module._deadman_expired(module._command_seq))
    assert halted.wait(2)
    wait_until(lambda: module.status().deadman_fired, timeout=2)
    halt.assert_called_once_with()
    callbacks[0](JointState(name=[f"{source}/{joints[0]}"], position=[0.7]))
    assert not module.status().deadman_fired
    assert writes.call_args.args[0][0].q == 0.7
    assert adapter.is_connected()


def test_halt_samples_feedback_and_shutdown_disables_once(rig, topology, mocker):
    module, adapter, _, _, writes, deactivate, disconnect, _ = rig
    with module._hardware_lock:
        module.halt()
    assert [c.q for c in writes.call_args.args[0]] == [0.2] * len(topology[2]) + [0.4] * len(
        topology[3]
    )
    module.stop()
    module.stop()
    deactivate.assert_called_once_with()
    disconnect.assert_called_once_with()
    assert not adapter.is_connected()
    assert not module.status().connected
    assert not module._reader.is_alive()


def test_feedback_failure_disables_and_blocks_further_commands(rig, topology, mocker):
    module, adapter, callbacks, _, writes, deactivate, _, _ = rig
    mocker.patch.object(adapter, "read_motor_states", side_effect=RuntimeError("bad feedback"))
    wait_until(lambda: deactivate.called, timeout=2)
    callbacks[0](JointState(name=[f"{module.config.source}/{topology[2][0]}"], position=[0.7]))
    writes.assert_not_called()
    assert "feedback" in module.status().last_error


def test_refused_write_records_error_and_attempts_hold(rig, topology, mocker):
    module, adapter, callbacks, _, _, _, _, _ = rig
    write = mocker.patch.object(adapter, "write_motor_commands", side_effect=[False, True])
    callbacks[0](JointState(name=[f"{module.config.source}/{topology[2][0]}"], position=[0.7]))
    assert write.call_count == 2
    assert write.call_args.args[0][0].q == 0.2
    assert "refused position" in module.status().last_error


def test_control_model_and_coordinator_agree_on_names_limits_and_grippers(model, topology):
    names = (*topology[2], *topology[3])
    qualified = model.with_renamed_joints({name: f"arm/{name}" for name in names})
    source = ConnectionSource("arm", ["position_command"], ["joint_state"], qualified, 1.0)
    source.on_message(
        "joint_state", JointState(name=[f"arm/{n}" for n in names], position=[0.2] * len(names))
    )
    assert source.ready_for_control()
    assert source.read_joints().joint_positions == {f"arm/{name}": 0.2 for name in names}
    assert [(j.name, j.lower, j.upper) for j in qualified.load().joints[-len(topology[3]) :]] == [
        (f"arm/{name}", 0.0, 1.0) for name in topology[3]
    ]
    restored = decode(encode(qualified))
    assert restored.load().xml == qualified.load().xml


@pytest.mark.parametrize(
    "blueprint, module_name, args",
    [
        (
            makermods_metal_connection_control,
            "metalconnection",
            ["--metalconnection.can-port", "test-can"],
        ),
        (
            openyam_connection_control,
            "openyamconnection",
            ["--openyamconnection.can-port", "test-can"],
        ),
        (
            openarm_connection_control,
            "openarmconnection",
            [
                "--openarmconnection.left-can-port",
                "left",
                "--openarmconnection.right-can-port",
                "right",
            ],
        ),
    ],
)
def test_new_entry_points_are_lazy_and_route_without_hardware(blueprint, module_name, args, mocker):
    load = mocker.patch.object(RobotModel, "load", side_effect=AssertionError("must stay lazy"))
    parsed = BlueprintConfigParser(blueprint).parse(args, environ={})
    assert parsed.module_kwargs(module_name)["source"] == "arm"
    coordinator = parsed.module_kwargs("ControlCoordinator")
    assert not coordinator.get("hardware")
    assert coordinator["tasks"][0]["joint_names"][0].startswith("arm/")
    load.assert_not_called()


@pytest.mark.parametrize(
    "cls, config",
    [
        (MetalConnection, {"can_port": " "}),
        (OpenYamConnection, {"can_port": ""}),
        (OpenArmConnection, {"left_can_port": "left"}),
        (OpenArmConnection, {"left_can_port": "same", "right_can_port": "same"}),
        (OpenArmConnection, {"left_can_port": " ", "right_can_port": "right"}),
    ],
)
def test_bad_can_selection_fails_before_hardware(cls, config, mocker):
    factory = mocker.patch.object(whole_body_adapter_registry, "create")
    with pytest.raises(ValueError):
        cls(**config)
    factory.assert_not_called()


@pytest.fixture
def coordinator(rig, model, topology, mocker):
    module, _, callbacks, states, _, _, _, _ = rig
    names = (*topology[2], *topology[3])
    qualified = model.with_renamed_joints({name: f"arm/{name}" for name in names})
    c = DamiaoCoordinator(
        model=qualified,
        publish_joint_state=False,
        tasks=[
            joint_trajectory_task([f"arm/{name}" for name in names]),
            *[
                TaskConfig(
                    name=f"arm_{name}",
                    type="gripper",
                    joint_names=[f"arm/{name}"],
                    stream_bind={"gripper_command": f"{side}_gripper_command"},
                )
                for name, side in zip(
                    topology[3],
                    ("left", "right") if len(topology[3]) == 2 else ("right",),
                    strict=True,
                )
            ],
        ],
    )
    readings = mocker.Mock(spec=Transport)
    received = []
    readings.subscribe.side_effect = lambda cb, *args: (
        received.append(cb) or (lambda: received.remove(cb))
    )
    c.set_transport("arm_joint_state", readings)
    commands = mocker.Mock(spec=Transport)
    commands.broadcast.side_effect = lambda stream, msg: callbacks[0](msg)
    c.set_transport("arm_position_command", commands)
    for name in ("left_gripper_command", "right_gripper_command"):
        port = mocker.Mock(spec=Transport)
        port.subscribe.return_value = lambda: None
        c.set_transport(name, port)
    mocker.patch.object(TickLoop, "start")
    try:
        c.start()
        states.broadcast.side_effect = lambda stream, msg: received[0](msg)
        wait_until(lambda: c._connections["arm"].ready_for_control(), timeout=2)
        yield c
    finally:
        c.stop()


def test_coordinator_routes_gripper_holds_and_estop_through_connection(coordinator, rig, topology):
    module, _, _, _, writes, _, _, factory = rig
    c = coordinator
    assert c._hardware == {}
    assert factory.call_args.args == ("mock_whole_body",)
    gripper = topology[3][0]
    assert c.task_invoke(f"arm_{gripper}", "set_normalized", {"values": [0.8]})
    c._tick_loop._tick()
    assert writes.call_args.args[0][len(topology[2])].q == 0.8
    assert [command.q for command in writes.call_args.args[0][: len(topology[2])]] == [0.2] * len(
        topology[2]
    )
    c.set_estop(True)
    assert module.status().last_rejection is None
    assert c.task_invoke(f"arm_{gripper}", "set_normalized", {"values": [0.1]}) is False
    c._tick_loop._tick()
    assert writes.call_args.args[0][len(topology[2])].q == 0.8


def test_activation_failure_disconnects_without_starting_reader(model, topology, mocker):
    adapter = mocker.Mock(spec=MockWholeBodyAdapter)
    adapter.connect.return_value = True
    adapter.activate.return_value = False
    factory = mocker.patch.object(whole_body_adapter_registry, "create", return_value=adapter)
    module = topology[0](model=model)
    try:
        with pytest.raises(ConnectionError, match="activate"):
            module.start()
        adapter.disconnect.assert_called_once_with()
        assert not module.status().connected
        assert module._reader is None
        factory.assert_called_once()
    finally:
        module.stop()


def test_malformed_position_column_is_reported_without_writing(rig, topology):
    module, _, callbacks, _, writes, _, _, _ = rig
    callbacks[0](
        JointState(name=[f"arm/{topology[2][0]}", f"arm/{topology[2][1]}"], position=[0.5])
    )
    writes.assert_not_called()
    assert "2 names but 1 position" in module.status().last_rejection


@pytest.mark.parametrize("simulation", ["", "mujoco"])
def test_explicit_can_selection_and_simulation_choose_adapter(
    model, topology, mocker, monkeypatch, simulation
):
    cls = topology[0]
    ports = (
        {"left_can_port": "left", "right_can_port": "right"}
        if cls is OpenArmConnection
        else {"can_port": "test-can"}
    )
    module = cls(model=model, **ports)
    monkeypatch.setattr(module.config.g, "simulation", simulation)
    adapter = MockWholeBodyAdapter(dof=len(topology[2]) + len(topology[3]))
    factory = mocker.patch.object(whole_body_adapter_registry, "create", return_value=adapter)
    try:
        module.connect()
        expected = {
            MetalConnection: "makermods_metal",
            OpenYamConnection: "openyam_damiao",
            OpenArmConnection: "openarm_damiao",
        }[cls]
        assert factory.call_args.args == ("mock_whole_body" if simulation else expected,)
        expected_devices = (
            {"left": "left", "right": "right"}
            if cls is OpenArmConnection
            else {"metal" if cls is MetalConnection else "openyam": "test-can"}
        )
        expected_runtime = (
            None
            if simulation
            else DamiaoRuntimeConfig(
                bus_devices=expected_devices, gravity_comp=cls is not MetalConnection
            )
        )
        assert factory.call_args.kwargs.get("runtime_config") == expected_runtime

    finally:
        module.stop()


def test_model_topology_mismatch_fails_before_opening_can(arm_model, topology, mocker):
    module = topology[0](model=arm_model)
    factory = mocker.patch.object(whole_body_adapter_registry, "create")
    try:
        with pytest.raises(ValueError, match="vendor topology"):
            module.start()
        factory.assert_not_called()
    finally:
        module.stop()
