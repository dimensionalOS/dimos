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

from pathlib import Path
import platform
import runpy

import numpy as np
import pytest
import pytest_asyncio

from dimos.control.sonic.sonic_pipeline import DEFAULT_ANGLES_DDS
from dimos.core.global_config import global_config
from dimos.hardware.whole_body.spec import IMUState, MotorState
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.String import String
from dimos.robot.unitree.g1.control_safety import damping_commands
from dimos.robot.unitree.g1.sonic_policy import G1SonicPolicyModule


@pytest.fixture
def clock_check(mocker):
    return mocker.patch("dimos.robot.unitree.g1.sonic_controller.require_jetson_clocks")


@pytest_asyncio.fixture
async def connection(mocker, tmp_path, clock_check):
    model = tmp_path / "model.onnx"
    model.touch()
    pipeline = mocker.patch("dimos.robot.unitree.g1.sonic_controller.SonicPipeline").return_value
    pipeline.step.return_value = np.ones(29, dtype=np.float32)
    pipeline.snapshot.return_value = {"stream_active": False}
    adapter = mocker.patch(
        "dimos.robot.unitree.g1.sonic_controller.SimMujocoG1WholeBodyAdapter"
    ).return_value
    adapter.read_motor_states.return_value = [MotorState()] * 29
    adapter.read_imu.return_value = IMUState()
    c = G1SonicPolicyModule(
        encoder_onnx=model,
        decoder_onnx=model,
        planner_onnx=model,
        simulation_address=tmp_path / "sim.xml",
        auto_dry_run=False,
    )
    # Clock ticks are explicit in these tests; ONNX and simulator IO are the boundaries.
    mocker.patch.object(c._controller, "_run")
    mocker.patch.object(c, "_auto_bind_handlers")
    mocker.patch.object(c.joint_state, "publish")
    mocker.patch.object(c.imu, "publish")
    yield c
    c.stop()


def _step(connection, t_now, positions=0.0):
    connection._controller._adapter.read_motor_states.return_value = [MotorState(q=positions)] * 29
    return connection._controller._step(t_now)


def _arm(connection):
    connection.start()
    connection.arm()
    _step(connection, 0.0)
    return _step(connection, 3.0)


def test_hold_and_measured_pose_ramp_precede_policy_output(connection):
    connection.start()
    hold = _step(connection, 9.0, positions=0.25)
    assert hold == pytest.approx([0.25] * 29)

    connection.arm()
    first = _step(connection, 10.0, positions=0.4)
    halfway = _step(connection, 11.5, positions=0.4)
    complete = _step(connection, 13.0, positions=0.4)

    assert first == pytest.approx([0.4] * 29)
    assert halfway == pytest.approx((0.4 + 0.5 * (DEFAULT_ANGLES_DDS - 0.4)).tolist())
    assert complete == pytest.approx(DEFAULT_ANGLES_DDS.tolist())
    connection._controller._policy.step.assert_not_called()
    assert _step(connection, 13.02) == [1.0] * 29


def test_dry_run_republishes_hold_while_policy_runs(connection):
    ramp = _arm(connection)
    connection.set_dry_run(True)

    first = _step(connection, 3.02)
    second = _step(connection, 3.04)

    assert first == second == ramp
    assert first != [1.0] * 29
    assert connection._controller._policy.step.call_count == 2


def test_hardware_start_rejects_unlocked_clocks_before_loading_models(
    connection, clock_check, monkeypatch, mocker
):
    monkeypatch.setattr(connection.config, "simulation_address", None)
    clock_check.side_effect = RuntimeError("SONIC requires locked Jetson CPU/GPU clocks")
    pipeline = mocker.patch("dimos.robot.unitree.g1.sonic_controller.SonicPipeline")
    publish = mocker.patch.object(connection.motor_command, "publish")

    with pytest.raises(RuntimeError, match="requires locked Jetson"):
        connection.start()

    assert connection.status()["active"] is False
    pipeline.assert_not_called()
    publish.assert_not_called()


def test_cpu_simulation_cannot_start_hardware(connection, monkeypatch, mocker):
    monkeypatch.setattr(connection.config, "simulation_address", None)
    monkeypatch.setattr(connection.config, "cpu_simulation", True)
    pipeline = mocker.patch("dimos.robot.unitree.g1.sonic_controller.SonicPipeline")
    publish = mocker.patch.object(connection.motor_command, "publish")

    with pytest.raises(ValueError, match="simulation, not hardware control"):
        connection.start()

    assert connection.status()["active"] is False
    pipeline.assert_not_called()
    publish.assert_not_called()


def test_simulation_passes_explicit_cpu_selection_to_inference(connection, monkeypatch, mocker):
    monkeypatch.setattr(connection.config, "cpu_simulation", True)
    pipeline = mocker.patch("dimos.robot.unitree.g1.sonic_controller.SonicPipeline")

    connection.start()

    pipeline.assert_called_once_with(
        encoder_path=connection.config.encoder_onnx,
        decoder_path=connection.config.decoder_onnx,
        planner_path=connection.config.planner_onnx,
        cpu_simulation=True,
    )
    assert connection.status()["active"] is True


@pytest.mark.parametrize("operation", ["arm", "activate", "live", "reactivate"])
def test_unlocked_clocks_reject_activation_without_changing_control_state(
    connection, clock_check, monkeypatch, operation
):
    connection.start()
    connection.set_dry_run(True)
    # Keep fake feedback IO but exercise the hardware clock prerequisite.
    monkeypatch.setattr(connection.config, "simulation_address", None)
    clock_check.side_effect = RuntimeError("SONIC requires locked Jetson CPU/GPU clocks")

    with pytest.raises(RuntimeError, match="requires locked Jetson"):
        if operation == "arm":
            connection.arm()
        elif operation == "activate":
            connection.activate()
        elif operation == "live":
            connection.set_dry_run(False)
        else:
            connection.reset_runtime_state(reactivate=True)

    state = connection.status()
    assert state["control_state"] == "unarmed"
    assert state["arm_pending"] is False
    assert state["dry_run"] is True
    assert state["fault_reason"] is None
    connection._controller._adapter.write_motor_commands.assert_not_called()


def test_simulation_skips_jetson_clock_requirements(connection, clock_check):
    clock_check.side_effect = RuntimeError("unlocked clocks")
    connection.start()
    connection.set_dry_run(True)

    connection.set_dry_run(False)
    assert connection.arm()
    assert connection.reset_runtime_state(reactivate=True)
    assert connection.status()["dry_run"] is False
    clock_check.assert_not_called()


def test_clock_changes_do_not_block_deactivation_or_stop(connection, clock_check, monkeypatch):
    _arm(connection)
    monkeypatch.setattr(connection.config, "simulation_address", None)
    clock_check.side_effect = RuntimeError("unlocked clocks")

    connection.set_dry_run(True)
    assert connection.reset_runtime_state(reactivate=False)
    assert connection.status()["control_state"] == "unarmed"
    assert connection.halt_robot()
    assert connection.status()["fault_reason"] == "operator stop"
    clock_check.assert_not_called()


def test_activate_enables_live_control_and_preserves_the_pose_ramp(connection):
    connection.start()
    connection.set_dry_run(True)

    assert connection.activate()
    assert connection.status()["arm_pending"]
    assert connection.status()["dry_run"] is False
    assert _step(connection, 10.0, positions=0.4) == pytest.approx([0.4] * 29)
    assert _step(connection, 13.0) == pytest.approx(DEFAULT_ANGLES_DDS.tolist())
    assert _step(connection, 13.02) == [1.0] * 29


@pytest.mark.parametrize("ramp", [-1.0, float("nan")])
def test_rejected_activation_does_not_disable_dry_run(connection, ramp):
    connection.start()
    connection.set_dry_run(True)

    with pytest.raises(ValueError):
        connection.activate(ramp)

    state = connection.status()
    assert state["dry_run"] is True
    assert state["arm_pending"] is False


def test_activation_requires_feedback_and_cannot_clear_a_fault(connection, mocker):
    connection.start()
    connection.set_dry_run(True)
    mocker.patch.object(connection._controller._adapter, "has_motor_states", return_value=False)

    with pytest.raises(RuntimeError, match="Waiting for joint"):
        connection.activate()
    assert connection.status()["dry_run"] is True
    assert connection.status()["arm_pending"] is False

    connection.halt_robot()
    with pytest.raises(RuntimeError, match="latched"):
        connection.activate()
    assert connection.status()["control_state"] == "fault"


def test_activate_does_not_change_an_already_armed_dry_run(connection):
    _arm(connection)
    connection.set_dry_run(True)

    with pytest.raises(RuntimeError, match="Deactivate"):
        connection.activate()

    assert connection.status()["dry_run"] is True


def test_stop_motion_clears_references_without_stopping_balance(connection):
    _arm(connection)
    connection.set_velocity_command(0.5, 0.2, 0.1, t_now=3.0)
    pipeline = connection._controller._policy

    connection.stop_motion()
    _step(connection, 3.02)

    pipeline.stop_clip.assert_called_once()
    pipeline.clear_vr_3point.assert_called_once()
    pipeline.set_velocity.assert_called_with(0.0, 0.0, 0.0)
    assert connection.status()["control_state"] == "control"


def test_runtime_reset_clears_velocity_and_repeats_the_measured_pose_ramp(connection):
    _arm(connection)
    connection.set_velocity_command(0.5, 0.0, 0.0, t_now=3.0)
    _step(connection, 3.02)
    connection._controller._policy.set_velocity.assert_called_with(0.5, 0.0, 0.0)

    assert connection.reset_runtime_state()
    first = _step(connection, 4.0, positions=0.4)
    complete = _step(connection, 7.0, positions=0.4)
    live = _step(connection, 7.02)

    assert first == pytest.approx([0.4] * 29)
    assert complete == pytest.approx(DEFAULT_ANGLES_DDS.tolist())
    assert live == [1.0] * 29
    connection._controller._policy.set_velocity.assert_called_with(0.0, 0.0, 0.0)


def test_disarm_holds_measured_pose_until_explicit_rearm(connection):
    _arm(connection)
    _step(connection, 3.02)
    connection._controller._policy.step.reset_mock()

    assert connection.disarm()
    assert _step(connection, 4.0, positions=0.6) == pytest.approx([0.6] * 29)
    assert _step(connection, 7.0, positions=0.6) == pytest.approx([0.6] * 29)
    assert connection.state_snapshot()["control_state"] == "unarmed"
    connection._controller._policy.step.assert_not_called()

    connection.arm(ramp_seconds=0.0)
    _step(connection, 8.0, positions=0.6)
    assert _step(connection, 8.02) == [1.0] * 29
    assert connection.state_snapshot()["control_state"] == "control"


def test_velocity_timeout_stops_motion_but_keeps_balancing(connection):
    _arm(connection)
    connection.set_velocity_command(0.5, 0.0, 0.0, t_now=3.0)
    _step(connection, 3.5)
    connection._controller._policy.set_velocity.assert_called_with(0.5, 0.0, 0.0)

    assert _step(connection, 4.02) == [1.0] * 29
    connection._controller._policy.set_velocity.assert_called_with(0.0, 0.0, 0.0)
    assert connection.status()["armed"]


@pytest.mark.parametrize(
    "operator_stop, reason", [(False, "decoder failed"), (True, "operator stop")]
)
def test_fault_during_inference_discards_targets_and_stays_latched(
    connection, operator_stop, reason
):
    _arm(connection)

    def inference(**kwargs):
        if operator_stop:
            connection.set_estop(True)
            return np.ones(29, dtype=np.float32)
        raise RuntimeError("decoder failed")

    connection._controller._policy.step.side_effect = inference

    assert _step(connection, 3.02) is None
    with pytest.raises(RuntimeError, match="restart"):
        connection.start()
    assert _step(connection, 3.04, positions=0.4) is None
    assert connection.joint_state.publish.call_args.args[0].position == pytest.approx([0.4] * 29)
    assert connection.status()["fault_reason"] == reason
    connection._controller._adapter.write_motor_commands.assert_called_with(damping_commands(29))
    connection._controller._policy.step.assert_called_once()
    assert connection.reset_runtime_state(reactivate=True) is False
    with pytest.raises(RuntimeError, match="restart"):
        connection.arm()
    with pytest.raises(RuntimeError, match="restart"):
        connection.set_estop(False)


@pytest.mark.asyncio
@pytest.mark.parametrize(
    "source, reason", [("operator", "operator stop"), ("hardware", "stale feedback")]
)
async def test_hardware_faults_do_not_trigger_motor_takeover(connection, mocker, source, reason):
    # Before startup there is no simulator adapter: the real DDS connection owns the stop.
    publish = mocker.patch.object(connection.controller_fault, "publish")
    motor_publish = mocker.patch.object(connection.motor_command, "publish")
    if source == "operator":
        connection.set_estop(True)
    else:
        await connection.handle_g1_fault(String(reason))
    assert connection.status()["fault_reason"] == reason
    assert publish.call_args.args[0].data == reason
    motor_publish.assert_not_called()


@pytest.mark.asyncio
async def test_hardware_feedback_drives_commands_and_staleness_latches_stop(
    connection, mocker, monkeypatch
):
    monkeypatch.setattr(connection.config, "simulation_address", None)
    publish = mocker.patch.object(connection.motor_command, "publish")
    fault = mocker.patch.object(connection.controller_fault, "publish")
    connection.start()
    await connection.handle_motor_states(
        JointState(position=[0.4] * 29, velocity=[0.0] * 29, effort=[0.0] * 29)
    )
    await connection.handle_low_level_imu(Imu(orientation=Quaternion(0, 0, 0, 1)))
    now = max(connection._controller._feedback_at, connection._controller._imu_at)
    connection.arm(ramp_seconds=0.0)
    connection._controller._step(now)
    connection._controller._step(now + 0.02)

    assert publish.call_args.args[0].q == [1.0] * 29
    np.testing.assert_array_equal(
        connection._controller._policy.step.call_args.kwargs["base_quat_wxyz"], [1, 0, 0, 0]
    )
    assert connection.joint_state.publish.call_args.args[0].position == [0.4] * 29
    publish.reset_mock()

    assert connection._controller._step(now + connection.config.feedback_timeout) is None
    assert fault.call_args.args[0].data == "SONIC feedback timeout"
    publish.assert_not_called()


@pytest.mark.asyncio
async def test_standard_inputs_drive_policy_and_publish_measured_state(connection):
    _arm(connection)
    await connection.handle_base_command(
        Twist(linear=Vector3(0.3, 0.1, 0), angular=Vector3(0, 0, 0.2))
    )
    arm_name = connection.config.joint_names[15]
    await connection.handle_position_command(JointState(name=[arm_name], position=[0.7]))
    _step(connection, 3.02, positions=0.4)

    assert connection._controller._policy.set_velocity.call_args.args == pytest.approx(
        (0.3, 0.1, 0.2)
    )
    expected = [0.02, 0.02, 0.0, 0.02, 0.0, 0.0, 0.0, 0.02, -0.02, 0.0, 0.02, 0.0, 0.0, 0.0]
    np.testing.assert_allclose(
        connection._controller._policy.set_upper_body.call_args.args[0], expected
    )
    feedback = connection.joint_state.publish.call_args.args[0]
    assert feedback.name == connection.config.joint_names
    assert feedback.position == pytest.approx([0.4] * 29)
    assert [
        m.q for m in connection._controller._adapter.write_motor_commands.call_args.args[0]
    ] == [1.0] * 29

    with pytest.raises(ValueError, match="only accepts arm"):
        await connection.handle_position_command(
            JointState(name=[connection.config.joint_names[0]], position=[0.7])
        )
    np.testing.assert_allclose(
        connection._controller._policy.set_upper_body.call_args.args[0], expected
    )


def test_arm_references_ramp_from_feedback_and_retarget_without_jumps(connection):
    _arm(connection)
    _step(connection, 3.02, positions=0.4)
    targets = [0.9, -0.1] + [0.43] * 12
    connection.set_upper_body(targets)
    _step(connection, 3.04, positions=0.4)
    np.testing.assert_allclose(
        connection._controller._policy.set_upper_body.call_args.args[0],
        [0.42, 0.38] + [0.42] * 12,
    )

    connection.set_upper_body(targets)
    _step(connection, 3.06, positions=0.4)
    np.testing.assert_allclose(
        connection._controller._policy.set_upper_body.call_args.args[0],
        [0.44, 0.36] + [0.43] * 12,
    )

    connection.clear_upper_body()
    _step(connection, 3.08, positions=0.4)
    np.testing.assert_allclose(
        connection._controller._policy.set_upper_body.call_args.args[0][:2], [0.42, 0.34]
    )


@pytest.mark.asyncio
async def test_stream_command_expires_without_an_upstream_task(connection, mocker):
    _arm(connection)
    mocker.patch("dimos.robot.unitree.g1.sonic_controller.time.perf_counter", return_value=3.0)
    await connection.handle_base_command(Twist(linear=Vector3(0.5, 0, 0)))
    _step(connection, 3.02)
    connection._controller._policy.set_velocity.assert_called_with(0.5, 0.0, 0.0)

    assert _step(connection, 4.02) == [1.0] * 29
    connection._controller._policy.set_velocity.assert_called_with(0.0, 0.0, 0.0)
    assert connection.status()["armed"]
    assert connection.status()["fault_reason"] is None


@pytest.mark.parametrize("command_while_disarmed", [False, True])
def test_rearm_does_not_replay_old_walking_commands(connection, command_while_disarmed):
    _arm(connection)
    connection.set_velocity_command(0.5, 0.0, 0.0, t_now=3.0)
    _step(connection, 3.02)
    connection._controller._policy.set_velocity.assert_called_with(0.5, 0.0, 0.0)
    assert connection.disarm()
    if command_while_disarmed:
        connection.set_velocity_command(0.8, 0.0, 0.0, t_now=3.03)

    assert connection.arm(ramp_seconds=0.0)
    _step(connection, 3.04)
    assert _step(connection, 3.06) == [1.0] * 29
    connection._controller._policy.set_velocity.assert_called_with(0.0, 0.0, 0.0)
    assert connection.status()["armed"]

    connection.set_velocity_command(-0.2, 0.0, 0.0, t_now=3.07)
    _step(connection, 3.08)
    assert connection._controller._policy.set_velocity.call_args.args == pytest.approx(
        (-0.2, 0.0, 0.0)
    )


def test_stopping_sonic_latches_damping(connection):
    _arm(connection)
    adapter = connection._controller._adapter
    connection.stop()

    adapter.write_motor_commands.assert_called_with(damping_commands(29))
    assert connection.status()["fault_reason"] == "SONIC module stopped"
    assert connection.reset_runtime_state(reactivate=True) is False


@pytest.mark.self_hosted
@pytest.mark.parametrize("system", ["Darwin", "Linux"])
@pytest.mark.parametrize("viewer", ["none", "rerun"])
@pytest.mark.parametrize(
    "simulation, backend", [("", "G1WholeBodyConnection"), ("mujoco", "MujocoSimModule")]
)
def test_blueprint_routes_policy_commands_without_coordinator_or_sim2(
    monkeypatch, mocker, simulation, backend, system, viewer
):
    mocker.patch.object(platform, "system", return_value=system)
    monkeypatch.setattr(global_config, "simulation", simulation)
    monkeypatch.setattr(global_config, "viewer", viewer)
    monkeypatch.setattr(global_config, "scene_package", None)
    monkeypatch.setattr(global_config, "transport", "zenoh")
    mocker.patch("dimos.utils.data.get_data", side_effect=AssertionError("unexpected asset load"))
    blueprint = runpy.run_path(
        str(Path(__file__).parent / "blueprints/basic/unitree_g1_sonic_wbc.py")
    )["unitree_g1_sonic_wbc"]

    modules = {atom.module.__name__ for atom in blueprint.active_blueprints}
    assert "G1SonicPolicyModule" in modules
    policy = next(
        atom for atom in blueprint.active_blueprints if atom.module is G1SonicPolicyModule
    )
    assert policy.kwargs["cpu_simulation"] is (simulation == "mujoco" and system == "Darwin")
    assert "MovementManager" in modules
    assert backend in modules
    assert not {"RerunBridgeModule", "RerunWebSocketServer"} & modules
    assert not {"SonicCoordinator", "ControlCoordinator"} & modules
    assert all(
        not atom.module.__module__.startswith("dimos.sim2") for atom in blueprint.active_blueprints
    )
    assert blueprint.remapping_map[(G1SonicPolicyModule.name, "base_command")] == "cmd_vel"
    assert (
        blueprint.transport_map[("cmd_vel", Twist)].channel == "dimos/cmd_vel/geometry_msgs.Twist"
    )
    assert blueprint.transport_map[("cmd_vel", Twist)].topic.queue_capacity == 1
    assert blueprint.transport_map[("position_command", JointState)].channel == (
        "dimos/g1/position_command/sensor_msgs.JointState"
    )
    assert ("joint_command", JointState) not in blueprint.transport_map
