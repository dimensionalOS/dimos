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

import threading

from dimos_behavior.runtime import BehaviorRuntime
import pytest

from dimos.experimental.isolated_python.bootstrap import validate_runtime
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.simulation.behavior.connection import BehaviorConnection
from dimos.simulation.behavior.types import ControlMode


@pytest.fixture
def runtime(mocker):
    module = BehaviorRuntime(_isolated_python_runtime=True)
    engine = mocker.Mock()
    engine.measured.return_value = {"arm": 0.1}
    engine.action.return_value = "hold"
    engine.step.return_value = (0.0, False, False, {})
    engine.observation.return_value = {"obs": {}}
    engine.ground_truth.return_value = {"objects": {}}
    engine.task = None
    engine.limits = {"arm": (-1, 1)}
    mocker.patch.object(module, "_engine", engine)
    module._description = {"physical_primitives": ["GRASP"], "symbolic_primitives": ["GRASP"]}
    module._ready.set()
    module._state.state = "running"
    module._hold(ControlMode.DIMOS)
    try:
        yield module, engine
    finally:
        module.stop()


def test_runtime_implements_every_contract_rpc():
    assert validate_runtime(BehaviorConnection, BehaviorRuntime) is None


def test_initialization_failure_preserves_error_and_closes_engine(runtime, mocker):
    module, engine = runtime
    engine.initialize.side_effect = RuntimeError("Camera calibration is invalid")
    mocker.patch(
        "dimos_behavior.runtime.load_class",
        return_value=mocker.Mock(return_value=engine),
    )
    stopping = threading.Event()
    stopping.set()

    module.run_runtime(stopping)

    assert module.get_status().error == "Camera calibration is invalid"
    engine.close.assert_called_once_with()


def test_primitive_requires_explicit_control_transfer(runtime):
    module, engine = runtime
    with pytest.raises(RuntimeError, match="take_control"):
        module.start_primitive("physical", "GRASP", "cup")
    handle = module.take_control(ControlMode.PRIMITIVE)
    module._tick()
    assert module.get_operation(handle).state == "succeeded"
    engine.primitive.return_value = iter(["grasp", "grasp"])
    action = module.start_primitive("physical", "GRASP", "cup")
    module._tick()
    assert module.get_operation(action).state == "running"
    with pytest.raises(RuntimeError, match="busy"):
        module.start_primitive("physical", "GRASP", "cup")


def test_takeover_cancels_primitive_and_clears_commands(runtime):
    module, engine = runtime
    module._hold()
    engine.primitive.return_value = iter(["grasp", "grasp"])
    action = module.start_primitive("physical", "GRASP", "cup")
    module._tick()
    module._command("velocity", Twist(), ControlMode.DIMOS)
    assert module._mailbox == {}
    takeover = module.take_control(ControlMode.DIMOS)
    module._tick()
    assert module.get_operation(action).state == "cancelled"
    assert module.get_operation(takeover).state == "succeeded"
    assert module.get_status().control == ControlMode.DIMOS
    assert engine.step.call_args.args == ("hold",)


def test_cancel_during_planning_discards_produced_action(runtime):
    module, engine = runtime
    module._hold()
    operation = module.start_primitive("physical", "GRASP", "cup")

    def commands():
        module.cancel_operation(operation)
        yield "must not execute"

    engine.primitive.return_value = commands()
    module._tick()
    assert module.get_operation(operation).state == "cancelled"
    engine.step.assert_called_once_with("hold")


def test_terminal_result_is_retained_until_reset(runtime):
    module, engine = runtime
    engine.step.return_value = (1.0, True, False, {"done": {"success": True}})
    module._tick()
    episode = module.get_status().episode.id
    module._tick()
    assert engine.step.call_count == 1
    assert module.get_status().episode.success
    reset = module.reset_task()
    engine.step.return_value = (0.0, False, False, {})
    module._tick()
    assert module.get_operation(reset).state == "succeeded"
    assert module.get_status().episode.id != episode
    assert not module.get_status().episode.success
    assert module.get_ground_truth()["episode"] == module.get_status().episode.id


def test_primitive_failure_does_not_claim_task_success(runtime):
    module, engine = runtime
    module._hold()
    engine.primitive.side_effect = ValueError("unreachable cup")
    operation = module.start_primitive("physical", "GRASP", "cup")
    module._tick()
    assert module.get_operation(operation).state == "failed"
    assert module.get_operation(operation).error == "unreachable cup"
    assert not module.get_status().episode.success


def test_reset_failure_leaves_inspectable_error(runtime):
    module, engine = runtime
    engine.reset.side_effect = RuntimeError("missing scene")
    operation = module.reset_task()
    module._tick()
    assert module.get_operation(operation).state == "failed"
    assert module.get_status().state == "error"
    engine.step.assert_not_called()


def test_stop_unsubscribes_streams(runtime, mocker):
    module, _ = runtime
    subscriptions = [mocker.Mock() for _ in range(3)]
    for stream, unsubscribe in zip(
        (module.cmd_vel, module.joint_command, module.native_action), subscriptions, strict=True
    ):
        mocker.patch.object(stream, "subscribe", return_value=unsubscribe)
    module.start()
    module.stop()
    for unsubscribe in subscriptions:
        unsubscribe.assert_called_once_with()
    assert module._shutdown.is_set()


def test_pause_blocks_physics_and_resume_drops_old_velocity(runtime):
    module, engine = runtime
    module._command("velocity", Twist(linear=[0.2, 0, 0]), ControlMode.DIMOS)
    paused = module.pause()
    module._tick()
    assert module.get_operation(paused).state == "succeeded"
    engine.step.assert_not_called()
    module._command("velocity", Twist(linear=[0.3, 0, 0]), ControlMode.DIMOS)
    resumed = module.resume()
    module._tick()
    assert module.get_operation(resumed).state == "succeeded"
    assert module._control.get_velocity(0) == (0, 0, 0)


def test_failed_generator_is_reported_and_owner_holds(runtime):
    module, engine = runtime
    module._hold()

    def commands():
        yield "first action"
        raise RuntimeError("planner failed")

    engine.primitive.return_value = commands()
    operation = module.start_primitive("physical", "GRASP", "cup")
    module._tick()
    module._tick()
    assert module.get_operation(operation).state == "failed"
    assert module.get_operation(operation).error == "planner failed"
    assert module.get_status().control == ControlMode.PRIMITIVE
    assert engine.step.call_args.args == ("hold",)
