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

import math
import threading
from types import SimpleNamespace

import pytest

from dimos.control.tasks.trajectory_task.trajectory_task import (
    JointTrajectoryTask,
    JointTrajectoryTaskConfig,
)
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.robot.galaxea.r1pro._connection_test_fakes import LocalTransport
from dimos.robot.galaxea.r1pro.joylo import JAWS, UPPER, JoyloController
from dimos.robot.galaxea.r1pro.joylo_coordinator import R1ProJoyloCoordinator
from dimos.robot.galaxea.r1pro.joylo_module import R1ProJoylo
from dimos.robot.galaxea.r1pro.motion import motion_groups


@pytest.fixture
def module(mocker):
    mocker.patch("dimos.robot.galaxea.r1pro.joylo_module.time.time", return_value=100)
    mocker.patch("dimos.robot.galaxea.r1pro.joylo_module.time.monotonic", return_value=10)
    instance = R1ProJoylo()
    coordinator = R1ProJoyloCoordinator(motion_limits=motion_groups())
    coordinator.add_task(
        JointTrajectoryTask(
            JointTrajectoryTaskConfig(
                joint_names=list(UPPER), velocity_limits=dict.fromkeys(UPPER, 3.0)
            )
        )
    )
    instance._motion_limits = coordinator
    instance._controller = JoyloController(dict.fromkeys(UPPER, (-3, 3)), instance.config)
    yield instance
    instance.dispose()
    coordinator.dispose()


@pytest.mark.parametrize("stamp", [99.7, 100.1, math.nan, math.inf])
def test_queued_old_future_or_nonfinite_samples_do_not_renew_input_authority(module, stamp):
    assert not module._fresh("leader", stamp)
    assert module._fresh("leader", 100)


def test_duplicates_and_reordering_are_rejected_per_input_source(module):
    assert module._fresh("leader", 99.9)
    assert not module._fresh("leader", 99.9)
    assert not module._fresh("leader", 99.8)
    assert module._fresh("joycon", 99.8)
    assert module._fresh("leader", 100)


def test_separate_hardware_feedback_packets_form_complete_measured_hold(module):
    module._on_robot_feedback(JointState(ts=99.9, name=list(UPPER), position=[0.2] * len(UPPER)))
    assert module._controller.step(10, 0.01) is None
    module._on_robot_feedback(JointState(ts=100, name=list(JAWS), position=[0.03, 0.07]))
    output = module._controller.step(10, 0.01)
    assert output.positions == dict.fromkeys(UPPER, 0.2)
    assert output.openings == pytest.approx((0.3, 0.7))
    assert output.velocity == (0, 0, 0)
    assert not module.status()["active"]
    assert module.status()["input_age_s"]["robot"] == pytest.approx(0.1)


@pytest.mark.parametrize("stale", [UPPER, JAWS])
def test_fresh_other_joints_cannot_renew_stale_feedback(module, mocker, stale):
    names = [*UPPER, *JAWS]
    module._on_robot_feedback(JointState(ts=100, name=names, position=[0.1] * len(names)))
    mocker.patch("dimos.robot.galaxea.r1pro.joylo_module.time.time", return_value=100.3)
    mocker.patch("dimos.robot.galaxea.r1pro.joylo_module.time.monotonic", return_value=10.3)
    fresh = [name for name in names if name not in stale]
    module._on_robot_feedback(JointState(ts=100.3, name=fresh, position=[0.2] * len(fresh)))
    assert module._controller.step(10.3, 0.01) is None
    assert "robot" in module.status()["reason"]
    assert module.status()["input_age_s"]["robot"] == pytest.approx(0.3)


def test_delayed_feedback_does_not_overwrite_new_pose_or_refresh_its_age(module):
    names = [*UPPER, *JAWS]
    module._on_robot_feedback(JointState(ts=99.95, name=names, position=[0.1] * len(names)))
    module._on_robot_feedback(JointState(ts=99.9, name=names, position=[0.9] * len(names)))
    output = module._controller.step(10, 0.01)
    assert output.positions == dict.fromkeys(UPPER, 0.1)
    assert module.status()["input_age_s"]["robot"] == pytest.approx(0.05)


@pytest.mark.asyncio
async def test_desktop_speed_input_changes_controller_without_enabling_it(module):
    await module.handle_arm_tracking_speed(Float32(data=0.75))
    await module.handle_drive_speed(Float32(data=0.3))
    assert module._motion_limits.get_motion_limits()["arm_rate"]["rate"] == 0.75
    assert module._controller.config.base_speed == 0.3
    assert not module.status()["active"]


@pytest.mark.parametrize("bad", [-1, math.nan, math.inf, 4])
def test_invalid_speed_request_changes_nothing(module, bad):
    before = module.set_motion_speeds()
    with pytest.raises(ValueError):
        module.set_motion_speeds(base_speed=0.3, arm_rate=bad)
    assert module.set_motion_speeds() == before


def test_deployment_speed_ceiling_and_zero_rate(module):
    with pytest.raises(ValueError):
        module.set_motion_speeds(arm_rate=1.1)
    assert module.set_motion_speeds(arm_rate=0)["arm_rate"] == 0


def test_speed_readback_describes_live_values_and_deployment_bounds(module):
    module.set_motion_speeds(base_speed=0.4, yaw_speed=0.8)
    settings = module.get_motion_speeds()
    assert settings["settings"][settings["drive"]["linear"]]["value"] == 0.4
    assert settings["settings"][settings["drive"]["angular"]]["value"] == 0.8
    assert settings["settings"]["arm_rate"]["max"] == 1.0
    assert not module.status()["active"]


def test_feedback_burst_keeps_arm_and_jaw_updates_while_async_loop_is_busy(module, mocker):
    model = SimpleNamespace(
        joints=[SimpleNamespace(name=name, lower=-3.0, upper=3.0) for name in UPPER]
    )
    mocker.patch.object(type(module.config.model), "load", return_value=model)
    mocker.patch.object(module, "_run")  # Only exercise feedback; no periodic command production.
    ports = {name: LocalTransport() for name in {**module.inputs, **module.outputs}}
    for name, transport in ports.items():
        module.set_transport(name, transport)
    module.start()
    entered, release = threading.Event(), threading.Event()

    def busy_loop():
        entered.set()
        release.wait(timeout=5)

    try:
        module._loop.call_soon_threadsafe(busy_loop)
        assert entered.wait(timeout=2)
        ports["robot_feedback"].publish(
            JointState(ts=99.9, name=list(UPPER), position=[0.2] * len(UPPER))
        )
        ports["robot_feedback"].publish(JointState(ts=100, name=list(JAWS), position=[0.03, 0.07]))
        # A chassis packet follows on the same topic, with velocity and no position.
        ports["robot_feedback"].publish(JointState(ts=100, name=["base"], velocity=[0]))
        output = module._controller.step(10, 0.01)
        assert output is not None
        assert output.positions == dict.fromkeys(UPPER, 0.2)
        assert output.openings == pytest.approx((0.3, 0.7))
        assert module.status()["input_age_s"]["robot"] == pytest.approx(0.1)
    finally:
        release.set()
        module.stop()
