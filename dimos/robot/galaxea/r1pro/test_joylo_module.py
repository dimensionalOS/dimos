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

import pytest

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.galaxea.r1pro.joylo import JAWS, UPPER, JoyloController
from dimos.robot.galaxea.r1pro.joylo_module import R1ProJoylo


@pytest.fixture
def module(mocker):
    mocker.patch("dimos.robot.galaxea.r1pro.joylo_module.time.time", return_value=100)
    mocker.patch("dimos.robot.galaxea.r1pro.joylo_module.time.monotonic", return_value=10)
    instance = R1ProJoylo()
    instance._controller = JoyloController(dict.fromkeys(UPPER, (-3, 3)), instance.config)
    yield instance
    instance.dispose()


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


@pytest.mark.asyncio
async def test_separate_hardware_feedback_packets_form_complete_measured_hold(module):
    await module.handle_robot_feedback(
        JointState(ts=99.9, name=list(UPPER), position=[0.2] * len(UPPER))
    )
    assert module._controller.step(10, 0.01) is None
    await module.handle_robot_feedback(JointState(ts=100, name=list(JAWS), position=[0.03, 0.07]))
    output = module._controller.step(10, 0.01)
    assert output.positions == dict.fromkeys(UPPER, 0.2)
    assert output.openings == pytest.approx((0.3, 0.7))
    assert output.velocity == (0, 0, 0)
    assert not module.status()["active"]
    assert module.status()["input_age_s"]["robot"] == pytest.approx(0.1)


@pytest.mark.asyncio
@pytest.mark.parametrize("stale", [UPPER, JAWS])
async def test_fresh_other_joints_cannot_renew_stale_feedback(module, mocker, stale):
    names = [*UPPER, *JAWS]
    await module.handle_robot_feedback(JointState(ts=100, name=names, position=[0.1] * len(names)))
    mocker.patch("dimos.robot.galaxea.r1pro.joylo_module.time.time", return_value=100.3)
    mocker.patch("dimos.robot.galaxea.r1pro.joylo_module.time.monotonic", return_value=10.3)
    fresh = [name for name in names if name not in stale]
    await module.handle_robot_feedback(
        JointState(ts=100.3, name=fresh, position=[0.2] * len(fresh))
    )
    assert module._controller.step(10.3, 0.01) is None
    assert "robot" in module.status()["reason"]
    assert module.status()["input_age_s"]["robot"] == pytest.approx(0.3)


@pytest.mark.asyncio
async def test_delayed_feedback_does_not_overwrite_new_pose_or_refresh_its_age(module):
    names = [*UPPER, *JAWS]
    await module.handle_robot_feedback(
        JointState(ts=99.95, name=names, position=[0.1] * len(names))
    )
    await module.handle_robot_feedback(JointState(ts=99.9, name=names, position=[0.9] * len(names)))
    output = module._controller.step(10, 0.01)
    assert output.positions == dict.fromkeys(UPPER, 0.1)
    assert module.status()["input_age_s"]["robot"] == pytest.approx(0.05)
