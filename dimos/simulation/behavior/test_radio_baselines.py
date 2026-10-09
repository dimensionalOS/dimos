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

"""Common baseline contract and sensor provenance checks, without a simulator."""

import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.simulation.behavior.radio_baselines import (
    PressBaseline,
    PressIntent,
    SensorContact,
    perception_intent,
)
from dimos.simulation.behavior.radio_policy import PolicyAction, RadioPolicy


@pytest.fixture
def policy(mocker):
    proxy = mocker.Mock()
    proxy.pose.return_value = PoseStamped(frame_id="base_link", position=[0.2, 0.3, 0.5])
    proxy.move_pose.side_effect = [
        PolicyAction("lift"),
        PolicyAction("cross"),
        PolicyAction("precontact"),
    ]
    proxy.press.return_value = PolicyAction("press")
    proxy.status.side_effect = [
        PolicyAction(x, "completed") for x in ("lift", "cross", "precontact", "press")
    ]
    return RadioPolicy(proxy), proxy


def test_locked_and_sensor_intents_share_checked_sdk_sequence(policy):
    facade, proxy = policy
    runner = PressBaseline(
        facade, PressIntent((0.4, 0.5, 0.6), (0, 0, 0, 1), (0, 1, 0), "locked development intent")
    )
    for _ in range(8):
        runner.tick()
    assert runner.completed
    assert [entry["action_id"] for entry in runner.evidence] == [
        "lift",
        "cross",
        "precontact",
        "press",
    ]
    calls = proxy.move_pose.call_args_list
    assert [call.args[0] for call in calls] == [
        (0.2, 0.3, 0.8),
        (0.4, 0.488, 0.75),
        (0.4, 0.488, 0.6),
    ]
    proxy.press.assert_called_once_with((0, 0.012, 0), 10)
    assert runner.tick() is None


@pytest.mark.parametrize("state", ["failed", "cancelled", "uncertain"])
def test_motion_failure_halts_before_next_stage(policy, state):
    facade, proxy = policy
    proxy.status.side_effect = None
    proxy.status.return_value = PolicyAction("lift", state)
    runner = PressBaseline(facade, PressIntent((0.4, 0.5, 0.6), (0, 0, 0, 1), (0, 1, 0), "test"))
    runner.tick()
    result = runner.tick()
    assert result.state == state
    assert runner.tick() is result
    assert not runner.completed
    assert proxy.move_pose.call_count == 1
    proxy.press.assert_not_called()


def test_pending_cancel_cannot_dispatch_another_stage(policy):
    facade, proxy = policy
    proxy.cancel.return_value = PolicyAction("lift", cancel_requested=True)
    runner = PressBaseline(facade, PressIntent((0.4, 0.5, 0.6), (0, 0, 0, 1), (0, 1, 0), "test"))
    runner.tick()
    cancelled = runner.cancel()
    assert runner.tick() is cancelled
    proxy.cancel.assert_called_once_with("lift")
    assert proxy.move_pose.call_count == 1
    assert not runner.completed


def test_perception_uses_selected_frame_and_robot_finger_offset(policy, mocker):
    facade, proxy = policy
    observation = {"id": "frame-A", "rgb": "actual camera payload"}
    proxy.observe.return_value = observation
    proxy.ground.return_value = {
        "frame": "base_link",
        "observation_id": "frame-A",
        "pixel": [12, 14],
        "position": [0.4, 0.5, 0.6],
    }
    estimator = mocker.Mock(
        return_value=SensorContact(12, 14, (0, 0, 0, 1), (0, 1, 0), (0, 0, -0.04))
    )
    intent = perception_intent(facade, estimator)
    estimator.assert_called_once_with(observation)
    proxy.ground.assert_called_once_with("frame-A", 12, 14)
    assert intent.position == pytest.approx((0.4, 0.5, 0.64))
    assert "frame-A" in intent.provenance
    proxy.move_pose.assert_not_called()


def test_grounding_frame_mismatch_rejected(policy, mocker):
    facade, proxy = policy
    proxy.observe.return_value = {"id": "frame-A"}
    proxy.ground.return_value = {"frame": "world", "observation_id": "frame-A", "pixel": [12, 14]}
    estimator = mocker.Mock(
        return_value=SensorContact(12, 14, (0, 0, 0, 1), (0, 1, 0), (0, 0, -0.04))
    )
    with pytest.raises(ValueError, match="does not match"):
        perception_intent(facade, estimator)
    proxy.move_pose.assert_not_called()


def test_sensor_target_follows_new_observation_instead_of_locked_coordinates(policy, mocker):
    facade, proxy = policy
    proxy.observe.side_effect = [{"id": "frame-A"}, {"id": "frame-B"}]
    proxy.ground.side_effect = [
        {
            "frame": "base_link",
            "observation_id": "frame-A",
            "pixel": [12, 14],
            "position": [0.4, 0.5, 0.6],
        },
        {
            "frame": "base_link",
            "observation_id": "frame-B",
            "pixel": [12, 14],
            "position": [0.48, 0.5, 0.6],
        },
    ]
    estimator = mocker.Mock(
        return_value=SensorContact(12, 14, (0, 0, 0, 1), (0, 1, 0), (0, 0, -0.04))
    )
    first, second = perception_intent(facade, estimator), perception_intent(facade, estimator)
    assert second.position[0] - first.position[0] == pytest.approx(0.08)
    assert second.position[1:] == first.position[1:]
    assert "frame-B" in second.provenance
    proxy.move_pose.assert_not_called()


def test_nonunit_press_direction_rejected():
    with pytest.raises(ValueError, match="unit inward"):
        PressIntent((0.4, 0.5, 0.6), (0, 0, 0, 1), (0, 2, 0), "test")
