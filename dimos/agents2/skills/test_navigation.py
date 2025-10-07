# Copyright 2025 Dimensional Inc.
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


from dimos.msgs.geometry_msgs import PoseStamped, Vector3
from dimos.utils.transform_utils import euler_to_quaternion


def test_stop_movement(fake_robot, create_navigation_agent):
    agent = create_navigation_agent(fixture="test_stop_movement.json")
    agent.query("stop")

    fake_robot.stop_exploration.assert_called_once_with()


def test_take_a_look_around(fake_robot, create_navigation_agent, mocker):
    fake_robot.explore.return_value = True
    fake_robot.is_exploration_active.side_effect = [True, False]
    mocker.patch("dimos.agents2.skills.navigation.time.sleep")
    agent = create_navigation_agent(fixture="test_take_a_look_around.json")

    agent.query("take a look around for 10 seconds")

    fake_robot.explore.assert_called_once_with()


def test_go_to_object(fake_robot, create_navigation_agent, mocker):
    fake_robot.object_tracker = mocker.MagicMock()
    fake_robot.object_tracker.is_tracking.side_effect = [True, True, True, True]  # Tracking active
    fake_robot.navigator = mocker.MagicMock()

    # Simulate navigation states: FOLLOWING_PATH -> IDLE (goal reached)
    from dimos.navigation.bt_navigator.navigator import NavigatorState

    fake_robot.navigator.get_state.side_effect = [
        NavigatorState.FOLLOWING_PATH,
        NavigatorState.FOLLOWING_PATH,
        NavigatorState.IDLE,
    ]
    fake_robot.navigator.is_goal_reached.return_value = True

    mocker.patch(
        "dimos.agents2.skills.navigation.NavigationSkillContainer._navigate_by_tagged_location",
        return_value=None,
    )
    mocker.patch(
        "dimos.agents2.skills.navigation.NavigationSkillContainer._navigate_using_semantic_map",
        return_value=None,
    )
    mocker.patch("dimos.agents2.skills.navigation.time.sleep")

    agent = create_navigation_agent(fixture="test_go_to_object.json")

    agent.query("go to the chair")

    fake_robot.object_tracker.track.assert_called_once()
    actual_bbox = fake_robot.object_tracker.track.call_args[0][0]
    expected_bbox = (82, 51, 163, 159)

    for actual_val, expected_val in zip(actual_bbox, expected_bbox):
        assert abs(actual_val - expected_val) <= 5, (
            f"BBox {actual_bbox} not within ±5 of {expected_bbox}"
        )

    fake_robot.object_tracker.stop_track.assert_called_once()


def test_go_to_semantic_location(fake_robot, create_navigation_agent, mocker):
    mocker.patch(
        "dimos.agents2.skills.navigation.NavigationSkillContainer._navigate_by_tagged_location",
        return_value=None,
    )
    mocker.patch(
        "dimos.agents2.skills.navigation.NavigationSkillContainer._navigate_to_object",
        return_value=None,
    )
    fake_robot.spatial_memory = mocker.Mock()
    fake_robot.spatial_memory.query_by_text.return_value = [
        {
            "distance": 0.5,
            "metadata": [
                {
                    "pos_x": 1,
                    "pos_y": 2,
                    "rot_z": 3,
                }
            ],
        }
    ]
    agent = create_navigation_agent(fixture="test_go_to_semantic_location.json")

    agent.query("go to the bookshelf")

    fake_robot.spatial_memory.query_by_text.assert_called_once_with("bookshelf")
    fake_robot.navigate_to.assert_called_once_with(
        PoseStamped(
            position=Vector3(1, 2, 0),
            orientation=euler_to_quaternion(Vector3(0, 0, 3)),
            frame_id="world",
        ),
        blocking=True,
    )
