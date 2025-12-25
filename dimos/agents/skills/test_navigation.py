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


# @pytest.mark.skip
def test_stop_movement(create_navigation_agent, navigation_skill_container, mocker) -> None:
    cancel_goal_mock = mocker.Mock()
    stop_exploration_mock = mocker.Mock()
    navigation_skill_container._bound_rpc_calls["NavigationInterface.cancel_goal"] = (
        cancel_goal_mock
    )
    navigation_skill_container._bound_rpc_calls["WavefrontFrontierExplorer.stop_exploration"] = (
        stop_exploration_mock
    )
    agent = create_navigation_agent(fixture="test_stop_movement.json")

    agent.query("stop")

    cancel_goal_mock.assert_called_once_with()
    stop_exploration_mock.assert_called_once_with()


def test_take_a_look_around(create_navigation_agent, navigation_skill_container, mocker) -> None:
    explore_mock = mocker.Mock()
    is_exploration_active_mock = mocker.Mock()
    navigation_skill_container._bound_rpc_calls["WavefrontFrontierExplorer.explore"] = explore_mock
    navigation_skill_container._bound_rpc_calls[
        "WavefrontFrontierExplorer.is_exploration_active"
    ] = is_exploration_active_mock
    mocker.patch("dimos.agents.skills.navigation.time.sleep")
    agent = create_navigation_agent(fixture="test_take_a_look_around.json")

    agent.query("take a look around for 10 seconds")

    explore_mock.assert_called_once_with()


def test_go_to_semantic_location(
    create_navigation_agent, navigation_skill_container, mocker
) -> None:
    mocker.patch(
        "dimos.agents.skills.navigation.NavigationSkillContainer._navigate_by_tagged_location",
        return_value=None,
    )
    mocker.patch(
        "dimos.agents.skills.navigation.NavigationSkillContainer._navigate_to_object",
        return_value=None,
    )
    navigate_to_mock = mocker.patch(
        "dimos.agents.skills.navigation.NavigationSkillContainer._navigate_to",
        return_value=True,
    )
    query_by_text_mock = mocker.Mock(
        return_value=[
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
    )
    navigation_skill_container._bound_rpc_calls["SpatialMemory.query_by_text"] = query_by_text_mock
    agent = create_navigation_agent(fixture="test_go_to_semantic_location.json")

    agent.query("go to the bookshelf")

    query_by_text_mock.assert_called_once_with("bookshelf")
    navigate_to_mock.assert_called_once_with(
        PoseStamped(
            position=Vector3(1, 2, 0),
            orientation=euler_to_quaternion(Vector3(0, 0, 3)),
            frame_id="world",
        ),
    )
