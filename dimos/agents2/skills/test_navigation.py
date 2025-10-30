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
    navigation_skill_container._cancel_goal = mocker.Mock()
    navigation_skill_container._stop_exploration = mocker.Mock()
    agent = create_navigation_agent(fixture="test_stop_movement.json")

    agent.query("stop")

    navigation_skill_container._cancel_goal.assert_called_once_with()
    navigation_skill_container._stop_exploration.assert_called_once_with()


def test_take_a_look_around(create_navigation_agent, navigation_skill_container, mocker) -> None:
    navigation_skill_container._explore = mocker.Mock()
    navigation_skill_container._is_exploration_active = mocker.Mock()
    mocker.patch("dimos.agents2.skills.navigation.time.sleep")
    agent = create_navigation_agent(fixture="test_take_a_look_around.json")

    agent.query("take a look around for 10 seconds")

    navigation_skill_container._explore.assert_called_once_with()


def test_go_to_semantic_location(
    create_navigation_agent, navigation_skill_container, mocker
) -> None:
    mocker.patch(
        "dimos.agents2.skills.navigation.NavigationSkillContainer._navigate_by_tagged_location",
        return_value=None,
    )
    mocker.patch(
        "dimos.agents2.skills.navigation.NavigationSkillContainer._navigate_to_object",
        return_value=None,
    )
    mocker.patch(
        "dimos.agents2.skills.navigation.NavigationSkillContainer._navigate_to",
        return_value=True,
    )
    navigation_skill_container._query_by_text = mocker.Mock(
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
    agent = create_navigation_agent(fixture="test_go_to_semantic_location.json")

    agent.query("go to the bookshelf")

    navigation_skill_container._query_by_text.assert_called_once_with("bookshelf")
    navigation_skill_container._navigate_to.assert_called_once_with(
        PoseStamped(
            position=Vector3(1, 2, 0),
            orientation=euler_to_quaternion(Vector3(0, 0, 3)),
            frame_id="world",
        ),
    )
