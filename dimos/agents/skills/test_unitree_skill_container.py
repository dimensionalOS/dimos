# Copyright 2025-2026 Dimensional Inc.
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


def test_pounce(mocker, create_unitree_skills_agent, unitree_skills) -> None:
    agent = create_unitree_skills_agent(fixture="test_pounce.json")
    publish_request_mock = mocker.Mock()
    unitree_skills.get_rpc_calls = mocker.Mock(return_value=publish_request_mock)

    response = agent.query("pounce")

    assert "front pounce" in response.lower()
    publish_request_mock.assert_called_once_with("rt/api/sport/request", {"api_id": 1032})


def test_did_you_mean(mocker, unitree_skills) -> None:
    unitree_skills.get_rpc_calls = mocker.Mock()
    assert (
        unitree_skills.execute_sport_command("Pounce")
        == "There's no 'Pounce' command. Did you mean: ['FrontPounce', 'Pose']"
    )
