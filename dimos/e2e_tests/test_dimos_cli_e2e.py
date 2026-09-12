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

import pickle

import pytest


@pytest.mark.skipif_in_ci
@pytest.mark.self_hosted
@pytest.mark.skipif_no_openai
def test_dimos_skills(lcm_spy, start_blueprint, wait_for_system_ready, human_input) -> None:
    lcm_spy.save_topic("/agent")

    call = start_blueprint("run", "demo-skill")

    wait_for_system_ready(call)

    human_input("what is 52983 + 587237")

    lcm_spy.wait_for_saved_topic_content("/agent", b"640220")

    # Zenoh RPC is a query, not a bus message, so the spy never sees the skill
    # call. The agent's tool result is what shows it ran.
    agent_messages = [pickle.loads(msg) for msg in lcm_spy.messages["/agent"]]
    assert any(m.name == "sum_numbers" and "640220" in m.content for m in agent_messages)
