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

import pytest


@pytest.mark.skipif_in_ci
@pytest.mark.skipif_no_openai
@pytest.mark.slow
def test_dimos_skills(lcm_spy, start_blueprint, human_input) -> None:
    lcm_spy.save_topic("/agent")
    lcm_spy.save_topic("/rpc/Agent/on_system_modules/res")
    lcm_spy.save_topic("/rpc/DemoCalculatorSkill/sum_numbers/req")
    lcm_spy.save_topic("/rpc/DemoCalculatorSkill/sum_numbers/res")

    start_blueprint("run", "demo-skill")

    lcm_spy.wait_for_saved_topic("/rpc/Agent/on_system_modules/res")

    human_input("what is 52983 + 587237")

    lcm_spy.wait_for_saved_topic_content("/agent", b"640220")

    assert "/rpc/DemoCalculatorSkill/sum_numbers/req" in lcm_spy.messages
    assert "/rpc/DemoCalculatorSkill/sum_numbers/res" in lcm_spy.messages
