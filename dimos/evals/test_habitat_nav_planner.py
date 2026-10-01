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


def test_a_case_with_no_world_state_is_an_error_not_a_miss() -> None:
    import pytest

    from dimos.evals.suites.habitat_nav import world_state_check

    world_state_check({})  # no bridge stats: nothing to say
    world_state_check({"ticks": 120, "errors": 3, "last_error": "x"})  # a few bad ticks are fine
    with pytest.raises(RuntimeError, match="no world state was published"):
        world_state_check(
            {"ticks": 0, "errors": 1200, "last_error": "IndexError: list index out of range"}
        )
    with pytest.raises(RuntimeError, match="0 ticks"):
        world_state_check(
            {"ticks": 0, "errors": 0, "last_error": ""}
        )  # no odometry ever reached the bridge
