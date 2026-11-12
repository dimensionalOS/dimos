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

"""The patrol file fits the planner it drives."""

from pathlib import Path

from dimos_lcm.actionlib_msgs import GoalStatus
from dimos_lcm.std_msgs import Time

from dimos.core.introspection.module.info import extract_rpc_info
from dimos.navigation.global_planner.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.orchestration.binding import Deployed, bind
from dimos.orchestration.loader import load_and_check

PATROL = Path(__file__).parent / "go2_sim_patrol.toml"


def test_the_patrol_file_fits_the_planner() -> None:
    deployed = Deployed(
        modules=frozenset({"MLSPlannerNative"}),
        rpcs={
            f"MLSPlannerNative.{name}": extract_rpc_info(fn)
            for name, fn in MLSPlannerNative.rpcs.items()
        },
        streams={"MLSPlannerNative.nav_status": GoalStatus},
    )

    assert bind(load_and_check(PATROL), deployed).problems == ()


def test_go_to_returns_a_stamp_the_file_can_compare() -> None:
    # A return type that does not resolve has no shape, and the stamp comparison goes unchecked.
    assert extract_rpc_info(MLSPlannerNative.go_to).return_annotation is Time
