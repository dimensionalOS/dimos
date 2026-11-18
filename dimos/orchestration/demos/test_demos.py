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

from dimos.core.introspection.module.info import extract_rpc_info
from dimos.msgs.sensor_msgs.BatteryState import BatteryState
from dimos.msgs.std_msgs.Bool import Bool
from dimos.orchestration.binding import Deployed, bind
from dimos.orchestration.demos.blueprints import HERE
from dimos.orchestration.demos.toy_robot import ToyRobot
from dimos.orchestration.loader import load_and_check

DEPLOYED = Deployed(
    modules=frozenset({"ToyRobot"}),
    rpcs={f"ToyRobot.{name}": extract_rpc_info(fn) for name, fn in ToyRobot.rpcs.items()},
    streams={
        "ToyRobot.battery": BatteryState,
        "ToyRobot.obstacle": Bool,
        "ToyRobot.estop": Bool,
        "ToyRobot.gripper_ok": Bool,
    },
)


@pytest.mark.parametrize(
    "name", ["delivery", "inspection", "shift", "safety", "jam", "patrol", "warehouse"]
)
def test_each_demo_fits_the_toy_robot(name: str) -> None:
    assert bind(load_and_check(HERE / f"{name}.toml"), DEPLOYED).problems == ()
