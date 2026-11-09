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

from contextlib import ExitStack

from dimos.core.global_config import global_config
from dimos.core.transport_factory import make_transport
from dimos.messages.geometry_msgs.msg import PoseStamped, Twist
from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_coordinator import r1pro_control


def test_r1_blueprint_chassis_pins_match_default_factory(monkeypatch):
    blueprint = r1pro_control()
    # The existing R1 control subtree selects Zenoh for both adapter and driver.
    assert blueprint.global_config_overrides["transport"] == "zenoh"
    monkeypatch.setattr(global_config, "transport", "zenoh")
    with ExitStack() as cleanup:
        for name, kind in [("cmd_vel", Twist), ("odom", PoseStamped)]:
            pin = blueprint.transport_map[("chassis_" + name, kind)].build()
            default = make_transport("/chassis/" + name, kind)
            cleanup.callback(pin.stop)
            cleanup.callback(default.stop)
            assert pin.channel == default.channel
            assert type(pin) is type(default)
