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

import json
from unittest.mock import Mock

from microduck_world.control import WorldControl


def test_manual_respawn_stops_navigation_and_returns_human_control(module_factory, monkeypatch):
    control = module_factory(WorldControl, default_mode="agent")
    navigation = Mock()
    monkeypatch.setattr(control, "_navigation", navigation, raising=False)
    mode = Mock()
    reset = Mock()
    velocity = Mock()
    monkeypatch.setattr(control.mode, "publish", mode)
    monkeypatch.setattr(control.respawn_request, "publish", reset)
    monkeypatch.setattr(control.cmd_vel, "publish", velocity)

    control._on_ui_command('{"name":"respawn","args":{}}')

    navigation.cancel_goal.assert_called_once_with()
    assert json.loads(mode.call_args.args[0])["mode"] == "teleop"
    assert velocity.call_args.args[0].linear.x == 0
    reset.assert_called_once_with(True)


def test_regular_policy_commands_still_use_the_stock_router(module_factory, monkeypatch):
    control = module_factory(WorldControl)
    policy = Mock()
    monkeypatch.setattr(control.policy_request, "publish", policy)
    control._on_ui_command('{"name":"policy","args":{"policy":"walk","action":"start"}}')
    request = json.loads(policy.call_args.args[0])
    assert (request["policy"], request["action"]) == ("walk", "start")


def test_ball_drop_passes_bridge_validation_without_changing_mode(module_factory, monkeypatch):
    from microduck_world.relay import WorldCommand

    control = module_factory(WorldControl)
    drop = Mock()
    mode = Mock()
    monkeypatch.setattr(control.ball_drop_request, "publish", drop)
    monkeypatch.setattr(control.mode, "publish", mode)
    raw = WorldCommand(name="drop_ball", args={"ball": "football_ball_1"}).model_dump_json()
    control._on_ui_command(raw)
    drop.assert_called_once_with("football_ball_1")
    mode.assert_not_called()


def test_ball_drop_rejects_unknown_ball(module_factory, monkeypatch):
    control = module_factory(WorldControl)
    drop = Mock()
    monkeypatch.setattr(control.ball_drop_request, "publish", drop)
    control._on_ui_command('{"name":"drop_ball","args":{"ball":"unknown"}}')
    drop.assert_not_called()
