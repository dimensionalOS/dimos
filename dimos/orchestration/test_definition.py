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

from pathlib import Path

from dimos.orchestration.definition import (
    ActionActivity,
    CallActivity,
    CallStep,
    PublishStep,
    load_definition,
)

MACHINES = Path(__file__).parent / "cases" / "machines"


def test_m20_patrol_loads() -> None:
    definition = load_definition(MACHINES / "m20_patrol.toml")
    states = {state.name: state for state in definition.states}

    assert definition.machine.initial == "active"
    assert definition.vars == {"wp": 0, "tries": 0, "skips": 0, "dock_tries": 0, "why": ""}
    assert states["goto"].parent == "patrol"
    assert states["patrol"].initial == "goto"

    startup = states["startup"].do
    assert isinstance(startup, CallActivity)
    assert (startup.call, startup.timeout) == ("robot.charge_state", 5.0)

    goto = states["goto"].do
    assert isinstance(goto, ActionActivity)
    assert (goto.action, goto.timeout) == ("nav.go_to_pose", 180.0)
    assert goto.args["x"] == "params.waypoints[vars.wp].x"

    retry = states["goto"].transitions[1]
    assert (retry.event, retry.guard, retry.to) == (
        "do.failed",
        "vars.tries < params.nav_retries",
        "goto",
    )
    assert retry.set == {"tries": "vars.tries + 1"}

    assert (
        states["charging"].transitions[0].when
        == "inputs.battery.percentage >= params.resume_battery"
    )
    stop, notice = states["help"].entry
    assert isinstance(stop, CallStep) and stop.call == "robot.stop_movement"
    assert isinstance(notice, PublishStep) and notice.publish == "notice"
