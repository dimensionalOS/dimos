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

"""Px4SkillContainer against a scripted connection: what the agent is told, and when.

One contract test ties the script to the real connection and the real supervisor status.
"""

from __future__ import annotations

from collections.abc import Iterator
from typing import Any

import pytest

from dimos.robot.px4.connection import Px4DroneConnection
from dimos.robot.px4.connection_spec import Px4DroneConnectionSpec
from dimos.robot.px4.skill_container import Px4SkillContainer, _settled
from dimos.robot.px4.supervisor_core import GOTO_ARRIVED
from dimos.robot.px4.test_supervisor_operator import Flight
from dimos.spec.utils import spec_annotation_compliance


def _status(state: str, reason: str = "", **fields: Any) -> dict[str, Any]:
    return {"state": state, "reason": reason, **fields}


class ScriptedConnection:
    """Accepts or refuses like Px4DroneConnection, then plays back supervisor statuses."""

    def __init__(self, statuses: list[dict[str, Any]], rejection: str | None = None) -> None:
        self.statuses = statuses
        self.rejection = rejection
        self.calls: list[tuple[Any, ...]] = []

    def _result(self, *call: Any) -> dict[str, Any]:
        self.calls.append(call)
        return {
            "accepted": self.rejection is None,
            "rejection": self.rejection,
            "state": self.statuses[0]["state"],
        }

    def takeoff(self, altitude_m: float | None = None) -> dict[str, Any]:
        return self._result("takeoff", altitude_m)

    def go_to(
        self,
        north_m: float = 0.0,
        east_m: float = 0.0,
        altitude_m: float | None = None,
        heading_deg: float | None = None,
        relative: bool = True,
    ) -> dict[str, Any]:
        return self._result("go_to", north_m, east_m, altitude_m, heading_deg, relative)

    def land(self) -> dict[str, Any]:
        return self._result("land")

    def set_guidance_mode(self, mode: str) -> dict[str, Any]:
        return self._result("set_guidance_mode", mode)

    def status(self) -> dict[str, Any]:
        """The next scripted status; the last one repeats."""
        return self.statuses.pop(0) if len(self.statuses) > 1 else self.statuses[0]


@pytest.fixture
def skills() -> Iterator[Px4SkillContainer]:
    s = Px4SkillContainer(poll_hz=500.0, wait_timeout_s=0.2)
    yield s
    s.stop()


def _fly(
    skills: Px4SkillContainer, statuses: list[dict[str, Any]], rejection: str | None = None
) -> ScriptedConnection:
    connection = ScriptedConnection(statuses, rejection)
    skills._connection = connection
    return connection


def test_takeoff_waits_for_the_hover_and_says_where(skills: Px4SkillContainer) -> None:
    connection = _fly(
        skills,
        [
            _status("PREFLIGHT"),
            _status("TAKEOFF", alt_m=0.8, north_m=0.0, east_m=0.0),
            _status("HOVER", "at 2.0 m", alt_m=1.96, north_m=0.02, east_m=-0.01),
        ],
    )
    assert skills.takeoff(2.0) == (
        "Took off. State HOVER: 2.0 m up, +0.0 m north and -0.0 m east of the takeoff point."
    )
    assert connection.calls == [("takeoff", 2.0)]


def test_a_refused_command_is_reported_without_waiting(skills: Px4SkillContainer) -> None:
    connection = _fly(skills, [_status("IDLE"), _status("IDLE")], rejection="estop_latched")
    assert skills.takeoff(2.0) == "Refused: estop_latched (state IDLE)."
    assert skills.go_to(north_m=-2.0) == "Refused: estop_latched (state IDLE)."
    assert skills.land() == "Refused: estop_latched (state IDLE)."
    assert skills.set_guidance_mode("TELEOP") == "Refused: estop_latched (state IDLE)."
    assert len(connection.statuses) == 2  # never polled: a poll would have consumed one


def test_takeoff_that_falls_back_to_idle_reports_the_reason(skills: Px4SkillContainer) -> None:
    _fly(skills, [_status("PREFLIGHT"), _status("IDLE", "preflight failed: enable switch off")])
    assert skills.takeoff() == "Takeoff failed: preflight failed: enable switch off. State IDLE."


def test_go_to_passes_the_goal_and_reports_arrival(skills: Px4SkillContainer) -> None:
    connection = _fly(
        skills,
        [
            _status("GOTO", "go-to: 2.0 m to go"),
            _status("HOVER", GOTO_ARRIVED, alt_m=3.0, north_m=-2.0, east_m=0.0),
        ],
    )
    assert skills.go_to(north_m=-2.0, altitude_m=3.0) == (
        "Arrived. State HOVER: 3.0 m up, -2.0 m north and +0.0 m east of the takeoff point."
    )
    assert connection.calls == [("go_to", -2.0, 0.0, 3.0, None, True)]


def test_arrival_is_reported_once_the_hover_has_closed_on_its_setpoint(
    skills: Px4SkillContainer,
) -> None:
    goal = {"kind": "pos", "n": -2.0, "e": 0.0, "d": -3.0}
    _fly(
        skills,
        [
            _status("GOTO"),
            # Arrived by the supervisor's tolerance, still 0.4 m short and closing.
            _status(
                "HOVER",
                GOTO_ARRIVED,
                setpoint=goal,
                local={"n": -1.6, "e": 0.0, "d": -2.9},
                alt_m=2.9,
                north_m=-1.6,
                east_m=0.0,
            ),
            _status(
                "HOVER",
                GOTO_ARRIVED,
                setpoint=goal,
                local={"n": -1.95, "e": 0.0, "d": -3.0},
                alt_m=3.0,
                north_m=-1.95,
                east_m=0.0,
            ),
        ],
    )
    assert skills.go_to(north_m=-2.0, altitude_m=3.0).startswith(
        "Arrived. State HOVER: 3.0 m up, -1.9 m north"
    )


def test_a_takeover_while_the_hover_settles_is_not_a_plain_success(
    skills: Px4SkillContainer,
) -> None:
    # Inside the supervisor's tolerance, 0.4 m short of the setpoint and closing.
    closing: dict[str, Any] = {
        "setpoint": {"kind": "pos", "n": -2.0, "e": 0.0, "d": -3.0},
        "local": {"n": -1.6, "e": 0.0, "d": -2.9},
    }
    arrived = _status("HOVER", GOTO_ARRIVED, **closing)
    _fly(
        skills,
        [_status("GOTO"), arrived, arrived, _status("IDLE", "PILOT_OVERRIDE (mode now POSCTL)")],
    )
    assert skills.go_to(north_m=-2.0, altitude_m=3.0) == (
        "Arrived, then interrupted: PILOT_OVERRIDE (mode now POSCTL). State IDLE."
    )
    hover = _status("HOVER", "at 3.0 m", **closing)
    _fly(skills, [_status("TAKEOFF"), hover, hover, _status("LANDING", "ESTOP land")])
    assert skills.takeoff(3.0) == "Took off, then interrupted: ESTOP land. State LANDING."


def test_go_to_cut_short_by_the_pilot_is_not_an_arrival(skills: Px4SkillContainer) -> None:
    _fly(skills, [_status("GOTO"), _status("IDLE", "PILOT_OVERRIDE (mode now POSCTL)")])
    assert skills.go_to(north_m=5.0) == (
        "Did not arrive: PILOT_OVERRIDE (mode now POSCTL). State IDLE."
    )


def test_a_wait_that_times_out_reports_the_flight_as_it_is(skills: Px4SkillContainer) -> None:
    _fly(skills, [_status("GOTO", "go-to: 3.0 m to go", alt_m=2.0, north_m=1.0, east_m=0.0)])
    assert skills.go_to(north_m=4.0).startswith("Did not arrive: go-to: 3.0 m to go. State GOTO")


def test_a_takeoff_or_landing_that_outlasts_the_wait_is_still_running(
    skills: Px4SkillContainer,
) -> None:
    _fly(skills, [_status("TAKEOFF", "armed, climbing")])
    assert skills.takeoff(2.0) == "Still taking off: armed, climbing. State TAKEOFF."
    _fly(skills, [_status("LANDING", "operator land")])
    assert skills.land() == "Still landing: operator land. State LANDING."


def test_land_waits_for_the_ground(skills: Px4SkillContainer) -> None:
    _fly(skills, [_status("LANDING"), _status("IDLE", "landed and disarmed")])
    assert skills.land() == "Landing ended: landed and disarmed. State IDLE."


def test_guidance_mode_is_passed_through(skills: Px4SkillContainer) -> None:
    connection = _fly(skills, [_status("TELEOP")])
    assert skills.set_guidance_mode("teleop") == "Guidance mode TELEOP, state TELEOP."
    assert connection.calls == [("set_guidance_mode", "teleop")]


def test_flight_status_without_a_battery_reading(skills: Px4SkillContainer) -> None:
    for batt_pct in (None, -1):  # no SYS_STATUS yet; PX4 does not know
        _fly(skills, [_status("IDLE", armed=False, px4_mode="POSCTL", batt_pct=batt_pct)])
        assert skills.flight_status() == (
            "State IDLE. Reason: . Armed: False, PX4 mode POSCTL, battery unknown."
        )


def test_the_skills_read_the_real_connection_and_status(skills: Px4SkillContainer) -> None:
    assert spec_annotation_compliance(Px4DroneConnection, Px4DroneConnectionSpec)
    assert spec_annotation_compliance(ScriptedConnection, Px4DroneConnectionSpec)
    f = Flight()
    assert f.core.takeoff_cmd(f.snap(), f.now, alt_m=2.0) is None
    f.run_until("HOVER", 30.0)
    assert not _settled(f.core.status(f.snap(), f.now))  # inside hover_tolerance_m, closing
    f.run(5.0)
    status = f.core.status(f.snap(), f.now)
    assert _settled(status)
    _fly(skills, [status])
    assert skills.flight_status() == (
        "State HOVER: 2.0 m up, +0.0 m north and +0.0 m east of the takeoff point. "
        "Reason: at 2.0 m. Armed: True, PX4 mode OFFBOARD, battery 80%."
    )
