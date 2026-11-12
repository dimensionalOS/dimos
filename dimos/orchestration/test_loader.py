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

from pathlib import Path

import pytest

from dimos.orchestration.definition import OrchestratorDefinition
from dimos.orchestration.loader import LoadError, load_and_check

PATROL = """\
root = "Demo"
resources = ["base"]

[constants]
route = ["lobby", "lab", "kitchen"]
SUCCEEDED = 3

[vars]
"patrol.index" = 0

[[signal]]
name = "estop_button"
stream = "FakeEstop.pressed"
field = "data"

[[signal]]
name = "nav_status"
stream = "FakeBase.status"

[[event]]
name = "estop"
when = "estop_button"

[[event]]
name = "estop_cleared"
when = "!estop_button"

[[event]]
name = "arrived"
when = "nav_status.status == SUCCEEDED && nav_status.goal_id.id == nav.goal"

[[activity]]
name = "go_to"
start = "FakeBase.go_to"
cancel = "FakeBase.stop"
uses = ["base"]

[[node]]
name = "Demo"
children = ["Patrolling", "Estopped"]
initial = "Patrolling"

[[node]]
name = "Patrolling"
children = ["NextWaypoint", "Navigating"]
initial = "NextWaypoint"

[[node.transition]]
trigger = "estop"
mode = "immediate"
goto = "Estopped"

[[node]]
name = "NextWaypoint"

[[node.transition]]
if = "patrol.index < size(route)"
goto = "Navigating"

[[node.transition]]
goto = "Navigating"
set = { "patrol.index" = 0 }

[[node]]
name = "Navigating"
activity = "go_to"
args = { name = "route[patrol.index]" }
result = "nav.goal"

[[node.transition]]
trigger = "arrived"
goto = "NextWaypoint"
set = { "patrol.index" = "patrol.index + 1" }

[[node]]
name = "Estopped"

[[node.transition]]
trigger = "estop_cleared"
goto = "Patrolling"
"""

# Each row replaces one piece of the patrol file and gives the problem that must follow.
BROKEN: list[tuple[str, str, str]] = [
    ('root = "Demo"', "root =", "not valid TOML: Invalid value (at line 1, column 7)"),
    ('goto = "Estopped"\n', "", "node `Patrolling`: transition 1: `goto` is required"),
    ('stream = "FakeBase.status"\n', "", "signal `nav_status`: `stream` is required"),
    (
        'name = "Estopped"',
        'name = "Estopped"\nwind_down = "Parked"',
        "node `Estopped`: `wind_down` is not a field",
    ),
    (
        'trigger = "estop_cleared"',
        'trigger = "estop_cleared"\nafter = 5',
        "node `Estopped`: transition 1: a transition has at most one of `trigger` and `after`",
    ),
    (
        'mode = "immediate"',
        'mode = "now"',
        "node `Patrolling`: transition 1: `mode` should be 'graceful' or 'immediate'",
    ),
    (
        'initial = "NextWaypoint"',
        'initial = "Estopped"',
        "node `Patrolling`: `initial` must name one of its children",
    ),
    (
        '[vars]\n"patrol.index" = 0',
        "[vars]\npatrol.index = 0",
        '`[vars]` holds it as a field of the map `patrol`. Write the key in quotes there: `"patrol.index"`',
    ),
]


def test_the_patrol_file_loads_to_the_patrol_definition(
    tmp_path: Path, patrol: OrchestratorDefinition
) -> None:
    path = tmp_path / "patrol.toml"
    path.write_text(PATROL)
    assert load_and_check(path) == patrol


@pytest.mark.parametrize(("old", "new", "problem"), BROKEN)
def test_broken_files(tmp_path: Path, old: str, new: str, problem: str) -> None:
    assert PATROL.count(old) == 1
    path = tmp_path / "patrol.toml"
    path.write_text(PATROL.replace(old, new))
    with pytest.raises(LoadError) as raised:
        load_and_check(path)
    assert any(problem in found for found in raised.value.problems), raised.value.problems
