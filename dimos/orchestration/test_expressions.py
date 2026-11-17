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

import sys

import pytest

# TODO: remove this skip once the pin is cel-expr-python 0.1.4, which supports Python 3.10.
if sys.version_info < (3, 11):
    pytest.skip("cel-expr-python 0.1.3 needs Python 3.11", allow_module_level=True)

from dimos.orchestration.expressions import (
    UNKNOWN,
    Compiler,
    ExpressionError,
    Plain,
    Wanted,
    evaluate,
    evaluate_bool,
)

STAMP: Plain = {"sec": 0, "nsec": 0}
SHAPES = {
    "estop_button": False,
    "battery": UNKNOWN,
    "battery_pct": 0.0,
    "low_battery": 20.0,
    "patrol.index": 0,
    "route": ["lobby"],
    "dock": {"x": 0.0, "y": 0.0},
    "nav_status": {"status": 0, "text": "", "goal_id": {"stamp": STAMP, "id": ""}},
    "nav.goal": STAMP,
    "untyped_result": UNKNOWN,
    "untyped_flag": UNKNOWN,
}
BOARD: dict[str, Plain] = {
    "estop_button": True,
    "low_battery": 20.0,
    "patrol.index": 1,
    "route": ["lobby", "lab", "kitchen"],
    "dock": {"x": 1.0, "y": 2.0},
    "nav_status": {"status": 3, "text": "", "goal_id": {"stamp": {"sec": 5, "nsec": 7}, "id": "a"}},
    "nav.goal": {"sec": 5, "nsec": 7},
    "untyped_result": {"anything": 4},
    "untyped_flag": True,
}
UNSET = object()
ERROR = object()

REJECTED: list[tuple[str, Wanted, str]] = [
    ("typo_key < 3", UNKNOWN, "undeclared reference to 'typo_key'"),
    ("nav_status.goalid == 1", UNKNOWN, "cannot be the operand of a select"),
    ("battery_pct < 20", UNKNOWN, r"'_<_' applied to '\(double, int\)'"),
    ("patrol.index + 1", True, "gives INT, and BOOL is needed"),
]

EVALUATED: list[tuple[str, Wanted, dict[str, object], object]] = [
    ("battery_pct < 20.0", True, {}, False),
    ("battery_pct < 20.0 || estop_button", True, {}, True),
    ("battery_pct < low_battery", True, {"battery_pct": 19.5}, True),
    ("nav_status.goal_id.stamp == nav.goal", True, {}, True),
    ("nav_status.goal_id.stamp == nav.goal", True, {"nav_status": {"status": 3}}, ERROR),
    (
        "estop_button || nav_status.goal_id.stamp == nav.goal",
        True,
        {"nav_status": {"status": 3}},
        True,
    ),
    ("dock", {"x": 0.0, "y": 0.0}, {}, {"x": 1.0, "y": 2.0}),
    ("route[patrol.index]", "", {}, "lab"),
    ("untyped_result.anything + 1 == 5", True, {}, True),
    ("untyped_flag", True, {"untyped_flag": "false"}, ERROR),
    ("route[5]", UNKNOWN, {}, ERROR),
    ("low_battery > 1.0 && route[5] == 'lab'", True, {}, ERROR),
]


@pytest.fixture(scope="module")
def compiler() -> Compiler:
    return Compiler(SHAPES)


def test_compile_gives_the_keys_read(compiler: Compiler) -> None:
    source = "nav_status.goal_id.stamp.sec == nav.goal.sec && battery_pct < low_battery"
    expression = compiler.compile(source, True)
    assert set(expression.reads) == {
        "nav_status.goal_id.stamp.sec",
        "nav.goal.sec",
        "battery_pct",
        "low_battery",
    }
    assert evaluate_bool(expression, {**BOARD, "battery_pct": 19.5}) is True


@pytest.mark.parametrize(("source", "gives", "reason"), REJECTED)
def test_compile_rejects(compiler: Compiler, source: str, gives: Wanted, reason: str) -> None:
    with pytest.raises(ExpressionError, match=reason):
        compiler.compile(source, gives)


@pytest.mark.parametrize(("source", "gives", "changes", "expected"), EVALUATED)
def test_evaluate(
    compiler: Compiler, source: str, gives: Wanted, changes: dict[str, object], expected: object
) -> None:
    board = {key: value for key, value in {**BOARD, **changes}.items() if value is not UNSET}
    expression = compiler.compile(source, gives)
    run = evaluate_bool if gives is True else evaluate
    if expected is ERROR:
        with pytest.raises(ExpressionError):
            run(expression, board)
    else:
        assert run(expression, board) == expected
