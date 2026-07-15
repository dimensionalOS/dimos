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

"""Fast, no-robot correctness test for the mapping-benchmark metric port.

Normal pytest collection (no `tool_`/`test_` exclusion applies to this file --
it's a real `test_*.py`), unlike tool_benchmark.py. Asserts that
compute_run_metrics(), ported from trial/scripts/report.py + bench.py,
reproduces each testdata.py fixture's known-correct numbers exactly. Metrics
math only -- no dimos runtime, no transport, no robot.
"""

from __future__ import annotations

from typing import Any

import pytest

from dimos.mapping.benchmark.testdata import FixtureCase, testcases
from dimos.mapping.benchmark.tool_benchmark import compute_run_metrics
from dimos.mapping.benchmark.type import RunMetrics


@pytest.mark.parametrize("case", testcases, ids=[c.name for c in testcases])
def test_fixture_reproduces_expected_metrics(case: FixtureCase) -> None:
    metrics = compute_run_metrics(
        case.records,
        run_id=case.name,
        route=case.route,
        mode=case.mode,
        duration_s=case.duration_s,
    )

    assert metrics.route == case.route
    assert metrics.mode == case.mode
    assert metrics.duration_s == round(case.duration_s, 1)

    for field_name, expected_value in case.expected.items():
        actual_value: Any = getattr(metrics, field_name)
        assert actual_value == expected_value, (
            f"{case.name}: {field_name} = {actual_value!r}, expected {expected_value!r}"
        )


def test_holdout_fields_default_to_none() -> None:
    """The extension point tool_benchmark.py marks: not computed yet, must
    stay nullable rather than silently defaulting to a misleading 0."""
    case = testcases[0]
    metrics: RunMetrics = compute_run_metrics(
        case.records,
        run_id=case.name,
        route=case.route,
        mode=case.mode,
        duration_s=case.duration_s,
    )
    assert metrics.holdout_tag_id is None
    assert metrics.holdout_error_m is None
