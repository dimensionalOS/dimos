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

"""Integration test for the planner eval. Drives the default suite on xArm6.

Skips when Drake or the xArm catalog entry is not available, so it's safe
to leave on in environments without the full simulation stack.
"""

from __future__ import annotations

import importlib.util

import pytest


def _drake_available() -> bool:
    return importlib.util.find_spec("pydrake") is not None


@pytest.mark.skipif(not _drake_available(), reason="Drake not installed")
def test_xarm6_default_suite_passes():
    """All default scenarios plan successfully on xArm6 with default tolerances."""
    from dimos.manipulation.eval import evaluate, default_scenarios
    from dimos.robot.catalog.ufactory import xarm6

    try:
        arm = xarm6(adapter_type="mock")
    except Exception as exc:
        pytest.skip(f"xArm6 catalog not available: {exc}")

    scores = evaluate(arm, default_scenarios())

    failures = [(s.name, s.reason) for s in scores if not s.passed]
    assert not failures, f"failed scenarios: {failures}"

    # Every passed scenario should have populated the standard metric keys.
    for s in scores:
        if s.passed:
            assert "planning_time_s" in s.metrics
            assert "position_error_m" in s.metrics
            assert "path_length_rad" in s.metrics
            assert "n_waypoints" in s.metrics
