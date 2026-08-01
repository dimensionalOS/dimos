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

import pytest
from rich.console import Console

from .scheduler_display import operational_json, operational_projection, render_status
from .scheduler_models import OperationalCount, OperationalFailure, OperationalSnapshot


def _snapshot() -> OperationalSnapshot:
    return OperationalSnapshot(
        experiment_id="exp-1",
        workers=4,
        jobs=8,
        active=1,
        observation="reconciled",
        counts=OperationalCount(
            pending=2, running=1, succeeded=3, failed=1, interrupted=1, cancelled=0
        ),
        failures=(OperationalFailure(job_id="job-7", state="failed", reason="executor_failed"),),
    )


def test_projection_is_exact_typed_schema_and_json_safe() -> None:
    snapshot = _snapshot()
    projected = operational_projection(snapshot)
    assert projected == {
        "record_type": "pi-operational-snapshot",
        "schema_version": "1.0",
        "experiment_id": "exp-1",
        "workers": 4,
        "jobs": 8,
        "active": 1,
        "observation": "reconciled",
        "states": {
            "pending": 2,
            "running": 1,
            "succeeded": 3,
            "failed": 1,
            "interrupted": 1,
            "cancelled": 0,
        },
        "failures": [{"job_id": "job-7", "state": "failed", "reason": "executor_failed"}],
    }
    assert json.loads(operational_json(snapshot)) == projected
    assert operational_json(snapshot) == operational_json(snapshot)


def test_rich_and_json_use_the_same_snapshot_content() -> None:
    console = Console(record=True, width=100)
    console.print(render_status(_snapshot()))
    output = console.export_text()
    assert all(state.title() in output for state in ("pending", "running", "succeeded"))
    assert all(state.title() in output for state in ("failed", "interrupted", "cancelled"))
    assert "executor_failed" in output
    assert "elapsed" not in output.lower()
    assert "score" not in output.lower()


def test_legacy_dict_input_is_rejected() -> None:
    with pytest.raises(TypeError):
        operational_projection({"experiment_id": "forged"})  # type: ignore[arg-type]
    with pytest.raises(TypeError):
        render_status({"states": {}})  # type: ignore[arg-type]
