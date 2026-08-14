# Copyright 2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

import json
from pathlib import Path

import pytest

from dimos.benchmark.vlnce_r2r.models import VlnceTaskManifest
from dimos.benchmark.vlnce_r2r.native_result import validate_native_result


def test_native_result_preserves_official_score_and_checks_attempt() -> None:
    case = VlnceTaskManifest.model_validate_json(
        (Path(__file__).parent / "cases/mp3d-example-episode-515/task.json").read_bytes()
    )
    result = {
        "schema_version": "vlnce-result.v1",
        "attempt_id": "attempt-1",
        "case_id": case.case_id,
        "terminal_reason": "submitted",
        "duration_seconds": 12.0,
        "trajectory": {"sha256": "a" * 64, "points": 4},
        "metrics": {
            "DISTANCE_TO_GOAL": 0.8,
            "SUCCESS": 1.0,
            "SPL": 0.7,
            "NDTW": 0.9,
            "PATH_LENGTH": 4.2,
            "ORACLE_SUCCESS": 1.0,
            "STEPS_TAKEN": 42.0,
        },
        "runtime": {"habitat_sim": "0.1.7"},
    }
    payload = json.dumps(result).encode()

    assert validate_native_result(payload, case=case, attempt_id="attempt-1").metrics.SUCCESS == 1
    with pytest.raises(ValueError, match="attempt_id"):
        validate_native_result(payload, case=case, attempt_id="attempt-2")
