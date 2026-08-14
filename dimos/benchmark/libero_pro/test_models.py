from pathlib import Path

import pytest

from dimos.benchmark.libero_pro.models import LiberoTaskManifest

CASE = Path(__file__).parent / "cases" / "goal-task-0-single-trial" / "task.json"


def test_smoke_manifest_selects_same_task_with_fresh_rows() -> None:
    manifest = LiberoTaskManifest.model_validate_json(CASE.read_bytes())

    assert manifest.task.instruction == "open the bottom drawer of the cabinet"
    assert manifest.episodes.debug_init_state_indices == (1, 2, 3, 4, 5)
    assert manifest.episodes.scored_init_state_index == 0
    assert manifest.contract.horizon_ticks == 300


def test_manifest_rejects_scored_row_used_for_debugging() -> None:
    payload = LiberoTaskManifest.model_validate_json(CASE.read_bytes()).model_dump()
    payload["episodes"]["scored_init_state_index"] = 1

    with pytest.raises(ValueError, match="must not be used for debugging"):
        LiberoTaskManifest.model_validate(payload)
