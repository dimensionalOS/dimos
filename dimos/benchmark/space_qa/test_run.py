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

"""The parts of a run that decide what it means, tested without running one.

Nothing here starts SPACE, imports it, or touches the release: what is under
test is what gets written before the first worker starts and what gets checked
after the last one finishes.
"""

import json
from pathlib import Path
import subprocess
import sys
import textwrap
from typing import Any

import pytest

from dimos.benchmark.agent_eval.models import EvalCase
from dimos.benchmark.space_qa.adapter import SubsetSpec
from dimos.benchmark.space_qa.manifest import RECORD_NAME, case_dir, read_manifest, subset_path
from dimos.benchmark.space_qa.run import (
    collect_records,
    cross_check_predictions,
    locate_results,
    prepare_run,
)
from dimos.benchmark.space_qa.sampling import DEFAULT_GROUP_SIZE
from dimos.benchmark.space_qa.source import SPACE_REVISION
from dimos.benchmark.space_qa.suite import SpaceQAAdapter
from dimos.benchmark.space_qa.tasks import SpaceTextTask

TASK = SpaceTextTask(name="FAKE_text", groups=6)
SEED = 20260808
GROUPS = 2


def _rows() -> list[dict[str, Any]]:
    return [
        {
            "question": f"Where is marker {ordinal}?",
            "answer": ordinal % DEFAULT_GROUP_SIZE + 1,
            "task": "FAKE",
        }
        for ordinal in range(TASK.expected_rows)
    ]


def _prepared(run_dir: Path) -> tuple[SpaceQAAdapter, Any]:
    adapter = SpaceQAAdapter(TASK, _rows())
    items = adapter.iter_items(SubsetSpec(seed=SEED, groups=GROUPS))
    manifest = prepare_run(
        run_dir,
        adapter,
        items,
        task_name=TASK.name,
        seed=SEED,
        groups=GROUPS,
        data_sha256="deadbeef",
    )
    return adapter, manifest


def _record(manifest: Any, subset_index: int, **overrides: Any) -> dict[str, Any]:
    row = manifest.row_for(subset_index)
    return {
        "case_id": f"space-{TASK.name}-{row.ordinal:05d}-{row.question_sha256[:8]}",
        "ordinal": row.ordinal,
        "question_sha256": row.question_sha256,
        "pred": 1,
        "gt": 1,
        "space_parse_status": "parsed",
        "prediction_status": "not_evaluated",
        "infra_error": None,
        "tool_call_count": 3,
        "duration_seconds": 1.5,
        "final_text": '{"answer": 1}',
        **overrides,
    }


def _write_records(results_dir: Path, manifest: Any, **overrides: Any) -> None:
    for row in manifest.rows:
        directory = results_dir / f"qa_{row.subset_index:05d}"
        directory.mkdir(parents=True, exist_ok=True)
        (directory / RECORD_NAME).write_text(
            json.dumps(_record(manifest, row.subset_index, **overrides)), encoding="utf-8"
        )


def test_importing_the_run_path_does_not_import_space() -> None:
    """A checkout of SPACE is a run-time dependency, never an import-time one."""
    script = textwrap.dedent(
        """
        import sys
        import dimos.benchmark.space_qa.run
        import dimos.benchmark.space_qa.suite
        leaked = sorted(name for name in sys.modules if name.split('.')[0] == 'space')
        assert not leaked, leaked
        """
    )
    completed = subprocess.run(
        [sys.executable, "-c", script], capture_output=True, text=True, check=False
    )

    assert completed.returncode == 0, completed.stdout + completed.stderr


def test_the_subset_file_holds_the_drawn_rows_in_the_order_the_workers_see_them(tmp_path) -> None:
    adapter, manifest = _prepared(tmp_path)
    written = json.loads(subset_path(tmp_path).read_text(encoding="utf-8"))
    rows = _rows()

    assert written == [rows[row.ordinal] for row in manifest.rows]


def test_the_subset_file_is_ascii_because_space_opens_it_without_an_encoding(tmp_path) -> None:
    _prepared(tmp_path)

    subset_path(tmp_path).read_bytes().decode("ascii")


def test_every_drawn_question_gets_a_case_the_contract_accepts(tmp_path) -> None:
    _adapter, manifest = _prepared(tmp_path)

    assert len(manifest.rows) == GROUPS * DEFAULT_GROUP_SIZE
    for row in manifest.rows:
        case = EvalCase.model_validate_json(
            (case_dir(tmp_path, row.subset_index) / "case.json").read_bytes()
        )
        assert case.case_id.startswith(f"space-{TASK.name}-{row.ordinal:05d}")
        assert case.task.prompt == f"Where is marker {row.ordinal}?"


def test_the_manifest_records_what_a_score_would_have_to_be_reproduced_from(tmp_path) -> None:
    _adapter, manifest = _prepared(tmp_path)
    reread = read_manifest(tmp_path)

    assert reread == manifest
    assert (reread.benchmark, reread.task, reread.seed, reread.groups) == (
        f"space-{TASK.name}",
        TASK.name,
        SEED,
        GROUPS,
    )
    assert reread.space_revision == SPACE_REVISION
    assert reread.data_sha256 == "deadbeef"
    assert [row.subset_index for row in reread.rows] == list(range(GROUPS * DEFAULT_GROUP_SIZE))


def test_a_run_with_no_questions_is_refused(tmp_path) -> None:
    with pytest.raises(ValueError, match="at least one question"):
        prepare_run(
            tmp_path,
            SpaceQAAdapter(TASK, _rows()),
            [],
            task_name=TASK.name,
            seed=SEED,
            groups=GROUPS,
            data_sha256=None,
        )


def test_records_come_back_in_subset_order(tmp_path) -> None:
    _adapter, manifest = _prepared(tmp_path)
    results = tmp_path / "results"
    _write_records(results, manifest)

    records = collect_records(results, manifest)

    assert [record["ordinal"] for record in records] == [row.ordinal for row in manifest.rows]


def test_a_record_for_a_question_we_did_not_draw_fails_the_run(tmp_path) -> None:
    _adapter, manifest = _prepared(tmp_path)
    results = tmp_path / "results"
    _write_records(results, manifest, question_sha256="f" * 64)

    with pytest.raises(ValueError, match="but the manifest selected"):
        collect_records(results, manifest)


def test_a_question_that_left_no_record_fails_the_run(tmp_path) -> None:
    _adapter, manifest = _prepared(tmp_path)
    results = tmp_path / "results"
    _write_records(results, manifest)
    (results / "qa_00003" / RECORD_NAME).unlink()

    with pytest.raises(FileNotFoundError, match="left no record"):
        collect_records(results, manifest)


def test_matching_records_and_official_predictions_pass() -> None:
    records = [{"pred": 1}, {"pred": None}, {"pred": 4}]

    cross_check_predictions(records, {"all_predictions": [1, None, 4]})


def test_a_prediction_the_two_sides_disagree_on_fails_the_run() -> None:
    records = [{"pred": 1}, {"pred": 2}]

    with pytest.raises(ValueError, match="question 1 was recorded as 2 but scored as 3"):
        cross_check_predictions(records, {"all_predictions": [1, 3]})


def test_a_results_file_that_scored_a_different_number_of_questions_fails_the_run() -> None:
    with pytest.raises(ValueError, match="scored 1 questions, but 2 were recorded"):
        cross_check_predictions([{"pred": 1}, {"pred": 2}], {"all_predictions": [1]})


def test_a_results_file_without_predictions_fails_the_run() -> None:
    with pytest.raises(ValueError, match="no all_predictions list"):
        cross_check_predictions([{"pred": 1}], {"mean_metrics": {"accuracy": 100.0}})


def test_the_official_results_file_is_found_under_the_timestamp_space_chose(tmp_path) -> None:
    results = tmp_path / "dimos_qa" / "20260808_010203" / "results.json"
    results.parent.mkdir(parents=True)
    results.write_text("{}", encoding="utf-8")

    assert locate_results(tmp_path) == results


def test_a_run_that_produced_no_results_file_is_reported(tmp_path) -> None:
    with pytest.raises(FileNotFoundError, match="expected exactly one results.json"):
        locate_results(tmp_path)
