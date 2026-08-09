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

"""What the SPACE adapter owes the execution path, checked on invented rows.

Rows are built here rather than read from the release: the shape is the
contract under test, and none of Apple's data belongs in this repository.
"""

import json
from typing import Any

import pytest

from dimos.benchmark.agent_eval.models import EvalCase
from dimos.benchmark.space_qa.adapter import BenchmarkAdapter, BenchmarkItem, SubsetSpec
from dimos.benchmark.space_qa.sampling import DEFAULT_GROUP_SIZE
from dimos.benchmark.space_qa.source import SPACE_REVISION
from dimos.benchmark.space_qa.suite import SpaceQAAdapter
from dimos.benchmark.space_qa.tasks import SpaceTextTask

TASK = SpaceTextTask(name="FAKE_text", groups=6)


def _rows(count: int = TASK.expected_rows) -> list[dict[str, Any]]:
    return [
        {
            "question": f"Where is marker {ordinal}?",
            "answer": ordinal % DEFAULT_GROUP_SIZE + 1,
            "task": "FAKE",
        }
        for ordinal in range(count)
    ]


def _adapter() -> SpaceQAAdapter:
    return SpaceQAAdapter(TASK, _rows())


def test_the_adapter_satisfies_the_protocol_at_runtime() -> None:
    assert isinstance(_adapter(), BenchmarkAdapter)


def test_the_suite_names_the_task_it_scores_and_the_scorer_it_pins() -> None:
    adapter = _adapter()

    assert adapter.name == f"space-{TASK.name}"
    assert adapter.revision == SPACE_REVISION


def test_a_subset_comes_back_as_whole_groups_of_consecutive_rows() -> None:
    ordinals = [item.ordinal for item in _adapter().iter_items(SubsetSpec(seed=7, groups=2))]

    assert len(ordinals) == 2 * DEFAULT_GROUP_SIZE
    for start in range(0, len(ordinals), DEFAULT_GROUP_SIZE):
        group = ordinals[start : start + DEFAULT_GROUP_SIZE]
        assert group == [group[0] + offset for offset in range(DEFAULT_GROUP_SIZE)]


def test_an_item_carries_the_upstream_question_verbatim() -> None:
    rows = _rows()
    adapter = SpaceQAAdapter(TASK, rows)

    for item in adapter.iter_items(SubsetSpec(seed=7, groups=2)):
        assert item.question == rows[item.ordinal]["question"]


def test_the_same_seed_draws_the_same_rows() -> None:
    subset = SubsetSpec(seed=20260808, groups=3)

    assert _adapter().iter_items(subset) == _adapter().iter_items(subset)
    assert _adapter().iter_items(subset) != _adapter().iter_items(SubsetSpec(seed=1, groups=3))


def test_a_case_asks_the_question_with_no_environment_and_no_local_verdict() -> None:
    adapter = _adapter()
    item = adapter.iter_items(SubsetSpec(seed=7, groups=1))[0]

    assert adapter.to_case(item).model_dump(mode="json") == {
        "schema_version": "1.0",
        "case_id": f"space-{TASK.name}-{item.ordinal:05d}-{item.question_sha256[:8]}",
        "source": {"schema_version": "1.0", "kind": "none"},
        "task": {"schema_version": "1.0", "kind": "verbatim_prompt", "prompt": item.question},
        "validator": {
            "schema_version": "1.0",
            "kind": "external_evaluator",
            "benchmark": f"space-{TASK.name}",
            "revision": SPACE_REVISION,
        },
    }


def test_a_case_round_trips_through_the_shipped_contract() -> None:
    adapter = _adapter()

    for item in adapter.iter_items(SubsetSpec(seed=7, groups=2)):
        case = adapter.to_case(item)
        assert EvalCase.model_validate(case.model_dump(mode="json")) == case


def test_cases_differ_only_in_the_question_they_ask() -> None:
    """The answer key stays with the adapter: nothing outside the prompt varies with it."""
    adapter = _adapter()
    shapes = set()
    for item in adapter.iter_items(SubsetSpec(seed=7, groups=3)):
        dumped = adapter.to_case(item).model_dump(mode="json")
        del dumped["case_id"], dumped["task"]
        shapes.add(json.dumps(dumped, sort_keys=True))

    assert len(shapes) == 1


def test_the_subset_file_keeps_the_upstream_rows_whole() -> None:
    """SPACE reads this file itself, answer key and stale metadata included."""
    rows = _rows()
    adapter = SpaceQAAdapter(TASK, rows)
    items = adapter.iter_items(SubsetSpec(seed=7, groups=2))

    assert adapter.upstream_rows(items) == [rows[item.ordinal] for item in items]


def test_unregistered_row_count_is_refused() -> None:
    with pytest.raises(ValueError, match=f"expects {TASK.expected_rows} rows"):
        SpaceQAAdapter(TASK, _rows(TASK.expected_rows - 1))


def test_an_item_from_elsewhere_is_refused_rather_than_answered() -> None:
    adapter = _adapter()
    stranger = BenchmarkItem(ordinal=TASK.expected_rows, question="Where is marker 999?")

    with pytest.raises(KeyError, match=f"was not issued by space-{TASK.name}"):
        adapter.upstream_rows([stranger])
