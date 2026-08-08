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

"""What an adapter owes the execution path, checked on the generated suite."""

import json
import re

from pydantic import ValidationError
import pytest

from dimos.benchmark.agent_eval.models import EvalCase
from dimos.benchmark.space_qa.adapter import (
    BenchmarkAdapter,
    BenchmarkItem,
    ItemScore,
    SubsetSpec,
)
from dimos.benchmark.space_qa.sampling import DEFAULT_GROUP_SIZE
from dimos.benchmark.space_qa.synthetic import (
    DEFAULT_TOTAL_GROUPS,
    SyntheticGridAdapter,
    parse_json_answer,
)

CASE_ID = re.compile(r"^synthetic-grid-\d{5}-[0-9a-f]{8}$")
SLOTS = range(1, DEFAULT_GROUP_SIZE + 1)


def _one_group(adapter: SyntheticGridAdapter) -> tuple[BenchmarkItem, ...]:
    return tuple(adapter.iter_items(SubsetSpec(seed=3, groups=1)))


def _correct_slot(adapter: SyntheticGridAdapter, item: BenchmarkItem) -> int:
    """Recover the answer the only way a caller can: by asking the adapter to grade one."""
    accepted = [slot for slot in SLOTS if adapter.score(item, json.dumps({"answer": slot})).correct]
    assert len(accepted) == 1
    return accepted[0]


def _options(question: str) -> list[str]:
    return [line.split(". ", 1)[1] for line in question.splitlines() if re.match(r"^\d+\. ", line)]


def test_the_adapter_satisfies_the_protocol_at_runtime() -> None:
    assert isinstance(SyntheticGridAdapter(), BenchmarkAdapter)


def test_a_subset_comes_back_as_whole_groups_of_consecutive_rows() -> None:
    items = SyntheticGridAdapter(total_groups=8).iter_items(SubsetSpec(seed=1, groups=3))
    ordinals = [item.ordinal for item in items]
    assert len(ordinals) == 3 * DEFAULT_GROUP_SIZE
    for start in range(0, len(ordinals), DEFAULT_GROUP_SIZE):
        group = ordinals[start : start + DEFAULT_GROUP_SIZE]
        assert group == [group[0] + offset for offset in range(DEFAULT_GROUP_SIZE)]


def test_a_group_asks_one_stimulus_once_per_answer_slot() -> None:
    adapter = SyntheticGridAdapter()
    group = _one_group(adapter)
    assert len(group) == DEFAULT_GROUP_SIZE
    stimuli = {item.question.split("\n\n")[0] for item in group}
    assert len(stimuli) == 1
    assert {frozenset(_options(item.question)) for item in group} == {
        frozenset(_options(group[0].question))
    }
    assert len({item.question for item in group}) == DEFAULT_GROUP_SIZE
    assert {_correct_slot(adapter, item) for item in group} == set(SLOTS)


def test_a_case_carries_the_question_and_nothing_else() -> None:
    adapter = SyntheticGridAdapter()
    item = _one_group(adapter)[0]
    assert adapter.to_case(item).model_dump(mode="json") == {
        "schema_version": "1.0",
        "case_id": f"synthetic-grid-{item.ordinal:05d}-{item.question_sha256[:8]}",
        "source": {"schema_version": "1.0", "kind": "none"},
        "task": {"schema_version": "1.0", "kind": "verbatim_prompt", "prompt": item.question},
        "validator": {
            "schema_version": "1.0",
            "kind": "external_evaluator",
            "benchmark": "synthetic-grid",
            "revision": "synthetic-v1",
        },
    }


def test_a_case_round_trips_through_the_shipped_contract() -> None:
    adapter = SyntheticGridAdapter()
    for item in _one_group(adapter):
        case = adapter.to_case(item)
        assert EvalCase.model_validate(case.model_dump(mode="json")) == case
        assert CASE_ID.match(case.case_id)


def test_case_ids_separate_rows_that_share_a_stimulus() -> None:
    adapter = SyntheticGridAdapter()
    group = _one_group(adapter)
    assert len({adapter.to_case(item).case_id for item in group}) == DEFAULT_GROUP_SIZE


def test_the_answer_key_never_reaches_an_item() -> None:
    assert set(BenchmarkItem.model_fields) == {"ordinal", "question"}


def test_cases_from_one_group_differ_only_in_the_question_they_ask() -> None:
    """Nothing outside the prompt records where the right option sits."""
    adapter = SyntheticGridAdapter()
    shapes = set()
    for item in _one_group(adapter):
        dumped = adapter.to_case(item).model_dump(mode="json")
        del dumped["case_id"], dumped["task"]
        shapes.add(json.dumps(dumped, sort_keys=True))
    assert len(shapes) == 1


def test_the_answer_slot_is_not_a_function_of_the_ordinal() -> None:
    """The ordinal is public (it sits in the case_id), so it must not encode the answer."""
    adapter = SyntheticGridAdapter()
    items = sorted(
        adapter.iter_items(SubsetSpec(seed=1, groups=DEFAULT_TOTAL_GROUPS)),
        key=lambda item: item.ordinal,
    )
    slots = [_correct_slot(adapter, item) for item in items]
    assert slots != [item.ordinal % DEFAULT_GROUP_SIZE + 1 for item in items]


def test_the_same_seed_regenerates_the_same_suite() -> None:
    subset = SubsetSpec(seed=9, groups=2)
    first = SyntheticGridAdapter(seed=4).iter_items(subset)
    assert first == tuple(SyntheticGridAdapter(seed=4).iter_items(subset))
    assert first != tuple(SyntheticGridAdapter(seed=5).iter_items(subset))


def test_a_reply_is_graded_against_the_key_the_adapter_kept() -> None:
    adapter = SyntheticGridAdapter()
    item = _one_group(adapter)[0]
    correct = _correct_slot(adapter, item)
    wrong = next(slot for slot in SLOTS if slot != correct)
    assert adapter.score(item, f'{{"answer": {correct}}}') == ItemScore(
        ordinal=item.ordinal, parsed_answer=correct, correct=True
    )
    assert adapter.score(item, f'{{"answer": {wrong}}}') == ItemScore(
        ordinal=item.ordinal, parsed_answer=wrong, correct=False
    )


@pytest.mark.parametrize(
    ("reply", "expected"),
    [
        ('{"answer": 2}', 2),
        ('```json\n{"answer": 2}\n```', 2),
        ('I checked each cell.\n{"answer": 2}\nHappy to explain further.', 2),
        ('{"answer": "2"}', 2),
        ('{"answer": 1}\nOn reflection: {"answer": 2}', 2),
        ('{"reasoning": "the K sits mid-grid", "answer": 2}', 2),
        ("The second option.", None),
        ("", None),
        ('{"answer": true}', None),
        ('{"choice": 2}', None),
        ('{"answer": [2]}', None),
        ('Reply with {"answer": <option number>}.', None),
    ],
)
def test_the_reply_parser_reads_the_shapes_a_model_actually_produces(
    reply: str, expected: int | None
) -> None:
    assert parse_json_answer(reply) == expected


def test_an_unparsed_reply_is_reported_as_ungraded() -> None:
    adapter = SyntheticGridAdapter()
    item = _one_group(adapter)[0]
    score = adapter.score(item, "The second option.")
    assert score == ItemScore(ordinal=item.ordinal)
    assert score.parsed_answer is None
    assert score.correct is None


def test_a_score_cannot_claim_a_verdict_it_never_parsed() -> None:
    with pytest.raises(ValidationError, match="must stay ungraded"):
        ItemScore(ordinal=0, correct=True)


def test_an_item_from_elsewhere_is_refused_rather_than_graded() -> None:
    adapter = SyntheticGridAdapter(total_groups=1)
    stranger = BenchmarkItem(ordinal=99, question="Which cell holds the letter Z?")
    with pytest.raises(KeyError, match="was not issued by synthetic-grid"):
        adapter.score(stranger, '{"answer": 1}')


def test_a_suite_with_no_questions_is_refused() -> None:
    with pytest.raises(ValueError, match="total_groups must be at least 1"):
        SyntheticGridAdapter(total_groups=0)
