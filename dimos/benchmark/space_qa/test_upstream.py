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

"""Claims about upstream that only real SPACE can settle.

Gated twice over. The `self_hosted` marker keeps these out of the default run,
and the environment check keeps them from cloning a repository or downloading
3.6 GB on a runner that was never set up for them. Nothing here calls a model:

    DIMOS_SPACE_SOURCE=~/src/ml-space-benchmark \\
    DIMOS_SPACE_DATA=~/src/space-benchmark-data/SPACE_data_release \\
    pytest dimos/benchmark/space_qa/test_upstream.py -m self_hosted
"""

import os
import sys

import pytest

from dimos.benchmark.space_qa.data import SPACE_DATA_ENV, load_task_rows, resolve_space_data
from dimos.benchmark.space_qa.source import SPACE_SOURCE_ENV, ensure_space_source
from dimos.benchmark.space_qa.suite import SpaceQAAdapter, parse_official_answer
from dimos.benchmark.space_qa.tasks import SPACE_TEXT_TASKS, space_text_task

TASK = space_text_task("SAtt_text")

pytestmark = [
    pytest.mark.self_hosted,
    pytest.mark.skipif(
        not (os.environ.get(SPACE_SOURCE_ENV) and os.environ.get(SPACE_DATA_ENV)),
        reason=f"set {SPACE_SOURCE_ENV} and {SPACE_DATA_ENV} to run against real SPACE",
    ),
]


@pytest.fixture(scope="module")
def space_on_path() -> None:
    source = str(ensure_space_source())
    if source not in sys.path:
        sys.path.insert(0, source)


def test_the_agent_inherits_the_upstream_parser_rather_than_reimplementing_it(
    space_on_path,
) -> None:
    """The half of SPACE that decides whether an answer is right stays untouched."""
    from space.agents.qa_agent import QA_Agent

    from dimos.benchmark.space_qa.agent import DimosQAAgent

    assert issubclass(DimosQAAgent, QA_Agent)
    assert DimosQAAgent.parse_answer_from_response is QA_Agent.parse_answer_from_response
    assert DimosQAAgent.postprocess_response is QA_Agent.postprocess_response
    assert DimosQAAgent.preprocess_question is QA_Agent.preprocess_question
    assert DimosQAAgent.reset is QA_Agent.reset


def test_the_agent_and_its_config_reach_the_upstream_registries(space_on_path) -> None:
    from space import get_config
    from space.registry import AGENTS_REGISTRY

    from dimos.benchmark.space_qa.agent import CONFIG_NAME, DimosQAAgent

    config = get_config(CONFIG_NAME)

    assert AGENTS_REGISTRY[config["agent_name"]] is DimosQAAgent
    # evaluate_qas reads these two itself, and skips per-question directories
    # (which is where every record on this side lands) without save_dir.
    assert config["use_vllm"] is False
    assert "save_dir" in config
    # QA_Agent branches on these prefixes when it builds the dialog.
    assert not config["model_name"].startswith(("claude", "mistralai"))


def test_this_side_grades_a_reply_the_way_the_official_scorer_does(space_on_path) -> None:
    assert parse_official_answer('{"answer": 3}') == 3
    assert parse_official_answer('Reasoning...\n```json\n{"answer": 2}\n```') == 2
    assert parse_official_answer("The third choice.") is None


@pytest.mark.parametrize("task", SPACE_TEXT_TASKS, ids=lambda task: task.name)
def test_every_registered_task_still_has_the_row_count_it_is_registered_with(task) -> None:
    """The registry is what turns a row index into a question; it has to match the release."""
    assert len(load_task_rows(task, resolve_space_data())) == task.expected_rows


def test_a_real_subset_becomes_cases_that_ask_the_release_questions_verbatim() -> None:
    from dimos.benchmark.space_qa.adapter import SubsetSpec

    adapter = SpaceQAAdapter.from_data_root(TASK, resolve_space_data())
    items = adapter.iter_items(SubsetSpec(seed=20260808, groups=2))
    rows = adapter.upstream_rows(items)

    assert len(items) == 2 * TASK.group_size
    for item, row in zip(items, rows, strict=True):
        assert adapter.to_case(item).task.prompt == row["question"]
