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

"""Guarantees the task registry owes the loader that will read it."""

import dataclasses

import pytest

from dimos.benchmark.space_qa.sampling import DEFAULT_GROUP_SIZE, sample_group_rows
from dimos.benchmark.space_qa.tasks import SPACE_TEXT_TASKS, SpaceTextTask, space_text_task

PAPER_GROUP_COUNTS = {
    "PTT_text": 100,
    "JLO_text": 50,
    "MPFB_text": 50,
    "MRT_text": 40,
    "SAtt_text": 100,
    "SAdd_text": 50,
    "CBTT_text": 50,
    "DirectionEstimationBEVText": 150,
    "DistanceEstimationBEVText": 135,
    "MapSketchingBEVText": 30,
}


def test_every_text_task_is_registered_once_with_its_paper_group_count() -> None:
    assert {task.name: task.groups for task in SPACE_TEXT_TASKS} == PAPER_GROUP_COUNTS
    assert len(SPACE_TEXT_TASKS) == len(PAPER_GROUP_COUNTS)


def test_a_task_names_its_own_question_file() -> None:
    for task in SPACE_TEXT_TASKS:
        assert task.qas_relative_path == f"{task.name}/qas.json"


def test_expected_rows_is_what_the_sampler_will_be_handed() -> None:
    for task in SPACE_TEXT_TASKS:
        assert task.group_size == DEFAULT_GROUP_SIZE
        assert task.expected_rows == task.groups * DEFAULT_GROUP_SIZE
        rows = sample_group_rows(task.expected_rows, groups=1, seed=0)
        assert len(rows) == DEFAULT_GROUP_SIZE


def test_the_registry_and_its_entries_are_immutable() -> None:
    assert isinstance(SPACE_TEXT_TASKS, tuple)
    with pytest.raises(dataclasses.FrozenInstanceError):
        SPACE_TEXT_TASKS[0].groups = 1


def test_lookup_finds_a_registered_task_and_refuses_anything_else() -> None:
    assert space_text_task("MRT_text").groups == 40
    with pytest.raises(KeyError, match="unregistered task"):
        space_text_task("MRT")


@pytest.mark.parametrize(("name", "groups", "group_size"), [("", 1, 4), ("x", 0, 4), ("x", 1, 0)])
def test_an_unusable_registration_fails_at_construction(
    name: str, groups: int, group_size: int
) -> None:
    with pytest.raises(ValueError, match="invalid task registration"):
        SpaceTextTask(name=name, groups=groups, group_size=group_size)
