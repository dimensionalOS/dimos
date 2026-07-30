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

"""Dataset discovery, and the shape of what it discovers.

Discovery decides which cases the ``self_hosted`` sweep even has, so it is
worth a test that does not need a recording: the sweep's own lane cannot check
it, because a machine without an ingested store skips before it would notice
that a dataset went missing.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from dimos.agents.evals.reference_sets import (
    QUESTIONS_NAME,
    dataset_dir,
    dataset_names,
    load_question_set,
)

#: What a dataset directory has to hold. ``crops/`` is deliberately absent: the
#: review images are presentation, some questions ship without them (see
#: ``reference/go2_short/README.md``), and the sweep reads none of them.
DATASET_ARTIFACTS = ("refs.jsonl", "manifest.json", "review.json", QUESTIONS_NAME)


def test_a_dataset_is_a_directory_that_ships_a_question_set(tmp_path: Path) -> None:
    """Discovery is the glob, sorted -- and a directory without questions is not one.

    The half-finished case is the one that matters: a directory holding a
    reference table and a review but no question set yet is *work in progress*,
    not a dataset the sweep should try to run and fail on.
    """
    for name in ("zulu", "alpha"):
        (tmp_path / name).mkdir()
        (tmp_path / name / QUESTIONS_NAME).write_text("")
    (tmp_path / "mid-review").mkdir()
    (tmp_path / "mid-review" / "refs.jsonl").write_text("")

    assert dataset_names(tmp_path) == ["alpha", "zulu"]
    assert dataset_names(tmp_path / "nothing-here") == []
    assert dataset_dir("alpha", tmp_path) == tmp_path / "alpha"


def test_the_committed_tree_ships_at_least_one_dataset() -> None:
    """An empty reference tree would make the sweep collect zero cases and pass."""
    assert dataset_names()


@pytest.mark.parametrize("dataset", dataset_names())
def test_every_committed_dataset_ships_the_whole_artifact_set(dataset: str) -> None:
    """Questions, and the evidence they rest on, travel together.

    A question set committed without the reference table and review overlay it
    came from would still run the sweep and still produce numbers -- it would
    just be unauditable, and ``test_questions`` could no longer re-derive it.
    """
    directory = dataset_dir(dataset)
    for artifact in DATASET_ARTIFACTS:
        path = directory / artifact
        assert path.is_file(), f"{dataset!r} ships no {artifact}"
        assert path.stat().st_size > 0, f"{dataset!r} ships an empty {artifact}"
    assert load_question_set(dataset), f"{dataset!r} ships an empty question set"
