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

"""Which recordings the eval ships a question set for, discovered from the tree.

``reference/`` holds one directory per recording -- ``reference/<dataset>/`` with
``refs.jsonl``, ``manifest.json``, ``review.json``, ``questions.jsonl`` and
``crops/`` inside it -- and the directory name *is* the dataset name: the one an
operator passes to ``teacher.py``/``ingest.py`` as ``--dataset``, and the one
``ingest.py`` derives its chroma collection name from. Keeping those three
spellings the same is what lets a store, a shard directory and a question set be
matched up without a lookup table.

A recording is part of the eval exactly when its directory holds a
``questions.jsonl``: that is the only file the sweep reads, so a half-finished
dataset directory (refs and a review, no question set yet) is discovered as
"not shipping questions" rather than as a broken dataset. Discovery is by glob
rather than by a committed list because the list would be a second place to
forget: adding a recording is adding a directory, and every test that iterates
:func:`dataset_names` picks it up without an edit.

The order is sorted, not filesystem order, so a parametrized sweep enumerates
its cases identically on every machine -- pytest's case ids, and therefore the
shard directories named after them, would otherwise depend on directory
iteration order.
"""

from __future__ import annotations

from pathlib import Path

from dimos.agents.evals.contracts import QuestionSpec

#: The committed reference tree: one sub-directory per recording.
REFERENCE_DIR = Path(__file__).parent / "reference"

#: The file whose presence makes a directory a dataset of this eval.
QUESTIONS_NAME = "questions.jsonl"


def dataset_names(reference_dir: Path = REFERENCE_DIR) -> list[str]:
    """Every dataset *reference_dir* ships a question set for, sorted by name."""
    return sorted(path.parent.name for path in reference_dir.glob(f"*/{QUESTIONS_NAME}"))


def dataset_dir(dataset: str, reference_dir: Path = REFERENCE_DIR) -> Path:
    """The committed artifact directory of *dataset*."""
    return reference_dir / dataset


def load_question_set(dataset: str, reference_dir: Path = REFERENCE_DIR) -> list[QuestionSpec]:
    """Read *dataset*'s committed question set, in file order.

    File order is ``questions.py``'s sort by ``question_id``, and the sweep
    writes its shard in the order it reads them, so a shard's line order is
    reproducible from the artifact alone.
    """
    path = dataset_dir(dataset, reference_dir) / QUESTIONS_NAME
    return [QuestionSpec.from_json(line) for line in path.read_text().splitlines() if line.strip()]
