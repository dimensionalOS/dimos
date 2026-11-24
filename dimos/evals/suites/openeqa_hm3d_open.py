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

"""OpenEQA HM3D open-phrase cases over preprocessed episode stores.

Media lives outside the git tree. Prepare with::

    python dimos-data/tools/prepare_openeqa.py --split hm3d

Then point ``OPENEQA_ROOT`` at the output (default
``~/dimos-data/evals/openeqa``) and run::

    dimos evals run dimos.evals.suites.openeqa_hm3d_open --agent dimos.evals.agents.dimos_agent

``SUITE`` is empty until ``cases.jsonl`` and the referenced ``stores/*.db`` exist,
so import stays safe for CI.
"""

from __future__ import annotations

import json
import os
from pathlib import Path

from dimos.evals.environments.dataset import Dataset
from dimos.evals.open_answer import JUDGE_MODEL, OPEN_ANSWER_PASS, grade_open_answer
from dimos.evals.types import EvalCase, Suite

_PHRASE = "Answer in a short phrase. There are no choices."
_DEFAULT_TIMEOUT_S = 480.0


def default_root() -> Path:
    """Prepared OpenEQA tree: ``cases.jsonl`` + ``stores/<episode>.db``."""
    override = os.environ.get("OPENEQA_ROOT", "").strip()
    if override:
        return Path(override).expanduser()
    return Path.home() / "dimos-data/evals/openeqa"


def build_suite(
    root: Path | None = None,
    *,
    limit: int | None = None,
    episode_prefix: str = "hm3d-v0/",
) -> Suite:
    """Load open-phrase cases that reference stores under *root*.

    Each ``cases.jsonl`` line needs ``id``, ``question``, ``answer``, ``source``
    (path to a ``.db``, absolute or relative to *root*), and optional ``category``.
    """
    base = default_root() if root is None else Path(root)
    cases_path = base / "cases.jsonl"
    if not cases_path.is_file():
        return []

    suite: list[EvalCase] = []
    for line in cases_path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        history = str(row.get("episode_history") or "")
        if episode_prefix and not history.startswith(episode_prefix):
            continue
        source = Path(str(row["source"]))
        recording = source if source.is_file() else (base / source)
        if not recording.is_file():
            raise FileNotFoundError(f"OpenEQA store missing for {row['id']}: {recording}")
        category = str(row.get("category") or "open")
        tags = frozenset({"open", "openeqa", "hm3d", category.replace(" ", "_")})
        suite.append(
            EvalCase(
                id=str(row["id"]),
                inputs=f"{row['question']}\n{_PHRASE}",
                environment=Dataset(
                    str(recording.resolve()),
                    select=(lambda s: s.streams.color_image,),
                ),
                grade=grade_open_answer(str(row["question"]), str(row["answer"])),
                tags=tags,
                timeout_s=float(row.get("timeout_s") or _DEFAULT_TIMEOUT_S),
                threshold=OPEN_ANSWER_PASS,
                judge_model=JUDGE_MODEL,
            )
        )
        if limit is not None and len(suite) >= limit:
            break
    return suite


# Populated when prepared data is present; otherwise empty (safe to import).
SUITE: Suite = build_suite()
