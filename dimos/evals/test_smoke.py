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

"""Live smoke: real model, real LFS recordings. Self-hosted + API key.

Runs only the ``numeric`` cases (odom + pointcloud str encodings) — the
self-hosted ros-dev container has no libturbojpeg, so image-encoding cases
are exercised locally via ``dimos evals run dimos.evals.suites.examples``.
"""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest

pytestmark = [pytest.mark.self_hosted, pytest.mark.skipif_no_openai]


def test_question_answer_smoke(tmp_path: Path) -> None:
    from dimos.evals.agents.question_answer import QuestionAnswer
    from dimos.evals.runner import EvalRunner, summarize
    from dimos.evals.suites.go2_smoke import SUITE
    from dimos.utils.data import get_data

    get_data("go2_short.db")

    runner = EvalRunner(out_dir=tmp_path / "evals")
    results = runner.run(
        SUITE, replace(QuestionAnswer(), model="gpt-4o-mini"), tags=frozenset({"numeric"})
    )

    assert not any(r.error for r in results), [r.error for r in results]
    s = summarize(results)
    # The lidar-points case reads a number embedded in the str() encoding —
    # a competent model clears it outright; displacement earns graded credit.
    assert s.mean_score >= 0.5
