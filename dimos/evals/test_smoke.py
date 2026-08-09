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

"""Live passive smoke: real model, real LFS recordings. Self-hosted + API key."""

from __future__ import annotations

import pytest

pytestmark = [pytest.mark.self_hosted, pytest.mark.skipif_no_openai]


def test_passive_smoke(tmp_path) -> None:  # type: ignore[no-untyped-def]
    from dimos.evals.runner import EvalRunner, summarize
    from dimos.evals.suites.examples import SUITE
    from dimos.utils.data import get_data

    get_data("go2_short.db")

    runner = EvalRunner(model="gpt-4o-mini", out_dir=tmp_path / "evals")
    results = runner.run(SUITE)

    assert not any(r.error for r in results), [r.error for r in results]
    s = summarize(results)
    # The lidar-points case reads a number embedded in the str() encoding and
    # the image case is unambiguous — a competent VLM should clear both.
    assert s.mean_score >= 0.5
