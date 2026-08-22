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

"""Bounded CI consumption of the product-facing PimSim InteractiveEval suite."""

from pathlib import Path

from pimsim_dimos.evaluation.canonical_suite import SUITE
import pytest

from dimos.evals.runner import EvalRunner
from dimos.evals.types import InteractiveEval

pytestmark = [pytest.mark.self_hosted_large, pytest.mark.mujoco]


@pytest.mark.parametrize("case", SUITE, ids=lambda case: case.id)
def test_pimsim_canonical_interactive_eval(case: InteractiveEval, tmp_path: Path) -> None:
    result = EvalRunner(out_dir=tmp_path, launch_timeout_s=1200.0).run((case,))[0]

    assert not result.error, result.error
    assert result.passed, result
