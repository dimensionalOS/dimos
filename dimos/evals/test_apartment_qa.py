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

import pytest

from dimos.evals.scorers import numeric
from dimos.evals.suites.dimsim_apartment_qa import SUITE
from dimos.evals.types import AgentInfo, FinalMetrics, Outcome, RunExtra, Step, Trajectory


def _outcome(answer: str) -> Outcome:
    return Outcome(
        trajectory=Trajectory(
            agent=AgentInfo(name="test", version="1", model_name="test"),
            steps=(Step(step_id=1, timestamp="", source="agent", message=answer),),
            final_metrics=FinalMetrics(
                total_prompt_tokens=0,
                total_completion_tokens=0,
                total_cached_tokens=0,
                total_cost_usd=0,
                total_steps=1,
            ),
            extra=RunExtra(ended_by="answer"),
        ),
        artifacts={},
    )


@pytest.mark.parametrize(
    "value,score",
    [(349, 0), (350, 0), (370, 0.5), (390, 1), (400, 1), (410, 1), (430, 0.5), (450, 0), (451, 0)],
)
def test_numeric_credit_bands(value: int, score: float) -> None:
    assert numeric(400, value, tolerance=10, band=50) == score


@pytest.mark.parametrize("value", [float("nan"), float("inf"), float("-inf")])
def test_nonfinite_numeric_values(value: float) -> None:
    assert numeric(4, value, tolerance=0.1, band=1) == 0


@pytest.mark.parametrize(
    "answer,score",
    [
        ("4", 1),
        ("4.0", 1),
        ("There are 4 chairs.", 1),
        ("3", 0),
        ("4.5", 0),
        ("-4", 0),
        ("unknown", 0),
    ],
)
def test_count_answers(answer: str, score: float) -> None:
    case = next(c for c in SUITE if c.id.endswith("dining_chair_count"))
    assert case.grade(_outcome(answer)) == score


@pytest.mark.parametrize(
    "answer,score",
    [("yes", 1), (" YES ", 1), ("Yes, there is.", 1), ("no", 0), ("unknown", 0), ("true", 0)],
)
def test_boolean_answers(answer: str, score: float) -> None:
    case = next(c for c in SUITE if c.id.endswith("bathtub_exists"))
    assert case.grade(_outcome(answer)) == score


@pytest.mark.parametrize("value,score", [(1.7, 1), (1.9, 1), (1.4, 0), (2.2, 0)])
def test_decimal_boundaries(value: float, score: float) -> None:
    assert numeric(1.8, value, tolerance=0.1, band=0.4) == score


@pytest.mark.parametrize("tolerance,band", [(-1, 2), (1, 1), (2, 1), (0, float("inf"))])
def test_invalid_bands(tolerance: float, band: float) -> None:
    with pytest.raises(ValueError):
        numeric(1, 1, tolerance=tolerance, band=band)


def test_tiny_band_retains_partial_credit() -> None:
    assert numeric(0, 2e-15, tolerance=1e-15, band=3e-15) == pytest.approx(0.5)


@pytest.mark.parametrize(
    "answer,score",
    [("1.80", 1), ("About 1.8 meters", 1), ("1.55", 0.5), ("-1.8", 0), ("unknown", 0)],
)
def test_measurement_answers(answer: str, score: float) -> None:
    case = next(c for c in SUITE if c.id.endswith("refrigerator_height"))
    assert case.grade(_outcome(answer)) == pytest.approx(score)


@pytest.mark.parametrize(
    "answer,score",
    [
        ("2.2, 1.1", 1),
        ("1.1, 2.2", 1),
        ("3.0, 1.1", 0.5),
        ("2.2", 0),
        ("true, 1.1", 0),
        ("2.2, -1.1", 0),
        ("2.2, 1.1, 0.5", 0),
        ("2.2, nan", 0),
        ("inf, 1.1", 0),
    ],
)
def test_table_dimension_scoring(answer: str, score: float) -> None:
    case = next(c for c in SUITE if c.id.endswith("dining_table_dimensions"))
    assert case.grade(_outcome(answer)) == pytest.approx(score)


def test_suite_contract() -> None:
    assert len(SUITE) == 21
    assert len({c.id for c in SUITE}) == 21
    assert len({id(c.environment) for c in SUITE}) == 21
    for case in SUITE:
        assert {"dimsim", "apartment", "qa"} <= case.tags
        assert case.timeout_s == 1200
        assert case.threshold == 1
        assert case.grade(_outcome("invalid answer")) == 0
        assert case.environment.config.scene == "apartment"
        assert "JSON" not in case.inputs
