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

from pydantic import BaseModel
import pytest

from dimos.benchmark.evaluation.models import EvaluationReport, InlineNativeResult
from dimos.benchmark.evaluation.protocol import EvaluationContext
import dimos.benchmark.evaluation.registry as registry


class Config(BaseModel):
    value: int


class PluginEvaluation:
    name = "sample"
    config_model: type[BaseModel] = Config

    def run(self, config: BaseModel, context: EvaluationContext) -> EvaluationReport:
        return EvaluationReport(native_result=InlineNativeResult(value=None))


class Distribution:
    metadata = {"Name": "Acme_Evals"}
    version = "2.0"


class EntryPoint:
    name = "sample"
    value = "acme.evals:sample"
    dist = Distribution()

    def __init__(self, target) -> None:
        self.target = target

    def load(self):
        return self.target


def test_external_evaluation_uses_distribution_namespace(monkeypatch) -> None:
    entry = EntryPoint(PluginEvaluation())
    monkeypatch.setattr(
        registry.importlib_metadata,
        "entry_points",
        lambda **_kwargs: [entry],
    )

    resolved = registry.resolve_evaluation("acme-evals.sample")

    assert resolved.provider == "Acme_Evals"
    assert resolved.version == "2.0"
    assert resolved.evaluation is entry.target


def test_unknown_evaluation_lists_available_names(monkeypatch) -> None:
    monkeypatch.setattr(
        registry.importlib_metadata,
        "entry_points",
        lambda **_kwargs: [],
    )

    with pytest.raises(registry.EvaluationRegistryError, match="frozen-integer-qa"):
        registry.resolve_evaluation("missing")


def test_external_target_must_implement_whole_evaluation(monkeypatch) -> None:
    entry = EntryPoint(object())
    monkeypatch.setattr(
        registry.importlib_metadata,
        "entry_points",
        lambda **_kwargs: [entry],
    )

    with pytest.raises(registry.EvaluationRegistryError, match="name, config_model, and run"):
        registry.resolve_evaluation("acme-evals.sample")


def test_duplicate_external_evaluation_names_are_rejected(monkeypatch) -> None:
    monkeypatch.setattr(
        registry.importlib_metadata,
        "entry_points",
        lambda **_kwargs: [EntryPoint(PluginEvaluation()), EntryPoint(PluginEvaluation())],
    )

    with pytest.raises(registry.EvaluationRegistryError, match="Multiple installed"):
        registry.available_evaluations()
