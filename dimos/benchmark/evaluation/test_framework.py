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

import json
from pathlib import Path

from pydantic import BaseModel, ConfigDict
import pytest

from dimos.benchmark.evaluation.models import (
    EvaluationReport,
    InlineNativeResult,
    RuntimeIdentity,
    SummaryItem,
)
from dimos.benchmark.evaluation.protocol import EvaluationContext
from dimos.benchmark.evaluation.registry import ResolvedEvaluation
import dimos.benchmark.evaluation.runner as runner


class HarnessConfig(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, strict=True)
    expected: int


class NativeHarnessEvaluation:
    name = "native-harness"
    config_model: type[BaseModel] = HarnessConfig

    def run(self, config: BaseModel, context: EvaluationContext) -> EvaluationReport:
        assert isinstance(config, HarnessConfig)
        assert context.runtime is not None
        return EvaluationReport(
            summary=(SummaryItem(key="score", label="Score", value=config.expected),),
            native_result=InlineNativeResult(value={"native_score": config.expected}),
        )


class FakeRuntime:
    prompt_evidence = ()
    runtime_artifacts = ()

    def __init__(self, **_kwargs) -> None:
        pass

    @property
    def identity(self) -> RuntimeIdentity:
        return RuntimeIdentity(
            driver_version="test",
            model="gpt-5.6-luna",
            thinking_level="medium",
        )


def _write_spec(tmp_path: Path) -> Path:
    path = tmp_path / "spec.json"
    path.write_text(
        json.dumps(
            {
                "schema_version": "1.0",
                "evaluation": {
                    "name": "native-harness",
                    "config": {"expected": 7},
                },
            }
        )
    )
    return path


def test_native_evaluation_owns_result_and_uses_fixed_runtime(monkeypatch, tmp_path: Path) -> None:
    monkeypatch.setenv("OPENAI_API_KEY", "secret")
    monkeypatch.setattr(runner, "CodePolicyRuntimeFactory", FakeRuntime)
    monkeypatch.setattr(
        runner,
        "resolve_evaluation",
        lambda _name: ResolvedEvaluation(
            name="native-harness",
            provider="fixture",
            version="1",
            evaluation=NativeHarnessEvaluation(),
        ),
    )
    output = tmp_path / "output"

    result = runner.execute_evaluation(_write_spec(tmp_path), output=output)

    assert result.status == "completed"
    assert result.report is not None
    assert result.report.native_result.value == {"native_score": 7}
    assert json.loads((output / "run.json").read_text())["runtime"]["driver"] == "pi"


def test_run_specification_rejects_agent_customization(tmp_path: Path) -> None:
    specification = json.loads(_write_spec(tmp_path).read_text())
    specification["agent"] = {"model": "another-model"}
    path = tmp_path / "custom.json"
    path.write_text(json.dumps(specification))

    with pytest.raises(Exception, match="Extra inputs are not permitted"):
        runner.execute_evaluation(path, output=tmp_path / "output")
