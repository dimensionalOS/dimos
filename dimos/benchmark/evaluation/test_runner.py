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
    CodePolicyAgentConfig,
    EvaluationReport,
    InlineNativeResult,
    RuntimeIdentity,
    SummaryItem,
)
from dimos.benchmark.evaluation.protocol import AgentOutcome, EvaluationContext
from dimos.benchmark.evaluation.registry import ResolvedEvaluation
import dimos.benchmark.evaluation.runner as runner


class HarnessConfig(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, strict=True)
    samples: tuple[int, ...]


class Environment(BaseModel):
    sample: int


class NativeHarnessEvaluation:
    name = "native-harness"
    config_model: type[BaseModel] = HarnessConfig

    def run(self, config: BaseModel, context: EvaluationContext) -> EvaluationReport:
        assert isinstance(config, HarnessConfig)
        predictions = []
        for sample in config.samples:
            with context.agent.open_session(Environment(sample=sample)) as session:
                outcome = session.run(
                    evaluation_protocol="Return the native benchmark answer.",
                    task_input=str(sample),
                )
            predictions.append(int(outcome.final_text))
        native = {
            "benchmark": "fixture",
            "predictions": predictions,
            "aggregate": {"sum": sum(predictions)},
        }
        return EvaluationReport(
            summary=(SummaryItem(key="native_sum", label="Native sum", value=sum(predictions)),),
            native_result=InlineNativeResult(value=native),
        )


class FailingEvaluation:
    name = "native-harness"
    config_model: type[BaseModel] = HarnessConfig

    def run(self, config: BaseModel, context: EvaluationContext) -> EvaluationReport:
        raise RuntimeError("credential=secret")


class FakeSession:
    def __init__(self, sample: int) -> None:
        self.sample = sample

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return None

    def run(self, *, evaluation_protocol: str, task_input: str) -> AgentOutcome:
        assert evaluation_protocol == "Return the native benchmark answer."
        assert task_input == str(self.sample)
        return AgentOutcome(str(self.sample * 2), 1, 0.01)


class FakeRuntime:
    def __init__(self, *, config: CodePolicyAgentConfig, **_kwargs) -> None:
        self.config = config
        self.prompt_evidence = ()
        self.runtime_artifacts = ()

    @property
    def identity(self) -> RuntimeIdentity:
        return RuntimeIdentity(
            driver_version="test",
            model=self.config.model,
            thinking_level=self.config.thinking_level,
        )

    def open_session(self, environment: BaseModel) -> FakeSession:
        assert isinstance(environment, Environment)
        return FakeSession(environment.sample)


def _write_spec(tmp_path: Path) -> Path:
    path = tmp_path / "spec.json"
    path.write_text(
        json.dumps(
            {
                "schema_version": "1.0",
                "evaluation": {
                    "name": "native-harness",
                    "config": {"samples": [2, 3]},
                },
                "agent": {"profile": "code-policy-v1"},
            }
        )
    )
    return path


def test_native_harness_owns_loop_scoring_and_aggregation(monkeypatch, tmp_path: Path) -> None:
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
    assert result.report.native_result.value == {
        "benchmark": "fixture",
        "predictions": [4, 6],
        "aggregate": {"sum": 10},
    }
    assert json.loads((output / "run.json").read_text())["status"] == "completed"


def test_nonempty_output_is_rejected_before_execution(tmp_path: Path) -> None:
    output = tmp_path / "output"
    output.mkdir()
    (output / "keep").write_text("user data")

    with pytest.raises(FileExistsError, match="absent or an empty"):
        runner.execute_evaluation(_write_spec(tmp_path), output=output)

    assert (output / "keep").read_text() == "user data"


def test_started_failure_is_published_with_credentials_redacted(
    monkeypatch,
    tmp_path: Path,
) -> None:
    monkeypatch.setenv("OPENAI_API_KEY", "secret")
    monkeypatch.setattr(runner, "CodePolicyRuntimeFactory", FakeRuntime)
    monkeypatch.setattr(
        runner,
        "resolve_evaluation",
        lambda _name: ResolvedEvaluation(
            name="native-harness",
            provider="fixture",
            version="1",
            evaluation=FailingEvaluation(),
        ),
    )
    output = tmp_path / "output"

    result = runner.execute_evaluation(_write_spec(tmp_path), output=output)

    assert result.status == "failed"
    assert result.error is not None
    assert result.error.message == "credential=[REDACTED]"
    assert "secret" not in (output / "run.json").read_text()
