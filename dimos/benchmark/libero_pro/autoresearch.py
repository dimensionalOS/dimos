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

"""Frozen multi-case measurement harness for LIBERO-PRO autoresearch."""

from __future__ import annotations

import argparse
from collections.abc import Callable, Sequence
from pathlib import Path, PurePosixPath
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

from dimos.benchmark.evaluation.models import EvaluationRun, InlineNativeResult
from dimos.benchmark.evaluation.runner import execute_evaluation


class ResearchModel(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, strict=True)


class PanelCase(ResearchModel):
    id: str = Field(min_length=1, pattern=r"^[a-z][a-z0-9-]*$")
    family: Literal["goal", "spatial", "object", "libero_10"]
    specification: str = Field(min_length=1)

    @model_validator(mode="after")
    def specification_is_safe(self) -> PanelCase:
        path = PurePosixPath(self.specification)
        if path.is_absolute() or ".." in path.parts:
            raise ValueError("case specification must be a safe relative path")
        return self


class ResearchPanel(ResearchModel):
    schema_version: Literal["1.0"] = "1.0"
    name: str = Field(min_length=1)
    cases: tuple[PanelCase, ...]

    @model_validator(mode="after")
    def cases_are_representative(self) -> ResearchPanel:
        if len(self.cases) != 4:
            raise ValueError("research panels require exactly four cases")
        ids = [case.id for case in self.cases]
        if len(set(ids)) != len(ids):
            raise ValueError("research panel case ids must be unique")
        families = [case.family for case in self.cases]
        if set(families) != {"goal", "spatial", "object", "libero_10"}:
            raise ValueError("research panels require one case from each suite family")
        return self


class PanelCaseResult(ResearchModel):
    id: str
    family: str
    output: str
    status: Literal["completed", "failed", "cancelled", "preflight_error", "invalid_result"]
    success: bool
    native_score: float
    error: str | None = None


class PanelScore(ResearchModel):
    schema_version: Literal["1.0"] = "1.0"
    panel: str
    native_successes: int = Field(ge=0)
    attempts: int = Field(gt=0)
    success_rate: float = Field(ge=0.0, le=1.0)
    infrastructure_or_policy_failures: int = Field(ge=0)
    cases: tuple[PanelCaseResult, ...]


EvaluationExecutor = Callable[..., EvaluationRun]


def run_panel(
    panel_path: Path,
    *,
    output: Path,
    api_key_env: str = "OPENAI_API_KEY",
    executor: EvaluationExecutor = execute_evaluation,
) -> PanelScore:
    """Run every frozen case and aggregate only its native result."""
    panel_path = panel_path.expanduser().resolve()
    output = output.expanduser().resolve()
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        raise FileExistsError(f"Output must be absent or an empty directory: {output}")
    output.mkdir(parents=True, exist_ok=True)
    panel = ResearchPanel.model_validate_json(panel_path.read_bytes())
    results = tuple(
        _run_case(
            case,
            panel_dir=panel_path.parent,
            output=output,
            api_key_env=api_key_env,
            executor=executor,
        )
        for case in panel.cases
    )
    successes = sum(result.success for result in results)
    failures = sum(result.status != "completed" for result in results)
    score = PanelScore(
        panel=panel.name,
        native_successes=successes,
        attempts=len(results),
        success_rate=successes / len(results),
        infrastructure_or_policy_failures=failures,
        cases=results,
    )
    (output / "panel-score.json").write_text(
        score.model_dump_json(indent=2) + "\n",
        encoding="utf-8",
    )
    return score


def _run_case(
    case: PanelCase,
    *,
    panel_dir: Path,
    output: Path,
    api_key_env: str,
    executor: EvaluationExecutor,
) -> PanelCaseResult:
    case_output = output / case.id
    try:
        run = executor(
            panel_dir / case.specification,
            output=case_output,
            api_key_env=api_key_env,
            progress=None,
        )
    except Exception as exc:
        return PanelCaseResult(
            id=case.id,
            family=case.family,
            output=case_output.name,
            status="preflight_error",
            success=False,
            native_score=0.0,
            error=f"{type(exc).__name__}: {exc}",
        )
    if run.status != "completed" or run.report is None:
        return PanelCaseResult(
            id=case.id,
            family=case.family,
            output=case_output.name,
            status=run.status,
            success=False,
            native_score=0.0,
            error=run.error.message if run.error is not None else None,
        )
    native = run.report.native_result
    if not isinstance(native, InlineNativeResult) or not isinstance(native.value, dict):
        return _invalid_result(case, case_output, "native result is not an inline mapping")
    value: dict[str, Any] = native.value
    if not isinstance(value.get("success"), bool) or not isinstance(
        value.get("score"), (int, float)
    ):
        return _invalid_result(case, case_output, "native result lacks success or score")
    return PanelCaseResult(
        id=case.id,
        family=case.family,
        output=case_output.name,
        status="completed",
        success=value["success"],
        native_score=float(value["score"]),
    )


def _invalid_result(case: PanelCase, output: Path, error: str) -> PanelCaseResult:
    return PanelCaseResult(
        id=case.id,
        family=case.family,
        output=output.name,
        status="invalid_result",
        success=False,
        native_score=0.0,
        error=error,
    )


def main(argv: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--panel", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--api-key-env", default="OPENAI_API_KEY")
    parser.add_argument("--json", action="store_true", dest="json_output")
    args = parser.parse_args(argv)
    result = run_panel(args.panel, output=args.output, api_key_env=args.api_key_env)
    if args.json_output:
        print(result.model_dump_json())
    else:
        print(
            f"{result.panel}: {result.native_successes}/{result.attempts} "
            f"({result.success_rate:.1%})"
        )


if __name__ == "__main__":
    main()
