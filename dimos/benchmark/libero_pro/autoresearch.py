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
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

from dimos.benchmark.evaluation.models import EvaluationRun, InlineNativeResult
from dimos.benchmark.evaluation.runner import execute_evaluation


class ResearchModel(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, strict=True)


class PanelCase(ResearchModel):
    id: str = Field(min_length=1, pattern=r"^[a-z][a-z0-9-]*$")
    family: str = Field(min_length=1)
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
    def cases_are_valid(self) -> ResearchPanel:
        if not self.cases:
            raise ValueError("research panels require at least one case")
        ids = [case.id for case in self.cases]
        if len(set(ids)) != len(ids):
            raise ValueError("research panel case ids must be unique")
        return self


class PanelCaseResult(ResearchModel):
    id: str
    family: str
    output: str
    status: Literal["completed", "failed", "cancelled", "preflight_error", "invalid_result"]
    success: bool
    native_score: float
    error: str | None = None
    failure_category: str | None = None
    run_id: str | None = None
    runtime: dict[str, str] | None = None
    artifacts: dict[str, tuple[str, ...]]


class PanelScore(ResearchModel):
    schema_version: Literal["1.0"] = "1.0"
    panel: str
    panel_hash: str = Field(min_length=64, max_length=64)
    started_at: str
    ended_at: str
    native_successes: int = Field(ge=0)
    attempts: int = Field(gt=0)
    macro_score: float
    infrastructure_or_policy_failures: int = Field(ge=0)
    cases: tuple[PanelCaseResult, ...]


EvaluationExecutor = Callable[..., EvaluationRun]


class InvalidMeasurementError(RuntimeError):
    """Raised when a panel cannot produce comparable native measurements."""


def run_panel(
    panel_path: Path,
    *,
    output: Path,
    api_key_env: str = "OPENAI_API_KEY",
    executor: EvaluationExecutor = execute_evaluation,
) -> PanelScore:
    """Run every frozen case and aggregate only its native result."""
    panel_path = panel_path.expanduser().resolve()
    started_at = datetime.now(timezone.utc).isoformat(timespec="seconds")
    output = output.expanduser().resolve()
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        raise FileExistsError(f"Output must be absent or an empty directory: {output}")
    output.mkdir(parents=True, exist_ok=True)
    panel_bytes = panel_path.read_bytes()
    panel = ResearchPanel.model_validate_json(panel_bytes)
    resolved_specs = _resolve_specifications(panel, panel_path.parent)
    panel_hash = _panel_hash(panel_bytes, resolved_specs)
    results = tuple(
        _run_case(
            case,
            specification_path=specification_path,
            output=output,
            api_key_env=api_key_env,
            executor=executor,
        )
        for case, specification_path, _content in resolved_specs
    )
    successes = sum(result.success for result in results)
    failures = sum(result.status != "completed" for result in results)
    score = PanelScore(
        panel=panel.name,
        panel_hash=panel_hash,
        started_at=started_at,
        ended_at=datetime.now(timezone.utc).isoformat(timespec="seconds"),
        native_successes=successes,
        attempts=len(results),
        macro_score=sum(result.native_score for result in results) / len(results),
        infrastructure_or_policy_failures=failures,
        cases=results,
    )
    (output / "panel-score.json").write_text(
        score.model_dump_json(indent=2) + "\n",
        encoding="utf-8",
    )
    return score


def _resolve_specifications(
    panel: ResearchPanel, panel_dir: Path
) -> tuple[tuple[PanelCase, Path, bytes], ...]:
    resolved: list[tuple[PanelCase, Path, bytes]] = []
    for case in panel.cases:
        path = (panel_dir / case.specification).resolve()
        if not path.is_relative_to(panel_dir.resolve()):
            raise ValueError(f"case specification escapes panel directory: {case.id}")
        resolved.append((case, path, path.read_bytes()))
    return tuple(resolved)


def _panel_hash(panel_bytes: bytes, specs: tuple[tuple[PanelCase, Path, bytes], ...]) -> str:
    digest = hashlib.sha256()
    digest.update(b"panel\0")
    digest.update(panel_bytes)
    for case, _path, content in specs:
        digest.update(b"\0case\0")
        digest.update(case.id.encode())
        digest.update(b"\0")
        digest.update(content)
    return digest.hexdigest()


def _run_case(
    case: PanelCase,
    *,
    specification_path: Path,
    output: Path,
    api_key_env: str,
    executor: EvaluationExecutor,
) -> PanelCaseResult:
    case_output = output / case.id
    try:
        run = executor(
            specification_path,
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
            failure_category="preflight_error",
            artifacts=_artifact_index(case_output),
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
            failure_category="measurement_failure",
            run_id=run.run_id,
            runtime=_runtime_identity(run),
            artifacts=_artifact_index(case_output),
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
        failure_category=None if value["success"] else "native_failure",
        run_id=run.run_id,
        runtime=_runtime_identity(run),
        artifacts=_artifact_index(case_output),
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
        failure_category="invalid_result",
        artifacts=_artifact_index(output),
    )


def _runtime_identity(run: EvaluationRun) -> dict[str, str]:
    return {
        "profile": run.runtime.profile,
        "driver": run.runtime.driver,
        "driver_version": run.runtime.driver_version,
        "model": run.runtime.model,
        "thinking_level": run.runtime.thinking_level,
    }


def _artifact_index(output: Path) -> dict[str, tuple[str, ...]]:
    patterns = {
        "policy": ("policy.py", "policy.pkl"),
        "logs": ("*.jsonl", "*.log"),
        "videos": ("*.mp4",),
        "memory": ("*.db",),
        "results": ("*.json",),
    }
    if not output.exists():
        return {category: () for category in patterns}
    return {
        category: tuple(
            sorted(
                path.resolve().as_posix()
                for pattern in category_patterns
                for path in output.rglob(pattern)
            )
        )
        for category, category_patterns in patterns.items()
    }


def publish_evo_result(
    score: PanelScore,
    *,
    result_path: Path | None,
    traces_dir: Path | None,
    experiment_id: str,
) -> None:
    """Publish Evo's inline result and per-task traces without importing Evo."""
    if traces_dir is not None:
        traces_dir.mkdir(parents=True, exist_ok=True)
        for case in score.cases:
            trace = {
                "experiment_id": experiment_id,
                "task_id": case.id,
                "score": case.native_score,
                "status": "passed" if case.success else "failed",
                "ended_at": score.ended_at,
                "native_status": case.status,
                "success": case.success,
                "failure_reason": case.failure_category,
                "error": case.error,
                "run_id": case.run_id,
                "runtime": case.runtime,
                "panel_hash": score.panel_hash,
                "artifacts": case.artifacts,
                "output": case.output,
            }
            (traces_dir / f"task_{case.id}.json").write_text(
                json.dumps(trace, indent=2, sort_keys=True) + "\n", encoding="utf-8"
            )
    invalid = [case for case in score.cases if case.status != "completed"]
    if invalid:
        ids = ", ".join(case.id for case in invalid)
        raise InvalidMeasurementError(f"invalid panel measurement for: {ids}")
    payload = {
        "score": score.macro_score,
        "tasks": {case.id: case.native_score for case in score.cases},
        "panel_hash": score.panel_hash,
        "started_at": score.started_at,
        "ended_at": score.ended_at,
    }
    if result_path is None:
        print(json.dumps(payload, sort_keys=True))
    else:
        _atomic_publish_json(result_path, payload)


def _atomic_publish_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    descriptor = os.open(
        temporary,
        os.O_CREAT | os.O_EXCL | os.O_WRONLY | os.O_CLOEXEC,
        0o644,
    )
    try:
        remaining = memoryview((json.dumps(payload, sort_keys=True) + "\n").encode())
        while remaining:
            remaining = remaining[os.write(descriptor, remaining) :]
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    try:
        os.link(temporary, path)
        directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    except FileExistsError as exc:
        raise FileExistsError(f"Evo result already claimed: {path}") from exc
    finally:
        temporary.unlink(missing_ok=True)


def main(argv: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--panel", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--api-key-env", default="OPENAI_API_KEY")
    parser.add_argument("--json", action="store_true", dest="json_output")
    args = parser.parse_args(argv)
    result = run_panel(args.panel, output=args.output, api_key_env=args.api_key_env)
    result_path = os.environ.get("EVO_RESULT_PATH")
    traces_dir = os.environ.get("EVO_TRACES_DIR")
    evo_environment = result_path is not None or traces_dir is not None
    if evo_environment:
        publish_evo_result(
            result,
            result_path=Path(result_path) if result_path is not None else None,
            traces_dir=Path(traces_dir) if traces_dir is not None else None,
            experiment_id=os.environ.get("EVO_EXPERIMENT_ID", "unknown"),
        )
    if evo_environment and result_path is None:
        return
    if args.json_output:
        print(result.model_dump_json())
    else:
        print(
            f"{result.panel}: {result.native_successes}/{result.attempts} "
            f"(macro score {result.macro_score:g})"
        )


if __name__ == "__main__":
    main()
