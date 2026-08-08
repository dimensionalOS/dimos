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

"""Resolve, execute, and atomically publish one Evaluation Run."""

from __future__ import annotations

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import tempfile
import time
from typing import Literal
from uuid import uuid4

from dimos.benchmark.evaluation.models import (
    EvaluationIdentity,
    EvaluationRun,
    EvaluationRunError,
    EvaluationRunSpecification,
)
from dimos.benchmark.evaluation.progress import ProgressSink, StatusProgress, emit_progress
from dimos.benchmark.evaluation.protocol import EvaluationContext
from dimos.benchmark.evaluation.registry import resolve_evaluation
from dimos.benchmark.evaluation.runtime import CodePolicyRuntimeFactory


def execute_evaluation(
    specification_path: Path,
    *,
    output: Path,
    api_key_env: str = "OPENAI_API_KEY",
    progress: ProgressSink | None = None,
) -> EvaluationRun:
    """Run one resolved Evaluation and publish its immutable record."""
    specification_path = specification_path.expanduser().resolve()
    output = output.expanduser().resolve()
    _validate_output(output)
    specification = EvaluationRunSpecification.model_validate_json(specification_path.read_bytes())
    resolved = resolve_evaluation(specification.evaluation.name)
    config = resolved.evaluation.config_model.model_validate_json(
        json.dumps(specification.evaluation.config),
        strict=True,
    )
    api_key = os.environ.get(api_key_env)
    if not api_key:
        raise ValueError(f"API key environment variable {api_key_env!r} is unset")

    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=f".{output.name}-", dir=output.parent))
    run_id = str(uuid4())
    started_at = datetime.now(timezone.utc)
    started = time.monotonic()
    runtime = CodePolicyRuntimeFactory(
        config=specification.agent,
        api_key=api_key,
        workspace=temporary,
        progress=progress,
    )
    context = EvaluationContext(
        run_id=run_id,
        spec_dir=specification_path.parent,
        workspace=temporary,
        agent=runtime,
        progress=progress,
    )
    emit_progress(progress, StatusProgress(channel="eval", message="evaluation started"))
    try:
        status: Literal["completed", "failed", "cancelled"]
        try:
            report = resolved.evaluation.run(config, context)
            status = "completed"
            error = None
        except KeyboardInterrupt:
            report = None
            status = "cancelled"
            error = EvaluationRunError(
                stage="evaluation",
                error_type="KeyboardInterrupt",
                message="Evaluation cancelled by user",
            )
        except Exception as exc:
            report = None
            status = "failed"
            error = EvaluationRunError(
                stage="evaluation",
                error_type=type(exc).__name__,
                message=_redact_error(str(exc) or type(exc).__name__, api_key),
            )
        finished_at = datetime.now(timezone.utc)
        run = EvaluationRun(
            run_id=run_id,
            specification=specification,
            evaluation=EvaluationIdentity(
                name=resolved.name,
                provider=resolved.provider,
                version=resolved.version,
            ),
            runtime=runtime.identity,
            status=status,
            started_at=started_at,
            finished_at=finished_at,
            duration_seconds=time.monotonic() - started,
            report=report,
            error=error,
            runtime_artifacts=runtime.runtime_artifacts,
            prompt_evidence=runtime.prompt_evidence,
        )
        (temporary / "run.json").write_text(
            run.model_dump_json(indent=2) + "\n",
            encoding="utf-8",
        )
        if output.exists():
            output.rmdir()
        os.replace(temporary, output)
        emit_progress(progress, StatusProgress(channel="eval", message="run published"))
        return run
    finally:
        if temporary.exists():
            shutil.rmtree(temporary)


def _validate_output(output: Path) -> None:
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        raise FileExistsError(f"Output must be absent or an empty directory: {output}")


def _redact_error(message: str, api_key: str) -> str:
    return message.replace(api_key, "[REDACTED]")
