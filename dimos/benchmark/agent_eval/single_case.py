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

"""Direct runner for one frozen-memory Pi evaluation case."""

from __future__ import annotations

import errno
import json
import os
from pathlib import Path
import re
import shutil
import tempfile
import time

from dimos.agents.code_policy_core import CodePolicySessionConfig, FrozenMemoryEnvironment
from dimos.agents.code_policy_server import CodePolicyMcpServer
from dimos.benchmark.agent_eval.models import CompactEvalResult, EvalCase, EvalRunConfig
from dimos.benchmark.agent_eval.pi_process import PiCliRunner, PiRunError
from dimos.benchmark.agent_eval.progress import ProgressSink, StatusProgress, emit_progress
from dimos.benchmark.short_horizon_qa.eval import (
    load_exact_integer_oracle,
    parse_integer_prediction,
)
from dimos.benchmark.short_horizon_qa.models import MapperSettings
from dimos.benchmark.short_horizon_qa.prepare import prepare_bundle
from dimos.benchmark.short_horizon_qa.service import load_bundle
from dimos.constants import CACHE_DIR
from dimos.memory2.cli.dataset import resolve_dataset

TURN_TIMEOUT_SECONDS = 600.0

SYSTEM_PROMPT = """You are answering a question about a frozen robot recording.

You have exactly one tool, `python_exec`. It runs trusted, unsandboxed Python in a
persistent Jupyter kernel with a read-only `memory` object. Inspect Memory2 streams
and compute the answer from the recording. Do not guess. End with exactly one line:
ANSWER: <integer>
"""


def execute_single_case(
    case_path: Path,
    *,
    config: EvalRunConfig,
    output: Path,
    progress: ProgressSink | None = None,
) -> CompactEvalResult:
    """Preflight, run, and atomically publish exactly one result directory."""
    path = case_path.expanduser().resolve()
    output = output.expanduser().resolve()
    _validate_output(output)
    emit_progress(progress, StatusProgress(channel="eval", message="loading case"))
    case = EvalCase.model_validate_json(path.read_bytes())
    oracle = load_exact_integer_oracle(case, path.parent)
    api_key = os.environ.get(config.agent.api_key_env)
    if not api_key:
        raise ValueError(f"API key environment variable {config.agent.api_key_env!r} is unset")
    bundle = _materialize_frozen_memory(case, progress)
    _, cutoff, source_path, derived_path = load_bundle(bundle, progress=case.source.progress)
    emit_progress(progress, StatusProgress(channel="eval", message="memory ready"))
    cli, extension = _pi_paths()
    runner = PiCliRunner(
        cli=cli,
        extension=extension,
        model=config.agent.model,
        thinking_level=config.agent.thinking_level,
        timeout_s=TURN_TIMEOUT_SECONDS,
        progress=progress,
    )

    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=f".{output.name}-", dir=output.parent))
    runtime_dir = temporary / "runtime"
    runtime_dir.mkdir()
    started = time.monotonic()
    stderr = ""
    server: CodePolicyMcpServer | None = None
    try:
        emit_progress(progress, StatusProgress(channel="eval", message="starting agent"))
        server = CodePolicyMcpServer(
            CodePolicySessionConfig(
                environment=FrozenMemoryEnvironment(
                    recording_path=str(source_path),
                    derived_recording_path=str(derived_path),
                    memory_cutoff_timestamp=cutoff.cutoff_timestamp,
                )
            )
        )
        try:
            server.start()
            pi_result = runner.run(
                prompt=_agent_prompt(case),
                system_prompt=SYSTEM_PROMPT,
                mcp_url=server.mcp_url,
                api_key=api_key,
                run_dir=runtime_dir,
            )
            stderr = pi_result.stderr
            if pi_result.transcript_path is not None:
                shutil.copy2(pi_result.transcript_path, temporary / "pi-transcript.jsonl")
            prediction = parse_integer_prediction(pi_result.final_text)
            passed = (
                prediction.status == "parsed" and prediction.integer_answer == oracle.expected_count
            )
            result = CompactEvalResult(
                case_id=case.case_id,
                recording=case.source.recording,
                progress=case.source.progress,
                model=config.agent.model,
                thinking_level=config.agent.thinking_level,
                final_response=pi_result.final_text,
                prediction_status=prediction.status,
                integer_answer=prediction.integer_answer,
                passed=passed,
                validator_revision=case.validator.revision,
                tool_call_count=pi_result.tool_call_count,
                duration_seconds=time.monotonic() - started,
            )
        finally:
            server.stop()
    except Exception as exc:
        if isinstance(exc, PiRunError):
            stderr = exc.stderr
        result = CompactEvalResult(
            case_id=case.case_id,
            recording=case.source.recording,
            progress=case.source.progress,
            model=config.agent.model,
            thinking_level=config.agent.thinking_level,
            prediction_status="not_evaluated",
            passed=None,
            validator_revision=case.validator.revision,
            tool_call_count=server.session.execution_count if server is not None else 0,
            duration_seconds=time.monotonic() - started,
            infra_error=f"{type(exc).__name__}: {exc}",
        )
    finally:
        shutil.rmtree(runtime_dir, ignore_errors=True)

    if stderr:
        (temporary / "stderr.log").write_text(stderr, encoding="utf-8")
    (temporary / "result.json").write_text(
        json.dumps(result.model_dump(mode="json"), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    if output.exists():
        output.rmdir()
    os.replace(temporary, output)
    emit_progress(progress, StatusProgress(channel="eval", message="result published"))
    return result


def _validate_output(output: Path) -> None:
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        raise FileExistsError(f"Output must be absent or an empty directory: {output}")


def _materialize_frozen_memory(case: EvalCase, progress: ProgressSink | None) -> Path:
    source_path = resolve_dataset(case.source.recording).resolve()
    stat = source_path.stat()
    stem = re.sub(r"[^A-Za-z0-9_.-]+", "-", source_path.stem)[:64]
    mapper = MapperSettings()
    raw_key = (
        f"{stem}-{stat.st_size}-{stat.st_mtime_ns}-p{case.source.progress:.9f}-"
        f"v{mapper.voxel_size_m}-b{mapper.block_count}-d{mapper.device}-"
        f"c{int(mapper.carve_columns)}-f{mapper.frame_id}-e{mapper.emit_every}"
    )
    key = re.sub(r"[^A-Za-z0-9_.-]+", "-", raw_key)
    bundle = CACHE_DIR / "agent_eval" / "frozen_memory" / key
    manifest = bundle / "manifest.v1.json"
    if not manifest.is_file():
        emit_progress(progress, StatusProgress(channel="eval", message="preparing memory"))
        bundle.parent.mkdir(parents=True, exist_ok=True)
        try:
            prepare_bundle(
                case.source.recording,
                [],
                bundle,
                progress=[case.source.progress],
                mapper=mapper,
                map_progress=lambda current, total: emit_progress(
                    progress,
                    StatusProgress(channel="eval", message=f"mapping {current}/{total} frames"),
                ),
            )
        except OSError as exc:
            concurrent_publish = isinstance(exc, FileExistsError) or exc.errno in {
                errno.EEXIST,
                errno.ENOTEMPTY,
            }
            if not concurrent_publish or not manifest.is_file():
                raise
    return bundle


def _pi_paths() -> tuple[Path, Path]:
    package = Path(__file__).resolve().parents[3] / "packages" / "pi-code-policy-extension"
    cli = package / "node_modules" / "@earendil-works" / "pi-coding-agent" / "dist" / "cli.js"
    extension = package / "dist" / "python-exec.js"
    return cli, extension


def _agent_prompt(case: EvalCase) -> str:
    return (
        f"{case.task.prompt}\n\n"
        "Use python_exec to inspect the read-only recording. "
        f"End with `{case.task.answer_marker} <integer>`."
    )
