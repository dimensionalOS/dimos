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

"""Live Pi policy execution against a running DimOS environment."""

from __future__ import annotations

from collections.abc import Callable
import hashlib
import json
from pathlib import Path
import shutil
import threading
import time
from typing import Literal

from dimos.agents.code_policy_server import CodePolicyMcpServer
from dimos.benchmark.evaluation.models import ArtifactReference, RuntimeCondition, RuntimeIdentity
from dimos.benchmark.evaluation.pi_process import (
    PI_VERSION,
    PiCliRunner,
    PiRunCancelledError,
    PiRunError,
    PiRunResult,
)
from dimos.benchmark.evaluation.progress import ProgressSink
from dimos.benchmark.evaluation.protocol import LiveAgentOutcome
from dimos.benchmark.evaluation.runtime import _artifact, _pi_paths

LIVE_AGENT_PROFILE: Literal["live-agent-v1"] = "live-agent-v1"


class LiveAgentRuntimeFactory:
    """Prepare one persistent model and Python workspace for a live episode."""

    def __init__(
        self,
        *,
        api_key: str,
        workspace: Path,
        condition: RuntimeCondition,
        progress: ProgressSink | None = None,
    ) -> None:
        self.api_key = api_key
        self.workspace = workspace
        self.condition = condition
        self.progress = progress
        self._prompt_evidence: list[ArtifactReference] = []
        self._runtime_artifacts: list[ArtifactReference] = []
        self._prepared = False

    @property
    def identity(self) -> RuntimeIdentity:
        return RuntimeIdentity(
            profile=LIVE_AGENT_PROFILE,
            driver_version=PI_VERSION,
            model=self.condition.model,
            thinking_level=self.condition.thinking_level,
        )

    @property
    def prompt_evidence(self) -> tuple[ArtifactReference, ...]:
        return tuple(self._prompt_evidence)

    @property
    def runtime_artifacts(self) -> tuple[ArtifactReference, ...]:
        return tuple(self._runtime_artifacts)

    def prepare(
        self,
        *,
        prompt: str,
        system_prompt: str,
        memory_path: Path,
        episode_timeout_s: float,
    ) -> _LiveAgentExecution:
        if self._prepared:
            raise RuntimeError("live-agent-v1 supports one episode per Evaluation Run")
        if not prompt.strip() or not system_prompt.strip() or episode_timeout_s <= 0:
            raise ValueError("live agent prompt, system prompt, and timeout are required")
        self._prepared = True
        relative = Path("runtime") / "live-agent"
        path = self.workspace / relative
        path.mkdir(parents=True)
        self._record_prompt(path, relative, "task-prompt.txt", prompt, "Task prompt")
        self._record_prompt(
            path, relative, "runtime-system.txt", system_prompt, "Runtime system prompt"
        )
        manifest = {
            "schema_version": "1.0",
            "runtime_profile": LIVE_AGENT_PROFILE,
            "model": self.condition.model,
            "thinking_level": self.condition.thinking_level,
        }
        (path / "prompt-assembly.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        reference = _artifact(
            relative / "prompt-assembly.json", "Prompt assembly", "application/json"
        )
        self._prompt_evidence.append(reference)
        self._runtime_artifacts.append(reference)
        return _LiveAgentExecution(
            api_key=self.api_key,
            model=self.condition.model,
            thinking_level=self.condition.thinking_level,
            prompt=prompt,
            system_prompt=system_prompt,
            memory_path=memory_path,
            timeout_s=episode_timeout_s,
            path=path,
            relative=relative,
            progress=self.progress,
            record_artifact=self._runtime_artifacts.append,
        )

    def _record_prompt(
        self,
        path: Path,
        relative: Path,
        filename: str,
        value: str,
        label: str,
    ) -> None:
        (path / filename).write_text(value, encoding="utf-8")
        reference = _artifact(relative / filename, label, "text/plain")
        self._prompt_evidence.append(reference)
        self._runtime_artifacts.append(reference)


class _LiveAgentExecution:
    def __init__(
        self,
        *,
        api_key: str,
        model: str,
        thinking_level: str,
        prompt: str,
        system_prompt: str,
        memory_path: Path,
        timeout_s: float,
        path: Path,
        relative: Path,
        progress: ProgressSink | None,
        record_artifact: Callable[[ArtifactReference], None],
    ) -> None:
        self.api_key = api_key
        self.prompt = prompt
        self.system_prompt = system_prompt
        self.timeout_s = timeout_s
        self.path = path
        self.relative = relative
        self.record_artifact = record_artifact
        cli, extension = _pi_paths()
        self.runner = PiCliRunner(
            cli=cli,
            extension=extension,
            model=model,
            thinking_level=thinking_level,
            timeout_s=timeout_s,
            progress=progress,
        )
        self.server = CodePolicyMcpServer(live_recording_path=str(memory_path))
        self.cancel = threading.Event()
        self.thread: threading.Thread | None = None
        self.result: PiRunResult | None = None
        self.error: BaseException | None = None
        self.started_at = 0.0
        self.server.start()

    def start(self) -> None:
        if self.thread is not None:
            raise RuntimeError("live agent already started")
        self.started_at = time.monotonic()
        self.thread = threading.Thread(target=self._run, name="evaluation-live-agent", daemon=True)
        self.thread.start()

    def _run(self) -> None:
        try:
            self.result = self.runner.run(
                prompt=self.prompt,
                system_prompt=self.system_prompt,
                mcp_url=self.server.mcp_url,
                api_key=self.api_key,
                run_dir=self.path,
                cancel=self.cancel,
            )
        except PiRunCancelledError as exc:
            self.result = PiRunResult(
                final_text="",
                tool_call_count=exc.tool_call_count,
                duration_seconds=exc.duration_seconds,
                transcript_path=exc.transcript_path,
                stderr=exc.stderr,
            )
        except PiRunError as exc:
            self.result = PiRunResult(
                final_text="",
                tool_call_count=exc.tool_call_count,
                duration_seconds=exc.duration_seconds,
                transcript_path=exc.transcript_path,
                stderr=exc.stderr,
            )
            self.error = exc
        except BaseException as exc:
            self.error = exc

    def failure(self) -> BaseException | None:
        return self.error

    def finish(self) -> LiveAgentOutcome:
        self.cancel.set()
        try:
            if self.thread is not None:
                self.thread.join(timeout=10)
                if self.thread.is_alive():
                    raise TimeoutError("live Pi session did not stop")
            result = self.result
            transcript_path = result.transcript_path if result is not None else None
            if transcript_path is not None:
                target = self.path / "pi-transcript.jsonl"
                shutil.copy2(transcript_path, target)
                self.record_artifact(
                    _artifact(
                        self.relative / target.name,
                        "Pi transcript",
                        "application/x-ndjson",
                    )
                )
                viewer = self.path / "pi-transcript.html"
                self.runner.export_transcript(
                    transcript_path=transcript_path,
                    output_path=viewer,
                    mcp_url=self.server.mcp_url,
                    api_key=self.api_key,
                )
                self.record_artifact(
                    _artifact(self.relative / viewer.name, "Pi transcript viewer", "text/html")
                )
            manifest = self.path / "live-agent.json"
            outcome = LiveAgentOutcome(
                final_text=result.final_text if result is not None else "",
                tool_call_count=result.tool_call_count if result is not None else 0,
                duration_seconds=(
                    max(0.0, time.monotonic() - self.started_at) if self.started_at else 0.0
                ),
            )
            manifest.write_text(
                json.dumps(
                    {
                        "final_text_sha256": hashlib.sha256(
                            outcome.final_text.encode()
                        ).hexdigest(),
                        "tool_call_count": outcome.tool_call_count,
                        "duration_seconds": outcome.duration_seconds,
                    },
                    indent=2,
                    sort_keys=True,
                )
                + "\n",
                encoding="utf-8",
            )
            self.record_artifact(
                _artifact(self.relative / manifest.name, "Live agent outcome", "application/json")
            )
            if self.error is not None:
                raise RuntimeError(f"live Pi failed: {type(self.error).__name__}: {self.error}")
            return outcome
        finally:
            self.server.stop()
