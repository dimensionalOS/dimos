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

"""Evaluate any artifact with a project command using one JSON request and response."""

from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess

from pydantic import BaseModel, Field, ValidationError

from dimos.autoresearch.session import Evaluation, SessionInfo, Trial


class CommandEvaluator(BaseModel):
    argv: list[str] = Field(min_length=1)
    cwd: Path
    timeout_s: float = Field(default=120, gt=0, allow_inf_nan=False)

    @classmethod
    def from_session(cls, info: SessionInfo) -> CommandEvaluator:
        if info.experiment != "command":
            raise ValueError(
                f"this session uses the {info.experiment!r} evaluator; run trials through its integration"
            )
        return cls.model_validate(info.settings)

    def __call__(self, trial: Trial) -> Evaluation:
        log = trial.directory / "stderr.log"
        output = trial.directory / "evaluation.json"
        with log.open("wb") as stderr, output.open("w+b") as stdout:
            process = subprocess.Popen(
                self.argv,
                cwd=self.cwd,
                stdin=subprocess.PIPE,
                stdout=stdout,
                stderr=stderr,
                start_new_session=True,
            )
            try:
                process.communicate(trial.model_dump_json().encode(), timeout=self.timeout_s)
            except BaseException:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait()
                raise
            if process.returncode:
                raise RuntimeError(f"evaluator exited with code {process.returncode}; see {log}")
            stdout.seek(0)
            try:
                return Evaluation.model_validate_json(stdout.read())
            except ValidationError as exc:
                raise ValueError(f"invalid evaluator result in {output}: {exc}") from exc
