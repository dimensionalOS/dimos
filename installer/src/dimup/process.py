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

"""Run installation steps with a readable console and a detailed log."""

from collections.abc import Sequence
import os
from pathlib import Path
import shlex
import shutil
import subprocess


class SetupError(RuntimeError):
    pass


def executable(name: str) -> str:
    found = shutil.which(name)
    if found:
        return found
    for directory in (
        Path.home() / ".local/bin",
        Path.home() / ".cargo/bin",
        Path.home() / ".nix-profile/bin",
        Path("/nix/var/nix/profiles/default/bin"),
        Path("/opt/homebrew/bin"),
    ):
        candidate = directory / name
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return str(candidate)
    raise SetupError(f"{name} is missing. Run dimup setup.")


class Runner:
    def __init__(self, log: Path) -> None:
        self.log = log

    def run(
        self,
        stage: str,
        command: Sequence[str],
        *,
        cwd: Path | None = None,
        env: dict[str, str] | None = None,
    ) -> str:
        print(f"{stage}...", flush=True)
        self.log.parent.mkdir(parents=True, exist_ok=True)
        with self.log.open("a") as output:
            output.write(f"\n{stage}\n$ {shlex.join(command)}\n")
            output.flush()
            try:
                result = subprocess.run(
                    command,
                    cwd=cwd,
                    env=env,
                    stdout=subprocess.PIPE,
                    stderr=output,
                    text=True,
                    check=False,
                )
            except OSError as error:
                raise SetupError(
                    f"Failed: {stage}\nCommand: {shlex.join(command)}\nLog: {self.log}\n{error}"
                ) from error
            output.write(result.stdout)
        if result.returncode:
            raise SetupError(f"Failed: {stage}\nCommand: {shlex.join(command)}\nLog: {self.log}")
        return result.stdout.strip()
