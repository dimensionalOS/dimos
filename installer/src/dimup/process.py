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

import codecs
from collections.abc import Iterator, Sequence
from contextlib import contextmanager, nullcontext, suppress
import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import time

from rich.console import Console
from rich.progress import Progress, SpinnerColumn, TextColumn, TimeElapsedColumn
from rich.text import Text


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


def tool_text(output: str) -> Text:
    rendered = Text.from_ansi(output)
    # Rich's ANSI decoder drops one final newline; retain tool line boundaries.
    if output.endswith("\n"):
        rendered.append("\n")
    return rendered


class Runner:
    def __init__(self, log: Path) -> None:
        self.log = log
        self.console = Console(
            highlight=False, force_terminal=False if os.environ.get("CI") else None
        )
        self._in_stage = False

    @contextmanager
    def stage(self, title: str) -> Iterator[None]:
        started = time.monotonic()
        self.console.print(Text(f"\n  {title}", style="bold cyan"))
        progress = Progress(
            SpinnerColumn(),
            TextColumn("{task.description}", style="cyan", markup=False),
            TimeElapsedColumn(),
            console=self.console,
            transient=True,
        )
        progress.add_task(title, total=None)
        status = progress if self.console.is_terminal else nullcontext()
        self._in_stage = True
        try:
            with status:
                yield
        except (Exception, KeyboardInterrupt):
            self.console.print(Text(f"✗ {title}", style="bold red"))
            raise
        else:
            line = Text(f"✓ {title}", style="green")
            line.append(f"  {time.monotonic() - started:.1f}s", style="dim")
            self.console.print(line)
        finally:
            self._in_stage = False

    def run(
        self,
        stage: str,
        command: Sequence[str],
        *,
        cwd: Path | None = None,
        env: dict[str, str] | None = None,
        capture: bool = False,
    ) -> str:
        context = nullcontext() if self._in_stage else self.stage(stage)
        with context:
            return self._run(stage, command, cwd=cwd, env=env, capture=capture)

    def _run(
        self,
        stage: str,
        command: Sequence[str],
        *,
        cwd: Path | None,
        env: dict[str, str] | None,
        capture: bool,
    ) -> str:
        self.log.parent.mkdir(parents=True, exist_ok=True)
        tail = ""
        stdout = ""
        with self.log.open("a", encoding="utf-8") as output:
            output.write(f"\n{stage}\n$ {shlex.join(command)}\n")
            output.flush()
            try:
                # Pipes keep tool output append-only; dimup owns terminal animation.
                child_env = {**(os.environ if env is None else env), "NO_COLOR": "1"}
                with subprocess.Popen(
                    command,
                    cwd=cwd,
                    env=child_env,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE if capture else subprocess.STDOUT,
                    start_new_session=True,
                ) as process:
                    try:
                        if capture:
                            raw_stdout, raw_stderr = process.communicate()
                            stdout = raw_stdout.decode("utf-8", errors="replace")
                            stderr = raw_stderr.decode("utf-8", errors="replace")
                            plain = tool_text(stdout + stderr).plain
                            output.write(plain)
                            tail = plain[-8000:]
                        else:
                            assert process.stdout is not None
                            decoder = codecs.getincrementaldecoder("utf-8")(errors="replace")
                            while True:
                                chunk = os.read(process.stdout.fileno(), 65536)
                                rendered = tool_text(decoder.decode(chunk, final=not chunk))
                                output.write(rendered.plain)
                                output.flush()
                                tail = (tail + rendered.plain)[-8000:]
                                if rendered:
                                    self.console.print(rendered, end="", soft_wrap=True)
                                if not chunk:
                                    break
                            process.wait()
                    except KeyboardInterrupt:
                        with suppress(ProcessLookupError):
                            os.killpg(process.pid, signal.SIGTERM)
                        try:
                            process.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            os.killpg(process.pid, signal.SIGKILL)
                            process.wait()
                        output.write("\nInterrupted.\n")
                        raise
                    returncode = process.returncode
            except OSError as error:
                raise SetupError(
                    f"Failed: {stage}\nCommand: {shlex.join(command)}\nLog: {self.log}\n{error}"
                ) from error
        if not capture and tail and not tail.endswith("\n"):
            self.console.print()
        if returncode:
            detail = "\n" + "\n".join(tail.splitlines()[-12:]) if capture else ""
            raise SetupError(
                f"Failed: {stage} (exit {returncode})\nCommand: {shlex.join(command)}"
                f"{detail}\nLog: {self.log}"
            )
        return stdout.strip()
