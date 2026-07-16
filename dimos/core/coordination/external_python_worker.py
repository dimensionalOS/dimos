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

from __future__ import annotations

import base64
from collections.abc import Sequence
import inspect
import io
import os
from pathlib import Path
import pickle
import select
import signal
import subprocess
import threading
import time

from dimos.core.external_python_module import ExternalPythonModule


class ExternalPythonRuntime:
    """Resolve, prepare, and own one external Python process."""

    startup_timeout = 30.0
    shutdown_timeout = 5.0
    # Keep the most recent 64 KiB for each child output stream.
    output_limit = 64 * 1024

    def __init__(
        self,
        declaration: type[ExternalPythonModule],
        constructor_kwargs: dict[str, object],
    ) -> None:
        self.declaration = declaration
        self.constructor_kwargs = constructor_kwargs
        source = Path(inspect.getfile(declaration)).resolve()
        self.project = source.parent / "python"
        self.pyproject = self.project / "pyproject.toml"
        self.pixi = self.project / "pixi.toml"
        if not self.project.is_dir():
            raise FileNotFoundError(
                f"External Python runtime project is missing: {self.project}; "
                "create the declaration sibling 'python/' directory."
            )
        if not self.pyproject.is_file():
            raise FileNotFoundError(
                f"External Python runtime manifest is missing: {self.pyproject}; "
                "add pyproject.toml to the sibling python/ project."
            )
        self._process: subprocess.Popen[bytes] | None = None
        self._stdout = bytearray()
        self._stderr = bytearray()
        self._reader_threads: list[threading.Thread] = []
        self._lock = threading.Lock()

    @property
    def locked(self) -> bool:
        return (self.project / "uv.lock").is_file()

    def _uv(self, *args: str) -> list[str]:
        command = ["uv", *args]
        return ["pixi", "run", "--executable", *command] if self.pixi.is_file() else command

    def prepare_command(self) -> list[str]:
        args = ["sync"]
        if self.locked:
            args.append("--locked")
        return self._uv(*args)

    def launch_command(
        self, declaration_ref: str, implementation_ref: str, handshake_fd: int
    ) -> list[str]:
        args = [
            "run",
            "python",
            "-m",
            "dimos.core.external_python_bootstrap",
            "--declaration",
            declaration_ref,
            "--implementation",
            implementation_ref,
            "--handshake-fd",
            str(handshake_fd),
            "--kwargs",
            base64.b64encode(pickle.dumps(self.constructor_kwargs)).decode("ascii"),
        ]
        if self.locked:
            args.insert(1, "--locked")
        return self._uv(*args)

    def _run(self, command: Sequence[str]) -> None:
        env = os.environ.copy()
        result = subprocess.run(command, cwd=self.project, env=env, capture_output=True, text=True)
        if result.returncode:
            output = (result.stdout + "\n" + result.stderr).strip()
            raise RuntimeError(
                f"External Python command failed ({' '.join(command)}), exit {result.returncode}: {output[-self.output_limit :]}"
            )

    def start(self) -> None:
        declaration_ref = f"{self.declaration.__module__}:{self.declaration.__name__}"
        parent_read, child_write = os.pipe()
        os.set_inheritable(child_write, True)
        try:
            implementation = self.declaration.implementation
            self._run(self.prepare_command())
            command = self.launch_command(declaration_ref, implementation, child_write)
            self._process = subprocess.Popen(
                command,
                cwd=self.project,
                env=os.environ.copy(),
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                pass_fds=(child_write,),
                start_new_session=True,
            )
            os.close(child_write)
            child_write = -1
            self._reader_threads = [
                threading.Thread(
                    target=self._capture, args=(self._process.stdout, self._stdout), daemon=True
                ),
                threading.Thread(
                    target=self._capture, args=(self._process.stderr, self._stderr), daemon=True
                ),
            ]
            for thread in self._reader_threads:
                thread.start()
            deadline = time.monotonic() + self.startup_timeout
            with os.fdopen(parent_read, "rb") as ready:
                parent_read = -1
                while time.monotonic() < deadline:
                    if select.select([ready], [], [], 0.1)[0]:
                        message = ready.readline().decode(errors="replace").strip()
                        if message.startswith("READY"):
                            return
                        self.stop()
                        raise RuntimeError(
                            f"External Python runtime failed to start: {message}; {self.diagnostics()}"
                        )
                    if self._process.poll() is not None:
                        break
            self.stop()
            raise RuntimeError(
                "External Python runtime exited before becoming ready; " + self.diagnostics()
            )
        except BaseException:
            if self._process is not None:
                self.stop()
            raise
        finally:
            if parent_read >= 0:
                os.close(parent_read)
            if child_write >= 0:
                os.close(child_write)

    def _capture(self, stream: io.BufferedReader | None, target: bytearray) -> None:
        if stream is None:
            return
        while True:
            chunk = stream.read(4096)
            if not chunk:
                return
            with self._lock:
                target.extend(chunk)
                del target[: -self.output_limit]

    def diagnostics(self) -> str:
        with self._lock:
            return f"stdout={bytes(self._stdout).decode(errors='replace')[-4000:]!r}, stderr={bytes(self._stderr).decode(errors='replace')[-4000:]!r}"

    def stop(self) -> None:
        process = self._process
        if process is None:
            return
        try:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=self.shutdown_timeout)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=1)
        finally:
            for thread in self._reader_threads:
                thread.join(timeout=1)
            for stream in (getattr(process, "stdout", None), getattr(process, "stderr", None)):
                if stream is not None:
                    stream.close()
            for thread in self._reader_threads:
                thread.join(timeout=0.1)
            self._reader_threads.clear()
            self._process = None

    @property
    def pid(self) -> int | None:
        return (
            None
            if self._process is None
            else self._process.pid
            if self._process.poll() is None
            else None
        )


class ExternalPythonWorker:
    """Private one-process worker for one external declaration."""

    def __init__(self, declaration: type, constructor_kwargs: dict[str, object]) -> None:
        self.runtime = ExternalPythonRuntime(declaration, constructor_kwargs)
        self.declaration = declaration

    def start(self) -> None:
        self.runtime.start()

    def stop(self) -> None:
        self.runtime.stop()

    @property
    def pid(self) -> int | None:
        return self.runtime.pid

    def diagnostics(self) -> str:
        return self.runtime.diagnostics()
