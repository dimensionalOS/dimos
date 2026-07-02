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

from abc import ABC, abstractmethod
from multiprocessing.connection import Connection, Listener, wait
import os
from pathlib import Path
import secrets
import signal
import subprocess
import tempfile

from dimos.core.runtime_environment import PythonProjectLaunchMaterial


class WorkerLaunchError(RuntimeError):
    pass


class WorkerProcessHandle(ABC):
    connection: Connection
    supports_parent_actor_ref: bool = True

    @property
    @abstractmethod
    def pid(self) -> int | None: ...

    @abstractmethod
    def join(self, timeout: float | None = None) -> None: ...

    @abstractmethod
    def is_alive(self) -> bool: ...

    @abstractmethod
    def terminate(self) -> None: ...


class WorkerLauncher(ABC):
    @abstractmethod
    def launch(self, worker_id: int) -> WorkerProcessHandle: ...


class ForkserverWorkerProcessHandle(WorkerProcessHandle):
    def __init__(self, process: object, connection: Connection) -> None:
        self._process = process
        self.connection = connection

    @property
    def pid(self) -> int | None:
        return getattr(self._process, "pid", None)

    def join(self, timeout: float | None = None) -> None:
        self._process.join(timeout=timeout)  # type: ignore[attr-defined]

    def is_alive(self) -> bool:
        return bool(self._process.is_alive())  # type: ignore[attr-defined]

    def terminate(self) -> None:
        self._process.terminate()  # type: ignore[attr-defined]


class ForkserverWorkerLauncher(WorkerLauncher):
    def launch(self, worker_id: int) -> WorkerProcessHandle:
        from dimos.core.coordination.python_worker import _worker_entrypoint, get_forkserver_context

        ctx = get_forkserver_context()
        parent_conn, child_conn = ctx.Pipe()
        process = ctx.Process(target=_worker_entrypoint, args=(child_conn, worker_id), daemon=True)
        process.start()
        return ForkserverWorkerProcessHandle(process, parent_conn)


class SubprocessWorkerProcessHandle(WorkerProcessHandle):
    supports_parent_actor_ref = False

    def __init__(
        self,
        process: subprocess.Popen[bytes],
        connection: Connection,
        *,
        terminate_process_group: bool = False,
    ) -> None:
        self._process = process
        self.connection = connection
        self._terminate_process_group = terminate_process_group

    @property
    def pid(self) -> int | None:
        return self._process.pid

    def join(self, timeout: float | None = None) -> None:
        try:
            self._process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            return

    def is_alive(self) -> bool:
        return self._process.poll() is None

    def terminate(self) -> None:
        if self._terminate_process_group and self._process.pid is not None:
            _terminate_process_group(self._process.pid)
            return
        self._process.terminate()


class VenvWorkerLauncher(WorkerLauncher):
    def __init__(
        self,
        python_executable: Path,
        env: dict[str, str] | None = None,
        *,
        runtime_name: str = "",
        startup_timeout: float = 10.0,
    ) -> None:
        self._python_executable = python_executable
        self._env = env or {}
        self._runtime_name = runtime_name
        self._startup_timeout = startup_timeout

    def launch(self, worker_id: int) -> WorkerProcessHandle:
        executable = self._python_executable
        if not executable.exists():
            raise WorkerLaunchError(
                f"Runtime {self._runtime_name!r} Python executable does not exist: {executable}"
            )
        if not os.access(executable, os.X_OK):
            raise WorkerLaunchError(
                f"Runtime {self._runtime_name!r} Python executable is not executable: {executable}"
            )
        return _launch_subprocess_worker(
            argv=(str(executable), "-m", "dimos.core.coordination.venv_worker_entrypoint"),
            env=self._env,
            cwd=None,
            worker_id=worker_id,
            runtime_name=self._runtime_name,
            startup_timeout=self._startup_timeout,
            terminate_process_group=False,
        )


class CommandWorkerLauncher(WorkerLauncher):
    def __init__(
        self,
        material: PythonProjectLaunchMaterial,
        *,
        startup_timeout: float = 10.0,
    ) -> None:
        self._material = material
        self._startup_timeout = startup_timeout

    def launch(self, worker_id: int) -> WorkerProcessHandle:
        return _launch_subprocess_worker(
            argv=(
                *self._material.argv_prefix,
                "-m",
                "dimos.core.coordination.venv_worker_entrypoint",
            ),
            env=dict(self._material.env),
            cwd=self._material.cwd,
            worker_id=worker_id,
            runtime_name=self._material.runtime_name,
            startup_timeout=self._startup_timeout,
            terminate_process_group=True,
        )


def _launch_subprocess_worker(
    *,
    argv: tuple[str, ...],
    env: dict[str, str],
    cwd: Path | None,
    worker_id: int,
    runtime_name: str,
    startup_timeout: float,
    terminate_process_group: bool,
) -> WorkerProcessHandle:
    with tempfile.TemporaryDirectory(prefix="dimos-runtime-worker-") as tmpdir:
        address = str(Path(tmpdir) / "worker.sock")
        authkey = secrets.token_bytes(32)
        listener = Listener(address, family="AF_UNIX", authkey=authkey)
        process_env = {**os.environ, **env}
        full_argv = (
            *argv,
            "--address",
            address,
            "--authkey-hex",
            authkey.hex(),
            "--worker-id",
            str(worker_id),
        )
        process: subprocess.Popen[bytes] | None = None
        try:
            process = subprocess.Popen(
                full_argv,
                cwd=cwd,
                env=process_env,
                start_new_session=terminate_process_group,
            )
            listener_socket = listener._listener._socket  # type: ignore[attr-defined]
            if not wait([listener_socket], timeout=startup_timeout):
                process.terminate()
                raise WorkerLaunchError(
                    f"Runtime {runtime_name!r} worker did not connect within {startup_timeout}s"
                )
            connection = listener.accept()
            return SubprocessWorkerProcessHandle(
                process,
                connection,
                terminate_process_group=terminate_process_group,
            )
        except Exception:
            if process is not None and process.poll() is None:
                process.terminate()
            raise
        finally:
            listener.close()


def _terminate_process_group(pid: int) -> None:
    try:
        os.killpg(pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        os.waitpid(pid, os.WNOHANG)
    except ChildProcessError:
        return
