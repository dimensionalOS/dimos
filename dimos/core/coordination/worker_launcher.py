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

from dataclasses import dataclass
from multiprocessing.connection import Connection, Listener, wait
import os
import secrets
import signal
import subprocess
import tempfile
import time
from typing import Protocol

from dimos.core.coordination.worker_messages import WorkerRequest, WorkerResponse
from dimos.core.runtime_environment import PythonProjectLaunchMaterial


class WorkerProcessHandle(Protocol):
    @property
    def pid(self) -> int | None: ...

    def send(self, request: WorkerRequest) -> None: ...

    def recv(self) -> WorkerResponse: ...

    def poll(self, timeout: float) -> bool: ...

    def close(self) -> None: ...

    def join(self, timeout: float) -> None: ...

    def is_alive(self) -> bool: ...

    def terminate(self) -> None: ...


class WorkerLauncher(Protocol):
    def launch(self, worker_id: int) -> WorkerProcessHandle: ...


@dataclass
class ForkserverWorkerProcessHandle:
    process: object
    connection: object

    @property
    def pid(self) -> int | None:
        pid = getattr(self.process, "pid", None)
        if pid is None:
            return None
        try:
            os.kill(pid, 0)
            return pid
        except OSError:
            return None

    def send(self, request: WorkerRequest) -> None:
        self.connection.send(request)  # type: ignore[attr-defined]

    def recv(self) -> WorkerResponse:
        return self.connection.recv()  # type: ignore[attr-defined,no-any-return]

    def poll(self, timeout: float) -> bool:
        return bool(self.connection.poll(timeout=timeout))  # type: ignore[attr-defined]

    def close(self) -> None:
        self.connection.close()  # type: ignore[attr-defined]

    def join(self, timeout: float) -> None:
        self.process.join(timeout=timeout)  # type: ignore[attr-defined]

    def is_alive(self) -> bool:
        return bool(self.process.is_alive())  # type: ignore[attr-defined]

    def terminate(self) -> None:
        self.process.terminate()  # type: ignore[attr-defined]


class ForkserverWorkerLauncher:
    def launch(self, worker_id: int) -> WorkerProcessHandle:
        from dimos.core.coordination.python_worker import _worker_entrypoint, get_forkserver_context

        ctx = get_forkserver_context()
        parent_conn, child_conn = ctx.Pipe()
        process = ctx.Process(target=_worker_entrypoint, args=(child_conn, worker_id), daemon=True)
        process.start()
        return ForkserverWorkerProcessHandle(process=process, connection=parent_conn)


@dataclass
class VenvWorkerProcessHandle:
    process: subprocess.Popen[bytes]
    connection: Connection

    @property
    def pid(self) -> int | None:
        if self.process.poll() is not None:
            return None
        return self.process.pid

    def send(self, request: WorkerRequest) -> None:
        self.connection.send(request)

    def recv(self) -> WorkerResponse:
        return self.connection.recv()

    def poll(self, timeout: float) -> bool:
        return bool(self.connection.poll(timeout=timeout))

    def close(self) -> None:
        self.connection.close()

    def join(self, timeout: float) -> None:
        try:
            self.process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            pass

    def is_alive(self) -> bool:
        return self.process.poll() is None

    def terminate(self) -> None:
        self.process.terminate()


class VenvWorkerLauncher:
    def __init__(
        self,
        python_executable: str,
        env: dict[str, str] | None = None,
        startup_timeout: float = 10.0,
    ) -> None:
        self.python_executable = python_executable
        self.env = env or {}
        self.startup_timeout = startup_timeout

    def launch(self, worker_id: int) -> WorkerProcessHandle:
        if not os.path.isfile(self.python_executable) or not os.access(
            self.python_executable, os.X_OK
        ):
            raise FileNotFoundError(
                f"Python executable not found or not executable: {self.python_executable}"
            )

        with tempfile.TemporaryDirectory(prefix="dimos-venv-worker-") as temp_dir:
            address = os.path.join(temp_dir, "worker.sock")
            authkey = secrets.token_bytes(32)
            listener = Listener(address=address, family="AF_UNIX", authkey=authkey)
            process: subprocess.Popen[bytes] | None = None
            try:
                child_env = os.environ.copy()
                child_env.update(self.env)
                process = subprocess.Popen(
                    [
                        self.python_executable,
                        "-m",
                        "dimos.core.coordination.venv_worker_entrypoint",
                        "--address",
                        address,
                        "--authkey-hex",
                        authkey.hex(),
                        "--worker-id",
                        str(worker_id),
                    ],
                    env=child_env,
                )
                conn = self._accept_with_timeout(listener, process)
                return VenvWorkerProcessHandle(process=process, connection=conn)
            except Exception:
                if process is not None and process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=2)
                raise
            finally:
                listener.close()

    def _accept_with_timeout(
        self, listener: Listener, process: subprocess.Popen[bytes]
    ) -> Connection:
        deadline = time.monotonic() + self.startup_timeout
        while True:
            returncode = process.poll()
            if returncode is not None:
                command = str(process.args)
                raise RuntimeError(
                    "Venv worker exited before connecting "
                    f"with exit code {returncode}. Command: {command}"
                )

            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(
                    f"Timed out waiting {self.startup_timeout}s for venv worker to connect"
                )

            ready = wait([listener._listener._socket], timeout=min(0.05, remaining))  # type: ignore[attr-defined]
            if ready:
                break

        result = listener.accept()
        if process.poll() is not None:
            result.close()
            raise RuntimeError(
                f"Venv worker exited during startup with exit code {process.returncode}"
            )
        return result


@dataclass
class CommandWorkerProcessHandle(VenvWorkerProcessHandle):
    process_group_id: int

    def terminate(self) -> None:
        _terminate_process_group(self.process, self.process_group_id, terminate_timeout=0.5)


def _process_group_exists(process_group_id: int) -> bool:
    try:
        os.killpg(process_group_id, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _terminate_process_group(
    process: subprocess.Popen[bytes],
    process_group_id: int,
    *,
    terminate_timeout: float,
) -> None:
    if _process_group_exists(process_group_id):
        try:
            os.killpg(process_group_id, signal.SIGTERM)
        except ProcessLookupError:
            pass
        except OSError:
            if process.poll() is None:
                process.terminate()

    deadline = time.monotonic() + terminate_timeout
    while time.monotonic() < deadline:
        if process.poll() is not None and not _process_group_exists(process_group_id):
            return
        time.sleep(0.02)

    if _process_group_exists(process_group_id):
        try:
            os.killpg(process_group_id, signal.SIGKILL)
        except ProcessLookupError:
            pass
        except OSError:
            if process.poll() is None:
                process.kill()

    try:
        process.wait(timeout=2)
    except subprocess.TimeoutExpired:
        pass
    except ChildProcessError:
        pass


class CommandWorkerLauncher:
    def __init__(
        self,
        material: PythonProjectLaunchMaterial,
        startup_timeout: float = 10.0,
    ) -> None:
        self.material = material
        self.startup_timeout = startup_timeout

    def launch(self, worker_id: int) -> WorkerProcessHandle:
        with tempfile.TemporaryDirectory(prefix="dimos-project-worker-") as temp_dir:
            address = os.path.join(temp_dir, "worker.sock")
            authkey = secrets.token_bytes(32)
            listener = Listener(address=address, family="AF_UNIX", authkey=authkey)
            process: subprocess.Popen[bytes] | None = None
            try:
                child_env = os.environ.copy()
                child_env.update(self.material.env)
                argv = [
                    *self.material.argv_prefix,
                    "-m",
                    "dimos.core.coordination.venv_worker_entrypoint",
                    "--address",
                    address,
                    "--authkey-hex",
                    authkey.hex(),
                    "--worker-id",
                    str(worker_id),
                ]
                process = subprocess.Popen(
                    argv,
                    cwd=self.material.cwd,
                    env=child_env,
                    start_new_session=True,
                )
                process_group_id = os.getpgid(process.pid)
                conn = self._accept_with_timeout(listener, process)
                return CommandWorkerProcessHandle(
                    process=process, connection=conn, process_group_id=process_group_id
                )
            except Exception:
                if process is not None:
                    self._terminate_process_group(process)
                raise
            finally:
                listener.close()

    def _accept_with_timeout(
        self, listener: Listener, process: subprocess.Popen[bytes]
    ) -> Connection:
        deadline = time.monotonic() + self.startup_timeout
        while True:
            returncode = process.poll()
            if returncode is not None:
                command = str(process.args)
                raise RuntimeError(
                    "Command worker exited before connecting "
                    f"with exit code {returncode}. Command: {command}"
                )

            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(
                    f"Timed out waiting {self.startup_timeout}s for command worker to connect"
                )

            ready = wait([listener._listener._socket], timeout=min(0.05, remaining))  # type: ignore[attr-defined]
            if ready:
                break

        result = listener.accept()
        if process.poll() is not None:
            result.close()
            raise RuntimeError(
                f"Command worker exited during startup with exit code {process.returncode}"
            )
        return result

    def _terminate_process_group(self, process: subprocess.Popen[bytes]) -> None:
        _terminate_process_group(process, process.pid, terminate_timeout=2)
