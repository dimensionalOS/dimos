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

"""Minimal supervisor for one hosted deployment."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import multiprocessing
from multiprocessing.connection import Connection
from multiprocessing.process import BaseProcess
import os
from pathlib import Path
import pickle
import signal
import socket
import threading
from typing import TYPE_CHECKING, Literal
import uuid

from dimos.constants import STATE_DIR
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.coordination.process_lifecycle import DIMOS_RUN_ID_ENV, kill_run_processes
from dimos.core.core import rpc
from dimos.utils.logging_config import set_run_log_dir

if TYPE_CHECKING:
    from dimos.core.coordination.blueprint_config.parsed import ParsedBlueprintConfig

HostState = Literal["available", "starting", "running", "stopping", "failed"]

HOST_PROTOCOL_VERSION = 1
FRAGMENT_SCHEMA_VERSION = 1
FRAGMENT_FORMAT = "python-blueprint"
HOST_LIVELINESS_KEY = "dimos/hosts/{host_id}/live"
HOST_CONTROL_RPC_NAME = "hosts/{host_id}"
DEFAULT_STARTUP_TIMEOUT = 60.0
DEFAULT_STOP_TIMEOUT = 5.0
DEFAULT_LOG_ROOT = STATE_DIR / "hosted" / "runs"


@dataclass(frozen=True, slots=True)
class HostDescriptor:
    host_id: str
    epoch: str
    name: str
    tags: frozenset[str]
    versions: dict[str, str | int]
    state: HostState
    active_run_id: str | None


@dataclass(frozen=True, slots=True)
class HostFragment:
    run_id: str
    generation: int
    host_id: str
    payload_digest: str
    blueprint_payload: bytes
    config: ParsedBlueprintConfig | None = None
    schema_version: int = FRAGMENT_SCHEMA_VERSION
    format: str = FRAGMENT_FORMAT


@dataclass(frozen=True, slots=True)
class DeploymentStatus:
    state: HostState
    run_id: str | None
    generation: int | None
    pid: int | None
    log_dir: str | None
    error: str | None


@dataclass(slots=True)
class _Deployment:
    fragment: HostFragment
    state: HostState
    log_dir: Path
    process: BaseProcess
    error: str | None = None


class HostDaemon:
    """Supervise at most one deployment process."""

    def __init__(
        self,
        host_id: str,
        *,
        name: str | None = None,
        tags: set[str] | frozenset[str] = frozenset(),
        versions: dict[str, str | int] | None = None,
        log_root: Path = DEFAULT_LOG_ROOT,
        startup_timeout: float = DEFAULT_STARTUP_TIMEOUT,
        stop_timeout: float = DEFAULT_STOP_TIMEOUT,
    ) -> None:
        self._host_id = host_id
        self._name = name or socket.gethostname()
        self._tags = frozenset(tags)
        self._versions = dict(versions or {})
        self._log_root = log_root
        self._startup_timeout = startup_timeout
        self._stop_timeout = stop_timeout
        self._process_context = multiprocessing.get_context("spawn")
        self._epoch = uuid.uuid4().hex
        self._deployment: _Deployment | None = None
        self._lock = threading.RLock()

    @rpc
    def describe(self) -> HostDescriptor:
        with self._lock:
            self._refresh_locked()
            deployment = self._deployment
            return HostDescriptor(
                host_id=self._host_id,
                epoch=self._epoch,
                name=self._name,
                tags=self._tags,
                versions=dict(self._versions),
                state=deployment.state if deployment else "available",
                active_run_id=deployment.fragment.run_id if deployment else None,
            )

    @rpc
    def start(self, epoch: str, fragment: HostFragment) -> DeploymentStatus:
        self._check_epoch(epoch)
        self._check_fragment(fragment)

        with self._lock:
            self._refresh_locked()
            if self._deployment is not None:
                current = self._deployment.fragment
                if (current.run_id, current.generation) != (
                    fragment.run_id,
                    fragment.generation,
                ):
                    raise RuntimeError(f"Host {self._host_id} is running {current.run_id}")
                if current.payload_digest != fragment.payload_digest:
                    raise ValueError("Fragment digest conflicts with the accepted deployment")
                return self._status_locked()

            log_dir = self._log_root / fragment.run_id
            log_dir.mkdir(parents=True, exist_ok=True)
            receive_ready, send_ready = self._process_context.Pipe(duplex=False)
            process = self._process_context.Process(
                target=_run_fragment,
                args=(fragment, log_dir, send_ready),
                daemon=False,
            )
            deployment = _Deployment(fragment, "starting", log_dir, process)
            self._deployment = deployment
            try:
                process.start()
            except Exception as exc:
                receive_ready.close()
                send_ready.close()
                deployment.state = "failed"
                deployment.error = str(exc)
                return self._status_locked()
            send_ready.close()

        error = self._wait_for_start(receive_ready)
        if error is not None:
            _terminate(process, self._stop_timeout)
            kill_run_processes(fragment.run_id)

        with self._lock:
            if self._deployment is not deployment:
                return self._status_locked()
            if error is not None:
                deployment.state = "failed"
                deployment.error = error
            elif process.exitcode is not None:
                deployment.state = "failed"
                deployment.error = f"Deployment process exited with code {process.exitcode}"
            else:
                deployment.state = "running"
            return self._status_locked()

    @rpc
    def status(self, epoch: str, run_id: str) -> DeploymentStatus:
        self._check_epoch(epoch)
        with self._lock:
            self._refresh_locked()
            if self._deployment and self._deployment.fragment.run_id != run_id:
                raise ValueError(f"Host is assigned to {self._deployment.fragment.run_id}")
            return self._status_locked()

    @rpc
    def stop(
        self,
        epoch: str,
        run_id: str,
        generation: int,
        fragment_digest: str,
    ) -> DeploymentStatus:
        self._check_epoch(epoch)
        with self._lock:
            deployment = self._deployment
            if deployment is None:
                return self._status_locked()
            fragment = deployment.fragment
            if (fragment.run_id, fragment.generation, fragment.payload_digest) != (
                run_id,
                generation,
                fragment_digest,
            ):
                raise ValueError("Stop request does not match the active deployment")
            if deployment.state == "stopping":
                return self._status_locked()
            deployment.state = "stopping"

        _terminate(deployment.process, self._stop_timeout)
        kill_run_processes(run_id)

        with self._lock:
            if self._deployment is deployment:
                self._deployment = None
            return self._status_locked()

    def shutdown(self) -> None:
        """Stop the active deployment when the Host service exits."""
        with self._lock:
            deployment = self._deployment
            self._deployment = None
        if deployment is not None:
            _terminate(deployment.process, self._stop_timeout)
            kill_run_processes(deployment.fragment.run_id)

    def _check_epoch(self, epoch: str) -> None:
        if epoch != self._epoch:
            raise ValueError("Host epoch does not match the current daemon instance")

    def _check_fragment(self, fragment: HostFragment) -> None:
        if fragment.host_id != self._host_id:
            raise ValueError(f"Fragment targets Host {fragment.host_id}, not {self._host_id}")
        if fragment.schema_version != FRAGMENT_SCHEMA_VERSION:
            raise ValueError(f"Unsupported fragment schema: {fragment.schema_version}")
        if fragment.format != FRAGMENT_FORMAT:
            raise ValueError(f"Unsupported fragment format: {fragment.format}")
        if hashlib.sha256(fragment.blueprint_payload).hexdigest() != fragment.payload_digest:
            raise ValueError("Fragment payload digest does not match its payload")

    def _wait_for_start(self, ready: Connection) -> str | None:
        try:
            if not ready.poll(self._startup_timeout):
                return f"Deployment startup timed out after {self._startup_timeout}s"
            started, error = ready.recv()
            return None if started else error or "Deployment failed during startup"
        except EOFError:
            return "Deployment process exited before reporting startup status"
        finally:
            ready.close()

    def _refresh_locked(self) -> None:
        deployment = self._deployment
        if deployment is None or deployment.state not in {"starting", "running"}:
            return
        if deployment.process.exitcode is not None:
            deployment.state = "failed"
            deployment.error = f"Deployment process exited with code {deployment.process.exitcode}"

    def _status_locked(self) -> DeploymentStatus:
        deployment = self._deployment
        if deployment is None:
            return DeploymentStatus("available", None, None, None, None, None)
        return DeploymentStatus(
            state=deployment.state,
            run_id=deployment.fragment.run_id,
            generation=deployment.fragment.generation,
            pid=deployment.process.pid,
            log_dir=str(deployment.log_dir),
            error=deployment.error,
        )


def _terminate(process: BaseProcess, timeout: float) -> None:
    if not process.is_alive():
        process.join(timeout=0)
        return
    process.terminate()
    process.join(timeout=timeout)
    if process.is_alive():
        process.kill()
        process.join(timeout=timeout)


def _run_fragment(fragment: HostFragment, log_dir: Path, ready: Connection) -> None:
    os.environ[DIMOS_RUN_ID_ENV] = fragment.run_id
    set_run_log_dir(log_dir)
    stop_requested = threading.Event()
    coordinator: ModuleCoordinator | None = None

    def stop(_signum: int, _frame: object) -> None:
        stop_requested.set()

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        blueprint = pickle.loads(fragment.blueprint_payload)
        coordinator = ModuleCoordinator.build(blueprint, fragment.config)
        coordinator.start_rpc_service()
        if not coordinator.health_check():
            raise RuntimeError("Deployment failed its initial health check")
        ready.send((True, None))
        stop_requested.wait()
    except Exception as exc:
        try:
            ready.send((False, str(exc)))
        except (BrokenPipeError, OSError):
            pass
    finally:
        ready.close()
        if coordinator is not None:
            coordinator.stop()
