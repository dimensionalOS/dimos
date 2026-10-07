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

"""Supervisor for hosted deployments on one machine."""

from __future__ import annotations

from collections.abc import Iterator, Sequence
from contextlib import ExitStack, contextmanager
from dataclasses import dataclass
from importlib.metadata import version as package_version
import multiprocessing
from multiprocessing.connection import Connection
from multiprocessing.process import BaseProcess
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading
from typing import TYPE_CHECKING, Any, Literal
import uuid

from pydantic import Field
from pydantic_settings import BaseSettings, SettingsConfigDict

from dimos.constants import DIMOS_PROJECT_ROOT, STATE_DIR
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.coordination.process_lifecycle import DIMOS_RUN_ID_ENV, kill_run_processes
from dimos.core.global_config import ENV_FILE
from dimos.hosted.fragment import (
    FRAGMENT_FORMAT,
    FRAGMENT_SCHEMA_VERSION,
    HostFragment,
    run_coordinator_rpc_name,
)
from dimos.utils.logging_config import set_run_log_dir

if TYPE_CHECKING:
    from dimos.protocol.rpc.zenohrpc import ZenohRPC

HostState = Literal["available", "starting", "running", "stopping", "failed"]

HOST_PROTOCOL_VERSION = 2
HOST_LIVELINESS_KEY = "dimos/hosts/{host_id}/live"
HOST_CONTROL_RPC_NAME = "hosts/{host_id}"
DEFAULT_STARTUP_TIMEOUT = 60.0
DEFAULT_STOP_TIMEOUT = 5.0
DEFAULT_LOG_ROOT = STATE_DIR / "hosted" / "runs"
# The daemon is its machine's zenoh router, on zenoh's own port.
DEFAULT_LISTEN = "tcp/0.0.0.0:7447"
# Hosts scout each other on their own multicast group, apart from any other zenoh fabric.
DIMOS_SCOUT_ADDR = "224.0.0.224:7449"


class HostConfig(BaseSettings):
    """`dimos host serve` settings, also set by HOST__<FIELD> in the environment or .env."""

    model_config = SettingsConfigDict(
        env_prefix="HOST__", env_file=ENV_FILE, env_file_encoding="utf-8", extra="ignore"
    )

    name: str = Field(default_factory=socket.gethostname)
    # Comma-separated placement tags, added to the auto-detected ones.
    tags: str = ""
    # Taken: the next free port up is used instead.
    listen: str = DEFAULT_LISTEN
    # Comma-separated routers this one always links to.
    connect: str = ""
    # Find other Hosts by scouting and the Go2 LAN probe, and link to them.
    autodiscovery: bool = True
    discovery_interval: float = Field(default=10.0, gt=0)
    scout_addr: str = DIMOS_SCOUT_ADDR
    # Empty scouts every interface.
    scout_interface: str = ""


def split_csv(value: str) -> list[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


def free_listen(endpoint: str, tries: int = 10) -> str:
    """``endpoint`` if its port is free, else the same endpoint on the next free port up."""
    protocol, _, address = endpoint.partition("/")
    host, _, port = address.rpartition(":")
    bind_host = host.strip("[]") or "0.0.0.0"
    for candidate in range(int(port), int(port) + tries):
        with socket.socket(socket.AF_INET6 if ":" in bind_host else socket.AF_INET) as probe:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                probe.bind((bind_host, candidate))
            except OSError:
                continue
        return f"{protocol}/{host}:{candidate}"
    raise OSError(f"No free port in {port}..{int(port) + tries - 1} for {endpoint}")


def code_revision() -> str:
    """Commit of the dimos checkout this process runs, else the installed package version."""
    try:
        return subprocess.run(
            ["git", "-C", str(DIMOS_PROJECT_ROOT), "rev-parse", "HEAD"],
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return package_version("dimos")


@dataclass(frozen=True, slots=True)
class HostDescriptor:
    host_id: str
    epoch: str
    name: str
    tags: frozenset[str]
    versions: dict[str, str | int]
    state: HostState
    active_run_ids: tuple[str, ...]
    # The zid of the router this Host serves, and that router's listen endpoints.
    router_zid: str = ""
    listen: tuple[str, ...] = ()


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
    """Supervise independent deployment processes for multiple runs."""

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
        listen: Sequence[str] = (DEFAULT_LISTEN,),
        connect: Sequence[str] = (),
        autodiscovery: bool = False,
        scout_addr: str = DIMOS_SCOUT_ADDR,
        scout_interface: str = "",
    ) -> None:
        if not listen:
            raise ValueError("HostDaemon is a zenoh router and needs a listen endpoint")
        self._host_id = host_id
        self.listen = list(listen)
        self.connect = list(connect)
        self.autodiscovery = autodiscovery
        self.scout_addr = scout_addr
        self.scout_interface = scout_interface
        self.router_zid = ""
        self._name = name or socket.gethostname()
        self._tags = frozenset(tags)
        self._versions = dict(versions or {})
        self._log_root = log_root
        self._startup_timeout = startup_timeout
        self._stop_timeout = stop_timeout
        self._process_context = multiprocessing.get_context("spawn")
        self._epoch = uuid.uuid4().hex
        self._deployments: dict[str, _Deployment] = {}
        self._lock = threading.RLock()

    @property
    def client_endpoint(self) -> str:
        """The locator this daemon's fragments dial, its first listener on loopback."""
        protocol, _, address = self.listen[0].partition("/")
        host, _, port = address.rpartition(":")
        if not port or port == "0":
            raise ValueError(f"Listen endpoint {self.listen[0]!r} needs a fixed port")
        if host in ("0.0.0.0", "[::]", ""):
            host = "127.0.0.1"
        return f"{protocol}/{host}:{port}"

    def fragment_global_overrides(self) -> dict[str, Any]:
        """Run every fragment's sessions as zenoh clients of this daemon's router."""
        return {
            "zenoh_mode": "client",
            "zenoh_connect": self.client_endpoint,
            "zenoh_multicast": False,
            "zenoh_scouting": False,
        }

    @contextmanager
    def serve(self) -> Iterator[ZenohRPC]:
        """Open this machine's zenoh router and serve the Host control plane on it."""
        from dimos.protocol.rpc.zenohrpc import ZenohRPC
        from dimos.protocol.service.zenohservice import ZenohSessionPool

        pool = ZenohSessionPool()
        # connect_timeout 0: zenoh keeps redialing absent routers in the background.
        rpc = ZenohRPC(
            session_pool=pool,
            mode="router",
            listen=self.listen,
            connect=self.connect,
            multicast=self.autodiscovery,
            scouting=self.autodiscovery,
            scouting_interface=self.scout_interface,
            scout_addr=self.scout_addr,
            router_autoconnect=self.autodiscovery,
            connect_timeout=0,
            adminspace=True,
        )
        with ExitStack() as cleanup:
            cleanup.callback(pool.close_all)
            cleanup.callback(self.shutdown)
            rpc.start()
            cleanup.callback(rpc.stop)
            self.router_zid = str(rpc.session.zid())
            control_name = HOST_CONTROL_RPC_NAME.format(host_id=self._host_id)
            rpc.serve_rpc(self.describe, f"{control_name}/describe")  # type: ignore[arg-type]
            rpc.serve_rpc(self.start, f"{control_name}/start")  # type: ignore[arg-type]
            rpc.serve_rpc(self.status, f"{control_name}/status")  # type: ignore[arg-type]
            rpc.serve_rpc(self.stop, f"{control_name}/stop")  # type: ignore[arg-type]
            token = rpc.session.liveliness().declare_token(
                HOST_LIVELINESS_KEY.format(host_id=self._host_id)
            )
            cleanup.callback(token.undeclare)
            yield rpc

    def describe(self) -> HostDescriptor:
        with self._lock:
            self._refresh_locked()
            return HostDescriptor(
                host_id=self._host_id,
                epoch=self._epoch,
                name=self._name,
                tags=self._tags,
                versions=dict(self._versions),
                state=self._host_state_locked(),
                active_run_ids=tuple(sorted(self._deployments)),
                router_zid=self.router_zid,
                listen=tuple(self.listen),
            )

    def start(self, epoch: str, fragment: HostFragment) -> DeploymentStatus:
        self._check_epoch(epoch)
        self._check_fragment(fragment)

        with self._lock:
            self._refresh_locked()
            existing = self._deployments.get(fragment.run_id)
            if existing is not None:
                current = existing.fragment
                if current.generation != fragment.generation:
                    raise RuntimeError(
                        f"Run {fragment.run_id} is already running generation {current.generation}"
                    )
                if current.payload_digest != fragment.payload_digest:
                    raise ValueError("Fragment digest conflicts with the accepted deployment")
                return self._status_locked(fragment.run_id)

            log_dir = self._log_root / fragment.run_id
            log_dir.mkdir(parents=True, exist_ok=True)
            receive_ready, send_ready = self._process_context.Pipe(duplex=False)
            process = self._process_context.Process(
                target=_run_fragment,
                args=(fragment, log_dir, send_ready, self.fragment_global_overrides()),
                daemon=False,
            )
            deployment = _Deployment(fragment, "starting", log_dir, process)
            self._deployments[fragment.run_id] = deployment
            try:
                process.start()
            except Exception as exc:
                receive_ready.close()
                send_ready.close()
                deployment.state = "failed"
                deployment.error = str(exc)
                return self._status_locked(fragment.run_id)
            send_ready.close()

        error = self._wait_for_start(receive_ready)
        if error is not None:
            _terminate(process, self._stop_timeout)
            kill_run_processes(fragment.run_id)

        with self._lock:
            if self._deployments.get(fragment.run_id) is not deployment:
                return self._status_locked(fragment.run_id)
            if error is not None:
                deployment.state = "failed"
                deployment.error = error
            elif process.exitcode is not None:
                deployment.state = "failed"
                deployment.error = f"Deployment process exited with code {process.exitcode}"
            else:
                deployment.state = "running"
            return self._status_locked(fragment.run_id)

    def status(self, epoch: str, run_id: str) -> DeploymentStatus:
        self._check_epoch(epoch)
        with self._lock:
            self._refresh_locked()
            return self._status_locked(run_id)

    def stop(
        self,
        epoch: str,
        run_id: str,
        generation: int,
        fragment_digest: str,
    ) -> DeploymentStatus:
        self._check_epoch(epoch)
        with self._lock:
            deployment = self._deployments.get(run_id)
            if deployment is None:
                return self._status_locked(run_id)
            fragment = deployment.fragment
            if (fragment.run_id, fragment.generation, fragment.payload_digest) != (
                run_id,
                generation,
                fragment_digest,
            ):
                raise ValueError("Stop request does not match the active deployment")
            if deployment.state == "stopping":
                return self._status_locked(run_id)
            deployment.state = "stopping"

        _terminate(deployment.process, self._stop_timeout)
        kill_run_processes(run_id)

        with self._lock:
            if self._deployments.get(run_id) is deployment:
                del self._deployments[run_id]
            return self._status_locked(run_id)

    def shutdown(self) -> None:
        """Stop every active deployment when the Host service exits."""
        with self._lock:
            deployments = tuple(self._deployments.values())
            self._deployments.clear()
        for deployment in deployments:
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
        required_revision = self._versions.get(
            "application_revision",
            self._versions.get("dimos"),
        )
        if required_revision is not None and fragment.application_revision != str(
            required_revision
        ):
            raise ValueError(
                f"Fragment requires application revision {fragment.application_revision}, "
                f"Host has {required_revision}"
            )
        fragment.validate_digest()

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
        for deployment in self._deployments.values():
            if deployment.state not in {"starting", "running"}:
                continue
            if deployment.process.exitcode is not None:
                deployment.state = "failed"
                deployment.error = (
                    f"Deployment process exited with code {deployment.process.exitcode}"
                )

    def _host_state_locked(self) -> HostState:
        states = {deployment.state for deployment in self._deployments.values()}
        for state in ("failed", "stopping", "starting", "running"):
            if state in states:
                return state
        return "available"

    def _status_locked(self, run_id: str) -> DeploymentStatus:
        deployment = self._deployments.get(run_id)
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


def _run_fragment(
    fragment: HostFragment,
    log_dir: Path,
    ready: Connection,
    global_overrides: dict[str, Any] | None = None,
) -> None:
    os.environ[DIMOS_RUN_ID_ENV] = fragment.run_id
    set_run_log_dir(log_dir)
    stop_requested = threading.Event()
    coordinator: ModuleCoordinator | None = None

    def stop(_signum: int, _frame: object) -> None:
        stop_requested.set()

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        payload = fragment.load_payload()
        remote_module_refs = {
            (reference.consumer_name, reference.reference_name): (
                reference.provider_type,
                reference.rpc_name,
            )
            for reference in payload.remote_module_references
        }
        config = payload.config
        if global_overrides:
            config = config.subset_for(payload.blueprint, global_overrides=global_overrides)
        coordinator = ModuleCoordinator.build(
            payload.blueprint,
            config,
            remote_module_refs=remote_module_refs,
        )
        coordinator.start_rpc_service(
            name=run_coordinator_rpc_name(fragment.run_id, fragment.host_id)
        )
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
