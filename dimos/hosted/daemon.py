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

"""Supervisor for hosted deployments on one machine, and its machine's zenoh router."""

from __future__ import annotations

from collections.abc import Callable, Iterator, Mapping
import contextlib
from contextlib import ExitStack, contextmanager
from dataclasses import dataclass
import multiprocessing
from multiprocessing.connection import Connection
from multiprocessing.process import BaseProcess
import os
from pathlib import Path
import socket
import sys
import threading
import time
from typing import TYPE_CHECKING, Any

import psutil

from dimos.constants import LOG_DIR
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.daemon import run_coordinator
from dimos.hosted.config import HostConfig
from dimos.hosted.fragment import HostFragment
from dimos.hosted.protocol import (
    HOST_CONTROL_RPC_NAME,
    HOST_LIVELINESS_KEY,
    RUN_LIVELINESS_KEY,
    RUN_LOG_KEY,
    DeploymentState,
    DeploymentStatus,
    HostDescriptor,
    run_coordinator_rpc_name,
)
from dimos.hosted.tags import as_tags, live_tags
from dimos.protocol.service.zenohservice import ZenohSessionPool, split_csv
from dimos.utils.logging_config import set_run_log_dir, setup_logger
from dimos.utils.safe_thread_map import safe_thread_map

if TYPE_CHECKING:
    import zenoh

    from dimos.protocol.rpc.zenohrpc import ZenohRPC

logger = setup_logger()


@dataclass(slots=True)
class _Deployment:
    fragment: HostFragment
    state: DeploymentState
    log_dir: Path
    process: BaseProcess


class HostDaemon:
    """Supervise independent deployment processes for multiple runs."""

    def __init__(
        self,
        config: HostConfig,
        *,
        host_id: str,
        revision: str,
        tags: Mapping[str, str] | None = None,
        log_root: Path = LOG_DIR,
        sync_checkout: Callable[[str, str], None] | None = None,
    ) -> None:
        if not config.listen_endpoints:
            raise ValueError("HostDaemon is a zenoh router and needs a listen endpoint")
        self.config = config
        self._sync_checkout = sync_checkout
        self.host_id = host_id
        self.revision = revision
        self.listen = config.listen_endpoints
        self.connect = split_csv(config.connect)
        self.router_zid = ""
        self.tags = {**(tags or {}), **as_tags(split_csv(config.tags))}
        self._log_root = log_root
        self._liveliness: zenoh.Liveliness | None = None
        self._process_context = multiprocessing.get_context("spawn")
        self._deployments: dict[str, _Deployment] = {}
        # Why a run's deployment died, reported by the next status call for it and forgotten.
        self._failed: dict[str, str] = {}
        self._lock = threading.RLock()
        self._closing = False
        self._pool = ZenohSessionPool()

    @property
    def client_endpoint(self) -> str:
        """The locator this daemon's fragments dial: its first listener, on loopback or its interface."""
        endpoint, _, options = self.listen[0].partition("#")
        protocol, _, address = endpoint.partition("/")
        host, _, port = address.rpartition(":")
        if not port or port == "0":
            raise ValueError(f"Listen endpoint {self.listen[0]!r} needs a fixed port")
        if host in ("0.0.0.0", "[::]", ""):
            iface = dict(o.partition("=")[::2] for o in options.split(";") if o).get("iface")
            host = _ipv4(iface) if iface else "127.0.0.1"
        return f"{protocol}/{host}:{port}"

    @contextmanager
    def serve(self) -> Iterator[ZenohRPC]:
        """Open this machine's zenoh router and serve the Host control plane on it."""
        from dimos.protocol.rpc.zenohrpc import ZenohRPC

        # connect_timeout 0: zenoh dials the connect routers itself, before and after any loss.
        rpc = ZenohRPC(
            session_pool=self._pool,
            mode="router",
            listen=self.listen,
            connect=self.connect,
            multicast=self.config.autodiscovery,
            scouting=self.config.autodiscovery,
            scouting_interface=self.config.scout_interface,
            scout_addr=self.config.scout_addr,
            multicast_autoconnect=self.config.autodiscovery,
            gossip_autoconnect=self.config.gossip,
            connect_timeout=0,
            # The routing test reads this router's subscriber table through it.
            adminspace=True,
        )
        self._closing = False
        with ExitStack() as cleanup:
            cleanup.callback(self._pool.close_all)
            rpc.start()
            cleanup.callback(rpc.stop)
            # Before rpc.stop: a start call in flight returns once its fragment is gone.
            cleanup.callback(self.shutdown)
            self.router_zid = str(rpc.session.zid())
            control_name = HOST_CONTROL_RPC_NAME.format(host_id=self.host_id)
            rpc.serve_rpc(self.describe, f"{control_name}/describe")
            rpc.serve_rpc(self.start, f"{control_name}/start")
            rpc.serve_rpc(self.status, f"{control_name}/status")
            rpc.serve_rpc(self.stop, f"{control_name}/stop")
            rpc.serve_rpc(self.sync, f"{control_name}/sync")
            self._liveliness = rpc.session.liveliness()
            token = self._liveliness.declare_token(HOST_LIVELINESS_KEY.format(host_id=self.host_id))
            cleanup.callback(token.undeclare)
            serving = threading.Event()
            cleanup.callback(serving.set)
            threading.Thread(target=self._watch_runs, args=(serving,), daemon=True).start()
            yield rpc

    def _watch_runs(self, stop: threading.Event) -> None:
        lost: dict[str, float] = {}
        while not stop.wait(min(self.config.run_loss_grace / 2, 5.0)):
            self._watch_tick(lost)

    def _watch_tick(self, lost: dict[str, float]) -> None:
        """Reap exited fragments; stop a run whose blueprint root has been gone for the grace period."""
        with self._lock:
            self._refresh_locked()
            run_ids = [r for r, d in self._deployments.items() if d.state == "running"]
        for run_id in run_ids:
            if self._root_alive(run_id):
                lost.pop(run_id, None)
            elif (
                time.monotonic() - lost.setdefault(run_id, time.monotonic())
                >= self.config.run_loss_grace
            ):
                logger.warning("Blueprint root gone, stopping its run", run_id=run_id)
                lost.pop(run_id)
                self._stop_run(run_id)

    def _root_alive(self, run_id: str) -> bool:
        """Whether the run's blueprint root still holds its token; a failed query counts as alive."""
        try:
            assert self._liveliness is not None
            replies = self._liveliness.get(RUN_LIVELINESS_KEY.format(run_id=run_id), timeout=2.0)
            return any(reply.ok is not None for reply in replies)
        except Exception:
            return True

    def run(self, stop: threading.Event) -> None:
        """Serve until ``stop``; zenoh links this router to scouted and configured ones."""
        with self.serve():
            logger.info(
                "Host up",
                name=self.config.name,
                host_id=self.host_id,
                tags=self.tags,
                listen=self.listen,
                connect=self.connect,
            )
            stop.wait()

    def describe(self) -> HostDescriptor:
        live = live_tags()
        with self._lock:
            self._refresh_locked()
            return HostDescriptor(
                host_id=self.host_id,
                name=self.config.name,
                tags={**self.tags, **live},
                revision=self.revision,
                active_run_ids=tuple(sorted(self._deployments)),
                router_zid=self.router_zid,
                listen=tuple(self.listen),
                start_timeout=self.start_timeout,
            )

    @property
    def start_timeout(self) -> float:
        """A start is answered by then: the fragment reported ready, or timed out and was killed."""
        return self.config.startup_timeout + 3 * self.config.stop_timeout

    def start(self, fragment: HostFragment) -> DeploymentStatus:
        """Start the idempotent deployment of ``fragment``, or report the one already there."""
        self._check_fragment(fragment)

        with self._lock:
            self._refresh_locked()
            if fragment.run_id in self._deployments:
                return self._status_locked(fragment.run_id)
            if self._closing:
                raise RuntimeError(f"Host {self.host_id} is shutting down")

            log_dir = self._log_root / fragment.run_id
            log_dir.mkdir(parents=True, exist_ok=True)
            receive_ready, send_ready = self._process_context.Pipe(duplex=False)
            process = self._process_context.Process(
                target=_run_fragment,
                args=(
                    fragment,
                    log_dir,
                    send_ready,
                    self.client_endpoint,
                    RUN_LOG_KEY.format(run_id=fragment.run_id, host=self.host_id),
                ),
                daemon=False,
            )
            deployment = _Deployment(fragment, "starting", log_dir, process)
            self._deployments[fragment.run_id] = deployment
            try:
                process.start()
            except Exception as exc:
                receive_ready.close()
                send_ready.close()
                self._fail_locked(deployment, str(exc))
                return self._status_locked(fragment.run_id)
            send_ready.close()

        error = self._wait_for_start(receive_ready)
        if error is not None:
            _terminate(process, self.config.stop_timeout)

        with self._lock:
            if self._deployments.get(fragment.run_id) is not deployment:
                return self._status_locked(fragment.run_id)
            if error is None and process.exitcode is not None:
                error = f"Deployment process exited with code {process.exitcode}"
            if error is not None:
                self._fail_locked(deployment, error)
            else:
                deployment.state = "running"
            return self._status_locked(fragment.run_id)

    def status(self, run_id: str) -> DeploymentStatus:
        with self._lock:
            self._refresh_locked()
            return self._status_locked(run_id)

    def stop(self, run_id: str) -> DeploymentStatus:
        return self._stop_run(run_id)

    def sync(self, branch: str, commit: str) -> str:
        """Check out ``commit`` and restart into it; refused while fragments run."""
        if self._sync_checkout is None:
            raise RuntimeError("This Host cannot sync its checkout")
        with self._lock:
            self._refresh_locked()
            if self._deployments:
                raise RuntimeError(f"Busy with runs {', '.join(sorted(self._deployments))}")
        self._sync_checkout(branch, commit)
        return commit

    def _stop_run(self, run_id: str) -> DeploymentStatus:
        with self._lock:
            deployment = self._deployments.get(run_id)
            if deployment is None or deployment.state == "stopping":
                return self._status_locked(run_id)
            deployment.state = "stopping"

        _terminate(deployment.process, self.config.stop_timeout)

        with self._lock:
            if self._deployments.get(run_id) is deployment:
                del self._deployments[run_id]
            return self._status_locked(run_id)

    def shutdown(self) -> None:
        """Stop every active deployment, side by side, when the Host service exits."""
        with self._lock:
            self._closing = True
            deployments = tuple(self._deployments.values())
            self._deployments.clear()
        safe_thread_map(deployments, lambda d: _terminate(d.process, self.config.stop_timeout))

    def _check_fragment(self, fragment: HostFragment) -> None:
        if fragment.host_id != self.host_id:
            raise ValueError(f"Fragment targets Host {fragment.host_id}, not {self.host_id}")
        if fragment.revision != self.revision:
            raise ValueError(
                f"Fragment requires revision {fragment.revision}, Host has {self.revision}"
            )

    def _wait_for_start(self, ready: Connection) -> str | None:
        try:
            if not ready.poll(self.config.startup_timeout):
                return f"Deployment startup timed out after {self.config.startup_timeout}s"
            started, error = ready.recv()
            return None if started else error or "Deployment failed during startup"
        except EOFError:
            return "Deployment process exited before reporting startup status"
        finally:
            ready.close()

    def _refresh_locked(self) -> None:
        """Drop running deployments whose process exited; a starting one is its start call's."""
        for deployment in list(self._deployments.values()):
            if deployment.state == "running" and deployment.process.exitcode is not None:
                self._fail_locked(
                    deployment, f"Deployment process exited with code {deployment.process.exitcode}"
                )

    def _fail_locked(self, deployment: _Deployment, error: str) -> None:
        del self._deployments[deployment.fragment.run_id]
        self._failed[deployment.fragment.run_id] = error

    def _status_locked(self, run_id: str) -> DeploymentStatus:
        deployment = self._deployments.get(run_id)
        if deployment is None:
            error = self._failed.pop(run_id, None)
            if error is None:
                return DeploymentStatus("absent", None, None, None, None)
            return DeploymentStatus("failed", run_id, None, None, error)
        return DeploymentStatus(
            state=deployment.state,
            run_id=deployment.fragment.run_id,
            pid=deployment.process.pid,
            log_dir=str(deployment.log_dir),
            error=None,
        )


def _ipv4(iface: str) -> str:
    """An interface's IPv4 address; a listener bound to it answers nowhere else."""
    for address in psutil.net_if_addrs().get(iface, ()):
        if address.family == socket.AF_INET:
            return str(address.address)
    raise ValueError(f"Interface {iface!r} has no IPv4 address to reach this Host on")


def _terminate(process: BaseProcess, timeout: float) -> None:
    """Stop a fragment process and every process under it."""
    if process.pid is None:
        return
    # ponytail: a fragment that already crashed has orphaned its children to init, so they are missed
    try:
        tree = psutil.Process(process.pid).children(recursive=True)
    except psutil.NoSuchProcess:
        tree = []
    if process.is_alive():
        process.terminate()
        process.join(timeout=timeout)
        if process.is_alive():
            process.kill()
            process.join(timeout=timeout)
    else:
        process.join(timeout=0)
    for child in tree:
        with contextlib.suppress(psutil.NoSuchProcess):
            child.terminate()
    _, alive = psutil.wait_procs(tree, timeout=timeout)
    for child in alive:
        with contextlib.suppress(psutil.NoSuchProcess):
            child.kill()


def _run_fragment(
    fragment: HostFragment,
    log_dir: Path,
    ready: Connection,
    router: str | None = None,
    log_key: str | None = None,
) -> None:
    """Run one fragment as zenoh clients of ``router``, teeing output onto ``log_key``."""
    set_run_log_dir(log_dir)
    try:
        if log_key is not None and router is not None:
            _publish_output(log_key, router)
        payload = fragment.load_payload()
        remote_module_refs = {
            (reference.consumer_name, reference.reference_name): (
                reference.rpcs,
                reference.rpc_name,
            )
            for reference in payload.remote_module_references
        }
        # This Host's own environment, the blueprint root's explicit values on top, the run's pins last.
        config = BlueprintConfigParser(payload.blueprint).parse(
            overrides=payload.overrides,
            global_overrides={**(payload.global_overrides or {}), **_pins(fragment.run_id, router)},
        )
        run_coordinator(
            payload.blueprint,
            config,
            run_id=fragment.run_id,
            log_dir=log_dir,
            rpc_name=run_coordinator_rpc_name(fragment.run_id, fragment.host_id),
            remote_module_refs=remote_module_refs,
            on_ready=lambda _: ready.send((True, None)),
        )
    except Exception as exc:
        with contextlib.suppress(OSError):
            ready.send((False, str(exc)))
        # Unwinds through multiprocessing, which prints the traceback and exits 1.
        raise
    finally:
        ready.close()
        sys.stdout.flush()
        sys.stderr.flush()
        # ponytail: fixed grace for the tee thread to publish the last lines.
        time.sleep(0.3)


def _pins(run_id: str, router: str | None) -> dict[str, Any]:
    """Globals every fragment of a run gets last: zenoh in the run's domain, via its Host."""
    pins: dict[str, Any] = {"transport": "zenoh", "zenoh_domain": run_id}
    if router is not None:
        pins.update(
            zenoh_mode="client", zenoh_connect=router, zenoh_multicast=False, zenoh_scouting=False
        )
    return pins


def _publish_output(key: str, router: str) -> None:
    """Tee stdout and stderr, inherited by every worker, onto ``key``; the original still gets all."""
    from dimos.protocol.service.zenohservice import ZenohConfig, default_session_pool

    session = default_session_pool.acquire(
        ZenohConfig(mode="client", connect=[router], multicast=False, scouting=False)
    )
    publisher = session.declare_publisher(key)
    original = os.dup(1)
    read_end, write_end = os.pipe()
    sys.stdout.flush()
    sys.stderr.flush()
    os.dup2(write_end, 1)
    os.dup2(write_end, 2)
    os.close(write_end)
    sys.stdout.reconfigure(line_buffering=True)  # type: ignore[union-attr]
    sys.stderr.reconfigure(line_buffering=True)  # type: ignore[union-attr]

    def tee() -> None:
        with os.fdopen(read_end, "rb") as lines:
            for line in lines:
                with contextlib.suppress(Exception):
                    os.write(original, line)
                with contextlib.suppress(Exception):
                    publisher.put(line)

    threading.Thread(target=tee, name="fragment-log-tee", daemon=True).start()
