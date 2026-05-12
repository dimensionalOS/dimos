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
import logging
import multiprocessing
from multiprocessing.connection import Connection
import os
import signal
import sys
import threading
import time
import traceback
from typing import TYPE_CHECKING, Any

from rpyc.utils.server import ThreadedServer

from dimos.core.coordination.rpyc_services import WorkerRpycService
from dimos.core.coordination.worker_messages import (
    CallMethodRequest,
    DeployModuleRequest,
    GetAttrRequest,
    SetRefRequest,
    ShutdownRequest,
    StartRpycRequest,
    SuppressConsoleRequest,
    UndeployModuleRequest,
    WorkerRequest,
    WorkerResponse,
)
from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.library_config import apply_library_config
from dimos.utils.logging_config import setup_logger
from dimos.utils.sequential_ids import SequentialIds

if TYPE_CHECKING:
    from dimos.core.module import ModuleBase

logger = setup_logger()


class ActorFuture:
    """Mimics Dask's ActorFuture - wraps a result with .result() method."""

    def __init__(self, value: Any) -> None:
        self._value = value

    def result(self, _timeout: float | None = None) -> Any:
        return self._value


class MethodCallProxy:
    """Proxy that wraps an Actor to support method calls returning ActorFuture.

    Used as the owner of RemoteOut/RemoteIn on the parent side so that calls like
    `owner.set_transport(name, value).result()` work through the pipe to the worker.
    """

    def __init__(self, actor: Actor) -> None:
        self._actor = actor

    def __reduce__(self) -> tuple[type, tuple[Actor]]:
        return (MethodCallProxy, (self._actor,))

    def __getattr__(self, name: str) -> Any:
        # Don't intercept private/dunder attributes - they must follow normal lookup.
        if name.startswith("_"):
            raise AttributeError(name)

        def _call(*args: Any, **kwargs: Any) -> ActorFuture:
            result = self._actor._send_request_to_worker(
                CallMethodRequest(
                    module_id=self._actor._module_id,
                    name=name,
                    args=args,
                    kwargs=kwargs,
                )
            )
            return ActorFuture(result)

        return _call


class Actor:
    """Proxy that forwards method calls to the worker process."""

    def __init__(
        self,
        conn: Connection | None,
        module_class: type[ModuleBase],
        worker_id: int,
        module_id: int = 0,
        lock: threading.Lock | None = None,
    ) -> None:
        self._conn = conn
        self._cls = module_class
        self._worker_id = worker_id
        self._module_id = module_id
        self._lock = lock

    def __reduce__(self) -> tuple[type, tuple[None, type, int, int, None]]:
        """Exclude the connection and lock when pickling."""
        return (Actor, (None, self._cls, self._worker_id, self._module_id, None))

    def _send_request_to_worker(self, request: WorkerRequest) -> Any:
        if self._conn is None:
            raise RuntimeError("Actor connection not available - cannot send requests")
        if self._lock is not None:
            with self._lock:
                self._conn.send(request)
                response: WorkerResponse = self._conn.recv()
        else:
            self._conn.send(request)
            response = self._conn.recv()
        if response.error:
            if "AttributeError" in response.error:  # TODO: better error handling
                raise AttributeError(response.error)
            raise RuntimeError(f"Worker error: {response.error}")
        return response.result

    def set_ref(self, ref: Any) -> ActorFuture:
        """Set the actor reference on the remote module."""
        result = self._send_request_to_worker(SetRefRequest(module_id=self._module_id, ref=ref))
        return ActorFuture(result)

    def start_rpyc(self) -> int:
        port: int = self._send_request_to_worker(StartRpycRequest())
        return port

    def __getattr__(self, name: str) -> Any:
        """Proxy attribute access to the worker process."""
        if name.startswith("_"):
            raise AttributeError(f"'{type(self).__name__}' object has no attribute '{name}'")

        return self._send_request_to_worker(GetAttrRequest(module_id=self._module_id, name=name))


# Global forkserver context. Using `forkserver` instead of `fork` because it
# avoids CUDA context corruption issues.
_forkserver_ctx: Any = None


def get_forkserver_context() -> Any:
    global _forkserver_ctx
    if _forkserver_ctx is None:
        _forkserver_ctx = multiprocessing.get_context("forkserver")
    return _forkserver_ctx


def reset_forkserver_context() -> None:
    """Reset the forkserver context. Used in tests to ensure clean state."""
    global _forkserver_ctx
    _forkserver_ctx = None


_worker_ids = SequentialIds()
_module_ids = SequentialIds()


class PythonWorker:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._modules: dict[int, Actor] = {}
        self._reserved: int = 0
        self._process: Any = None
        self._conn: Connection | None = None
        self._worker_id: int = _worker_ids.next()

    @property
    def module_count(self) -> int:
        return len(self._modules) + self._reserved

    @property
    def pid(self) -> int | None:
        """PID of the worker process, or ``None`` if not alive."""
        if self._process is None:
            return None
        try:
            # Signal 0 just checks if the process is alive.
            pid: int | None = self._process.pid
            if pid is None:
                return None
            os.kill(pid, 0)
            return pid
        except OSError:
            return None

    @property
    def worker_id(self) -> int:
        return self._worker_id

    @property
    def module_names(self) -> list[str]:
        return [actor._cls.__name__ for actor in self._modules.values()]

    def reserve_slot(self) -> None:
        """Reserve a slot so _select_worker() sees the pending load."""
        self._reserved += 1

    def start_process(self) -> None:
        ctx = get_forkserver_context()
        parent_conn, child_conn = ctx.Pipe()
        self._conn = parent_conn

        self._process = ctx.Process(
            target=_worker_entrypoint,
            args=(child_conn, self._worker_id),
            daemon=True,
        )
        self._process.start()

    def deploy_module(
        self,
        module_class: type[ModuleBase],
        global_config: GlobalConfig = global_config,
        kwargs: dict[str, Any] | None = None,
    ) -> Actor:
        if self._conn is None:
            raise RuntimeError("Worker process not started")

        kwargs = kwargs or {}
        kwargs["g"] = global_config
        module_id = _module_ids.next()

        request = DeployModuleRequest(module_id=module_id, module_class=module_class, kwargs=kwargs)
        try:
            with self._lock:
                self._conn.send(request)
                response: WorkerResponse = self._conn.recv()

            if response.error:
                raise RuntimeError(f"Failed to deploy module: {response.error}")

            actor = Actor(self._conn, module_class, self._worker_id, module_id, self._lock)
            actor.set_ref(actor).result()

            self._modules[module_id] = actor
            logger.info(
                "Deployed module.",
                module=module_class.__name__,
                worker_id=self._worker_id,
                module_id=module_id,
            )
            return actor
        finally:
            self._reserved = max(0, self._reserved - 1)

    def undeploy_module(self, module_id: int) -> None:
        """Stop and remove a single module from the worker process."""
        if self._conn is None:
            raise RuntimeError("Worker process not started")

        with self._lock:
            self._conn.send(UndeployModuleRequest(module_id=module_id))
            response: WorkerResponse = self._conn.recv()

        if response.error:
            raise RuntimeError(f"Failed to undeploy module: {response.error}")

        self._modules.pop(module_id, None)

    def suppress_console(self) -> None:
        if self._conn is None:
            return
        try:
            with self._lock:
                self._conn.send(SuppressConsoleRequest())
                self._conn.recv()
        except (BrokenPipeError, EOFError, ConnectionResetError):
            pass

    def shutdown(self) -> None:
        if self._conn is not None:
            try:
                with self._lock:
                    self._conn.send(ShutdownRequest())
                    if self._conn.poll(timeout=5):
                        self._conn.recv()
                    else:
                        logger.warning(
                            "Worker did not respond to shutdown within 5s, closing pipe.",
                            worker_id=self._worker_id,
                        )
            except (BrokenPipeError, EOFError, ConnectionResetError):
                pass
            finally:
                self._conn.close()
                self._conn = None

        if self._process is not None:
            self._process.join(timeout=5)
            if self._process.is_alive():
                logger.warning(
                    "Worker still alive after 5s, terminating.",
                    worker_id=self._worker_id,
                )
                self._process.terminate()
                self._process.join(timeout=1)
            self._process = None


def _suppress_console_output() -> None:
    """Redirect stdout/stderr to /dev/null and strip console handlers."""
    devnull = open(os.devnull, "w")
    os.dup2(devnull.fileno(), sys.stdout.fileno())
    os.dup2(devnull.fileno(), sys.stderr.fileno())
    devnull.close()

    # Remove StreamHandlers.
    for name in list(logging.Logger.manager.loggerDict):
        lg = logging.getLogger(name)
        lg.handlers = [
            h
            for h in lg.handlers
            if not isinstance(h, logging.StreamHandler) or isinstance(h, logging.FileHandler)
        ]


@dataclass
class _WorkerState:
    instances: dict[int, Any]
    worker_id: int
    rpyc_server: ThreadedServer | None = None
    rpyc_thread: threading.Thread | None = None
    should_stop: bool = False


def _worker_entrypoint(conn: Connection, worker_id: int) -> None:
    apply_library_config()
    signal.signal(signal.SIGINT, signal.SIG_IGN)  # coordinator handles shutdown
    state = _WorkerState(instances={}, worker_id=worker_id)

    try:
        _worker_loop(conn, state)
    except Exception as e:
        # `f"... {e}"` is often empty (some exceptions stringify to "")
        # and structlog's `exc_info` rendering may be elided in captured
        # CI logs. Always emit the full traceback to stderr so the cause
        # is visible regardless of the structured-log handler config.
        traceback.print_exception(e, file=sys.stderr)
        sys.stderr.flush()
        logger.error(
            "Worker process error",
            worker_id=worker_id,
            error_type=type(e).__name__,
            error_repr=repr(e),
            exc_info=True,
        )
    finally:
        for module_id, instance in reversed(list(state.instances.items())):
            try:
                logger.info(
                    "Worker stopping module...",
                    module=type(instance).__name__,
                    worker_id=worker_id,
                    module_id=module_id,
                )
                instance.stop()
                logger.info(
                    "Worker module stopped.",
                    module=type(instance).__name__,
                    worker_id=worker_id,
                    module_id=module_id,
                )
            except Exception:
                logger.error("Error during worker shutdown", exc_info=True)


def _handle_request(request: Any, state: _WorkerState) -> WorkerResponse:
    match request:
        case DeployModuleRequest(module_id=module_id, module_class=module_class, kwargs=kwargs):
            state.instances[module_id] = module_class(**kwargs)
            return WorkerResponse(result=module_id)

        case SetRefRequest(module_id=module_id, ref=ref):
            state.instances[module_id].ref = ref
            return WorkerResponse(result=state.worker_id)

        case GetAttrRequest(module_id=module_id, name=name):
            return WorkerResponse(result=getattr(state.instances[module_id], name))

        case CallMethodRequest(module_id=module_id, name=name, args=args, kwargs=kwargs):
            method = getattr(state.instances[module_id], name)
            return WorkerResponse(result=method(*args, **kwargs))

        case UndeployModuleRequest(module_id=module_id):
            instance = state.instances.pop(module_id, None)
            if instance is not None:
                instance.stop()
            return WorkerResponse(result=True)

        case SuppressConsoleRequest():
            _suppress_console_output()
            return WorkerResponse(result=True)

        case StartRpycRequest():
            if state.rpyc_server is not None:
                return WorkerResponse(result=state.rpyc_server.port)
            WorkerRpycService._instances = state.instances
            state.rpyc_server = ThreadedServer(
                WorkerRpycService,
                port=0,
                hostname=global_config.listen_host,
                protocol_config={
                    "allow_all_attrs": True,
                    "allow_public_attrs": True,
                    "allow_pickle": True,
                },
            )
            # `ThreadedServer.__init__` binds the socket but `listen()` only
            # runs once `start()` executes on the thread, which sets
            # `active=True` immediately after. Wait on that flag so callers
            # never see a Connection refused before the accept loop is live.
            state.rpyc_thread = threading.Thread(target=state.rpyc_server.start, daemon=True)
            state.rpyc_thread.start()
            deadline = time.monotonic() + 5.0
            while not state.rpyc_server.active:
                if not state.rpyc_thread.is_alive():
                    raise RuntimeError("rpyc server thread died before listening")
                if time.monotonic() > deadline:
                    raise RuntimeError("rpyc server failed to start listening within 5s")
                time.sleep(0.001)
            return WorkerResponse(result=state.rpyc_server.port)

        case ShutdownRequest():
            if state.rpyc_server is not None:
                state.rpyc_server.close()
                if state.rpyc_thread is not None:
                    state.rpyc_thread.join(timeout=5)
            state.should_stop = True
            return WorkerResponse(result=True)

        case _:
            return WorkerResponse(error=f"Unknown request type: {type(request)}")


def _worker_loop(conn: Connection, state: _WorkerState) -> None:
    while True:
        try:
            if not conn.poll(timeout=0.1):
                continue
            request = conn.recv()
        except EOFError:
            break

        try:
            response = _handle_request(request, state)
        except Exception as e:
            response = WorkerResponse(
                error=f"{e.__class__.__name__}: {e}\n{traceback.format_exc()}"
            )

        try:
            conn.send(response)
        except (BrokenPipeError, EOFError):
            break

        if state.should_stop:
            break
