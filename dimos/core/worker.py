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

import logging
import multiprocessing
import os
import sys
import threading
import traceback
from typing import TYPE_CHECKING, Any

from dimos.utils.logging_config import setup_logger
from dimos.utils.sequential_ids import SequentialIds

if TYPE_CHECKING:
    from multiprocessing.connection import Connection

    from dimos.core.module import ModuleT

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
                {"type": "call_method", "name": name, "args": args, "kwargs": kwargs}
            )
            return ActorFuture(result)

        return _call


class Actor:
    """Proxy that forwards method calls to the worker process."""

    def __init__(
        self,
        conn: Connection | None,
        module_class: type[ModuleT],
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

    def _send_request_to_worker(self, request: dict[str, Any]) -> Any:
        if self._conn is None:
            raise RuntimeError("Actor connection not available - cannot send requests")
        request["module_id"] = self._module_id
        if self._lock is not None:
            with self._lock:
                self._conn.send(request)
                response = self._conn.recv()
        else:
            self._conn.send(request)
            response = self._conn.recv()
        if response.get("error"):
            if "AttributeError" in response["error"]:  # TODO: better error handling
                raise AttributeError(response["error"])
            raise RuntimeError(f"Worker error: {response['error']}")
        return response.get("result")

    def set_ref(self, ref: Any) -> ActorFuture:
        """Set the actor reference on the remote module."""
        result = self._send_request_to_worker({"type": "set_ref", "ref": ref})
        return ActorFuture(result)

    def __getattr__(self, name: str) -> Any:
        """Proxy attribute access to the worker process."""
        if name.startswith("_"):
            raise AttributeError(f"'{type(self).__name__}' object has no attribute '{name}'")

        return self._send_request_to_worker({"type": "getattr", "name": name})


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


class Worker:
    """Generic worker process that can host multiple modules."""

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
        module_class: type[ModuleT],
        args: tuple[Any, ...] = (),
        kwargs: dict[Any, Any] | None = None,
    ) -> Actor:
        if self._conn is None:
            raise RuntimeError("Worker process not started")

        kwargs = kwargs or {}
        module_id = _module_ids.next()

        # Send deploy_module request to the worker process
        request = {
            "type": "deploy_module",
            "module_id": module_id,
            "module_class": module_class,
            "args": args,
            "kwargs": kwargs,
        }
        with self._lock:
            self._conn.send(request)
            response = self._conn.recv()

        if response.get("error"):
            raise RuntimeError(f"Failed to deploy module: {response['error']}")

        actor = Actor(self._conn, module_class, self._worker_id, module_id, self._lock)
        actor.set_ref(actor).result()

        self._modules[module_id] = actor
        self._reserved = max(0, self._reserved - 1)
        logger.info(
            "Deployed module.",
            module=module_class.__name__,
            worker_id=self._worker_id,
            module_id=module_id,
        )
        return actor

    def suppress_console(self) -> None:
        if self._conn is None:
            return
        try:
            with self._lock:
                self._conn.send({"type": "suppress_console"})
                self._conn.recv()
        except (BrokenPipeError, EOFError, ConnectionResetError):
            pass

    def shutdown(self) -> None:
        if self._conn is not None:
            try:
                with self._lock:
                    self._conn.send({"type": "shutdown"})
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


def _worker_entrypoint(
    conn: Connection,
    worker_id: int,
) -> None:
    instances: dict[int, Any] = {}

    try:
        _worker_loop(conn, instances, worker_id)
    except KeyboardInterrupt:
        logger.info("Worker got KeyboardInterrupt.", worker_id=worker_id)
    except Exception as e:
        logger.error(f"Worker process error: {e}", exc_info=True)
    finally:
        for module_id, instance in reversed(list(instances.items())):
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
            except KeyboardInterrupt:
                logger.warning(
                    "KeyboardInterrupt during worker stop",
                    module=type(instance).__name__,
                    worker_id=worker_id,
                )
            except Exception:
                logger.error("Error during worker shutdown", exc_info=True)


def _worker_loop(conn: Connection, instances: dict[int, Any], worker_id: int) -> None:
    while True:
        try:
            if not conn.poll(timeout=0.1):
                continue
            request = conn.recv()
        except (EOFError, KeyboardInterrupt):
            break

        response: dict[str, Any] = {}
        try:
            req_type = request.get("type")

            if req_type == "deploy_module":
                module_class = request["module_class"]
                args = request.get("args", ())
                kwargs = request.get("kwargs", {})
                module_id = request["module_id"]
                instance = module_class(*args, **kwargs)
                instances[module_id] = instance
                response["result"] = module_id

            elif req_type == "set_ref":
                module_id = request["module_id"]
                instances[module_id].ref = request.get("ref")
                response["result"] = worker_id

            elif req_type == "getattr":
                module_id = request["module_id"]
                response["result"] = getattr(instances[module_id], request["name"])

            elif req_type == "call_method":
                module_id = request["module_id"]
                method = getattr(instances[module_id], request["name"])
                result = method(*request.get("args", ()), **request.get("kwargs", {}))
                response["result"] = result

            elif req_type == "suppress_console":
                _suppress_console_output()
                response["result"] = True

            elif req_type == "shutdown":
                response["result"] = True
                conn.send(response)
                break

            else:
                response["error"] = f"Unknown request type: {req_type}"

        except Exception as e:
            response["error"] = f"{e.__class__.__name__}: {e}\n{traceback.format_exc()}"

        try:
            conn.send(response)
        except (BrokenPipeError, EOFError):
            break
