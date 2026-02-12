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

import multiprocessing as mp
from multiprocessing.connection import Connection
import traceback
from typing import Any

from dimos.core.module import ModuleT
from dimos.core.rpc_client import RPCClient
from dimos.utils.actor_registry import ActorRegistry
from dimos.utils.logging_config import setup_logger
from dimos.utils.sequential_ids import SequentialIds

logger = setup_logger()


class ActorFuture:
    """Mimics Dask's ActorFuture - wraps a result with .result() method."""

    def __init__(self, value: Any) -> None:
        self._value = value

    def result(self, _timeout: float | None = None) -> Any:
        return self._value


class Actor:
    """Proxy that forwards method calls to the worker process."""

    def __init__(
        self, conn: Connection | None, module_class: type[ModuleT], worker_id: int
    ) -> None:
        self._conn = conn
        self._cls = module_class
        self._worker_id = worker_id

    def __reduce__(self) -> tuple[type, tuple[None, type, int]]:
        """Exclude the connection when pickling - it can't be used in other processes."""
        return (Actor, (None, self._cls, self._worker_id))

    def _send_request_to_worker(self, request: dict[str, Any]) -> Any:
        if self._conn is None:
            raise RuntimeError("Actor connection not available - cannot send requests")
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
        _forkserver_ctx = mp.get_context("forkserver")
    return _forkserver_ctx


def reset_forkserver_context() -> None:
    """Reset the forkserver context. Used in tests to ensure clean state."""
    global _forkserver_ctx
    _forkserver_ctx = None


_seq_ids = SequentialIds()


class Worker:
    def __init__(
        self,
        module_class: type[ModuleT],
        args: tuple[Any, ...] = (),
        kwargs: dict[Any, Any] | None = None,
    ) -> None:
        self._module_class: type[ModuleT] = module_class
        self._args: tuple[Any, ...] = args
        self._kwargs: dict[Any, Any] = kwargs or {}
        self._process: Any = None
        self._conn: Connection | None = None
        self._actor: Actor | None = None
        self._worker_id: int = _seq_ids.next()
        self._ready: bool = False

    def start_process(self) -> None:
        ctx = get_forkserver_context()
        parent_conn, child_conn = ctx.Pipe()
        self._conn = parent_conn

        self._process = ctx.Process(
            target=_worker_entrypoint,
            args=(child_conn, self._module_class, self._args, self._kwargs, self._worker_id),
            daemon=True,
        )
        self._process.start()
        self._actor = Actor(parent_conn, self._module_class, self._worker_id)

    def wait_until_ready(self) -> None:
        if self._ready:
            return
        if self._actor is None:
            raise RuntimeError("Worker process not started")

        worker_id = self._actor.set_ref(self._actor).result()
        ActorRegistry.update(str(self._actor), str(worker_id))
        self._ready = True

        logger.info(
            "Deployed module.", module=self._module_class.__name__, worker_id=self._worker_id
        )

    def deploy(self) -> None:
        self.start_process()
        self.wait_until_ready()

    def get_instance(self) -> RPCClient:
        if self._actor is None:
            raise RuntimeError("Worker not deployed")
        return RPCClient(self._actor, self._module_class)

    def shutdown(self) -> None:
        if self._conn is not None:
            try:
                self._conn.send({"type": "shutdown"})
                self._conn.recv()
            except (BrokenPipeError, EOFError):
                pass
            finally:
                self._conn.close()
                self._conn = None

        if self._process is not None:
            self._process.join(timeout=2)
            if self._process.is_alive():
                self._process.terminate()
                self._process.join(timeout=1)
            self._process = None


def _worker_entrypoint(
    conn: Connection,
    module_class: type[ModuleT],
    args: tuple[Any, ...],
    kwargs: dict[Any, Any],
    worker_id: int,
) -> None:
    instance = None

    try:
        instance = module_class(*args, **kwargs)
        instance.worker = worker_id

        _worker_loop(conn, instance, worker_id)
    except Exception as e:
        logger.error(f"Worker process error: {e}", exc_info=True)
    finally:
        if instance is not None:
            try:
                instance.stop()
            except Exception:
                logger.error("Error during worker shutdown", exc_info=True)


def _worker_loop(conn: Connection, instance: Any, worker_id: int) -> None:
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

            if req_type == "set_ref":
                instance.ref = request.get("ref")
                response["result"] = worker_id

            elif req_type == "getattr":
                response["result"] = getattr(instance, request["name"])

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
