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

from concurrent.futures import ThreadPoolExecutor
from itertools import count
import pickle
import threading
import time
from typing import TYPE_CHECKING, Any

import zenoh
import zenoh.handlers

from dimos.protocol.rpc.rpc_utils import deserialize_exception, serialize_exception
from dimos.protocol.rpc.spec import DEFAULT_RPC_TIMEOUT, DEFAULT_RPC_TIMEOUTS, Args, RPCSpec
from dimos.protocol.service.zenohservice import ZenohService
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable
    from types import FunctionType

logger = setup_logger()

EXCEPTION_ENCODING = zenoh.Encoding("dimos/rpc-exception")


class ZenohRPC(RPCSpec, ZenohService):
    def __init__(
        self,
        rpc_timeouts: dict[str, float] | None = None,
        default_rpc_timeout: float = DEFAULT_RPC_TIMEOUT,
        **kwargs: Any,
    ) -> None:
        if rpc_timeouts is None:
            rpc_timeouts = dict(DEFAULT_RPC_TIMEOUTS)
        ZenohService.__init__(self, **kwargs)
        self.rpc_timeouts = dict(rpc_timeouts)
        self.default_rpc_timeout = default_rpc_timeout
        self._call_thread_pool: ThreadPoolExecutor | None = None
        self._call_thread_pool_lock = threading.RLock()
        self._call_thread_pool_max_workers = 50
        self._queryables: list[zenoh.Queryable[None]] = []
        self._pending: dict[int, Callable[..., Any]] = {}
        self._call_counter = count(1)

    def __getstate__(self):  # type: ignore[no-untyped-def]
        """Drop the live queryables, call pool and in-flight calls on top of the session.

        A module carries its RPC service into the worker by pickle, and none of
        these survive the trip; the far side rebuilds them on first use.
        """
        state = super().__getstate__()  # type: ignore[no-untyped-call]
        for key in ("_call_thread_pool", "_call_thread_pool_lock", "_queryables", "_pending"):
            state.pop(key, None)
        return state

    def __setstate__(self, state) -> None:  # type: ignore[no-untyped-def]
        super().__setstate__(state)
        self._call_thread_pool = None
        self._call_thread_pool_lock = threading.RLock()
        self._queryables = []
        self._pending = {}

    def _get_call_thread_pool(self) -> ThreadPoolExecutor:
        """Get or create the thread pool for RPC handler execution (lazy initialization)."""
        with self._call_thread_pool_lock:
            if self._call_thread_pool is None:
                self._call_thread_pool = ThreadPoolExecutor(
                    max_workers=self._call_thread_pool_max_workers
                )
            return self._call_thread_pool

    def _shutdown_thread_pool(self) -> None:
        """Safely shutdown the thread pool with deadlock prevention."""
        with self._call_thread_pool_lock:
            if self._call_thread_pool:
                # Check if we're being called from within the thread pool
                # to avoid "cannot join current thread" error
                current_thread = threading.current_thread()
                is_pool_thread = False

                # Check if current thread is one of the pool's threads
                if hasattr(self._call_thread_pool, "_threads"):
                    is_pool_thread = current_thread in self._call_thread_pool._threads
                elif "ThreadPoolExecutor" in current_thread.name:
                    # Fallback: check thread name pattern
                    is_pool_thread = True

                # Don't wait if we're in a pool thread to avoid deadlock
                self._call_thread_pool.shutdown(wait=not is_pool_thread)
                self._call_thread_pool = None

    def call(self, name: str, arguments: Args, cb: Callable | None):  # type: ignore[no-untyped-def, type-arg]
        if cb is None:
            return self.call_nowait(name, arguments)
        return self.call_cb(name, arguments, cb)

    def call_cb(self, name: str, arguments: Args, cb: Callable[..., Any]) -> Callable[[], None]:
        method = name.rsplit("/", 1)[-1]
        timeout = self.rpc_timeouts.get(name) or self.rpc_timeouts.get(
            method, self.default_rpc_timeout
        )
        call_id = next(self._call_counter)
        self._pending[call_id] = cb
        self._issue_query(
            call_id, f"dimos/rpc/{name}", pickle.dumps(arguments), time.monotonic() + timeout
        )

        def unsubscribe_callback() -> None:
            self._pending.pop(call_id, None)

        return unsubscribe_callback

    def _issue_query(self, call_id: int, key: str, payload: bytes, deadline: float) -> None:
        def on_reply(reply: zenoh.Reply) -> None:
            err = reply.err
            if err is None:
                cb = self._pending.pop(call_id, None)
                if cb is not None:
                    cb(pickle.loads(reply.ok.payload.to_bytes()))  # type: ignore[union-attr]
                return
            if err.encoding == EXCEPTION_ENCODING:
                cb = self._pending.pop(call_id, None)
                if cb is not None:
                    cb(deserialize_exception(pickle.loads(err.payload.to_bytes())))
                return
            self._pending.pop(call_id, None)

        def on_finalize() -> None:
            if call_id not in self._pending:
                return
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                self._pending.pop(call_id, None)
                return
            time.sleep(min(0.05, remaining))
            if call_id in self._pending:
                self._issue_query(call_id, key, payload, deadline)

        self.session.get(
            key,
            zenoh.handlers.Callback(on_reply, drop=on_finalize),
            target=zenoh.QueryTarget.ALL,
            consolidation=zenoh.ConsolidationMode.NONE,
            congestion_control=zenoh.CongestionControl.BLOCK,
            timeout=max(deadline - time.monotonic(), 0.001),
            payload=payload,
        )

    def call_sync(
        self, name: str, arguments: Args, rpc_timeout: float | None = None
    ) -> tuple[Any, Callable[[], None]]:
        if rpc_timeout is None:
            method = name.rsplit("/", 1)[-1]
            rpc_timeout = self.rpc_timeouts.get(name) or self.rpc_timeouts.get(
                method, self.default_rpc_timeout
            )
        deadline = time.monotonic() + rpc_timeout
        self._wait_for_queryable(f"dimos/rpc/{name}", deadline)
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(f"RPC call to '{name}' timed out after {rpc_timeout} seconds")
        return super().call_sync(name, arguments, rpc_timeout=remaining)

    def _wait_for_queryable(self, key: str, deadline: float) -> None:
        matched = threading.Event()
        remaining = max(deadline - time.monotonic(), 0.001)
        querier = self.session.declare_querier(
            key,
            target=zenoh.QueryTarget.ALL,
            consolidation=zenoh.ConsolidationMode.NONE,
            congestion_control=zenoh.CongestionControl.BLOCK,
            timeout=remaining,
        )
        listener = querier.declare_matching_listener(
            lambda status: matched.set() if status.matching else None
        )
        try:
            if querier.matching_status.matching:  # type: ignore[attr-defined]
                matched.set()
            if not matched.wait(max(deadline - time.monotonic(), 0)):
                raise TimeoutError(f"RPC call to '{key}' timed out waiting for a Zenoh queryable")
        finally:
            listener.undeclare()
            querier.undeclare()

    def call_nowait(self, name: str, arguments: Args) -> None:
        method = name.rsplit("/", 1)[-1]
        timeout = self.rpc_timeouts.get(name) or self.rpc_timeouts.get(
            method, self.default_rpc_timeout
        )
        self.session.get(
            f"dimos/rpc/{name}",
            lambda _: None,
            target=zenoh.QueryTarget.ALL,
            consolidation=zenoh.ConsolidationMode.NONE,
            congestion_control=zenoh.CongestionControl.BLOCK,
            timeout=timeout,
            payload=pickle.dumps(arguments),
            attachment=b"nowait",
        )

    def serve_rpc(self, f: FunctionType, name: str | None = None):  # type: ignore[no-untyped-def, override]
        rpc_name = name or f.__name__

        def on_query(query: zenoh.Query) -> None:
            self._get_call_thread_pool().submit(self._execute_rpc, f, rpc_name, query)

        queryable = self.session.declare_queryable(f"dimos/rpc/{rpc_name}", on_query, complete=True)
        self._queryables.append(queryable)

        def unsubscribe() -> None:
            self._queryables.remove(queryable)
            queryable.undeclare()

        return unsubscribe

    def _execute_rpc(self, f: Callable[..., Any], name: str, query: zenoh.Query) -> None:
        args = pickle.loads(query.payload.to_bytes())  # type: ignore[union-attr]
        nowait = query.attachment is not None
        try:
            response = f(*args[0], **args[1])
            if not nowait:
                query.reply(query.key_expr, pickle.dumps(response))
        except Exception as e:
            logger.exception(f"Exception in RPC handler for {name}: {e}", exc_info=e)
            if not nowait:
                query.reply_err(pickle.dumps(serialize_exception(e)), encoding=EXCEPTION_ENCODING)
        query.drop()

    def stop(self) -> None:
        self._shutdown_thread_pool()
        queryables = self._queryables
        self._queryables = []
        for queryable in queryables:
            queryable.undeclare()
        self._pending.clear()
        super().stop()
