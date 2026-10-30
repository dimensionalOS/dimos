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
import time
from typing import TYPE_CHECKING, Any

import zenoh
import zenoh.handlers

from dimos.protocol.encode.cbor import decode, encode
from dimos.protocol.rpc.rpc_utils import deserialize_exception, serialize_exception
from dimos.protocol.rpc.spec import DEFAULT_RPC_TIMEOUT, DEFAULT_RPC_TIMEOUTS, Args, RPCSpec
from dimos.protocol.service.call_pool import CallPool
from dimos.protocol.service.zenohservice import ZenohService
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable

logger = setup_logger()

EXCEPTION_ENCODING = zenoh.Encoding("dimos/rpc-exception")


def _decode_reply(payload: bytes, convert: Callable[[Any], Any]) -> Any:
    """The decoded reply, or the failure itself so the caller raises instead of timing out."""
    try:
        return convert(decode(payload))
    except Exception as e:
        return e


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
        self._call_pool = CallPool(max_workers=50)
        self._queryables: list[zenoh.Queryable[None]] = []
        self._pending: dict[int, Callable[..., Any]] = {}
        self._call_counter = count(1)

    def _get_call_thread_pool(self) -> ThreadPoolExecutor:
        return self._call_pool.get()

    def start(self) -> None:
        with self._call_pool.lock:
            self._call_pool.start()
            super().start()

    def call(self, name: str, arguments: Args, cb: Callable | None):  # type: ignore[no-untyped-def, type-arg]
        if cb is None:
            return self.call_nowait(name, arguments)
        return self.call_cb(name, arguments, cb)

    def call_cb(self, name: str, arguments: Args, cb: Callable[..., Any]) -> Callable[[], None]:
        method = name.rsplit("/", 1)[-1]
        timeout = self.rpc_timeouts.get(name) or self.rpc_timeouts.get(
            method, self.default_rpc_timeout
        )
        # Encode before registering the callback: an unencodable argument raises
        # to the caller and leaves nothing pending.
        payload = encode(arguments)
        call_id = next(self._call_counter)
        self._pending[call_id] = cb
        self._issue_query(call_id, f"dimos/rpc/{name}", payload, time.monotonic() + timeout)

        def unsubscribe_callback() -> None:
            self._pending.pop(call_id, None)

        return unsubscribe_callback

    def _issue_query(self, call_id: int, key: str, payload: bytes, deadline: float) -> None:
        def on_reply(reply: zenoh.Reply) -> None:
            err = reply.err
            if err is None:
                cb = self._pending.pop(call_id, None)
                if cb is not None:
                    cb(_decode_reply(reply.ok.payload.to_bytes(), lambda value: value))  # type: ignore[union-attr]
                return
            if err.encoding == EXCEPTION_ENCODING:
                cb = self._pending.pop(call_id, None)
                if cb is not None:
                    cb(_decode_reply(err.payload.to_bytes(), deserialize_exception))
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
            payload=encode(arguments),
            attachment=b"nowait",
        )

    def serve_rpc(self, f: Callable[..., Any], name: str | None = None):  # type: ignore[no-untyped-def, override]
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
        nowait = query.attachment is not None
        try:
            args = decode(query.payload.to_bytes())  # type: ignore[union-attr]
            response = f(*args[0], **args[1])
            if not nowait:
                query.reply(query.key_expr, encode(response))
        except Exception as e:
            logger.exception(f"Exception in RPC handler for {name}: {e}", exc_info=e)
            if not nowait:
                query.reply_err(encode(serialize_exception(e)), encoding=EXCEPTION_ENCODING)
        finally:
            query.drop()

    def stop(self) -> None:
        with self._call_pool.stop_scope():
            self._call_pool.shutdown()
            with self._call_pool.lock:
                queryables = self._queryables
                self._queryables = []
            for queryable in queryables:
                queryable.undeclare()
            self._pending.clear()
            super().stop()
