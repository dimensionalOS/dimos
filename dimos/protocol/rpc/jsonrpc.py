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

from collections.abc import Callable
import inspect
import json
from queue import Empty, Queue
import threading
import time
from typing import Any, cast

import zenoh
import zenoh.handlers

from dimos.protocol.rpc.rpc_utils import deserialize_exception, serialize_exception
from dimos.protocol.rpc.spec import Args
from dimos.protocol.rpc.zenohrpc import ZenohRPC


class JsonRPCError(Exception):
    def __init__(self, code: int, message: str, data: Any = None) -> None:
        super().__init__(message)
        self.code = code
        self.data = data


class JsonRPC(ZenohRPC):
    named_params = True
    version = "v1"
    encoding = zenoh.Encoding.APPLICATION_JSON

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._pending_lock = threading.RLock()

    def __getstate__(self) -> dict[str, Any]:
        state: dict[str, Any] = super().__getstate__()  # type: ignore[no-untyped-call]
        state.pop("_pending_lock", None)
        return state

    def __setstate__(self, state: dict[str, Any]) -> None:
        super().__setstate__(state)
        self._pending_lock = threading.RLock()

    @staticmethod
    def encode(value: Any) -> bytes:
        return json.dumps(value, allow_nan=False, separators=(",", ":")).encode()

    @staticmethod
    def decode(payload: bytes) -> Any:
        return json.loads(payload, parse_constant=_reject_constant)

    def _route(self, name: str) -> str:
        if "*" in name:
            raise ValueError("RPC requires an exact method, not a wildcard")
        return f"dimos/rpc/{self.version}/{name}"

    def call_cb(self, name: str, arguments: Args, cb: Callable[..., Any]) -> Callable[[], None]:
        method = name.rsplit("/", 1)[-1]
        timeout = self.rpc_timeouts.get(name) or self.rpc_timeouts.get(
            method, self.default_rpc_timeout
        )
        call_id = next(self._call_counter)
        request = {
            "jsonrpc": "2.0",
            "id": call_id,
            "method": name,
            "params": self._params(arguments),
        }
        payload = self.encode(request)
        route = self._route(name)
        deadline = time.monotonic() + timeout
        messages: Queue[tuple[str, Any]] = Queue()
        started = threading.Event()

        def on_reply(reply: zenoh.Reply) -> None:
            messages.put(("reply", reply))

        def response(reply: zenoh.Reply) -> Any:
            if reply.err is not None:
                message = reply.err.payload.to_bytes().decode(errors="replace")
                if message == "Timeout" and reply.replier_id is None:
                    return TimeoutError(f"RPC call to '{name}' timed out")
                return ConnectionError(f"Zenoh error answering {name}: {message}")
            return self._response(self.decode(reply.ok.payload.to_bytes()), call_id)  # type: ignore[union-attr]

        def run() -> None:
            result: Any = TimeoutError(f"RPC call to '{name}' timed out")
            try:
                # Zenoh callbacks only enqueue; this thread owns all API calls and cleanup.
                with session.declare_querier(
                    route,
                    target=zenoh.QueryTarget.ALL,
                    consolidation=zenoh.ConsolidationMode.NONE,
                    # A full transmit queue must not block timeout or cancellation.
                    congestion_control=zenoh.CongestionControl.DROP,
                    timeout=timeout,
                ) as querier:
                    with querier.declare_matching_listener(
                        lambda status: messages.put(("match", status.matching))
                    ):
                        messages.put(
                            (
                                "match",
                                cast("zenoh.MatchingStatus", querier.matching_status).matching,
                            )
                        )
                        sent = False
                        while time.monotonic() < deadline:
                            kind, value = messages.get(timeout=max(0, deadline - time.monotonic()))
                            if time.monotonic() >= deadline or kind == "cancel":
                                break
                            if kind == "match" and value and not sent:
                                with self._pending_lock:
                                    if call_id not in self._pending:
                                        break
                                    sent = True
                                querier.get(
                                    zenoh.handlers.Callback(
                                        on_reply,
                                        drop=lambda: messages.put(("end", None)),
                                    ),
                                    payload=payload,
                                    encoding=self.encoding,
                                )
                            elif kind == "reply":
                                result = response(value)
                                break
                            elif kind == "end":
                                result = ConnectionError(
                                    f"RPC call to '{name}' received no reply; it may have executed"
                                )
                                break
            except Empty:
                pass  # The shared discovery/reply deadline expired.
            except Exception as error:
                result = error
            with self._pending_lock:
                callback = self._pending.pop(call_id, None)
            if callback is not None:
                if time.monotonic() >= deadline:
                    result = TimeoutError(f"RPC call to '{name}' timed out")
                cb(result)

        def unsubscribe_callback() -> None:
            with self._pending_lock:
                self._pending.pop(call_id, None)
            messages.put(("cancel", None))
            if threading.current_thread() is not worker:
                started.wait()
                if worker.ident is not None:
                    worker.join()

        worker = threading.Thread(target=run, daemon=True)
        with self._pending_lock:
            session = self.session
            self._pending[call_id] = unsubscribe_callback
        # Thread startup can wait for scheduling; do not hold up other calls.
        try:
            worker.start()
        except Exception:
            with self._pending_lock:
                self._pending.pop(call_id, None)
            raise
        finally:
            started.set()
        return unsubscribe_callback

    def stop(self) -> None:
        with self._pending_lock:
            self._session = None
            callbacks = list(self._pending.values())
            self._pending.clear()
        for cancel in callbacks:
            cancel()
        super().stop()

    def call_nowait(self, name: str, arguments: Args) -> None:
        method = name.rsplit("/", 1)[-1]
        timeout = self.rpc_timeouts.get(name) or self.rpc_timeouts.get(
            method, self.default_rpc_timeout
        )
        request = {
            "jsonrpc": "2.0",
            "method": name,
            "params": self._params(arguments),
        }
        self.session.get(
            self._route(name),
            lambda _: None,
            target=zenoh.QueryTarget.ALL,
            consolidation=zenoh.ConsolidationMode.NONE,
            congestion_control=zenoh.CongestionControl.BLOCK,
            timeout=timeout,
            payload=self.encode(request),
            encoding=self.encoding,
        )

    def serve_rpc(self, f: Callable[..., Any], name: str | None = None):  # type: ignore[no-untyped-def, override]
        rpc_name = name or f.__name__

        def on_query(query: zenoh.Query) -> None:
            self._get_call_thread_pool().submit(self._execute, f, rpc_name, query)

        queryable = self.session.declare_queryable(self._route(rpc_name), on_query, complete=True)
        self._queryables.append(queryable)

        def unsubscribe() -> None:
            self._queryables.remove(queryable)
            queryable.undeclare()

        return unsubscribe

    def _execute(self, f: Callable[..., Any], name: str, query: zenoh.Query) -> None:
        request_id: Any = None
        notification = False
        try:
            request = self.decode(query.payload.to_bytes())  # type: ignore[union-attr]
        except Exception:
            self._reply_error(query, None, -32700, "Parse error")
            query.drop()
            return
        try:
            if isinstance(request, dict):
                request_id = request.get("id")
                notification = "id" not in request
            args, kwargs = self._request(request, name)
            try:
                inspect.signature(f).bind(*args, **kwargs)
            except TypeError as error:
                raise JsonRPCError(-32602, str(error)) from error
            try:
                result = f(*args, **kwargs)
            except JsonRPCError as error:
                raise JsonRPCError(-32000, str(error), serialize_exception(error)) from error
            if not notification:
                self._reply(query, {"jsonrpc": "2.0", "id": request_id, "result": result})
        except JsonRPCError as error:
            if error.code == -32600:
                self._reply_error(query, None, error.code, str(error))
            elif not notification:
                self._reply_error(query, request_id, error.code, str(error), error.data)
        except Exception as error:
            if not notification:
                self._reply_error(
                    query,
                    request_id,
                    -32000,
                    str(error),
                    serialize_exception(error),
                )
        finally:
            query.drop()

    def _request(self, request: Any, name: str) -> tuple[list[Any], dict[str, Any]]:
        if not isinstance(request, dict):
            raise JsonRPCError(-32600, "Invalid Request")
        request_id = request.get("id")
        if (
            request.get("jsonrpc") != "2.0"
            or not isinstance(request.get("method"), str)
            or isinstance(request_id, bool)
            or (request_id is not None and not isinstance(request_id, (int, float, str)))
        ):
            raise JsonRPCError(-32600, "Invalid Request")
        if request["method"] != name:
            raise JsonRPCError(-32601, "Method not found")
        params = request.get("params", {})
        if isinstance(params, list):
            return params, {}
        if isinstance(params, dict):
            return [], params
        raise JsonRPCError(-32602, "Invalid params")

    @staticmethod
    def _params(arguments: Args) -> list[Any] | dict[str, Any]:
        args, kwargs = arguments
        if args and kwargs:
            raise TypeError("JSON-RPC calls cannot mix positional and named arguments")
        return kwargs or args or {}

    def _response(self, response: Any, call_id: int) -> Any:
        if not isinstance(response, dict) or response.get("jsonrpc") != "2.0":
            raise ValueError("Invalid JSON-RPC response")
        error = response.get("error")
        if "error" in response and (
            not isinstance(error, dict)
            or type(error.get("code")) is not int
            or not isinstance(error.get("message"), str)
        ):
            raise ValueError("Invalid JSON-RPC error")
        valid_id = type(response.get("id")) is int and response["id"] == call_id
        parse_error = (
            response.get("id") is None
            and isinstance(error, dict)
            and error.get("code") in {-32700, -32600}
        )
        if (
            "id" not in response
            or not (valid_id or parse_error)
            or ("result" in response) == ("error" in response)
        ):
            raise ValueError("Invalid JSON-RPC response")
        if "result" in response:
            return response["result"]
        assert isinstance(error, dict)
        data = error.get("data")
        if (
            isinstance(data, dict)
            and {"type_name", "type_module", "args", "traceback"} <= data.keys()
        ):
            return deserialize_exception(data)  # type: ignore[arg-type]
        return JsonRPCError(error["code"], error["message"], data)

    def _reply(self, query: zenoh.Query, response: dict[str, Any]) -> None:
        try:
            payload = self.encode(response)
        except (TypeError, ValueError) as error:
            payload = self.encode(
                {
                    "jsonrpc": "2.0",
                    "id": response.get("id"),
                    "error": {"code": -32603, "message": f"Cannot encode reply: {error}"},
                }
            )
        query.reply(query.key_expr, payload, encoding=self.encoding)

    def _reply_error(
        self,
        query: zenoh.Query,
        request_id: Any,
        code: int,
        message: str,
        data: Any = None,
    ) -> None:
        error = {"code": code, "message": message}
        if data is not None:
            error["data"] = data
        self._reply(query, {"jsonrpc": "2.0", "id": request_id, "error": error})


def _reject_constant(value: str) -> Any:
    raise ValueError(f"Invalid JSON constant: {value}")
