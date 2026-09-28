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
import contextlib
import inspect
import json
import math
from typing import Any

import zenoh

from dimos.protocol.rpc.rpc_utils import deserialize_exception, serialize_exception
from dimos.protocol.rpc.spec import Args
from dimos.protocol.rpc.zenohrpc import ZenohRPC
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class JsonRPCError(Exception):
    """A JSON-RPC error response, or an error a handler raises to send one."""

    def __init__(self, code: int, message: str, data: Any = None) -> None:
        super().__init__(message)
        self.code = code
        self.data = data


class JsonRPC(ZenohRPC):
    """ZenohRPC with JSON-RPC 2.0 messages on dimos/rpc/v1/<name>, for peers in any language.

    Only the messages change: a call is re-sent until a server answers or it
    times out, as on ZenohRPC. Calls send named params. A failed call raises
    the builtin exception a Python handler raised, a RemoteError for any other
    Python exception, or JsonRPCError.
    """

    named_params = True
    key_prefix = "dimos/rpc/v1"

    def _encode(self, name: str, arguments: Args, call_id: int | None) -> bytes:
        args, kwargs = arguments
        if args and kwargs:
            raise TypeError("JSON-RPC calls cannot mix positional and named arguments")
        request: dict[str, Any] = {"jsonrpc": "2.0", "method": name, "params": kwargs or args or {}}
        if call_id is not None:
            request["id"] = call_id
        return _dumps(request)

    def _decode(self, payload: bytes) -> Any:
        try:
            response = json.loads(payload)
            if "error" not in response:
                return response["result"]
            error = response["error"]
            code, message, data = error["code"], error["message"], error.get("data")
        except (ValueError, TypeError, KeyError, RecursionError) as invalid:
            return ValueError(f"Invalid JSON-RPC response: {invalid!r}")
        remote = None
        if isinstance(data, dict) and "type_name" in data:
            # A builtin exception, or a RemoteError for any other type. Data a peer
            # malformed must still answer the call, so it falls back to JsonRPCError.
            with contextlib.suppress(Exception):
                remote = deserialize_exception(data)  # type: ignore[arg-type]
        # A peer must not exit this process or raise control flow into a future.
        if isinstance(remote, Exception) and not isinstance(
            remote, (StopIteration, StopAsyncIteration)
        ):
            return remote
        return JsonRPCError(code, message, data)

    def _decode_error(self, payload: bytes, encoding: zenoh.Encoding) -> Any:
        return ValueError(f"Unexpected JSON-RPC transport error encoding: {encoding}")

    def _execute_rpc(self, f: Callable[..., Any], name: str, query: zenoh.Query) -> None:
        reply: dict[str, Any] = {"jsonrpc": "2.0", "id": None}
        notification = False
        try:
            request = _request(query, name)
            reply["id"] = request.get("id")
            notification = "id" not in request
            params = request.get("params", {})
            args, kwargs = (params, {}) if isinstance(params, list) else ([], params)
            try:
                inspect.signature(f).bind(*args, **kwargs)
            except TypeError as error:
                raise JsonRPCError(-32602, f"Invalid params: {error}") from None
            reply["result"] = f(*args, **kwargs)
        except JsonRPCError as error:
            reply["error"] = {"code": error.code, "message": str(error), "data": error.data}
        except Exception as error:
            logger.exception(f"Exception in JSON-RPC handler for {name}: {error}")
            reply["error"] = {
                "code": -32000,
                "message": str(error),
                "data": serialize_exception(error),
            }
        if not notification:
            _reply(query, reply)
        elif "error" in reply:
            logger.warning(f"JSON-RPC notification {name} failed: {reply['error']['message']}")
        query.drop()


def _request(query: zenoh.Query, name: str) -> dict[str, Any]:
    try:
        # NaN and Infinity are not JSON; an id carrying one could not be echoed back.
        request = json.loads(query.payload.to_bytes(), parse_constant=_not_json)  # type: ignore[union-attr]
    except (ValueError, AttributeError):
        raise JsonRPCError(-32700, "Parse error") from None
    if (
        not isinstance(request, dict)
        or request.get("jsonrpc") != "2.0"
        or request.get("method") != name
    ):
        raise JsonRPCError(-32600, "Invalid Request")
    if "id" in request:
        call_id = request["id"]
        if call_id is not None and (
            isinstance(call_id, bool)
            or not isinstance(call_id, (str, int, float))
            or (isinstance(call_id, float) and not math.isfinite(call_id))
        ):
            raise JsonRPCError(-32600, "Invalid Request")
    return request


def _reply(query: zenoh.Query, reply: dict[str, Any]) -> None:
    # Every request gets a reply: a caller that hears nothing sends the request again.
    try:
        payload = _dumps(reply)
    except (TypeError, ValueError) as error:
        failure = {"code": -32603, "message": f"Cannot encode reply: {error}"}
        payload = _dumps({"jsonrpc": "2.0", "id": reply["id"], "error": failure})
    query.reply(query.key_expr, payload, encoding=zenoh.Encoding.APPLICATION_JSON)


def _dumps(value: Any) -> bytes:
    return json.dumps(value, allow_nan=False).encode()


def _not_json(token: str) -> Any:
    raise ValueError(f"{token} is not JSON")
