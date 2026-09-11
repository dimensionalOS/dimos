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

import json
from typing import Any

import zenoh

ROUTE: str = "dimos/rpc/v1"


class RpcError(Exception):
    def __init__(self, code: int, message: str, data: Any = None) -> None:
        super().__init__(message)
        self.code = code
        self.data = data


def reject_constant(value: str) -> Any:
    raise ValueError(f"Invalid JSON constant: {value}")


def call(
    session: zenoh.Session,
    method: str,
    params: dict[str, Any] | None = None,
    timeout: float = 2.0,
) -> Any:
    if "*" in method:
        raise ValueError("RPC requires an exact method, not a wildcard")
    request = {"jsonrpc": "2.0", "id": 1, "method": method, "params": params or {}}
    replies = list(
        session.get(
            zenoh.KeyExpr(f"{ROUTE}/{method}"),
            payload=json.dumps(request, allow_nan=False),
            encoding=zenoh.Encoding.APPLICATION_JSON,
            timeout=timeout,
            target=zenoh.QueryTarget.ALL,
            consolidation=zenoh.ConsolidationMode.NONE,
        )
    )
    if not replies:
        raise TimeoutError(f"No reply for {method}")
    if len(replies) != 1:
        raise ValueError(f"Expected one reply for {method}, got {len(replies)}")
    transport_error = replies[0].err
    if transport_error is not None:
        message = transport_error.payload.to_bytes().decode(errors="replace")
        if message == "Timeout" and replies[0].replier_id is None:
            raise TimeoutError(f"RPC timed out for {method}; it may have executed")
        raise ConnectionError(f"Zenoh error answering {method}: {message}")
    reply = replies[0].ok
    assert reply is not None
    response = json.loads(reply.payload.to_bytes(), parse_constant=reject_constant)
    if (
        not isinstance(response, dict)
        or response.get("jsonrpc") != "2.0"
        or "id" not in response
        or ("result" in response) == ("error" in response)
    ):
        raise ValueError("Invalid JSON-RPC response")
    error = response.get("error")
    if "error" in response and (
        not isinstance(error, dict)
        or type(error.get("code")) is not int
        or not isinstance(error.get("message"), str)
    ):
        raise ValueError("Invalid JSON-RPC error")
    if not (type(response["id"]) is int and response["id"] == request["id"]):
        if not (response["id"] is None and error and error["code"] in {-32700, -32600}):
            raise ValueError("Mismatched JSON-RPC response id")
    if error is not None:
        raise RpcError(error["code"], error["message"], error.get("data"))
    return response["result"]
