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

"""JSON-RPC 2.0 wire models for DimOS's ``args``/``kwargs`` parameter format.

Use ``model_validate_json`` to validate incoming payloads. These models check
message structure; the RPC backend owns routing, method binding and matching
response IDs to pending requests. Batch handling is outside these models.

Calculator data flow (proposed JSON-RPC transport; data examples only, no calls run):

1. Python call → arguments
   self.calc.compute1(2, 3)
   args = (2, 3), kwargs = {}

2. Arguments → JsonRpcRequest → UTF-8 JSON request body
   Zenoh key: dimos/jsonrpc/python-args/Calculator/compute1
   {
       "jsonrpc": "2.0",
       "id": "client1:1",
       "method": "Calculator/compute1",
       "params": {"args": [2, 3], "kwargs": {}}
   }

3. Zenoh routes by key → the server validates the request → extracts arguments
   JsonRpcRequest.params → JsonRpcParams(args=[2, 3], kwargs={})
   method matches Calculator/compute1 in the key
   compute1 receives a=2, b=3 → returns the Python integer 5

4. Return value → JsonRpcSuccessResponse → UTF-8 JSON response body
   Reply within the same Zenoh query, without a separate response topic:
   {"jsonrpc": "2.0", "id": "client1:1", "result": 5}

5. The client validates the response → matches the ID → completes the waiting call
   JsonRpcSuccessResponse(id="client1:1", result=5, jsonrpc="2.0")
   pending["client1:1"] → callback(5) → wakes call_sync()
   self.calc.compute1(2, 3) returns the Python integer 5
   pending is the proposed backend's request map, not state maintained by these models.

Another successful call, preserving floating-point values:
   self.calc.compute2(1.5, 2.5)
   Zenoh key: dimos/jsonrpc/python-args/Calculator/compute2
   Request: {"jsonrpc": "2.0", "id": "client1:2", "method": "Calculator/compute2",
             "params": {"args": [1.5, 2.5], "kwargs": {}}}
   Server arguments: a=1.5, b=2.5 → returns 4.0
   Response: {"jsonrpc": "2.0", "id": "client1:2", "result": 4.0}
   Client result: 4.0

Error path (assume compute1 rejects negative values; illustrative error data only):
   self.calc.compute1(-2, 3)
   Zenoh key: dimos/jsonrpc/python-args/Calculator/compute1
   Request: {"jsonrpc": "2.0", "id": "client1:3", "method": "Calculator/compute1",
             "params": {"args": [-2, 3], "kwargs": {}}}
   Server exception: ValueError("a must be non-negative")
   JsonRpcErrorResponse payload:
   {
       "jsonrpc": "2.0",
       "id": "client1:3",
       "error": {
           "code": -32000,
           "message": "a must be non-negative",
           "data": {
               "type_name": "ValueError",
               "type_module": "builtins",
               "args": ["a must be non-negative"],
               "traceback": "<remote traceback>"
           }
       }
   }
   Client validation → backend reconstructs ValueError → call_sync() raises it.

A call without a response (once stop supports the JSON transport):
   self.calc.stop()
   Zenoh key: dimos/jsonrpc/python-args/Calculator/stop
   JsonRpcNotification: {"jsonrpc": "2.0", "method": "Calculator/stop",
                         "params": {"args": [], "kwargs": {}}}
   Request has no ID → the server executes stop() → no response payload.
"""

import math
from typing import Annotated, Literal, TypeAlias

from pydantic import AfterValidator, BaseModel, ConfigDict, Field, JsonValue

# DimOS uses string or integer IDs; explicit null is distinct from an absent ID.
JsonRpcId: TypeAlias = str | int | None

JsonRpcValue: TypeAlias = Annotated[
    JsonValue,
    AfterValidator(
        lambda v: None if isinstance(v, float) and (math.isnan(v) or math.isinf(v)) else v
    ),
]


class _WireModel(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True, allow_inf_nan=False)


class JsonRpcParams(_WireModel):
    """JSON-compatible arguments to pass to ``f(*args, **kwargs)``."""

    args: list[JsonRpcValue] = Field(default_factory=list)
    kwargs: dict[str, JsonRpcValue] = Field(default_factory=dict)


class JsonRpcRequest(_WireModel):
    """A call requiring a response, including when its ID is explicitly null."""

    jsonrpc: Literal["2.0"]
    id: JsonRpcId
    method: str = Field(min_length=1)
    params: JsonRpcParams = Field(default_factory=JsonRpcParams)


class JsonRpcNotification(_WireModel):
    """A call without an ID; no response should be sent."""

    jsonrpc: Literal["2.0"]
    method: str = Field(min_length=1)
    params: JsonRpcParams = Field(default_factory=JsonRpcParams)


class JsonRpcError(_WireModel):
    """A protocol or application error with optional JSON-compatible details."""

    code: int
    message: str
    data: JsonRpcValue = None


class JsonRpcSuccessResponse(_WireModel):
    """A successful response; ``result`` is required even when it is null."""

    jsonrpc: Literal["2.0"]
    id: JsonRpcId
    result: JsonRpcValue


class JsonRpcErrorResponse(_WireModel):
    """An error response; an unknown request ID must be explicitly null."""

    jsonrpc: Literal["2.0"]
    id: JsonRpcId
    error: JsonRpcError


JsonRpcCall: TypeAlias = JsonRpcRequest | JsonRpcNotification
JsonRpcResponse: TypeAlias = JsonRpcSuccessResponse | JsonRpcErrorResponse
