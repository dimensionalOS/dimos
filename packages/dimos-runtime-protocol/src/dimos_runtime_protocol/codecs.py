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

"""Codec helpers for protocol models."""

from __future__ import annotations

from collections.abc import Callable
import json
from typing import TypeVar

from pydantic import BaseModel

ProtocolModel = TypeVar("ProtocolModel", bound=BaseModel)


def to_json_bytes(model: BaseModel) -> bytes:
    """Serialize a protocol model to UTF-8 JSON bytes."""

    return model.model_dump_json().encode("utf-8")


def from_json_bytes(data: bytes, model_type: type[ProtocolModel]) -> ProtocolModel:
    """Deserialize UTF-8 JSON bytes into a protocol model."""

    return model_type.model_validate_json(data)


def to_json_dict(model: BaseModel) -> dict[str, object]:
    """Serialize to a JSON-compatible dictionary for HTTP clients."""

    return json.loads(model.model_dump_json())


def to_msgpack_bytes(model: BaseModel) -> bytes:
    """Serialize a protocol model to msgpack bytes when msgpack is installed."""

    try:
        import msgpack
    except ImportError as exc:  # pragma: no cover - depends on optional extra
        raise RuntimeError("Install dimos-runtime-protocol[msgpack] to use msgpack codecs") from exc
    return msgpack.packb(to_json_dict(model), use_bin_type=True)


def from_msgpack_bytes(data: bytes, model_type: type[ProtocolModel]) -> ProtocolModel:
    """Deserialize msgpack bytes into a protocol model when msgpack is installed."""

    try:
        import msgpack
    except ImportError as exc:  # pragma: no cover - depends on optional extra
        raise RuntimeError("Install dimos-runtime-protocol[msgpack] to use msgpack codecs") from exc
    payload = msgpack.unpackb(data, raw=False)
    return model_type.model_validate(payload)


ModelDecoder = Callable[[bytes], ProtocolModel]
