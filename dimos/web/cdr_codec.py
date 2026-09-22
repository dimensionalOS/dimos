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

"""Generated CDR payloads and self-contained ROS2 schemas for web channels."""

from collections.abc import Mapping
import re
from typing import Any

CDR_V1_SUFFIX = ".cdr.v1"
_TYPE_NAME = re.compile(r"^[A-Za-z][A-Za-z0-9_]*/msg/[A-Za-z][A-Za-z0-9_]*$")


def cdr_type_name(message_type: type[Any]) -> str | None:
    name = getattr(message_type, "msg_name", None)
    if (
        isinstance(name, str)
        and _TYPE_NAME.fullmatch(name)
        and isinstance(getattr(message_type, "schema", None), str)
        and callable(getattr(message_type, "encode", None))
    ):
        return name
    return None


def default_encoding(message_type: type[Any], dir: str) -> str:
    name = cdr_type_name(message_type) if dir == "rx" else None
    if name is None:
        return "json.v1"
    if name == "sensor_msgs/msg/Image":
        raise ValueError(
            f"{name} has no default web encoding (raw pixels; use jpeg.v1 or explicit CDR)"
        )
    return f"{name}{CDR_V1_SUFFIX}"


def export_schema(message_type: type[Any]) -> dict[str, str]:
    name = cdr_type_name(message_type)
    if name is None:
        raise ValueError(f"{message_type.__qualname__} has no generated CDR schema")
    return {"type": name, "definition": message_type.schema}


def check_cdr_params(params: Mapping[str, Any]) -> None:
    schema = params.get("cdr")
    if not (
        isinstance(schema, Mapping)
        and isinstance(schema.get("type"), str)
        and _TYPE_NAME.fullmatch(schema["type"])
        and isinstance(schema.get("definition"), str)
    ):
        raise ValueError(
            "*.cdr.v1 channels need params['cdr'] = {type, definition}; cockpit() fills it in"
        )


def encode_cdr_v1(msg: Any, params: Mapping[str, Any]) -> bytes:
    if cdr_type_name(type(msg)) != params["cdr"]["type"]:
        raise ValueError(
            f"message type does not match declared channel type {params['cdr']['type']}"
        )
    payload: bytes = msg.encode()
    return payload
