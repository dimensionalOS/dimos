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

"""Explicit web codec for raw foreign LCM schema classes.

The payload and schema retain the external LCM ABI. Generated DimOS messages
use the CDR codec; this module never reconstructs retired message wrappers.
"""

from __future__ import annotations

from collections.abc import Mapping
import re
from typing import Any

LCM_V1_SUFFIX = ".lcm.v1"
LCM_PRIMITIVES = frozenset(
    {"int8_t", "int16_t", "int32_t", "int64_t", "float", "double", "string", "boolean", "byte"}
)

_FINGERPRINT_RE = re.compile(r"^[0-9a-f]{16}$")


def _is_generated(message_type: type[Any]) -> bool:
    """A generated class from the dimos_lcm wheel (or its vendored lcm_msgs copy)."""
    return message_type.__module__.partition(".")[0] in ("dimos_lcm", "lcm_msgs")


def lcm_type_name(message_type: type[Any]) -> str | None:
    """Return the raw third-party LCM class's canonical schema name."""
    if not callable(getattr(message_type, "lcm_encode", None)):
        return None
    return canonical_name(message_type) if _is_generated(message_type) else None


def schema_class_for(message_type: type[Any]) -> type[Any]:
    """The generated dimos_lcm class carrying the schema `message_type.lcm_encode()`
    writes: the class itself when it is generated, else the class its msg_name
    names (an overlay's own fingerprint method can be inherited from another
    message: PoseStamped carries Pose's). ValueError, with the reason, when the
    type has no LCM schema."""
    name = f"{message_type.__module__}.{message_type.__qualname__}"
    if not callable(getattr(message_type, "lcm_encode", None)):
        raise ValueError(f"{name} is not a DimOS message (no lcm_encode)")
    if not _is_generated(message_type):
        raise ValueError(
            f"{name} is not a raw generated LCM schema; register an explicit @web_encoder"
        )
    generated: Any = message_type
    type_name = canonical_name(message_type)
    if not isinstance(generated, type) or not all(
        hasattr(generated, attr)
        for attr in ("__slots__", "__typenames__", "__dimensions__", "_get_packed_fingerprint")
    ):
        raise ValueError(
            f"{name} ({type_name}) has no generated dimos_lcm.{type_name} class; "
            "register an explicit @web_encoder"
        )
    if generated is not message_type and "_get_packed_fingerprint" in vars(message_type):
        own = _fingerprint(message_type)
        expected = _fingerprint(generated)
        if own != expected:
            raise ValueError(
                f"{name} declares its own LCM fingerprint {own} but the {type_name} schema "
                f"has {expected}: its lcm_encode() does not write the schema's wire format"
            )
    return generated


def _fingerprint(cls: Any) -> str:
    """A generated class's packed fingerprint as the 16 hex chars of params["lcm"]["fp"]."""
    return str(cls._get_packed_fingerprint().hex())


def canonical_name(generated: type[Any]) -> str:
    """'<package>.<Type>' of a generated class, from its module path: the
    generated msg_name is unqualified for some types, and nested field types
    resolve into the wheel's vendored lcm_msgs copy of the same classes."""
    package = generated.__module__.split(".")[1]
    return f"{package}.{generated.__name__}"


def export_schema(generated: type[Any]) -> dict[str, Any]:
    """The compact wire schema {type, fp, structs} the browser decoder compiles:
    structs map canonical names to [field, type, dims] rows in wire order, dims
    None (scalar), [n] (fixed length) or ["<count field>"] (variable length)."""
    structs: dict[str, list[list[Any]]] = {}

    def walk(cls: type[Any]) -> str:
        name = canonical_name(cls)
        if name not in structs:
            rows: list[list[Any]] = []
            structs[name] = rows
            for slot, type_name, dims in zip(
                cls.__slots__, cls.__typenames__, cls.__dimensions__, strict=True
            ):
                field_type = (
                    type_name if type_name in LCM_PRIMITIVES else walk(cls._get_field_type(slot))
                )
                rows.append([slot, field_type, None if dims is None else list(dims)])
        return name

    root = walk(generated)
    return {"type": root, "fp": _fingerprint(generated), "structs": structs}


def check_lcm_params(params: Mapping[str, Any]) -> None:
    """The channel's params must carry the schema cockpit() exported."""
    schema = params.get("lcm")
    if not (
        isinstance(schema, Mapping)
        and isinstance(schema.get("type"), str)
        and isinstance(schema.get("fp"), str)
        and _FINGERPRINT_RE.match(schema["fp"])
        and isinstance(schema.get("structs"), Mapping)
    ):
        raise ValueError(
            "*.lcm.v1 channels need params['lcm'] = {type, fp, structs}; cockpit() fills it in"
        )


def encode_lcm_v1(msg: Any, params: Mapping[str, Any]) -> bytes:
    """The generic *.lcm.v1 encoder: the message's own LCM bytes, refused (the
    bridge drops the sample and logs) when their fingerprint is not the one the
    channel's schema promised the browser."""
    if not _is_generated(type(msg)):
        raise ValueError("LCM web encoding requires a raw generated foreign schema")
    data: bytes = msg.lcm_encode()
    schema = params["lcm"]
    fingerprint = data[:8].hex()
    if fingerprint != schema["fp"]:
        raise ValueError(
            f"{type(msg).__qualname__}.lcm_encode() wrote fingerprint {fingerprint}, "
            f"the channel schema {schema['type']} expects {schema['fp']}"
        )
    return data
