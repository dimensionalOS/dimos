# Copyright 2025-2026 Dimensional Inc.
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

from collections.abc import Callable
from typing import Any, cast

from dimos.core.transport import (
    JpegLcmTransport,
    JpegShmTransport,
    LCMTransport,
    PubSubTransport,
    SHMTransport,
    pLCMTransport,
    pSHMTransport,
)
from dimos.msgs.helpers import resolve_msg_type

_PROTOS = ("jpeg_lcm", "jpeg_shm", "lcm", "plcm", "pshm", "shm")


def supported_protos() -> list[str]:
    return list(_PROTOS)


def parse_pubsub_uri(uri: str) -> tuple[str, str, str | None]:
    if ":" not in uri:
        raise ValueError(f"Invalid pubsub URI {uri!r}; supported: {supported_protos()}")
    proto, value = uri.split(":", 1)
    if proto not in _PROTOS:
        raise ValueError(f"Unsupported proto {proto!r}; supported: {supported_protos()}")
    topic, separator, msg_type = value.partition("#")
    if not topic:
        raise ValueError(f"Invalid pubsub URI {uri!r}: empty topic")
    return proto, topic, msg_type if separator and msg_type else None


def make_pubsub_transport(uri: str, *, msg_type: type | None = None) -> PubSubTransport[Any]:
    proto, topic, type_name = parse_pubsub_uri(uri)
    if type_name is not None:
        msg_type = resolve_msg_type(type_name)
        if msg_type is None:
            raise ValueError(f"Could not resolve message type {type_name!r} from URI {uri!r}")
    if proto in ("lcm", "jpeg_lcm") and msg_type is None:
        raise ValueError(f"proto {proto!r} requires a message type")

    match proto:
        case "lcm":
            return LCMTransport(topic, cast("type", msg_type))
        case "jpeg_lcm":
            return JpegLcmTransport(topic, cast("type", msg_type))
        case "plcm":
            return pLCMTransport(topic)
        case "pshm":
            return pSHMTransport(topic)
        case "shm":
            return SHMTransport(topic)
        case _:
            return JpegShmTransport(topic)


def subscribe_pubsub_uri(
    uri: str, callback: Callable[[Any], Any], *, msg_type: type | None = None
) -> tuple[PubSubTransport[Any], Callable[[], None]]:
    transport = make_pubsub_transport(uri, msg_type=msg_type)
    transport.start()
    return transport, transport.subscribe(callback)
