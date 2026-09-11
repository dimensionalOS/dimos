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

"""Pubsub URI registry: ``"<proto>:<topic>[#<msg_type>]"`` -> started ``PubSubTransport``.

Maps user-facing protocol names onto the concrete transport classes in
``dimos.core.transport``. Used by CLIs and config files that need to accept
a single string describing both how and where to subscribe.

URI grammar::

    <proto>:<topic>[#<msg_type>]

- ``<proto>``: registry key, e.g. ``lcm``, ``jpeg_lcm``, ``plcm``, ``pshm``,
  ``shm``, ``jpeg_shm``.
- ``<topic>``: channel/key, passed verbatim to the transport constructor.
- ``<msg_type>``: optional ``module.ClassName`` resolved via
  ``dimos.msgs.helpers.resolve_msg_type`` (e.g. ``sensor_msgs.Image``).

Typed protos (``lcm``, ``jpeg_lcm``) require a message type — either from the
``#``-suffix or the ``msg_type`` kwarg. Pickled / self-describing protos
(``plcm``, ``pshm``, ``shm``, ``jpeg_shm``) don't.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from collections.abc import Callable

    from dimos.core.transport import PubSubTransport


def _make_lcm(topic: str, msg_type: type | None) -> Any:
    if msg_type is None:
        raise ValueError("proto 'lcm' requires a message type (URI '#suffix' or msg_type kwarg)")
    from dimos.core.transport import LCMTransport

    return LCMTransport(topic, msg_type)


def _make_jpeg_lcm(topic: str, msg_type: type | None) -> Any:
    if msg_type is None:
        raise ValueError(
            "proto 'jpeg_lcm' requires a message type (URI '#suffix' or msg_type kwarg)"
        )
    from dimos.core.transport import JpegLcmTransport

    return JpegLcmTransport(topic, msg_type)


def _make_plcm(topic: str, msg_type: type | None) -> Any:
    # pickled LCM: receivers unpickle Python objects, no type registration needed.
    from dimos.core.transport import pLCMTransport

    return pLCMTransport(topic)


def _make_pshm(topic: str, msg_type: type | None) -> Any:
    # pickled shared memory: same shape as plcm but over /dev/shm.
    from dimos.core.transport import pSHMTransport

    return pSHMTransport(topic)


def _make_shm(topic: str, msg_type: type | None) -> Any:
    # raw-bytes shared memory: subscribers receive bytes; caller decodes.
    from dimos.core.transport import SHMTransport

    return SHMTransport(topic)


def _make_jpeg_shm(topic: str, msg_type: type | None) -> Any:
    # JPEG-encoded shared memory: subscribers receive decoded Image objects.
    from dimos.core.transport import JpegShmTransport

    return JpegShmTransport(topic)


_REGISTRY: dict[str, Callable[[str, type | None], Any]] = {
    "lcm": _make_lcm,
    "jpeg_lcm": _make_jpeg_lcm,
    "plcm": _make_plcm,
    "pshm": _make_pshm,
    "shm": _make_shm,
    "jpeg_shm": _make_jpeg_shm,
}


def supported_protos() -> list[str]:
    """Return the sorted list of registered proto names."""
    return sorted(_REGISTRY.keys())


def parse_pubsub_uri(uri: str) -> tuple[str, str, str | None]:
    """Split ``"<proto>:<topic>[#<msg_type>]"`` into its three parts.

    Returns ``(proto, topic, msg_type_name_or_None)``. Raises ``ValueError``
    on malformed input or an unknown proto.
    """
    if ":" not in uri:
        raise ValueError(
            f"Invalid pubsub URI {uri!r}: expected '<proto>:<topic>'. "
            f"Supported protos: {supported_protos()}"
        )
    proto, rest = uri.split(":", 1)
    if not proto:
        raise ValueError(f"Invalid pubsub URI {uri!r}: empty proto")
    if proto not in _REGISTRY:
        raise ValueError(f"Unsupported proto {proto!r}; supported: {supported_protos()}")
    msg_type_name: str | None
    if "#" in rest:
        topic, suffix = rest.split("#", 1)
        msg_type_name = suffix or None
    else:
        topic, msg_type_name = rest, None
    if not topic:
        raise ValueError(f"Invalid pubsub URI {uri!r}: empty topic")
    return proto, topic, msg_type_name


def make_pubsub_transport(
    uri: str,
    *,
    msg_type: type | None = None,
) -> PubSubTransport[Any]:
    """Build a ``PubSubTransport`` from a URI (does not call ``start()``).

    The ``#``-suffix in the URI wins over the ``msg_type`` kwarg if both are
    present. Pickled / self-describing protos ignore ``msg_type``.
    """
    proto, topic, msg_type_name = parse_pubsub_uri(uri)
    resolved = msg_type
    if msg_type_name is not None:
        from dimos.msgs.helpers import resolve_msg_type

        resolved = resolve_msg_type(msg_type_name)
        if resolved is None:
            raise ValueError(f"Could not resolve message type {msg_type_name!r} from URI {uri!r}")
    transport: PubSubTransport[Any] = _REGISTRY[proto](topic, resolved)
    return transport


def subscribe_pubsub_uri(
    uri: str,
    callback: Callable[[Any], Any],
    *,
    msg_type: type | None = None,
) -> tuple[PubSubTransport[Any], Callable[[], None]]:
    """Construct + start + subscribe in one step.

    Returns ``(transport, unsubscribe)``. The caller is responsible for
    calling ``transport.stop()`` (and ``unsubscribe()`` if it needs to stop
    receiving messages before the transport itself stops).
    """
    transport = make_pubsub_transport(uri, msg_type=msg_type)
    transport.start()
    unsub = transport.subscribe(callback)
    return transport, unsub
