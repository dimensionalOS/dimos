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

from __future__ import annotations

from collections.abc import Callable
import re
import time
from types import SimpleNamespace
from typing import cast

import typer

from dimos.core.global_config import global_config
from dimos.core.transport import PubSubTransport
from dimos.core.transport_factory import make_transport, transport_topic
from dimos.message_codegen.registry import message_types
from dimos.msgs.helpers import resolve_msg_type
from dimos.protocol.pubsub.impl.lcmpubsub import LCMPubSubBase, Topic
from dimos.protocol.pubsub.impl.zenohpubsub import Zenoh


def _resolve_type(type_name: str) -> type:
    types = message_types()
    if type_name in types:
        return cast("type", types[type_name])
    matches = [value for name, value in types.items() if name.rsplit("/", 1)[-1] == type_name]
    if len(matches) == 1:
        return cast("type", matches[0])
    if matches:
        raise ValueError(f"Ambiguous message type {type_name!r}; use package/msg/Type")
    raise ValueError(f"Unknown installed message type {type_name!r}")


def _decode_typed_lcm_message(channel: str, data: bytes) -> object:
    _, msg_name = channel.split("#", 1)  # e.g. "nav_msgs/msg/Odometry"
    cls = resolve_msg_type(msg_name)
    if cls is None:
        raise ValueError(f"Could not resolve message type from channel: {channel}")
    return cls.decode(data)


def _listen_forever(listening_msg: str, on_stop: Callable[[], None] = lambda: None) -> None:
    """Print the banner and block until Ctrl+C, then run on_stop."""
    typer.echo(listening_msg)
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        on_stop()
        typer.echo("\nStopped.")


def topic_echo(topic: str, type_name: str | None) -> None:
    # Explicit message type: backend chosen by make_transport from global_config.
    if type_name is not None:
        msg_type = _resolve_type(type_name)
        transport: PubSubTransport[object] = make_transport(topic, msg_type)
        transport.subscribe(lambda msg: print(msg))
        _listen_forever(f"Listening on {topic} for {type_name} messages... (Ctrl+C to stop)")
        return

    # Inferred typed mode: decode each message from the type embedded in its
    # channel/key. The wire format is backend-specific, so dispatch on it.
    if global_config.transport == "zenoh":
        _topic_echo_inferred_zenoh(topic)
    else:
        _topic_echo_inferred_lcm(topic)


def _topic_echo_inferred_lcm(topic: str) -> None:
    # Warn about missing system config for standalone CLI usage.
    from dimos.protocol.service.lcmservice import autoconf

    autoconf(check_only=True)

    # Listen on /topic#pkg/msg/Type and decode from the msg_name suffix.
    bus = LCMPubSubBase()
    bus.start()  # starts threaded handle loop

    typed_pattern = rf"^{re.escape(topic)}#.*"

    def on_msg(channel: str, data: bytes) -> None:
        print(_decode_typed_lcm_message(channel, data))

    assert bus.l is not None
    bus.l.subscribe(typed_pattern, on_msg)

    _listen_forever(
        f"Listening on {topic} (inferring from typed LCM channels like '{topic}#pkg/msg/Type')... "
        "(Ctrl+C to stop)",
        bus.stop,
    )


def _topic_echo_inferred_zenoh(topic: str) -> None:
    key = transport_topic(topic)
    bus = Zenoh()
    bus.start()

    # Typed Zenoh keys embed the type as trailing segments ("dimos/topic/pkg/msg/Type");
    # a wildcard subscription decodes each message from that suffix. Untyped keys
    # don't resolve to a type and are skipped by the encoder. The ignore reflects the
    # pattern Topic vs the encoder's concrete-topic protocol (see lcmpubsub.py).
    bus.subscribe(Topic(f"{key}/**"), lambda msg, _topic: print(msg))  # type: ignore[arg-type]

    _listen_forever(
        f"Listening on {topic} (inferring from typed Zenoh keys like '{key}/pkg/msg/Type')... "
        "(Ctrl+C to stop)",
        bus.stop,
    )


def _build_eval_context() -> dict[str, object]:
    types = message_types()
    context: dict[str, object] = {}
    packages: dict[str, SimpleNamespace] = {}
    short_names: dict[str, list[type]] = {}
    for name, cls in types.items():
        package, _, short = name.split("/")
        namespace = packages.setdefault(package, SimpleNamespace(msg=SimpleNamespace()))
        setattr(namespace.msg, short, cls)
        short_names.setdefault(short, []).append(cls)
    context.update(packages)
    context.update({name: classes[0] for name, classes in short_names.items() if len(classes) == 1})
    return context


def topic_send(topic: str, message_expr: str) -> None:
    try:
        message = eval(message_expr, _build_eval_context())
    except Exception as e:
        typer.echo(f"Error parsing message: {e}", err=True)
        raise typer.Exit(1)

    msg_type = type(message)
    transport: PubSubTransport[object] = make_transport(topic, msg_type)

    transport.broadcast(None, message)
    typer.echo(f"Sent to {topic}: {message}")
