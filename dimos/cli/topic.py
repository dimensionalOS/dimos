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

from dimos_cdrgen.registry import message_types
import typer

from dimos.core.global_config import global_config
from dimos.core.transport import PubSubTransport
from dimos.core.transport_factory import make_transport, transport_topic
from dimos.porcelain.remote_module_source import RemoteModuleSource
from dimos.protocol.pubsub.codec import describe_message
from dimos.protocol.pubsub.impl.lcmpubsub import LCMPubSubBase, Topic
from dimos.protocol.pubsub.impl.zenohpubsub import Zenoh
from dimos.protocol.pubsub.message_types import resolve_message_type


def _resolve_type(type_name: str) -> type:
    resolved = resolve_message_type(type_name)
    if resolved is not None:
        return resolved
    matches = [
        kind for name, kind in message_types().items() if name.rsplit("/", 1)[-1] == type_name
    ]
    if len(matches) == 1:
        return matches[0]
    if len(matches) > 1:
        raise ValueError(f"Ambiguous message type '{type_name}'; use package/msg/Type")
    raise ValueError(f"Could not find registered message type '{type_name}'")


def _decode_typed_lcm_message(channel: str, data: bytes) -> object:
    _, msg_name = channel.split("#", 1)
    cls = _resolve_type(msg_name.removesuffix(".cdr.v1"))
    return describe_message(cls).codec.decode(data)


def _listen_forever(listening_msg: str, on_stop: Callable[[], None] = lambda: None) -> None:
    """Print the banner and block until Ctrl+C, then run on_stop."""
    typer.echo(listening_msg)
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        on_stop()
        typer.echo("\nStopped.")


def _topic_echo_from_graph(topic: str) -> bool:
    try:
        source = RemoteModuleSource(timeout=0.5)
    except RuntimeError:
        return False
    try:
        channels = {topic, transport_topic(topic)}
        matches = [
            descriptor
            for descriptor in source.list_stream_descriptors()
            if descriptor.channel in channels
        ]
        if not matches:
            return False
        if len({descriptor.type_name for descriptor in matches}) != 1:
            raise ValueError(f"Topic '{topic}' has multiple declared types; use --type")
        descriptor = matches[0]
        stream = source.get_module(descriptor.module).get_stream(descriptor.name)
        unsubscribe = stream.subscribe(lambda message: print(message))
        try:
            _listen_forever(
                f"Listening on {topic} using the running graph's declared type... (Ctrl+C to stop)"
            )
        finally:
            unsubscribe()
        return True
    finally:
        source.close()


def topic_echo(topic: str, type_name: str | None) -> None:
    # Explicit typed mode: backend chosen by make_transport from global_config.
    if type_name is not None:
        msg_type = _resolve_type(type_name)
        transport: PubSubTransport[object] = make_transport(topic, msg_type)
        transport.subscribe(lambda msg: print(msg))
        _listen_forever(f"Listening on {topic} for {type_name} messages... (Ctrl+C to stop)")
        return

    if _topic_echo_from_graph(topic):
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

    # Listen on /topic#pkg.Msg and decode from the msg_name suffix.
    bus = LCMPubSubBase()
    bus.start()  # starts threaded handle loop

    typed_pattern = rf"^{re.escape(topic)}#.*"

    def on_msg(channel: str, data: bytes) -> None:
        print(_decode_typed_lcm_message(channel, data))

    assert bus.l is not None
    bus.l.subscribe(typed_pattern, on_msg)

    _listen_forever(
        f"Listening on {topic} (inferring from typed LCM channels like '{topic}#pkg.Msg.cdr.v1')... "
        "Raw CDR topics without a running graph require --type. (Ctrl+C to stop)",
        bus.stop,
    )


def _topic_echo_inferred_zenoh(topic: str) -> None:
    key = transport_topic(topic)
    bus = Zenoh()
    bus.start()

    # Typed Zenoh keys embed the type as a trailing segment ("dimos/topic/pkg.Msg");
    # a wildcard subscription decodes each message from that suffix. Untyped keys
    # don't resolve to a type and are skipped by the encoder. The ignore reflects the
    # pattern Topic vs the encoder's concrete-topic protocol (see lcmpubsub.py).
    bus.subscribe(Topic(f"{key}/**"), lambda msg, _topic: print(msg))  # type: ignore[arg-type]

    _listen_forever(
        f"Listening on {topic} (inferring from typed Zenoh keys like '{key}/pkg.Msg')... "
        "Raw CDR topics without a running graph require --type. (Ctrl+C to stop)",
        bus.stop,
    )


def _build_eval_context() -> dict[str, object]:
    registered = message_types()
    context: dict[str, object] = {}
    names: dict[str, list[type]] = {}
    packages: dict[str, SimpleNamespace] = {}
    for canonical, kind in registered.items():
        package, _, name = canonical.split("/")
        names.setdefault(name, []).append(kind)
        setattr(packages.setdefault(package, SimpleNamespace()), name, kind)
    context.update(packages)
    context.update({name: kinds[0] for name, kinds in names.items() if len(kinds) == 1})
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
