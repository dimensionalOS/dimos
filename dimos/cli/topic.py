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

import importlib
import pkgutil
import re
import time
from typing import NoReturn

import typer
import zenoh

from dimos.core.global_config import global_config
from dimos.core.transport import PubSubTransport
from dimos.core.transport_factory import make_transport, transport_topic, zenoh_prefix
from dimos.msgs.helpers import resolve_msg_type
from dimos.protocol.pubsub.impl.lcmpubsub import LCMPubSubBase, Topic
from dimos.protocol.pubsub.impl.zenohpubsub import Zenoh

_MAX_LIST_ITEMS = 8

_modules_to_try = [
    "dimos.msgs.geometry_msgs",
    "dimos.msgs.nav_msgs",
    "dimos.msgs.sensor_msgs",
    "dimos.msgs.std_msgs",
    "dimos.msgs.vision_msgs",
    "dimos.msgs.tf2_msgs",
]


def _resolve_type(type_name: str) -> type:
    for module_name in _modules_to_try:
        try:
            module = importlib.import_module(f"{module_name}.{type_name}")
        except ImportError:
            continue
        if hasattr(module, type_name):
            return getattr(module, type_name)  # type: ignore[no-any-return]

    _fail(f"Could not find type '{type_name}' in any known message modules")


def _fail(message: str) -> NoReturn:
    typer.echo(f"Error: {message}", err=True)
    raise typer.Exit(1)


def _split_topic(topic: str) -> tuple[str, type | None]:
    """Split a topic argument into the channel name and the message type it embeds.

    Besides the bare name ("tf", "/tf"), this accepts the full forms that the logs
    and `dimos spy` print: the typed LCM channel ("/tf#tf2_msgs.TFMessage") and,
    on Zenoh, the key ("dimos/tf/tf2_msgs.TFMessage").
    """
    name, _, msg_name = topic.partition("#")
    if global_config.transport == "zenoh":
        name = name.removeprefix(zenoh_prefix())
        head, _, last = name.rpartition("/")
        if not msg_name and head and resolve_msg_type(last) is not None:
            name, msg_name = head, last
        try:
            zenoh.KeyExpr(transport_topic(name))
        except zenoh.ZError as e:
            _fail(f"Invalid topic '{topic}': {e}")
    if not msg_name:
        return name, None
    msg_type = resolve_msg_type(msg_name)
    if msg_type is None:
        _fail(f"Could not resolve message type '{msg_name}' in topic '{topic}'")
    return name, msg_type


def _decode_typed_lcm_message(channel: str, data: bytes) -> object:
    _, msg_name = channel.split("#", 1)  # e.g. "nav_msgs.Odometry"
    cls = resolve_msg_type(msg_name)
    if cls is None:
        raise ValueError(f"Could not resolve message type from channel: {channel}")
    return cls.lcm_decode(data)


def _format_message(msg: object) -> str:
    """Render a message as str() when its class defines one, else as TypeName(field=value, ...)."""
    cls = type(msg)
    if cls.__str__ is not object.__str__ or cls.__repr__ is not object.__repr__:
        return str(msg)
    names = [name for klass in reversed(cls.__mro__) for name in getattr(klass, "__slots__", ())]
    names += getattr(msg, "__dict__", {})
    fields = ", ".join(
        f"{name}={_format_field(getattr(msg, name))}"
        for name in dict.fromkeys(names)
        if not name.startswith("_")
    )
    return f"{cls.__name__}({fields})"


def _format_field(value: object) -> str:
    if isinstance(value, (list, tuple)):
        items = [_format_field(item) for item in value[:_MAX_LIST_ITEMS]]
        if len(value) > _MAX_LIST_ITEMS:
            items.append(f"... ({len(value) - _MAX_LIST_ITEMS} more)")
        return "[" + ", ".join(items) + "]"
    if isinstance(value, bytes):
        return f"<{len(value)} bytes>"
    if isinstance(value, str):
        return repr(value)
    return _format_message(value)


def _listen_forever(listening_msg: str) -> None:
    """Print the banner and block until Ctrl+C."""
    typer.echo(listening_msg)
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        typer.echo("\nStopped.")


def topic_echo(topic: str, type_name: str | None) -> None:
    topic, msg_type = _split_topic(topic)
    if type_name is not None:
        msg_type = _resolve_type(type_name)

    # Typed mode: backend chosen by make_transport from global_config.
    if msg_type is not None:
        transport: PubSubTransport[object] = make_transport(topic, msg_type)
        with transport:
            transport.subscribe(lambda msg: print(_format_message(msg), flush=True))
            _listen_forever(
                f"Listening on {topic} for {msg_type.__name__} messages... (Ctrl+C to stop)"
            )
    # Inferred typed mode: decode each message from the type embedded in its
    # channel/key. The wire format is backend-specific, so dispatch on it.
    elif global_config.transport == "zenoh":
        _topic_echo_inferred_zenoh(topic)
    else:
        _topic_echo_inferred_lcm(transport_topic(topic))


def _topic_echo_inferred_lcm(topic: str) -> None:
    # Warn about missing system config for standalone CLI usage.
    from dimos.protocol.service.lcmservice import autoconf

    autoconf(check_only=True)

    # Listen on /topic#pkg.Msg and decode from the msg_name suffix.
    typed_pattern = rf"^{re.escape(topic)}#.*"

    def on_msg(channel: str, data: bytes) -> None:
        print(_format_message(_decode_typed_lcm_message(channel, data)), flush=True)

    bus = LCMPubSubBase()
    try:
        bus.start()
        assert bus.l is not None
        bus.l.subscribe(typed_pattern, on_msg)

        _listen_forever(
            f"Listening on {topic} (inferring from typed LCM channels like '{topic}#pkg.Msg')... "
            "(Ctrl+C to stop)"
        )
    finally:
        bus.stop()


def _topic_echo_inferred_zenoh(topic: str) -> None:
    key = transport_topic(topic)
    # Typed Zenoh keys embed the type as a trailing segment ("dimos/topic/pkg.Msg");
    # a wildcard subscription decodes each message from that suffix. Untyped keys
    # don't resolve to a type and are skipped by the encoder. The ignore reflects the
    # pattern Topic vs the encoder's concrete-topic protocol (see lcmpubsub.py).
    bus = Zenoh()
    try:
        bus.start()
        bus.subscribe(
            Topic(f"{key}/**"),  # type: ignore[arg-type]
            lambda msg, _topic: print(_format_message(msg), flush=True),
        )

        _listen_forever(
            f"Listening on {topic} (inferring from typed Zenoh keys like '{key}/pkg.Msg')... "
            "(Ctrl+C to stop)"
        )
    finally:
        bus.stop()


def _build_eval_context() -> dict[str, object]:
    # The msgs packages are namespace packages (no __init__.py), so walk their
    # submodules; each message file defines a class of the same name.
    eval_context: dict[str, object] = {}
    for package_name in _modules_to_try:
        package = importlib.import_module(package_name)
        for module_info in pkgutil.iter_modules(package.__path__):
            name = module_info.name
            if name.startswith("test_"):
                continue
            try:
                submodule = importlib.import_module(f"{package_name}.{name}")
            except ImportError:
                continue
            obj = getattr(submodule, name, None)
            if obj is not None:
                eval_context[name] = obj
    return eval_context


def topic_send(topic: str, message_expr: str) -> None:
    try:
        message = eval(message_expr, _build_eval_context())
    except Exception as e:
        typer.echo(f"Error parsing message: {e}", err=True)
        raise typer.Exit(1)

    msg_type = type(message)
    topic, topic_type = _split_topic(topic)
    if topic_type is not None and getattr(topic_type, "msg_name", None) != getattr(
        msg_type, "msg_name", None
    ):
        _fail(f"Topic expects {topic_type.__name__} but the message is {msg_type.__name__}")
    transport: PubSubTransport[object] = make_transport(topic, msg_type)

    with transport:
        transport.broadcast(None, message)
    typer.echo(f"Sent to {topic}: {message}")
