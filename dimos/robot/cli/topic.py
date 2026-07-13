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
import importlib
import pkgutil
import re
import time

import typer

from dimos.core.global_config import global_config
from dimos.core.transport import PubSubTransport
from dimos.core.transport_factory import make_transport, transport_topic
from dimos.protocol.pubsub.impl.lcmpubsub import LCMPubSubBase, Topic
from dimos.protocol.pubsub.impl.zenohpubsub import Zenoh

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

    raise ValueError(f"Could not find type '{type_name}' in any known message modules")


def _decode_typed_lcm_message(channel: str, data: bytes) -> object:
    from dimos.msgs.helpers import resolve_msg_type

    _, msg_name = channel.split("#", 1)  # e.g. "nav_msgs.Odometry"
    cls = resolve_msg_type(msg_name)
    if cls is None:
        raise ValueError(f"Could not resolve message type from channel: {channel}")
    return cls.lcm_decode(data)


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
    # Explicit mode (legacy): backend chosen by make_transport from global_config.
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

    # Listen on /topic#pkg.Msg and decode from the msg_name suffix.
    bus = LCMPubSubBase()
    bus.start()  # starts threaded handle loop

    typed_pattern = rf"^{re.escape(topic)}#.*"

    def on_msg(channel: str, data: bytes) -> None:
        print(_decode_typed_lcm_message(channel, data))

    assert bus.l is not None
    bus.l.subscribe(typed_pattern, on_msg)

    _listen_forever(
        f"Listening on {topic} (inferring from typed LCM channels like '{topic}#pkg.Msg')... "
        "(Ctrl+C to stop)",
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
        "(Ctrl+C to stop)",
        bus.stop,
    )


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
    transport: PubSubTransport[object] = make_transport(topic, msg_type)

    transport.broadcast(None, message)
    typer.echo(f"Sent to {topic}: {message}")
