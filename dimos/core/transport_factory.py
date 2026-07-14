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

"""Backend-agnostic transport construction."""

from __future__ import annotations

import os
import sys
from typing import TYPE_CHECKING, Any, NoReturn, get_args

from dimos.core.global_config import GlobalConfig, TransportBackend, global_config
from dimos.core.transport import (
    LCMTransport,
    ZenohTransport,
    pLCMTransport,
    pZenohTransport,
)
from dimos.protocol.pubsub.impl.zenohpubsub import (
    QOS_LATEST_WINS,
    QOS_NEVER_DROP,
    Topic as ZenohTopic,
    ZenohQoS,
)
from dimos.protocol.rpc.pubsubrpc import LCMRPC, ZenohRPC
from dimos.protocol.tf.tf import LCMTF, ZenohTF

if TYPE_CHECKING:
    from dimos.core.transport import PubSubTransport
    from dimos.protocol.rpc.spec import RPCSpec
    from dimos.protocol.tf.tf import TFSpec


def transport_topic(name: str, g: GlobalConfig = global_config) -> str:
    """Map a logical channel name to the active backend's topic string.

    LCM channels are leading-slash paths (`/foo`).

    Zenoh key expressions can't start with `/` and are namespaced under `dimos`.
    """
    if g.transport == "zenoh":
        return "dimos/" + name.lstrip("/")
    return name if name.startswith("/") else "/" + name


# High-rate sensor streams: drop stale frames under congestion, never stall the
# publisher. Matched by message type since that is what makes them high-rate.
_LATEST_WINS_TYPES = ("sensor_msgs.Image", "sensor_msgs.PointCloud2")
# Agent/human conversation channels: low-rate, and a dropped message loses a
# whole turn of conversation.
_NEVER_DROP_CHANNELS = ("human_input", "agent", "agent_idle")


def default_zenoh_qos(name: str, msg_type: type | None = None) -> ZenohQoS | None:
    """Default publisher QoS for a logical channel; None = zenoh defaults."""
    if getattr(msg_type, "msg_name", None) in _LATEST_WINS_TYPES:
        return QOS_LATEST_WINS
    if name.lstrip("/") in _NEVER_DROP_CHANNELS:
        return QOS_NEVER_DROP
    return None


def make_transport(
    name: str, msg_type: type | None = None, *, g: GlobalConfig = global_config
) -> PubSubTransport[Any]:
    """Construct the active-backend pub/sub transport for a logical channel.

    A pickled (self-describing) transport is used when no `msg_type` is given or
    the type has no `lcm_encode`. Otherwise a typed transport is used.

    A channel name alone doesn't fully define a topic: backends have per-topic
    settings (Zenoh publisher QoS, for one) that live on their Topic objects.
    The factory fills those with `default_zenoh_qos`; a channel that needs more
    should pin an explicit transport built from a full Topic instead, e.g.
    `ZenohTransport(ZenohTopic("bla", Image, qos=...))` in the blueprint's
    transport map. LCM (UDP multicast) has no per-topic settings.
    """

    use_pickled = msg_type is None or getattr(msg_type, "lcm_encode", None) is None
    topic = transport_topic(name, g)
    if g.transport == "zenoh":
        ztopic = ZenohTopic(
            topic, None if use_pickled else msg_type, qos=default_zenoh_qos(name, msg_type)
        )
        return pZenohTransport(ztopic) if use_pickled else ZenohTransport(ztopic)
    if use_pickled:
        return pLCMTransport(topic)
    assert msg_type is not None  # not use_pickled implies a typed msg_type
    return LCMTransport(topic, msg_type)


def _transport_arg_error(argv: list[str], message: str) -> NoReturn:
    """Print an argparse-style CLI error for `--transport` and exit(2)."""
    prog = os.path.basename(argv[0]) if argv else "dimos"
    print(f"{prog}: error: {message}", file=sys.stderr)
    sys.exit(2)


def apply_transport_arg(argv: list[str], *, g: GlobalConfig = global_config) -> None:
    """Apply a `--transport <lcm|zenoh>` / `--transport=...` override from argv.

    Lets standalone CLIs (`humancli`, `agentspy`, `dtop`) flip the backend
    explicitly. Without it they follow `DIMOS_TRANSPORT` / `.env` via the
    global config, which is the single switch shared with the `dimos` process.

    A missing or invalid value exits(2) with a CLI-style message rather than
    letting the assignment raise a raw pydantic ValidationError (the field is
    validated on assignment).
    """
    choices = get_args(TransportBackend)
    for i, arg in enumerate(argv):
        if arg.startswith("--transport="):
            value = arg.split("=", 1)[1]
        elif arg == "--transport":
            if i + 1 >= len(argv) or argv[i + 1].startswith("-"):
                _transport_arg_error(argv, "argument --transport: expected one argument")
            value = argv[i + 1]
        else:
            continue
        if value not in choices:
            _transport_arg_error(
                argv,
                f"argument --transport: invalid choice: {value!r} "
                f"(choose from {', '.join(map(repr, choices))})",
            )
        g.update(transport=value)


def rpc_backend(g: GlobalConfig = global_config) -> type[RPCSpec]:
    """Return the RPC class (`LCMRPC` or `ZenohRPC`) for the active backend."""
    return ZenohRPC if g.transport == "zenoh" else LCMRPC


def tf_backend(g: GlobalConfig = global_config) -> type[TFSpec]:
    """Return the TF class (`LCMTF` or `ZenohTF`) for the active backend."""
    return ZenohTF if g.transport == "zenoh" else LCMTF
