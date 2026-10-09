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

"""Channel declarations for the relay bridge: the per-channel runtime
contract and the built-in channel tables that drive the no-manifest (auto)
mode and validate hand-written manifests."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Literal

from dimos.web.relay_bridge.manifest import Dir
from dimos.web.relay_bridge.protocol import Delivery

if TYPE_CHECKING:
    from dimos.web.relay_bridge.config import RelayBridgeConfig


@dataclass(frozen=True)
class RuntimeChannelSpec:
    """One channel's immutable runtime contract.

    Compiled by cockpit() in the parent (codecs resolved from the registry,
    ready to pickle by reference into the worker) or resolved from the
    manifest against BUILTIN_CHANNELS at module start. The bridge's rx
    encode behavior and its tx publish decode behavior are driven entirely
    by these specs.
    """

    ch: str
    message_type: type[Any]
    dir: Dir
    encoding: str
    delivery: Delivery
    max_hz: float
    params: dict[str, Any]
    publish: Literal["none", "shared", "exclusive"] = "none"
    required_scope: str | None = None
    # bytes | EncodedPayload | None (None skips the sample); None for tx.
    encoder: Callable[..., Any] | None = None
    encoder_takes_params: bool = False
    # browser JSON value (+ optional PublishContext) -> message; publish tx
    # channels only, None otherwise.
    decoder: Callable[..., Any] | None = None
    decoder_takes_context: bool = False
    # Keep an always-on raw-input cache (decode only, no encode) and replay
    # the newest message when the channel goes from zero viewers to some
    # viewer: a new session must not wait for the next publish (the producer
    # may have gone quiet, possibly before the first viewer ever attached).
    resend_on_subscribe: bool = False
    # Event channels (chat): the maxHz cap is met by spacing sends (a paced
    # FIFO on the loop), never by dropping, so a burst of messages crosses
    # complete and in order and a state flag keeps its newest value.
    paced: bool = False


def _no_default_params(config: RelayBridgeConfig) -> dict[str, Any]:
    return {}


def _jpeg_default_params(config: RelayBridgeConfig) -> dict[str, Any]:
    return {"quality": config.jpeg_quality}


@dataclass(frozen=True)
class BuiltinChannel:
    """Declaration of one static-port channel (no encoder here: codecs come
    from the dimos.web.codecs registry, the same one custom channels use).
    Drives the no-manifest (auto) mode and validates hand-written manifests;
    every entry needs a matching `In` on the module."""

    ch: str
    encoding: str
    delivery: Delivery
    max_hz: Callable[[RelayBridgeConfig], float]
    # Config-driven params merged under the manifest's (manifest wins), so
    # flat config fields keep working as fallbacks in every mode.
    default_params: Callable[[RelayBridgeConfig], dict[str, Any]] = _no_default_params
    resend_on_subscribe: bool = False


BUILTIN_CHANNELS: tuple[BuiltinChannel, ...] = (
    BuiltinChannel(
        "color_image", "jpeg.v1", "latest", lambda c: c.image_max_hz, _jpeg_default_params
    ),
    BuiltinChannel("odom", "pose.json.v1", "reliable", lambda c: c.odom_max_hz),
    BuiltinChannel(
        "global_costmap",
        "costmap.zlib.v1",
        "latest",
        lambda c: c.costmap_max_hz,
        resend_on_subscribe=True,
    ),
)

# The tx (viewer->robot) counterpart of BUILTIN_CHANNELS: stream ->
# (encoding, delivery). Every entry needs a matching `Out` on the module and
# a handler in RelayBridgeModule._supervise; it is also the delivery source
# for tx channels in authored manifests (dimos/web/cockpit.py).
TX_CHANNELS: tuple[tuple[str, str, Delivery], ...] = (("tele_cmd_vel", "twist.json.v1", "latest"),)
