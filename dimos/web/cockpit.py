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

"""Cockpit authoring: panels + layout that compile to a manifest-v1 dict.

Robot blueprints describe their cockpit here and the Cockpit web app (repo
root web/cockpit, not this module) renders whatever the manifest describes:

    unitree_go2_cockpit = autoconnect(
        unitree_go2,
        cockpit(layout=Row(Video("color_image"), Map2D(), shares=[2, 1])),
    )

The manifest dict travels through blueprint kwargs into RelayBridgeModule
(dimos/web/relay_bridge/), which advertises it to the relay. Authoring
errors (unknown streams, conflicting rates, malformed trees) raise at
blueprint definition time, not at robot start.

Import-light on purpose: the panel/layout classes work without the [web]
extra; only cockpit() touches the relay bridge module.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Mapping, Sequence, Set as AbstractSet
from dataclasses import dataclass, field, replace
import json
import math
from typing import TYPE_CHECKING, Any, ClassVar, Literal, Self

from dimos.web.relay_bridge.manifest import (
    MANIFEST_VERSION,
    MAX_MANIFEST_ID_LEN,
    RESERVED_CHANNEL_PREFIX,
    Delivery,
    Dir,
    parse_manifest,
)

if TYPE_CHECKING:
    from dimos.core.coordination.blueprints import Blueprint


def _check_stream(name: str, value: str) -> None:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{name} must be a non-empty stream name, got {value!r}")


def _check_rate(name: str, value: float) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number, got {value!r}")
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be a positive finite number, got {value!r}")


class _FrozenDict(dict[str, Any]):
    """Read-only dict for Channel.params: a frozen dataclass must not leak a
    mutable mapping whose edits would change what a later cockpit() emits.
    A dict subclass (not mappingproxy) so it stays picklable and
    json.dumps-able."""

    def __reduce__(self) -> tuple[Any, ...]:
        return (_FrozenDict, (dict(self),))

    def __setitem__(self, key: str, value: Any) -> None:
        raise TypeError("Channel params are immutable")

    def __delitem__(self, key: str) -> None:
        raise TypeError("Channel params are immutable")

    # dict.__ior__ mutates at the C level (bypassing update()), so it must be
    # blocked too; deliberately breaking the in-place contract is the whole
    # point here, which typeshed's dict overloads cannot express.
    def __ior__(self, other: Any) -> Self:  # type: ignore[override, misc]
        raise TypeError("Channel params are immutable")

    def clear(self) -> None:
        raise TypeError("Channel params are immutable")

    def pop(self, key: str, default: Any = None) -> Any:
        raise TypeError("Channel params are immutable")

    def popitem(self) -> tuple[str, Any]:
        raise TypeError("Channel params are immutable")

    def setdefault(self, key: str, default: Any = None) -> Any:
        raise TypeError("Channel params are immutable")

    def update(self, *args: Any, **kwargs: Any) -> None:
        raise TypeError("Channel params are immutable")


def _freeze_params(value: Any) -> Any:
    """Recursively immutable copy: dicts freeze, lists become tuples."""
    if isinstance(value, Mapping):
        return _FrozenDict({key: _freeze_params(item) for key, item in value.items()})
    if isinstance(value, (list, tuple)):
        return tuple(_freeze_params(item) for item in value)
    return value


def _thaw_params(value: Any) -> Any:
    """Plain-JSON deep copy of frozen params (manifests and runtime specs
    carry ordinary dicts/lists, and pinned manifests compare with ==)."""
    if isinstance(value, Mapping):
        return {key: _thaw_params(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_thaw_params(item) for item in value]
    return value


@dataclass(frozen=True)
class ChannelRequest:
    """One stream a panel needs advertised.

    The extension point for new panel types (chat, teleop, stats):
    a panel returns its requests from _channel_requests(), and the request
    order defines the panel's channel slot order in the manifest.
    """

    stream: str
    dir: Dir
    encoding: str
    max_hz: float
    params: Mapping[str, Any] = field(default_factory=dict)
    delivery: Delivery = field(default="reliable", kw_only=True)


@dataclass(frozen=True)
class Channel:
    """One explicitly exposed robot<->browser channel (Web SDK authoring).

    Declares a stream for custom frontends without requiring a cockpit
    panel; pass a sequence of these to cockpit(channels=[...]). A channel
    naming a stream some panel also requests merges with it: dir, encoding,
    delivery, and params must agree exactly, max_hz takes the max.

    `encoding` names a codec: a registered @web_encoder (dimos.web.codecs)
    whose message type must match `message_type`, or the generic "json.v1"
    for JSON-shaped types and dataclasses. dir/publish/required_scope beyond
    the rx defaults arrive with the publish ticket (W7).
    """

    stream: str
    message_type: type[Any]
    dir: Dir = field(default="rx", kw_only=True)
    encoding: str = field(default="json.v1", kw_only=True)
    delivery: Delivery = field(default="reliable", kw_only=True)
    max_hz: float = field(default=10.0, kw_only=True)
    params: Mapping[str, Any] | None = field(default=None, kw_only=True)
    publish: Literal["none", "shared", "exclusive"] = field(default="none", kw_only=True)
    required_scope: str | None = field(default=None, kw_only=True)
    # Replay the newest message to a channel's first viewer, so a page opened
    # mid-run shows current state instead of waiting for the next publish.
    resend_on_subscribe: bool = field(default=False, kw_only=True)
    # False = every message is encoded, maxHz notwithstanding: for an event
    # stream (a chat turn, a mode flip) a skipped sample is lost data, not a
    # dropped frame.
    rate_gate: bool = field(default=True, kw_only=True)
    # > 1 replays that many past messages to a first viewer (a transcript must
    # survive a page reload) instead of only the newest. The bridge numbers
    # the entries and ships the number as the frame's `n` meta.
    replay_depth: int = field(default=1, kw_only=True)

    def __post_init__(self) -> None:
        _check_stream("stream", self.stream)
        if len(self.stream) > MAX_MANIFEST_ID_LEN:
            raise ValueError(
                f"stream id {self.stream!r} is longer than {MAX_MANIFEST_ID_LEN} chars"
            )
        if self.stream.startswith(RESERVED_CHANNEL_PREFIX):
            raise ValueError(
                f"stream id {self.stream!r} is invalid: ids beginning with "
                f"{RESERVED_CHANNEL_PREFIX!r} are reserved for protocol control"
            )
        if not isinstance(self.message_type, type):
            raise TypeError(f"message_type must be a class, got {self.message_type!r}")
        if self.dir not in ("rx", "tx"):
            raise ValueError(f"dir must be 'rx' or 'tx', got {self.dir!r}")
        if self.dir == "tx":
            raise ValueError(
                'dir="tx" channels are not available yet: generic browser-to-robot '
                "publish is enabled by the publish ticket (W7)"
            )
        if not isinstance(self.encoding, str) or not 1 <= len(self.encoding) <= MAX_MANIFEST_ID_LEN:
            raise ValueError(
                f"encoding must be 1..{MAX_MANIFEST_ID_LEN} chars, got {self.encoding!r}"
            )
        if self.delivery not in ("latest", "reliable"):
            raise ValueError(f"delivery must be 'latest' or 'reliable', got {self.delivery!r}")
        _check_rate("max_hz", self.max_hz)
        if self.publish not in ("none", "shared", "exclusive"):
            raise ValueError(
                f"publish must be 'none', 'shared', or 'exclusive', got {self.publish!r}"
            )
        if self.publish != "none":
            raise ValueError("rx channels must use publish='none' (generic publish is tx-only, W7)")
        if self.required_scope is not None:
            raise ValueError(
                "rx channels cannot set required_scope (reserved for tx publish channels, W7)"
            )
        if not isinstance(self.replay_depth, int) or isinstance(self.replay_depth, bool):
            raise ValueError(f"replay_depth must be an int, got {self.replay_depth!r}")
        if self.replay_depth < 1:
            raise ValueError(f"replay_depth must be >= 1, got {self.replay_depth}")
        if self.replay_depth > 1 and not self.resend_on_subscribe:
            raise ValueError("replay_depth > 1 requires resend_on_subscribe=True")
        if self.params is not None and not isinstance(self.params, Mapping):
            raise ValueError(f"params must be a mapping or None, got {self.params!r}")
        params = {} if self.params is None else dict(self.params)
        if any(not isinstance(key, str) for key in params):
            raise ValueError("params keys must be strings")
        try:
            # Params ride the manifest verbatim; non-JSON values (bytes,
            # arrays) and non-finite numbers must fail at authoring, not at
            # manifest parse or on the wire.
            json.dumps(params, allow_nan=False)
        except (TypeError, ValueError) as e:
            raise ValueError(f"params must be JSON-shaped: {e}") from e
        object.__setattr__(self, "params", _freeze_params(params))


class Panel(ABC):
    """Base for cockpit panels. `kind` is the Cockpit component registry
    key; `title` "" means untitled (the panel frame falls back to the panel
    id)."""

    kind: ClassVar[str]
    title: str

    @abstractmethod
    def _channel_requests(self) -> tuple[ChannelRequest, ...]: ...

    def _panel_params(self) -> dict[str, Any]:
        return {}


@dataclass(frozen=True)
class Video(Panel):
    """JPEG video feed of one image stream."""

    kind: ClassVar[str] = "video"
    stream: str = "color_image"
    max_hz: float = field(default=30.0, kw_only=True)
    quality: int = field(default=75, kw_only=True)
    title: str = field(default="", kw_only=True)

    def __post_init__(self) -> None:
        _check_stream("stream", self.stream)
        _check_rate("max_hz", self.max_hz)
        if (
            isinstance(self.quality, bool)
            or not isinstance(self.quality, int)
            or not 0 <= self.quality <= 100
        ):
            raise ValueError(f"quality must be an int in 0..100, got {self.quality!r}")

    def _channel_requests(self) -> tuple[ChannelRequest, ...]:
        return (
            ChannelRequest(
                self.stream,
                "rx",
                "jpeg.v1",
                self.max_hz,
                {"quality": self.quality},
                delivery="latest",
            ),
        )


@dataclass(frozen=True)
class Map2D(Panel):
    """2D costmap with an optional pose overlay (pose=None drops it)."""

    kind: ClassVar[str] = "map2d"
    costmap: str = "global_costmap"
    pose: str | None = "odom"
    costmap_hz: float = field(default=5.0, kw_only=True)
    pose_hz: float = field(default=20.0, kw_only=True)
    title: str = field(default="", kw_only=True)

    def __post_init__(self) -> None:
        _check_stream("costmap", self.costmap)
        if self.pose is not None:
            _check_stream("pose", self.pose)
        _check_rate("costmap_hz", self.costmap_hz)
        _check_rate("pose_hz", self.pose_hz)

    def _channel_requests(self) -> tuple[ChannelRequest, ...]:
        requests = [
            ChannelRequest(
                self.costmap, "rx", "costmap.zlib.v1", self.costmap_hz, delivery="latest"
            )
        ]
        if self.pose is not None:
            requests.append(ChannelRequest(self.pose, "rx", "pose.json.v1", self.pose_hz))
        return tuple(requests)


@dataclass(frozen=True)
class Teleop(Panel):
    """Keyboard teleop: twist datagrams on one tx stream (WASD drive, Q/E
    strafe, Shift boost, Space e-stop; click the panel to arm).

    All params ride the tx channel in the manifest: the Cockpit machine
    reads speeds and cadence from there, the bridge reads watchdog_ms (its
    deadman window) and clamps incoming twists to max * boost.

    `mode` names a mode stream some other panel in the cockpit already
    subscribes (the store is keyed by channel, so the pad can read it without
    requesting it). Naming it lets the pad refuse to arm on a robot that
    ignores teleop in agent mode, instead of going green while the robot
    discards every key. Unset - every robot but the duck - keeps the pad
    exactly as it was: one tx channel, no extra slot, so the "teleop binds
    exactly one channel" manifest rule is untouched.
    """

    kind: ClassVar[str] = "teleop"
    stream: str = "tele_cmd_vel"
    max_linear: float = field(default=0.8, kw_only=True)
    max_angular: float = field(default=1.0, kw_only=True)
    boost: float = field(default=2.0, kw_only=True)
    publish_hz: float = field(default=15.0, kw_only=True)
    watchdog_ms: float = field(default=300.0, kw_only=True)
    mode: str | None = field(default=None, kw_only=True)
    title: str = field(default="", kw_only=True)

    def __post_init__(self) -> None:
        _check_stream("stream", self.stream)
        _check_rate("max_linear", self.max_linear)
        _check_rate("max_angular", self.max_angular)
        _check_rate("boost", self.boost)
        _check_rate("publish_hz", self.publish_hz)
        _check_rate("watchdog_ms", self.watchdog_ms)
        if self.mode is not None:
            _check_stream("mode", self.mode)

    def _channel_requests(self) -> tuple[ChannelRequest, ...]:
        return (
            ChannelRequest(
                self.stream,
                "tx",
                "twist.json.v1",
                self.publish_hz,
                {
                    "maxLinear": self.max_linear,
                    "maxAngular": self.max_angular,
                    "boost": self.boost,
                    "watchdogMs": self.watchdog_ms,
                },
                delivery="latest",
            ),
        )

    def _panel_params(self) -> dict[str, Any]:
        return {} if self.mode is None else {"mode": self.mode}


# The three panels below bind several streams each. Their role -> channel
# mapping rides `params` (keyed by role) so the web panel looks channels up
# by role instead of index-guessing; the channel slot order (request order)
# is what manifest.py / manifest.ts validate per kind. Rates are fixed: the
# bridge's channel table caps them anyway and none of these are tunable
# from the cockpit today.


@dataclass(frozen=True, kw_only=True)
class Chat(Panel):
    """Agent chat: the agent transcript, its idle flag and the control mode
    in; typed human input out."""

    kind: ClassVar[str] = "chat"
    chat: str = "agent"
    idle: str = "agent_idle"
    mode: str = "mode"
    input: str = "human_input"
    title: str = "Agent"

    def __post_init__(self) -> None:
        _check_stream("chat", self.chat)
        _check_stream("idle", self.idle)
        _check_stream("mode", self.mode)
        _check_stream("input", self.input)

    def _channel_requests(self) -> tuple[ChannelRequest, ...]:
        return (
            ChannelRequest(self.chat, "rx", "chat.json.v1", 30.0),
            ChannelRequest(self.idle, "rx", "flag.json.v1", 10.0),
            ChannelRequest(self.mode, "rx", "mode.json.v1", 10.0),
            ChannelRequest(self.input, "tx", "text.json.v1", 2.0),
        )

    def _panel_params(self) -> dict[str, Any]:
        return {"chat": self.chat, "idle": self.idle, "mode": self.mode, "input": self.input}


@dataclass(frozen=True, kw_only=True)
class NavMap(Panel):
    """Costmap with pose, planned path, named places and nav state overlays;
    click-to-goal and cancel go out as goal poses / UI commands."""

    kind: ClassVar[str] = "navmap"
    costmap: str = "global_costmap"
    pose: str = "odom"
    path: str = "path"
    places: str = "places"
    nav_state: str = "nav_state"
    goal: str = "goal_request"
    command: str = "ui_command"
    title: str = "Nav map"

    def __post_init__(self) -> None:
        _check_stream("costmap", self.costmap)
        _check_stream("pose", self.pose)
        _check_stream("path", self.path)
        _check_stream("places", self.places)
        _check_stream("nav_state", self.nav_state)
        _check_stream("goal", self.goal)
        _check_stream("command", self.command)

    def _channel_requests(self) -> tuple[ChannelRequest, ...]:
        return (
            ChannelRequest(self.costmap, "rx", "costmap.zlib.v1", 2.0, delivery="latest"),
            ChannelRequest(self.pose, "rx", "pose.json.v1", 10.0),
            ChannelRequest(self.path, "rx", "path.json.v1", 5.0, delivery="latest"),
            ChannelRequest(self.places, "rx", "places.json.v1", 2.0),
            # State, not a frame stream: reliable. A latest channel costs one
            # relay->viewer stream per frame out of a budget shared with the
            # cameras (web/README.md bug 12), and a dropped nav_state would
            # leave the chip showing a state the robot has already left.
            ChannelRequest(self.nav_state, "rx", "navstate.json.v1", 10.0),
            ChannelRequest(self.goal, "tx", "pose_goal.json.v1", 5.0),
            ChannelRequest(self.command, "tx", "command.json.v1", 10.0),
        )

    def _panel_params(self) -> dict[str, Any]:
        return {
            "costmap": self.costmap,
            "pose": self.pose,
            "path": self.path,
            "places": self.places,
            "navState": self.nav_state,
            "goal": self.goal,
            "command": self.command,
        }


@dataclass(frozen=True, kw_only=True)
class Control(Panel):
    """Top bar: teleop/agent mode switch, policy buttons and nav status in;
    UI commands (set_mode / policy / cancel_nav) out."""

    kind: ClassVar[str] = "control"
    mode: str = "mode"
    policies: str = "policy_state"
    nav_state: str = "nav_state"
    command: str = "ui_command"
    title: str = "Control"

    def __post_init__(self) -> None:
        _check_stream("mode", self.mode)
        _check_stream("policies", self.policies)
        _check_stream("nav_state", self.nav_state)
        _check_stream("command", self.command)

    def _channel_requests(self) -> tuple[ChannelRequest, ...]:
        return (
            ChannelRequest(self.mode, "rx", "mode.json.v1", 10.0),
            # Both are state, not frames: see the note in NavMap.
            ChannelRequest(self.policies, "rx", "policy.json.v1", 10.0),
            ChannelRequest(self.nav_state, "rx", "navstate.json.v1", 10.0),
            ChannelRequest(self.command, "tx", "command.json.v1", 10.0),
        )

    def _panel_params(self) -> dict[str, Any]:
        return {
            "mode": self.mode,
            "policies": self.policies,
            "navState": self.nav_state,
            "command": self.command,
        }


class _Split:
    """Base for Row/Col: children plus optional flex shares."""

    def __init__(self, *children: Panel | _Split, shares: Sequence[float] | None = None) -> None:
        if not children:
            raise ValueError(f"{type(self).__name__} needs at least one child")
        for child in children:
            if not isinstance(child, (Panel, _Split)):
                raise ValueError(
                    f"{type(self).__name__} children must be panels or Row/Col, got {child!r}"
                )
        checked: tuple[float, ...] | None = None
        if shares is not None:
            checked = tuple(shares)
            if len(checked) != len(children):
                raise ValueError(
                    f"shares needs one entry per child, got {len(checked)} for "
                    f"{len(children)} children"
                )
            for share in checked:
                _check_rate("share", share)
        self.children = children
        self.shares = checked


class Row(_Split):
    """Horizontal split; absent shares mean an equal split."""


class Col(_Split):
    """Vertical split; absent shares mean an equal split."""


def _default_preset() -> Row:
    """The no-layout cockpit: video left (2/3); costmap+pose over teleop
    right (1/3)."""
    return Row(
        Video("color_image"),
        Col(Map2D(costmap="global_costmap", pose="odom"), Teleop(), shares=[3, 1]),
        shares=[2, 1],
    )


def build_manifest_data(
    layout: Panel | Row | Col | None,
    pages: Sequence[Panel],
    *,
    registry: Mapping[str, tuple[str, str]],
    tx_streams: AbstractSet[str],
    tx_registry: Mapping[str, tuple[str, str]] | None = None,
    channels: Sequence[ChannelRequest] = (),
    extra_channels: Sequence[ChannelRequest] = (),
) -> dict[str, Any]:
    """Compile a layout tree + pages + explicit channels into a manifest-v1
    dict.

    Shared by cockpit() and the bridge's availability-driven default
    manifest (default_manifest in relay_bridge_module.py). `registry` and
    `tx_registry` map built-in stream name -> (encoding, delivery) in
    advertisement order (the bridge's BUILTIN_CHANNELS/TX_CHANNELS tables);
    tx_streams are the module's typed Out names. Panel ids are p0..pN in
    tree order (layout depth-first, then pages).

    Panel requests and `channels` (cockpit(channels=...) declarations) run
    through one merge: dir, encoding, delivery, and params must agree,
    max_hz takes the max. An rx stream outside `registry` is legal only when
    `channels` declares it (the caller resolves its codec and port type).
    `extra_channels` are advertised channel-only unless something already
    requested the stream.

    Advertisement order: rx built-ins in registry order, then rx customs in
    first-declaration order, then tx in tx_registry order - so manifests
    without custom channels keep their historical channel order.
    """
    tx_registry = {} if tx_registry is None else tx_registry
    merged: dict[str, ChannelRequest] = {}
    panels_out: list[dict[str, Any]] = []

    def merge(request: ChannelRequest) -> None:
        previous = merged.get(request.stream)
        if previous is None:
            merged[request.stream] = request
            return
        same = (previous.dir, previous.encoding, previous.delivery, dict(previous.params)) == (
            request.dir,
            request.encoding,
            request.delivery,
            dict(request.params),
        )
        if not same:
            raise ValueError(
                f"conflicting requirements for stream {request.stream!r}: "
                f"{previous!r} vs {request!r}"
            )
        if request.max_hz > previous.max_hz:
            merged[request.stream] = replace(previous, max_hz=request.max_hz)

    def add_panel(panel: Panel) -> str:
        panel_id = f"p{len(panels_out)}"
        requests = panel._channel_requests()
        for request in requests:
            merge(request)
        panels_out.append(
            {
                "id": panel_id,
                "kind": panel.kind,
                "title": panel.title,
                "channels": [request.stream for request in requests],
                "params": panel._panel_params(),
            }
        )
        return panel_id

    def emit(node: Panel | _Split) -> Any:
        if isinstance(node, _Split):
            out: dict[str, Any] = {
                "row" if isinstance(node, Row) else "col": [emit(child) for child in node.children]
            }
            if node.shares is not None:
                out["shares"] = list(node.shares)
            return out
        if isinstance(node, Panel):
            return add_panel(node)
        raise ValueError(f"layout must be a panel or Row/Col, got {node!r}")

    layout_node = None if layout is None else emit(layout)
    page_ids = []
    for page in pages:
        if not isinstance(page, Panel):
            raise ValueError(f"pages entries must be panels, got {page!r}")
        page_ids.append(add_panel(page))

    for request in channels:
        merge(request)

    declared = {request.stream for request in channels}
    for stream, request in merged.items():
        if request.dir == "rx":
            if stream in registry:
                expected_encoding, expected_delivery = registry[stream]
                if expected_encoding != request.encoding:
                    raise ValueError(
                        f"stream {stream!r} encodes {expected_encoding}, not {request.encoding} "
                        f"(wrong panel type for this stream?)"
                    )
                if expected_delivery != request.delivery:
                    raise ValueError(
                        f"stream {stream!r} delivers {expected_delivery}, not {request.delivery}"
                    )
            elif stream not in declared:
                raise ValueError(
                    f"unknown stream {stream!r}; this robot bridge supports: "
                    f"{', '.join(sorted(set(registry) | declared))}"
                )
        else:
            if stream not in tx_streams:
                raise ValueError(
                    f"unknown tx stream {stream!r}; this robot bridge has no matching "
                    f"output stream (outputs: {sorted(tx_streams) or 'none'})"
                )
            expected = tx_registry.get(stream)
            if expected is None:
                raise ValueError(
                    f"tx stream {stream!r} has no channel-table entry; this bridge "
                    f"sends: {sorted(tx_registry) or 'none'}"
                )
            if expected[0] != request.encoding:
                raise ValueError(
                    f"stream {stream!r} encodes {expected[0]}, not {request.encoding} "
                    f"(wrong panel type for this stream?)"
                )
            if expected[1] != request.delivery:
                raise ValueError(
                    f"stream {stream!r} delivers {expected[1]}, not {request.delivery}"
                )

    for request in extra_channels:
        if request.stream not in merged:
            merged[request.stream] = request

    ordered = [merged[stream] for stream in registry if stream in merged]
    ordered += [
        request
        for stream, request in merged.items()
        if request.dir == "rx" and stream not in registry
    ]
    ordered += [merged[stream] for stream in tx_registry if stream in merged]
    return {
        "version": MANIFEST_VERSION,
        "channels": [
            {
                "ch": request.stream,
                "dir": request.dir,
                "encoding": request.encoding,
                "delivery": request.delivery,
                "maxHz": request.max_hz,
                "params": dict(request.params),
            }
            for request in ordered
        ],
        "panels": panels_out,
        "layout": layout_node,
        "pages": page_ids,
    }


def cockpit(
    layout: Panel | Row | Col | None = None,
    pages: Sequence[Panel] = (),
    channels: Sequence[Channel] = (),
) -> Blueprint:
    """Cockpit blueprint for the given layout (default: `_default_preset`)
    plus explicitly exposed channels.

    Walks the tree, merges panel requests with the `channels` declarations,
    compiles the manifest, validates it eagerly, resolves every rx channel's
    codec, and returns a relay bridge blueprint carrying it all; compose
    onto a robot with `autoconnect(robot_blueprint, cockpit(...))`. Channels
    whose stream is not a built-in bridge port get a generated
    RelayBridgeModule subclass with matching typed ports (autoconnected by
    name + message type). Streams whose producer never publishes are still
    advertised: their panels show "waiting for data" (no runtime stream
    probing).

    The default preset layout applies only when neither a layout nor
    channels are given; cockpit(channels=[...]) alone compiles with no
    panels and no layout.
    """
    declared: dict[str, Channel] = {}
    for entry in channels:
        if not isinstance(entry, Channel):
            raise ValueError(f"channels entries must be Channel, got {entry!r}")
        if entry.stream in declared:
            raise ValueError(f"duplicate channel declaration for stream {entry.stream!r}")
        declared[entry.stream] = entry
    try:
        from dimos.web.relay_bridge.dynamic import DynamicPortSpec, make_relay_bridge_class
        from dimos.web.relay_bridge.relay_bridge_module import (
            BUILTIN_CHANNELS,
            TX_CHANNELS,
            RelayBridgeModule,
            RuntimeChannelSpec,
        )
    except ImportError as e:
        raise RuntimeError(
            "the cockpit blueprint needs the web extra: `uv sync --extra web --inexact`"
        ) from e
    from dimos.core.coordination.blueprints import autoconnect
    from dimos.web.codecs import resolve_encoder

    atom = RelayBridgeModule.blueprint().blueprints[0]
    port_types = {s.name: s.type for s in atom.streams}
    builtin_by_ch = {b.ch: b for b in BUILTIN_CHANNELS}
    data = build_manifest_data(
        _default_preset() if layout is None and not declared else layout,
        tuple(pages),
        registry={b.ch: (b.encoding, b.delivery) for b in BUILTIN_CHANNELS},
        tx_streams={s.name for s in atom.streams if s.direction == "out"},
        tx_registry={ch: (encoding, delivery) for ch, encoding, delivery in TX_CHANNELS},
        channels=tuple(
            ChannelRequest(
                c.stream,
                c.dir,
                c.encoding,
                c.max_hz,
                # Deep plain copy: the manifest and specs must not alias the
                # (frozen) authoring record's nested values.
                _thaw_params(c.params or {}),
                delivery=c.delivery,
            )
            for c in declared.values()
        ),
    )
    # The domain parser is the authority; authoring bugs must fail at
    # blueprint definition time, not at robot start.
    parse_manifest(data)

    specs: list[RuntimeChannelSpec] = []
    ports: list[DynamicPortSpec] = []
    for wire in data["channels"]:
        if wire["dir"] != "rx":
            continue
        ch = wire["ch"]
        explicit = declared.get(ch)
        builtin = builtin_by_ch.get(ch)
        if builtin is not None:
            message_type = port_types[ch]
            if explicit is not None and explicit.message_type is not message_type:
                raise ValueError(
                    f"channel {ch!r}: message type {explicit.message_type.__qualname__} "
                    f"does not match the bridge port type {message_type.__qualname__}"
                )
        else:
            assert explicit is not None  # build_manifest_data rejected the rest
            message_type = explicit.message_type
            ports.append(DynamicPortSpec(ch, message_type, "rx"))
        try:
            codec = resolve_encoder(wire["encoding"], message_type)
            if codec.check_params is not None:
                codec.check_params(wire["params"])
        except ValueError as e:
            raise ValueError(f"channel {ch!r}: {e}") from e
        specs.append(
            RuntimeChannelSpec(
                ch=ch,
                message_type=message_type,
                dir="rx",
                encoding=wire["encoding"],
                delivery=wire["delivery"],
                max_hz=float(wire["maxHz"]),
                params=dict(wire["params"]),
                encoder=codec.encode,
                encoder_takes_params=codec.takes_params,
                resend_on_subscribe=(
                    builtin.resend_on_subscribe
                    if builtin is not None
                    else explicit is not None and explicit.resend_on_subscribe
                ),
                rate_gate=explicit.rate_gate if explicit is not None else True,
                replay_depth=explicit.replay_depth if explicit is not None else 1,
            )
        )
    # Generated classes carry only the custom ports; the built-ins are
    # inherited. No custom streams means the plain static class.
    module_class = make_relay_bridge_class(ports) if ports else RelayBridgeModule
    return autoconnect(module_class.blueprint(manifest=data, channels=tuple(specs)))
