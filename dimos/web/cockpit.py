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
import math
from typing import TYPE_CHECKING, Any, ClassVar

from dimos.web.relay_bridge.manifest import MANIFEST_VERSION, Dir, parse_manifest

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
            ChannelRequest(self.stream, "rx", "jpeg.v1", self.max_hz, {"quality": self.quality}),
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
        requests = [ChannelRequest(self.costmap, "rx", "costmap.zlib.v1", self.costmap_hz)]
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
    """

    kind: ClassVar[str] = "teleop"
    stream: str = "tele_cmd_vel"
    max_linear: float = field(default=0.8, kw_only=True)
    max_angular: float = field(default=1.0, kw_only=True)
    boost: float = field(default=2.0, kw_only=True)
    publish_hz: float = field(default=15.0, kw_only=True)
    watchdog_ms: float = field(default=300.0, kw_only=True)
    title: str = field(default="", kw_only=True)

    def __post_init__(self) -> None:
        _check_stream("stream", self.stream)
        _check_rate("max_linear", self.max_linear)
        _check_rate("max_angular", self.max_angular)
        _check_rate("boost", self.boost)
        _check_rate("publish_hz", self.publish_hz)
        _check_rate("watchdog_ms", self.watchdog_ms)

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
            ),
        )


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
            ChannelRequest(self.costmap, "rx", "costmap.zlib.v1", 2.0),
            ChannelRequest(self.pose, "rx", "pose.json.v1", 10.0),
            ChannelRequest(self.path, "rx", "path.json.v1", 5.0),
            ChannelRequest(self.places, "rx", "places.json.v1", 2.0),
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
    rx_streams: AbstractSet[str],
    tx_streams: AbstractSet[str],
    tx_registry: Mapping[str, tuple[str, str]] | None = None,
    extra_channels: Sequence[ChannelRequest] = (),
) -> dict[str, Any]:
    """Compile a layout tree + pages into a plain manifest-v1 dict.

    Shared by cockpit() and the bridge's availability-driven default
    manifest (default_manifest in relay_bridge_module.py). `registry` and
    `tx_registry` map stream name -> (encoding, delivery) in advertisement
    order (the bridge's CHANNELS/TX_CHANNELS tables); rx/tx_streams are the
    module's typed In/Out names. Panel ids are p0..pN in tree order (layout
    depth-first, then pages). Channels requested by several panels merge:
    max_hz takes the max, any other disagreement raises. `extra_channels`
    are advertised channel-only unless a panel already requested the stream.
    """
    tx_registry = {} if tx_registry is None else tx_registry
    channels: dict[str, ChannelRequest] = {}
    panels_out: list[dict[str, Any]] = []

    def merge(request: ChannelRequest) -> None:
        previous = channels.get(request.stream)
        if previous is None:
            channels[request.stream] = request
            return
        same = (previous.dir, previous.encoding, dict(previous.params)) == (
            request.dir,
            request.encoding,
            dict(request.params),
        )
        if not same:
            raise ValueError(
                f"conflicting requirements for stream {request.stream!r}: "
                f"{previous!r} vs {request!r}"
            )
        if request.max_hz > previous.max_hz:
            channels[request.stream] = replace(previous, max_hz=request.max_hz)

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

    for stream, request in channels.items():
        if request.dir == "rx":
            if stream not in rx_streams or stream not in registry:
                raise ValueError(
                    f"unknown stream {stream!r}; this robot bridge supports: "
                    f"{', '.join(sorted(registry))}"
                )
            expected_encoding = registry[stream][0]
            if expected_encoding != request.encoding:
                raise ValueError(
                    f"stream {stream!r} encodes {expected_encoding}, not {request.encoding} "
                    f"(wrong panel type for this stream?)"
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

    for request in extra_channels:
        if request.stream not in channels:
            channels[request.stream] = request

    # Registry order, not first-request order: the advertisement order is the
    # bridge's channel-table order (rx table, then tx table) however panels
    # were nested.
    ordered = [channels[stream] for stream in registry if stream in channels]
    ordered += [channels[stream] for stream in tx_registry if stream in channels]
    delivery = {
        stream: pair[1] for table in (registry, tx_registry) for stream, pair in table.items()
    }
    return {
        "version": MANIFEST_VERSION,
        "channels": [
            {
                "ch": request.stream,
                "dir": request.dir,
                "encoding": request.encoding,
                "delivery": delivery[request.stream],
                "maxHz": request.max_hz,
                "params": dict(request.params),
            }
            for request in ordered
        ],
        "panels": panels_out,
        "layout": layout_node,
        "pages": page_ids,
    }


def _tx_registry(table: Sequence[Any]) -> dict[str, tuple[str, str]]:
    """stream -> (encoding, delivery) from the bridge's tx channel table,
    whose rows are `(ch, encoding, delivery)` tuples or channel-def objects
    with `.ch/.encoding/.delivery` attributes."""
    registry: dict[str, tuple[str, str]] = {}
    for row in table:
        if isinstance(row, tuple):
            ch, encoding, delivery = row[0], row[1], row[2]
        else:
            ch, encoding, delivery = row.ch, row.encoding, row.delivery
        registry[ch] = (encoding, delivery)
    return registry


def cockpit(layout: Panel | Row | Col | None = None, pages: Sequence[Panel] = ()) -> Blueprint:
    """Cockpit blueprint for the given layout (default: `_default_preset`).

    Walks the tree, compiles the manifest, validates it eagerly, and returns
    a RelayBridgeModule blueprint carrying it; compose onto a robot with
    `autoconnect(robot_blueprint, cockpit(...))`. Streams whose producer
    never publishes are still advertised: their panels show "waiting for
    data" (no runtime stream probing).
    """
    try:
        from dimos.web.relay_bridge.relay_bridge_module import (
            CHANNELS,
            TX_CHANNELS,
            RelayBridgeModule,
        )
    except ImportError as e:
        raise RuntimeError(
            "the cockpit blueprint needs the web extra: `uv sync --extra web --inexact`"
        ) from e
    from dimos.core.coordination.blueprints import autoconnect

    atom = RelayBridgeModule.blueprint().blueprints[0]
    data = build_manifest_data(
        _default_preset() if layout is None else layout,
        tuple(pages),
        registry={cd.ch: (cd.encoding, cd.delivery) for cd in CHANNELS},
        rx_streams={s.name for s in atom.streams if s.direction == "in"},
        tx_streams={s.name for s in atom.streams if s.direction == "out"},
        tx_registry=_tx_registry(TX_CHANNELS),
    )
    # The domain parser is the authority; authoring bugs must fail at
    # blueprint definition time, not at robot start.
    parse_manifest(data)
    return autoconnect(RelayBridgeModule.blueprint(manifest=data))
