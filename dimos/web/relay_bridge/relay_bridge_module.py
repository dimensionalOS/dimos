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

"""RelayBridgeModule: the robot side of the cockpit relay.

Registers the robot (id + manifest) with a relay - spawned locally in
--local-relay mode, or a remote one via relay_url - and forwards robot streams
to it. The advertised channels/panels/layout come from the manifest: authored
by a cockpit(...) blueprint (dimos/web/cockpit.py), or built at start by
default_manifest() from whatever inputs are available. Encoding is lazy: an
input is subscribed for encoding, and frames are encoded, only while the relay
reports at least one viewer subscribed to that channel, so a robot with no
open cockpit does no encode work. Channels with resend_on_subscribe
additionally keep one always-on raw subscription (decode only, never encode)
so the newest message can be replayed the moment a channel gains its first
viewer, even when the producer went quiet before that.

Threading: input callbacks fire on the transport (LCM) thread, which gates on
maxHz and encodes there (RerunBridge precedent, ~3 ms per JPEG), then hands
the payload to the module event loop; all relay/session state lives on the
loop. The supervisor task consumes relay subs snapshots and survives relay
restarts (respawning the local child when it died).
"""

from __future__ import annotations

import asyncio
from collections import deque
from collections.abc import AsyncIterator, Callable, Collection
from dataclasses import dataclass, field, replace
import functools
import json
import math
from pathlib import Path
import socket
import threading
import time
from typing import Any, Literal, TypeVar
import webbrowser

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.logging_config import setup_logger

# No import cycle: cockpit.py only imports this module lazily inside
# cockpit(), and its own module body is relay-free.
from dimos.web.cockpit import (
    ChannelRequest,
    Col,
    Map2D,
    Panel,
    Row,
    Teleop,
    Video,
    build_manifest_data,
)
from dimos.web.codecs import EncodedPayload, PublishContext, encoder_definition

# Imported for its registration side effect: the built-in encoders must be in
# the codec registry wherever this module runs (parent and worker).
from dimos.web.relay_bridge import builtin_codecs  # noqa: F401
from dimos.web.relay_bridge.locate import find_web_dir
from dimos.web.relay_bridge.manifest import Dir, parse_manifest
from dimos.web.relay_bridge.protocol import (
    MAX_REQUEST_ID_LEN,
    ChannelSpec,
    DataFrame,
    Delivery,
    Msg,
    PubAck,
    PubNack,
    RobotInfo,
    RobotManifest,
    Stop as WireStop,
    Subs,
    TeleopStart as WireTeleopStart,
    TeleopStop as WireTeleopStop,
    Twist as WireTwist,
)
from dimos.web.relay_bridge.relay_process import RelayProcess, ensure_web_dist
from dimos.web.relay_bridge.wt_client import (
    RelayClient,
    RelayRejectedError,
    connect_with_backoff,
)

logger = setup_logger()

_T = TypeVar("_T")
_FrameMeta = dict[str, Any] | None
# (payload, meta, ts); ts is None for live frames (stamped at send) and the
# source arrival time for replays, so a stale replay is honest about its age.
_Sender = Callable[[bytes, _FrameMeta, float | None], None]

_RECONNECT_PAUSE_S = 2.0

# A SIGKILLed relay child sends no CONNECTION_CLOSE, so the QUIC session only
# notices at idle timeout (tens of seconds). The child watchdog polls the
# process instead and force-closes the session to trigger a prompt respawn.
_CHILD_POLL_S = 1.0

# Bounded wait for the build worker thread after cancelling it (the child
# dies within the SIGTERM-to-SIGKILL grace; this adds a margin on top).
_BUILD_CANCEL_WAIT_S = 8.0

# Deadman poll granularity; small against the default 300 ms watchdog window
# so the zero lands close to the deadline.
_TELEOP_POLL_S = 0.05

# Per-channel floor between "encoder failed" logs (a broken encoder on a
# 30 Hz stream must not flood the log).
_ENCODE_ERROR_LOG_S = 5.0


@dataclass(frozen=True)
class _TeleopParams:
    """Teleop tx channel params resolved at start (manifest, with defaults)."""

    max_linear: float
    max_angular: float
    boost: float
    watchdog_s: float


# Manifest param keys and their defaults (matching the Teleop panel's).
_TELEOP_PARAM_DEFAULTS = {"maxLinear": 0.8, "maxAngular": 1.0, "boost": 2.0, "watchdogMs": 300.0}


def _clamp(value: float, bound: float) -> float:
    return max(-bound, min(bound, value))


def _probe_local_port(port: int) -> None:
    if port == 0:
        return
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
            # SO_REUSEADDR matches how the relay itself binds: a live
            # listener still fails the probe, but the FIN_WAIT/TIME_WAIT
            # remnants of a just-killed relay (a browser tab was
            # attached) must not block an immediate restart.
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            probe.bind(("127.0.0.1", port))
    except OSError as e:
        raise RuntimeError(f"cannot start local relay: port {port} is unavailable") from e


async def _blocking_call(func: Callable[..., _T], *args: Any) -> _T:
    """Run blocking process work without abandoning its thread on cancellation."""
    work = asyncio.create_task(asyncio.to_thread(func, *args))
    cancelled: asyncio.CancelledError | None = None
    while not work.done():
        try:
            await asyncio.shield(work)
        except asyncio.CancelledError as exc:
            cancelled = cancelled or exc
        except BaseException:
            break
    try:
        result = work.result()
    except BaseException as exc:
        if cancelled is not None:
            raise cancelled from exc
        raise
    if cancelled is not None:
        raise cancelled
    return result


async def _cancel_task(task: asyncio.Task[None] | None, name: str) -> None:
    if task is None:
        return
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass
    except Exception:
        # A task that already died re-raises here; teardown must continue.
        logger.exception(f"relay bridge {name} task failed during teardown")


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


class RelayBridgeConfig(ModuleConfig):
    relay_url: str | None = None
    """Attach to a running relay (wtUrl). None: spawn a local one."""
    local_port: int = 7780
    """HTTP port of the spawned local relay; 0 picks an ephemeral port (tests)."""
    open_browser: bool = True
    """Open the local relay's page once it is up (local mode only)."""
    web_build: bool = True
    """Build the web dists (SDK bundle + Cockpit) before spawning the local
    relay when they are missing or stale (checkouts only; wheels ship them
    pre-built)."""
    serve_dir: str | None = None
    """Directory the spawned local relay serves at / instead of the Cockpit
    (index.html for /); /api/* and /sdk.js keep precedence over it. Local
    relay only: rejected when relay_url attaches to an existing relay."""
    robot_id: str = ""
    """Relay identity; empty falls back to g.robot_id, then the hostname."""
    robot_name: str = ""
    """Display name; empty falls back to robot_id."""
    jpeg_quality: int = Field(default=75, ge=0, le=100)
    # MuJoCo publishes video at 20 Hz. Keep enough headroom for that source and
    # for camera jitter: a cap close to the nominal rate aliases slightly early
    # frames into an every-other-frame pattern.
    image_max_hz: float = Field(default=30.0, gt=0.0)
    odom_max_hz: float = Field(default=20.0, gt=0.0)
    costmap_max_hz: float = Field(default=5.0, gt=0.0)
    """Full-grid zlib frames; the go2 mapper publishes at ~7.6 Hz."""
    available_channels: tuple[str, ...] | None = None
    """Composition-provided channel allowlist for the no-manifest (auto)
    mode; None derives from bound inputs. Ignored when `manifest` is set."""
    manifest: dict[str, Any] | None = None
    """Full manifest-v1 dict (see dimos.web.cockpit): defines the advertised
    channels/panels/layout verbatim, with per-channel rates (maxHz) and jpeg
    quality (params) overriding the flat rate/quality fields above. None:
    default_manifest() builds one at start from the available inputs and
    those fields."""
    channels: tuple[RuntimeChannelSpec, ...] | None = None
    """Compiled rx runtime specs, set by cockpit() alongside `manifest` (they
    are authored together and cross-checked at start). None: encoders resolve
    from the manifest against BUILTIN_CHANNELS."""


def _passes_rate_gate(
    last_input: dict[str, float],
    ch: str,
    now: float,
    min_interval: float,
) -> bool:
    """Claim the current input when it is outside the channel's rate interval."""
    if now - last_input.get(ch, 0.0) < min_interval:
        return False
    last_input[ch] = now
    return True


def _matches_message_type(value: Any, message_type: type[Any]) -> bool:
    """Decoded-result check with JSON's number/bool subtleties: bool is exact
    (Python bool subclasses int), int excludes bool, float accepts int (JSON
    has one number type) but not bool."""
    if message_type is bool:
        return isinstance(value, bool)
    if message_type is int:
        return isinstance(value, int) and not isinstance(value, bool)
    if message_type is float:
        return isinstance(value, (int, float)) and not isinstance(value, bool)
    return isinstance(value, message_type)


# Publish values nesting deeper than this are rejected before json.loads; the
# SDK enforces the same cap, so both ends agree on what "too deep" means.
_MAX_PUB_DEPTH = 100


def _pub_depth_ok(payload: bytes) -> bool:
    """True when the JSON payload's bracket nesting stays within
    _MAX_PUB_DEPTH (string contents are skipped, so braces in text never
    count). Keeps deep-but-valid JSON from reaching json.loads, whose own
    depth bound is a RecursionError near the interpreter limit."""
    depth = 0
    in_string = False
    escaped = False
    for byte in payload:
        if in_string:
            if escaped:
                escaped = False
            elif byte == 0x5C:  # backslash
                escaped = True
            elif byte == 0x22:  # quote
                in_string = False
        elif byte == 0x22:  # quote
            in_string = True
        elif byte in (0x5B, 0x7B):  # [ {
            depth += 1
            if depth > _MAX_PUB_DEPTH:
                return False
        elif byte in (0x5D, 0x7D):  # ] }
            depth -= 1
    return True


def _parse_pub_meta(meta: dict[str, Any] | None) -> tuple[str, str, float, float | None] | None:
    """(request id, principal, relayTs, clientTs) from a publish frame's
    relay-stamped meta; None when the shape is unusable (a compliant relay
    never produces one)."""
    if not isinstance(meta, dict):
        return None
    request_id = meta.get("id")
    principal = meta.get("principal")
    relay_ts = meta.get("relayTs")
    client_ts = meta.get("clientTs")
    if not isinstance(request_id, str) or not 1 <= len(request_id) <= MAX_REQUEST_ID_LEN:
        return None
    if not isinstance(principal, str):
        return None
    if isinstance(relay_ts, bool) or not isinstance(relay_ts, (int, float)):
        return None
    if client_ts is not None and (
        isinstance(client_ts, bool) or not isinstance(client_ts, (int, float))
    ):
        return None
    return request_id, principal, float(relay_ts), None if client_ts is None else float(client_ts)


@dataclass(slots=True)
class _Session:
    client: RelayClient
    senders: dict[str, _Sender]
    last_n: int | float = 0
    unsubs: dict[str, Callable[[], None]] = field(default_factory=dict)
    retired: threading.Event = field(default_factory=threading.Event)


# A producer sustaining more than maxHz for this many frames is pathological
# (the agent chats at a few messages a second); beyond it the oldest go.
_PACED_QUEUE_MAX = 256


class _PacedSender:
    """Send cap by spacing, not sampling: frames queue on the loop and go out
    one per interval, in order, so an event channel (chat) never thins a
    burst. Wraps the channel's real sender."""

    def __init__(self, loop: asyncio.AbstractEventLoop, interval: float, send: _Sender) -> None:
        self._loop = loop
        self._interval = interval
        self._send = send
        self._queue: deque[tuple[bytes, _FrameMeta, float | None]] = deque()
        self._due = 0.0
        self._handle: asyncio.TimerHandle | None = None
        self.dropped = 0

    def __call__(self, payload: bytes, meta: _FrameMeta, ts: float | None) -> None:
        self._queue.append((payload, meta, ts))
        if len(self._queue) > _PACED_QUEUE_MAX:
            self._queue.popleft()
            self.dropped += 1
        if self._handle is None:
            self._drain()

    def _drain(self) -> None:
        self._handle = None
        now = self._loop.time()
        if now < self._due:
            self._handle = self._loop.call_at(self._due, self._drain)
            return
        payload, meta, ts = self._queue.popleft()
        self._due = now + self._interval
        if self._queue:
            self._handle = self._loop.call_at(self._due, self._drain)
        try:
            self._send(payload, meta, ts)
        except Exception:
            return  # session mid-teardown, same as _offer

    def close(self) -> None:
        if self._handle is not None:
            self._handle.cancel()
            self._handle = None
        self._queue.clear()


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
# a handler in _supervise; it is also the delivery source for tx channels in
# authored manifests (dimos/web/cockpit.py).
TX_CHANNELS: tuple[tuple[str, str, Delivery], ...] = (("tele_cmd_vel", "twist.json.v1", "latest"),)


def default_manifest(config: RelayBridgeConfig, available: Collection[str]) -> dict[str, Any]:
    """Availability-driven default cockpit: video/map2d/teleop panels for the
    channels in `available`, remaining rx channels advertised channel-only
    (raw rows in the cockpit's channel list). Rates and jpeg quality come
    from the config fields, so `-o relay-bridge-module.*` overrides keep
    working in the no-manifest (auto) mode."""
    present = frozenset(available)
    main: Panel | None = None
    if "color_image" in present:
        main = Video("color_image", max_hz=config.image_max_hz, quality=config.jpeg_quality)
    side_panels: list[Panel] = []
    if "global_costmap" in present:
        side_panels.append(
            Map2D(
                costmap="global_costmap",
                pose="odom" if "odom" in present else None,
                costmap_hz=config.costmap_max_hz,
                pose_hz=config.odom_max_hz,
            )
        )
    if "tele_cmd_vel" in present:
        side_panels.append(Teleop())
    side: Panel | Col | None
    if len(side_panels) > 1:
        side = Col(*side_panels, shares=[3, 1])
    elif side_panels:
        side = side_panels[0]
    else:
        side = None
    layout: Panel | Row | Col | None
    if main is not None and side is not None:
        layout = Row(main, side, shares=[2, 1])
    else:
        layout = main if main is not None else side
    registry = {b.ch: (b.encoding, b.delivery) for b in BUILTIN_CHANNELS}
    tx_registry = {ch: (encoding, delivery) for ch, encoding, delivery in TX_CHANNELS}
    return build_manifest_data(
        layout,
        (),
        registry=registry,
        tx_streams=frozenset(tx_registry),
        tx_registry=tx_registry,
        extra_channels=tuple(
            ChannelRequest(b.ch, "rx", b.encoding, b.max_hz(config), delivery=b.delivery)
            for b in BUILTIN_CHANNELS
            if b.ch in present
        ),
    )


def resolve_robot_info(config: RelayBridgeConfig) -> RobotInfo:
    robot_id = config.robot_id or config.g.robot_id or socket.gethostname()
    return RobotInfo(
        id=robot_id,
        name=config.robot_name or robot_id,
        model=config.g.robot_model or "",
    )


class RelayBridgeModule(Module):
    """Bridges robot streams to the relay; encodes only while viewers watch."""

    config: RelayBridgeConfig
    # Exact producer types (GO2Connection/CostMapper outputs) so autoconnect
    # matches.
    color_image: In[Image]
    odom: In[PoseStamped]
    global_costmap: In[OccupancyGrid]
    # tx: cockpit teleop twists, autoconnected by name+type to
    # MovementManager.tele_cmd_vel (publish is a no-op while unwired).
    tele_cmd_vel: Out[Twist]
    # NEVER add handle_color_image/handle_odom methods here: _auto_bind_handlers
    # subscribes any handle_<input> eagerly at start(), defeating lazy encode.

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._relay: RelayProcess | None = None
        self._build_cancel: threading.Event | None = None
        self._session: _Session | None = None
        self._url: str | None = None
        # Resolved config.serve_dir, kept for relay-child respawns.
        self._serve_dir: Path | None = None
        self._robot_info: RobotInfo | None = None
        self._manifest: RobotManifest | None = None
        self._channel_specs: tuple[RuntimeChannelSpec, ...] = ()
        # Generic-publish tx specs by channel id (decoder resolved); the rx
        # machinery (_reconcile, rate gates, encode counters) never sees them.
        self._pub_specs: dict[str, RuntimeChannelSpec] = {}
        self._min_interval: dict[str, float] = {}
        self._last_input: dict[str, float] = {}
        # Last "encoder failed" log time per channel: a broken encoder on a
        # 30 Hz stream must not flood the log (rate gated like the inputs).
        self._encode_error_logged: dict[str, float] = {}
        # Newest raw message (+ arrival wall time, the replay's frame ts) per
        # resend_on_subscribe channel; written on the transport thread, read
        # on the loop (GIL-atomic dict swap), and kept across sessions so a
        # reconnect replays too. Pins the full grid (MBs, one per channel);
        # encoding stays lazy.
        self._last_msg: dict[str, tuple[Any, float]] = {}
        # Teleop state, all touched on the module loop only. None params =
        # the manifest advertises no teleop channel; every teleop message is
        # then ignored. `driving` implements the release-edge rule: publish
        # a zero only after a nonzero (MovementManager cancels the nav goal
        # on EVERY teleop message, so idle zeros must never repeat).
        self._teleop_params: _TeleopParams | None = None
        self._teleop_driving = False
        # Lease-generation floor (relay-stamped, monotonic in a session):
        # anything below it is voided permanently, so a released holder's
        # delayed datagrams cannot restart motion after a stop.
        self._teleop_gen: int | float = -math.inf
        self._teleop_last_seq = -math.inf
        self._teleop_last_rx = 0.0
        # Live-path encode counters, keyed by the advertised rx channels at
        # start (main() fills it once the manifest resolves).
        self.encoded: dict[str, int] = {}
        # Publish frames dropped for an unusable meta shape (no correlatable
        # request id to nack with); a compliant relay never produces one.
        self._pub_invalid = 0

    async def main(self) -> AsyncIterator[None]:
        supervisor: asyncio.Task[None] | None = None
        try:
            self._robot_info = resolve_robot_info(self.config)
            manifest_data = self.config.manifest
            if self.config.channels is not None and manifest_data is None:
                raise RuntimeError(
                    "channels runtime specs require a manifest; author both through cockpit()"
                )
            if manifest_data is None:
                allowed = (
                    None
                    if self.config.available_channels is None
                    else frozenset(self.config.available_channels)
                )
                available = tuple(
                    b.ch
                    for b in BUILTIN_CHANNELS
                    if (allowed is None or b.ch in allowed)
                    and self.inputs[b.ch].transport is not None
                )
                # tx side: an Out gets a transport whether or not anything
                # consumes it, so availability comes from the composition
                # (with_relay_bridge scans the blueprint's consumers).
                available += tuple(
                    ch for ch, _, _ in TX_CHANNELS if allowed is not None and ch in allowed
                )
                manifest_data = default_manifest(self.config, available)
            # The domain parser is the authority: an invalid manifest fails
            # module start instead of poisoning the relay.
            manifest = parse_manifest(manifest_data)
            rx_wire = [spec for spec in manifest.channels if spec.dir == "rx"]
            if self.config.channels is not None:
                self._channel_specs, self._pub_specs = self._adopt_authored_specs(manifest.channels)
            else:
                self._channel_specs = self._resolve_builtin_specs(rx_wire)
            self._min_interval = {s.ch: 1.0 / s.max_hz for s in self._channel_specs}
            self.encoded = {s.ch: 0 for s in self._channel_specs}
            by_tx = {ch: (encoding, delivery) for ch, encoding, delivery in TX_CHANNELS}
            for spec in manifest.channels:
                if spec.dir != "tx" or spec.publish != "none":
                    # Publish tx channels are declaration-driven (adopted
                    # above); the static tx table covers only the
                    # specialized protocol paths.
                    continue
                if by_tx.get(spec.ch) != (spec.encoding, spec.delivery):
                    raise RuntimeError(
                        f"manifest tx channel {spec.ch!r} ({spec.encoding}/{spec.delivery}) has "
                        f"no matching handler; this bridge supports: {sorted(by_tx)}"
                    )
                if spec.ch == "tele_cmd_vel":
                    self._teleop_params = self._resolve_teleop_params(spec)
            # No runtime stream probing: an authored channel whose input got
            # no transport stays advertised (its panel shows "waiting for
            # data"); _reconcile just never subscribes it.
            unwired = sorted(
                s.ch for s in self._channel_specs if self.inputs[s.ch].transport is None
            )
            if unwired:
                logger.info(
                    f"relay bridge: channels {unwired} advertised without a wired input; "
                    "their panels will wait for data"
                )
            for s in self._channel_specs:
                if s.resend_on_subscribe and self.inputs[s.ch].transport is not None:
                    # Always-on raw cache: grids published before the first
                    # viewer, or while nobody watches, must still be
                    # replayable on the next 0->1 subscribe.
                    self.register_disposable(
                        Disposable(
                            self.inputs[s.ch].subscribe(functools.partial(self._cache_input, s))
                        )
                    )
            self._manifest = manifest.model_dump()
            self._url = self.config.relay_url or self.config.g.relay_url
            if self._url is not None and self.config.serve_dir is not None:
                raise RuntimeError(
                    "serve_dir requires the spawned local relay (--local-relay); "
                    "an external relay (--relay-url) cannot serve local files"
                )
            if self._url is None:
                # Probe before the (expensive) build: a start that will lose
                # the port must not rewrite the dist a running relay serves.
                _probe_local_port(self.config.local_port)
                # Before the build too: fail fast on a typo'd directory.
                self._serve_dir = self._resolve_serve_dir()
                if self.config.web_build:
                    try:
                        await self._build_web_dist()
                    except Exception:
                        logger.exception("web build failed; continuing with the relay only")
                self._url = await _blocking_call(
                    self._spawn_relay, self.config.open_browser, self._serve_dir
                )
            # The first connect fails fast: a relay that cannot be reached at
            # startup should fail the module start visibly, not retry forever.
            session = await self._connect_and_hello()
            self._session = session
            supervisor = asyncio.create_task(self._supervise(session))
            logger.info(f"relay bridge up: robot={self._robot_info.id} relay={self._url}")
            yield
        finally:
            try:
                await _cancel_task(supervisor, "supervisor")
                await self._disconnect()
            finally:
                if self._relay is not None:
                    try:
                        await _blocking_call(self._relay.stop)
                    except Exception:
                        # An orphaned Deno child would outlive this process holding
                        # its port (it has no PDEATHSIG), so surface cleanup failure.
                        logger.exception("relay bridge: stopping the local relay failed")

    def _resolve_serve_dir(self) -> Path | None:
        """Validated config.serve_dir; a clean labeled error beats the
        relay child dying with a stderr-tail dump."""
        if self.config.serve_dir is None:
            return None
        path = Path(self.config.serve_dir)
        if not path.is_dir():
            raise RuntimeError(
                f"serve_dir does not exist or is not a directory: {self.config.serve_dir}"
            )
        return path

    async def _build_web_dist(self) -> None:
        """Build the web dists (checkouts only) before spawning the relay.

        The blocking build runs in a thread but stays cancellable: on
        cancellation (or module close, via _close_module) the build child is
        killed and the worker reaped within a bounded grace, so teardown
        never waits out the 600 s build timeout.
        """
        cancel = threading.Event()
        self._build_cancel = cancel
        work = asyncio.create_task(
            asyncio.to_thread(ensure_web_dist, find_web_dir(), cancel=cancel)
        )
        try:
            await asyncio.shield(work)
        except (asyncio.CancelledError, GeneratorExit):
            # Generator finalization (aclose) delivers GeneratorExit instead
            # of CancelledError; both mean the same thing here.
            cancel.set()
            await asyncio.wait({work}, timeout=_BUILD_CANCEL_WAIT_S)
            raise
        finally:
            self._build_cancel = None

    def _close_module(self) -> None:
        # Runs on every stop path, including a stop() racing a still-starting
        # main(): an in-flight web build must die now, not at its timeout.
        cancel = self._build_cancel
        if cancel is not None:
            cancel.set()
        super()._close_module()

    def _adopt_authored_specs(
        self, wire_channels: list[ChannelSpec]
    ) -> tuple[tuple[RuntimeChannelSpec, ...], dict[str, RuntimeChannelSpec]]:
        """cockpit() compiled config.channels together with the manifest;
        cross-check the pair and finish config-dependent params.

        Every advertised field that drives runtime behavior must agree, so a
        stale or independently overridden half fails start instead of gating
        and encoding differently from what viewers were told. The codec
        callables and the internal resend flag are runtime-only and not
        advertised, so they have no wire side to compare against. Returns
        (rx specs, publish tx specs by channel); publish="none" tx channels
        (teleop) stay with the static tx table.
        """
        assert self.config.channels is not None
        by_ch = {s.ch: s for s in self.config.channels}
        covered = [w for w in wire_channels if w.dir == "rx" or w.publish != "none"]
        if {w.ch for w in covered} != set(by_ch):
            raise RuntimeError(
                f"manifest rx/publish channels {sorted(w.ch for w in covered)} do not match "
                f"the compiled runtime specs {sorted(by_ch)}; author both through cockpit()"
            )
        rx_specs = []
        pub_specs: dict[str, RuntimeChannelSpec] = {}
        for wire in covered:
            spec = by_ch[wire.ch]
            advertised = (
                wire.dir,
                wire.encoding,
                wire.delivery,
                float(wire.maxHz),
                wire.params,
                wire.publish,
                wire.requiredScope,
            )
            compiled = (
                spec.dir,
                spec.encoding,
                spec.delivery,
                spec.max_hz,
                spec.params,
                spec.publish,
                spec.required_scope,
            )
            if advertised != compiled:
                raise RuntimeError(
                    f"manifest channel {wire.ch!r} does not match its compiled runtime "
                    "spec; author both through cockpit()"
                )
            if spec.publish == "exclusive":
                raise RuntimeError(
                    f"runtime spec {wire.ch!r} sets publish='exclusive'; exclusive "
                    "publishers arrive with the lease ticket (W8)"
                )
            if spec.dir == "rx":
                if wire.ch not in self.inputs:
                    raise RuntimeError(
                        f"runtime spec {wire.ch!r} has no matching input port "
                        f"on {type(self).__name__}"
                    )
                port_type = self.inputs[wire.ch].type
                if port_type is not spec.message_type:
                    raise RuntimeError(
                        f"runtime spec {wire.ch!r} message type "
                        f"{spec.message_type.__qualname__} does not match the "
                        f"{type(self).__name__} port type {port_type.__qualname__}"
                    )
                rx_specs.append(self._finalize_spec(spec))
                continue
            if spec.decoder is None:
                raise RuntimeError(
                    f"publish spec {wire.ch!r} has no resolved decoder; author "
                    "both through cockpit()"
                )
            if wire.ch not in self.outputs:
                raise RuntimeError(
                    f"runtime spec {wire.ch!r} has no matching output port on {type(self).__name__}"
                )
            out_type = self.outputs[wire.ch].type
            if out_type is not spec.message_type:
                raise RuntimeError(
                    f"runtime spec {wire.ch!r} message type "
                    f"{spec.message_type.__qualname__} does not match the "
                    f"{type(self).__name__} port type {out_type.__qualname__}"
                )
            pub_specs[wire.ch] = spec
        return tuple(rx_specs), pub_specs

    def _resolve_builtin_specs(self, rx_wire: list[ChannelSpec]) -> tuple[RuntimeChannelSpec, ...]:
        """Runtime specs for a hand-written or auto (default_manifest)
        manifest: every rx channel must be a built-in declaration, with its
        encoder taken from the codec registry."""
        by_ch = {b.ch: b for b in BUILTIN_CHANNELS}
        specs = []
        for wire in rx_wire:
            builtin = by_ch.get(wire.ch)
            if (
                builtin is None
                or builtin.encoding != wire.encoding
                or builtin.delivery != wire.delivery
            ):
                raise RuntimeError(
                    f"manifest channel {wire.ch!r} ({wire.encoding}/{wire.delivery}) has no "
                    f"matching encoder; this bridge supports: {sorted(by_ch)}"
                )
            definition = encoder_definition(wire.encoding)
            assert definition is not None, wire.encoding  # built-ins register at import
            specs.append(
                self._finalize_spec(
                    RuntimeChannelSpec(
                        ch=wire.ch,
                        message_type=definition.message_type,
                        dir="rx",
                        encoding=wire.encoding,
                        delivery=wire.delivery,
                        max_hz=float(wire.maxHz),
                        params=dict(wire.params),
                        encoder=definition.encode,
                        encoder_takes_params=definition.takes_params,
                        resend_on_subscribe=builtin.resend_on_subscribe,
                    )
                )
            )
        return tuple(specs)

    def _finalize_spec(self, spec: RuntimeChannelSpec) -> RuntimeChannelSpec:
        """Merge config-driven defaults under a built-in channel's params and
        run the codec's params check, so a bad authored value (jpeg quality)
        fails module start, not the first frame."""
        params = dict(spec.params)
        builtin = next((b for b in BUILTIN_CHANNELS if b.ch == spec.ch), None)
        if builtin is not None:
            params = {**builtin.default_params(self.config), **params}
        definition = encoder_definition(spec.encoding)
        if definition is not None and definition.check_params is not None:
            try:
                definition.check_params(params)
            except ValueError as e:
                raise RuntimeError(f"manifest channel {spec.ch!r} {e}") from e
        return replace(spec, params=params)

    def _run_encoder(self, spec: RuntimeChannelSpec, msg: Any) -> tuple[bytes, _FrameMeta] | None:
        """One sample through the channel's encoder; None drops it (encoder
        skip, failure, or a bad return type - failures are logged at most
        once per channel per _ENCODE_ERROR_LOG_INTERVAL_S)."""
        assert spec.encoder is not None
        now = time.monotonic()
        try:
            out = spec.encoder(msg, spec.params) if spec.encoder_takes_params else spec.encoder(msg)
        except Exception:
            if _passes_rate_gate(self._encode_error_logged, spec.ch, now, _ENCODE_ERROR_LOG_S):
                logger.exception(f"relay bridge: encoding {spec.ch} failed")
            return None
        if out is None:
            return None
        if isinstance(out, EncodedPayload):
            return out.payload, dict(out.meta) if out.meta is not None else None
        if isinstance(out, (bytes, bytearray, memoryview)):
            return bytes(out), None
        if _passes_rate_gate(self._encode_error_logged, spec.ch, now, _ENCODE_ERROR_LOG_S):
            logger.error(
                f"relay bridge: {spec.ch} encoder returned {type(out).__name__}; "
                "expected bytes, EncodedPayload, or None"
            )
        return None

    def _spawn_relay(self, open_browser: bool, serve_dir: Path | None) -> str:
        """Start a fresh local relay child (blocking; run via to_thread)."""
        _probe_local_port(self.config.local_port)
        self._relay = RelayProcess(port=self.config.local_port, serve_dir=serve_dir)
        info = self._relay.start()
        logger.info(f"local relay ready: {info.open_url}")
        if open_browser:
            webbrowser.open_new_tab(info.open_url)
        return info.wt_url

    async def _connect_and_hello(self) -> _Session:
        assert self._url is not None and self._robot_info is not None and self._manifest is not None
        client = await connect_with_backoff(self._url, "robot", max_attempts=4)
        try:
            await client.hello(robot=self._robot_info, manifest=self._manifest)
            senders = self._build_senders(client)
        except BaseException:
            try:
                await client.close()
            except Exception:
                logger.exception("relay bridge: closing a failed relay session failed")
            raise
        return _Session(client, senders)

    def _build_senders(self, client: RelayClient) -> dict[str, _Sender]:
        senders: dict[str, _Sender] = {}
        for spec in self._channel_specs:
            sender: _Sender
            if spec.delivery == "latest":
                sender = client.latest_writer(spec.ch).offer
            else:
                sender = functools.partial(self._send_reliable, client, spec.ch)
            if spec.paced:
                sender = _PacedSender(asyncio.get_running_loop(), 1.0 / spec.max_hz, sender)
            senders[spec.ch] = sender
        return senders

    def _send_reliable(
        self,
        client: RelayClient,
        ch: str,
        payload: bytes,
        meta: dict[str, Any] | None,
        ts: float | None = None,
    ) -> None:
        client.send_frame(ch, payload, delivery="reliable", meta=meta, ts=ts)

    async def _supervise(self, session: _Session) -> None:
        """Consume relay control messages (subs snapshots, teleop); on session
        loss, reconnect (and respawn a dead local relay child) until the
        module stops."""
        watchdog: asyncio.Task[None] | None = None
        deadman: asyncio.Task[None] | None = None
        try:
            if self._relay is not None:
                watchdog = asyncio.create_task(self._watch_child())
            if self._teleop_params is not None:
                deadman = asyncio.create_task(self._teleop_watchdog())
            while True:
                crashed = False
                try:
                    async for msg in session.client.control_messages():
                        if isinstance(msg, Subs) and msg.n > session.last_n:
                            session.last_n = msg.n
                            self._reconcile(session, set(msg.chs))
                        elif isinstance(msg, DataFrame):
                            # A forwarded viewer publish (tx channel data on
                            # the carrier); never raises out of the loop.
                            self._on_pub_frame(session, msg)
                        elif isinstance(msg, WireTwist):
                            self._on_wire_twist(msg)
                        elif isinstance(msg, WireStop):
                            self._on_wire_stop(msg)
                        elif isinstance(msg, WireTeleopStart):
                            self._on_wire_teleop_start(msg)
                        elif isinstance(msg, WireTeleopStop):
                            self._on_wire_teleop_stop(msg)
                    # The iterator only ends when the session closed.
                except Exception:
                    # An unguarded error here would silently end supervision while
                    # the module stays "up" (never CancelledError: stop() cancels).
                    crashed = True
                    logger.exception("relay bridge supervisor error; recycling the relay session")
                await self._disconnect(session)
                if crashed:
                    # Reconnect only pauses on FAILED connects; without this a
                    # persistent reconcile error would recycle at handshake speed.
                    await asyncio.sleep(_RECONNECT_PAUSE_S)
                logger.warning("relay session lost; encoders stopped, reconnecting")
                reconnected = await self._reconnect()
                if reconnected is None:
                    return
                session = reconnected
                self._session = session
        finally:
            await _cancel_task(deadman, "teleop watchdog")
            await _cancel_task(watchdog, "watchdog")
            await self._disconnect(session)

    def _on_pub_frame(self, session: _Session, frame: DataFrame) -> None:
        """One forwarded viewer publish from the carrier: decode the JSON
        value, publish on the channel's Out port, then acknowledge on a
        robot-opened one-shot @control stream - pub_ack only after
        Out.publish() returned, pub_nack for decode/publish failures (bounded
        messages, no tracebacks). Failures never raise (an exception would
        recycle the whole relay session) and never touch other channels.
        """
        ch = frame.header.ch
        parsed = _parse_pub_meta(frame.header.meta)
        if parsed is None:
            # No trustworthy request id to nack with; the relay's publish
            # timeout settles the viewer side.
            self._pub_invalid += 1
            logger.warning(f"dropping publish frame on {ch!r}: unusable meta")
            return
        request_id, principal, relay_ts, client_ts = parsed

        def nack(code: str, error: Exception | str) -> None:
            message = error if isinstance(error, str) else f"{type(error).__name__}: {error}"
            self._send_pub_result(session, PubNack(id=request_id, code=code, message=message[:200]))

        spec = self._pub_specs.get(ch)
        if spec is None:
            nack("unknown_channel", f"no publishable channel {ch!r}")
            return
        if not _pub_depth_ok(frame.payload):
            nack("decode_failed", f"value nests deeper than {_MAX_PUB_DEPTH} levels")
            return
        try:
            value = json.loads(frame.payload)
        except (ValueError, RecursionError) as e:
            # RecursionError as backstop: escaping here would recycle the
            # whole relay session over one request's payload.
            nack("decode_failed", e)
            return
        assert self._robot_info is not None  # set before the supervisor starts
        assert spec.decoder is not None  # _adopt_authored_specs required it
        context = PublishContext(
            robot=self._robot_info.id,
            ch=ch,
            relay_ts=relay_ts,
            request_id=request_id,
            principal=principal,
            client_ts=client_ts,
        )
        try:
            if spec.decoder_takes_context:
                result = spec.decoder(value, context)
            else:
                result = spec.decoder(value)
        except Exception as e:
            nack("decode_failed", e)
            return
        if not _matches_message_type(result, spec.message_type):
            nack(
                "decode_failed",
                f"decoder returned {type(result).__name__}, not {spec.message_type.__name__}",
            )
            return
        try:
            self.outputs[ch].publish(result)
        except Exception as e:
            nack("publish_failed", e)
            return
        self._send_pub_result(
            session, PubAck(id=request_id, ch=ch, relayTs=relay_ts, bridgeTs=time.time())
        )

    def _send_pub_result(self, session: _Session, msg: Msg) -> None:
        try:
            session.client.send_control_frame(msg)
        except Exception as e:
            # A session death racing the result is routine; the relay's
            # publish timeout covers the viewer side.
            logger.warning(f"relay bridge: sending a publish result failed: {e}")

    async def _watch_child(self) -> None:
        """Close the session promptly when the local relay child dies (a kill
        sends no CONNECTION_CLOSE; waiting for QUIC idle timeout is too slow).
        The supervisor then respawns and reconnects."""
        while True:
            await asyncio.sleep(_CHILD_POLL_S)
            relay, session = self._relay, self._session
            if (
                relay is not None
                and not relay.is_running()
                and session is not None
                and not session.client.is_closed
            ):
                logger.warning("local relay child died; closing the session to reconnect")
                await session.client.close()

    def _resolve_teleop_params(self, spec: ChannelSpec) -> _TeleopParams:
        values: dict[str, float] = {}
        for key, default in _TELEOP_PARAM_DEFAULTS.items():
            candidate = spec.params.get(key, default)
            if (
                isinstance(candidate, bool)
                or not isinstance(candidate, (int, float))
                or not math.isfinite(candidate)
                or candidate <= 0
            ):
                raise RuntimeError(
                    f"manifest channel {spec.ch!r} {key} must be a positive number, "
                    f"got {candidate!r}"
                )
            values[key] = float(candidate)
        return _TeleopParams(
            max_linear=values["maxLinear"],
            max_angular=values["maxAngular"],
            boost=values["boost"],
            watchdog_s=values["watchdogMs"] / 1000.0,
        )

    def _on_wire_twist(self, msg: WireTwist) -> None:
        params = self._teleop_params
        if params is None:
            return
        # Non-finite components cannot reach here: the wire decoders reject
        # NaN/Infinity (allow_inf_nan=False).
        gen = msg.gen
        if gen is None or gen < self._teleop_gen:
            return  # unstamped, or in flight from a lease already voided
        if gen == self._teleop_gen and msg.seq <= self._teleop_last_seq:
            # Within a generation the high-water mark is permanent: a paused
            # holder resumes with rising seq, while a delayed pre-stop twist
            # stays dead even after watchdog silence. A new lease (gen above
            # the floor) rebaselines instead.
            return
        self._teleop_gen = gen
        self._teleop_last_seq = float(msg.seq)
        self._teleop_last_rx = time.monotonic()
        bound_linear = params.max_linear * params.boost
        vx = _clamp(float(msg.vx), bound_linear)
        vy = _clamp(float(msg.vy), bound_linear)
        wz = _clamp(float(msg.wz), params.max_angular * params.boost)
        if vx == 0.0 and vy == 0.0 and wz == 0.0:
            self._teleop_zero("release")
            return
        self._teleop_driving = True
        self.tele_cmd_vel.publish(Twist(linear=Vector3(vx, vy, 0.0), angular=Vector3(0.0, 0.0, wz)))

    def _on_wire_stop(self, msg: WireStop) -> None:
        """E-stop: unconditional zero, even from idle - it must also cancel
        an autonomous nav goal (MovementManager cancels on any teleop msg)."""
        if self._teleop_params is None:
            return
        gen = msg.gen
        if gen is None or gen < self._teleop_gen:
            # A voided lease's in-flight e-stop must not blip the current
            # holder (the relay gate already blocks post-release sends).
            return
        self._teleop_zero("stop message (e-stop)", force=True)
        if gen > self._teleop_gen:
            self._teleop_gen = gen
            self._teleop_last_seq = float(msg.seq)
        else:
            # max(): a stale reordered e-stop must not lower the high-water
            # mark and let an already-superseded twist re-apply.
            self._teleop_last_seq = max(self._teleop_last_seq, float(msg.seq))
        self._teleop_last_rx = time.monotonic()

    def _on_wire_teleop_start(self, msg: WireTeleopStart) -> None:
        """A relay-granted lease: adopt its generation, voiding the previous
        one. Heals a lost teleop_stop (zeroing if it arrived mid-drive)."""
        if self._teleop_params is None:
            return
        gen = msg.gen
        if gen is None or gen <= self._teleop_gen:
            return  # a duplicated start must not reset the high-water mid-lease
        self._teleop_zero("new teleop lease")
        self._teleop_gen = gen
        self._teleop_last_seq = -math.inf
        self._teleop_last_rx = 0.0

    def _on_wire_teleop_stop(self, msg: WireTeleopStop) -> None:
        """The lease ended (holder released, disconnected, or watched away)."""
        if self._teleop_params is None:
            return
        gen = msg.gen
        if gen is None or gen < self._teleop_gen:
            return  # a stale lease-end must not blip or reset the current holder
        self._teleop_zero("teleop lease ended")
        # The relay bumps by exactly 1 per grant, so this floor equals the
        # next lease's generation; the ended lease and everything below it
        # are voided permanently.
        self._teleop_gen = gen + 1
        self._teleop_last_seq = -math.inf
        self._teleop_last_rx = 0.0

    def _teleop_zero(self, reason: str, *, force: bool = False) -> None:
        """Publish one zero twist; edge-gated unless `force` (e-stop)."""
        if not force and not self._teleop_driving:
            return
        self._teleop_driving = False
        logger.warning(f"relay bridge teleop: zero twist ({reason})")
        self.tele_cmd_vel.publish(Twist.zero())

    def _teleop_reset(self) -> None:
        # Session teardown only: datagrams are QUIC-session-scoped and the
        # relay's lease generation dies with the robot registration, so
        # nothing stale can leak into the next session.
        self._teleop_gen = -math.inf
        self._teleop_last_seq = -math.inf
        self._teleop_last_rx = 0.0

    async def _teleop_watchdog(self) -> None:
        """Deadman: the cockpit repeats commands at publish_hz, so silence
        while driving means the chain broke (viewer gone, relay killed,
        datagrams lost) - zero within ~watchdog_s regardless of which hop
        failed."""
        assert self._teleop_params is not None
        watchdog_s = self._teleop_params.watchdog_s
        while True:
            await asyncio.sleep(_TELEOP_POLL_S)
            if self._teleop_driving and time.monotonic() - self._teleop_last_rx > watchdog_s:
                self._teleop_zero("watchdog: twist silence")

    async def _reconnect(self) -> _Session | None:
        while True:
            if self._relay is not None and not self._relay.is_running():
                # The child is gone (crash, kill, or a previous respawn that
                # failed): its QUIC port and cert die with it, so respawn and
                # re-read the ready line. The browser page reconnects itself
                # via the stable HTTP port. `not is_running()` rather than
                # `poll() is not None`: a failed start leaves no process and
                # poll() would read None forever, latching respawns off.
                logger.warning("local relay child died; respawning")
                try:
                    await _blocking_call(self._relay.stop)
                    self._url = await _blocking_call(self._spawn_relay, False, self._serve_dir)
                except Exception:
                    logger.exception("relay respawn failed; retrying")
                    await asyncio.sleep(_RECONNECT_PAUSE_S)
                    continue
            try:
                return await self._connect_and_hello()
            except RelayRejectedError as e:
                logger.error(f"relay rejected reconnect ({e.code}: {e.message}); not retrying")
                return None
            except Exception as e:
                logger.warning(f"relay reconnect failed ({e}); retrying")
                await asyncio.sleep(_RECONNECT_PAUSE_S)

    def _reconcile(self, session: _Session, want: set[str]) -> None:
        """Subscribe/unsubscribe inputs so exactly `want` is being encoded."""
        for spec in self._channel_specs:
            active = spec.ch in session.unsubs
            should = spec.ch in want
            if should and not active:
                if self.inputs[spec.ch].transport is None:
                    # Advertised but unwired (manifest-authored): nothing to
                    # subscribe; the panel shows "waiting for data".
                    continue
                cached = self._last_msg.get(spec.ch)
                if cached is not None:
                    # Replay precedes the subscribe: this offer runs
                    # synchronously on the loop, so a live frame - possible
                    # only once subscribed - always queues behind it and wins
                    # the 1-slot mailbox. Fires on 0->1 transitions only: the
                    # relay reports sub-set changes and stays cache-free, so
                    # an extra viewer on an already-active channel waits for
                    # the next publish (review issue 2, deferred).
                    msg, recv_ts = cached
                    encoded = self._run_encoder(spec, msg)
                    if encoded is not None:
                        # self.encoded counts live-path encodes only; the
                        # arrival ts keeps a stale replay honest about its age.
                        self._offer(session, session.senders[spec.ch], *encoded, recv_ts)
                session.unsubs[spec.ch] = self.inputs[spec.ch].subscribe(
                    functools.partial(self._on_input, session, spec, session.senders[spec.ch])
                )
                logger.info(f"relay bridge: viewer subscribed to {spec.ch}; encoding started")
            elif active and not should:
                unsubscribe = session.unsubs[spec.ch]
                unsubscribe()
                del session.unsubs[spec.ch]
                logger.info(f"relay bridge: no viewers on {spec.ch}; encoding stopped")
        unknown = want - {spec.ch for spec in self._channel_specs}
        if unknown:
            logger.debug(f"relay bridge: ignoring unknown channels {sorted(unknown)}")

    def _on_input(
        self, session: _Session, spec: RuntimeChannelSpec, sender: _Sender, msg: Any
    ) -> None:
        """Transport-thread callback: maxHz gate, encode, hand to the loop."""
        if session.retired.is_set():
            return
        now = time.monotonic()
        if not spec.paced and not _passes_rate_gate(
            self._last_input, spec.ch, now, self._min_interval[spec.ch]
        ):
            return
        encoded = self._run_encoder(spec, msg)
        if encoded is None:
            return
        payload, meta = encoded
        self.encoded[spec.ch] += 1
        loop = self._loop
        if loop is not None and loop.is_running():
            loop.call_soon_threadsafe(self._offer, session, sender, payload, meta)

    def _cache_input(self, spec: RuntimeChannelSpec, msg: Any) -> None:
        """Transport-thread callback: remember the newest raw message so a
        0->1 subscribe can replay it (its arrival time becomes the frame ts)."""
        self._last_msg[spec.ch] = (msg, time.time())

    def _offer(
        self,
        session: _Session,
        sender: _Sender,
        payload: bytes,
        meta: _FrameMeta,
        ts: float | None = None,
    ) -> None:
        if session.retired.is_set() or self._session is not session:
            return
        try:
            sender(payload, meta, ts)
        except Exception:
            # Session mid-teardown (dead writer pump / closed connection): the
            # supervisor is already reconnecting and will rebuild the senders.
            return

    async def _disconnect(self, session: _Session | None = None) -> None:
        target = self._session if session is None else session
        if target is None:
            return
        target.retired.set()
        if self._session is target:
            self._session = None
        for sender in target.senders.values():
            if isinstance(sender, _PacedSender):
                sender.close()
        # A driving teleop stream cannot outlive its session (this also
        # covers module stop, KeyboardTeleop.stop() parity). Idempotent:
        # the edge gate makes the second call of a double-disconnect a no-op.
        if self._teleop_params is not None:
            self._teleop_zero("relay session ended")
            self._teleop_reset()
        for ch, unsubscribe in tuple(target.unsubs.items()):
            try:
                unsubscribe()
            except Exception:
                logger.exception(f"relay bridge: unsubscribing {ch} failed")
            finally:
                target.unsubs.pop(ch, None)
        try:
            await target.client.close()
        except Exception:
            logger.exception("relay bridge: closing the relay session failed")


def with_relay_bridge(blueprint: Blueprint) -> Blueprint:
    """Append a relay bridge wired to the blueprint's channel producers."""
    # An external or explicitly composed blueprint may already own a customized
    # relay bridge (possibly a generated subclass with custom channel ports).
    # Preserve that atom rather than overriding its kwargs.
    if any(
        isinstance(atom.module, type) and issubclass(atom.module, RelayBridgeModule)
        for atom in blueprint.blueprints
    ):
        return blueprint

    producer_keys: set[tuple[str, type]] = set()
    consumer_keys: set[tuple[str, type]] = set()
    for atom in blueprint.active_blueprints:
        for stream in atom.streams:
            effective_name = blueprint.remapping_map.get((atom.name, stream.name), stream.name)
            if not isinstance(effective_name, str):
                continue
            keys = producer_keys if stream.direction == "out" else consumer_keys
            keys.add((effective_name, stream.type))

    bridge_atom = RelayBridgeModule.blueprint().blueprints[0]
    bridge_input_types = {
        stream.name: stream.type for stream in bridge_atom.streams if stream.direction == "in"
    }
    # Every declared Out gets a transport whether or not a counterparty
    # exists, so tx availability is derived from the blueprint's consumers
    # (mirroring the producer scan above), not from bound transports.
    bridge_output_types = {
        stream.name: stream.type for stream in bridge_atom.streams if stream.direction == "out"
    }
    available_channels = tuple(
        channel.ch
        for channel in BUILTIN_CHANNELS
        if (channel_type := bridge_input_types.get(channel.ch)) is not None
        and (channel.ch, channel_type) in producer_keys
    ) + tuple(
        ch
        for ch, _, _ in TX_CHANNELS
        if (tx_type := bridge_output_types.get(ch)) is not None and (ch, tx_type) in consumer_keys
    )
    return autoconnect(
        blueprint,
        RelayBridgeModule.blueprint(available_channels=available_channels),
    )
