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
from collections.abc import AsyncIterator, Callable, Collection
from dataclasses import dataclass, field
import functools
import json
import math
import socket
import threading
import time
from typing import Any, TypeVar
import webbrowser
import zlib

import numpy as np
from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid, block_max_reduce
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
from dimos.web.relay_bridge.locate import find_web_dir
from dimos.web.relay_bridge.manifest import parse_manifest
from dimos.web.relay_bridge.protocol import (
    ChannelSpec,
    Delivery,
    RobotInfo,
    RobotManifest,
    Stop as WireStop,
    Subs,
    TeleopStart as WireTeleopStart,
    TeleopStop as WireTeleopStop,
    Twist as WireTwist,
)
from dimos.web.relay_bridge.relay_process import RelayProcess, ensure_cockpit_dist
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


class RelayBridgeConfig(ModuleConfig):
    relay_url: str | None = None
    """Attach to a running relay (wtUrl). None: spawn a local one."""
    local_port: int = 7780
    """HTTP port of the spawned local relay; 0 picks an ephemeral port (tests)."""
    open_browser: bool = True
    """Open the local relay's page once it is up (local mode only)."""
    cockpit_build: bool = True
    """Build the Cockpit dist before spawning the local relay when it is
    missing or stale (checkouts only; wheels ship it pre-built)."""
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


def _encode_image(module: RelayBridgeModule, msg: Image) -> tuple[bytes, dict[str, Any] | None]:
    # TurboJPEG via the message's own encoder (handles BGR/RGB/gray inputs).
    # Quality resolved at start from the manifest channel params (falling
    # back to config.jpeg_quality).
    return (
        msg.to_jpeg_bytes(quality=module._jpeg_quality),
        {"w": msg.width, "h": msg.height},
    )


def _encode_odom(
    module: RelayBridgeModule, msg: PoseStamped
) -> tuple[bytes, dict[str, Any] | None]:
    pose = {
        "x": msg.position.x,
        "y": msg.position.y,
        "z": msg.position.z,
        "yaw": msg.yaw,
        "ts": msg.ts,
    }
    return json.dumps(pose, separators=(",", ":")).encode(), None


# The historical costmap encoder's choice (websocket_vis/optimized_costmap.py);
# full grids compress to ~10-30 KB at <= 5 Hz, so speed over ratio is fine.
_COSTMAP_ZLIB_LEVEL = 6
# Render budget shared with the cockpit decoder (MAX_COSTMAP_DIM in
# costmap.ts): larger grids are block-max downsampled before compression so
# every frame stays within what consumers accept and render. 2048^2 raw is
# 4 MiB, and zlib worst case adds ~0.01%, so the 8 MiB payload caps
# (_wt_session._MAX_PAYLOAD_BYTES and the cockpit's) are unreachable.
_COSTMAP_MAX_SIDE = 2048


def _encode_costmap(
    module: RelayBridgeModule, msg: OccupancyGrid
) -> tuple[bytes, dict[str, Any] | None] | None:
    grid = msg.grid
    if grid.size == 0:
        return None  # mapper still warming up; nothing to draw
    res = msg.resolution
    side = max(grid.shape)
    if side > _COSTMAP_MAX_SIDE:
        factor = -(-side // _COSTMAP_MAX_SIDE)
        grid = block_max_reduce(grid, factor)
        res *= factor
    h, w = grid.shape
    # Wire contract (costmap.zlib.v1): uint8 cells, ROS -1 unknown -> 255.
    # int8 -1 is byte 0xff and 0..100 are byte-identical, so the raw buffer
    # already is the wire payload - no mask/astype/tobytes copies.
    cells = np.ascontiguousarray(grid)
    origin = msg.origin
    meta = {
        "w": w,
        "h": h,
        "res": res,
        "origin": [origin.position.x, origin.position.y, origin.yaw],
    }
    return zlib.compress(cells, _COSTMAP_ZLIB_LEVEL), meta


@dataclass(frozen=True)
class ChannelDef:
    ch: str
    encoding: str
    delivery: Delivery
    max_hz: Callable[[RelayBridgeConfig], float]
    # Returning None skips the frame (an empty grid before mapping starts).
    encode: Callable[[RelayBridgeModule, Any], tuple[bytes, _FrameMeta] | None]
    # Keep an always-on raw-input cache (decode only, no encode) and replay
    # the newest message when the channel goes from zero viewers to some
    # viewer: a new session must not wait for the next publish (the producer
    # may have gone quiet, possibly before the first viewer ever attached).
    resend_on_subscribe: bool = False


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


@dataclass(slots=True)
class _Session:
    client: RelayClient
    senders: dict[str, _Sender]
    last_n: int | float = 0
    unsubs: dict[str, Callable[[], None]] = field(default_factory=dict)
    retired: threading.Event = field(default_factory=threading.Event)


# The stream -> encoder registry; every entry needs a matching `In` on the
# module. What actually gets advertised (and which panels bind it) is the
# manifest's business, not this table's.
CHANNELS: tuple[ChannelDef, ...] = (
    ChannelDef("color_image", "jpeg.v1", "latest", lambda c: c.image_max_hz, _encode_image),
    ChannelDef("odom", "pose.json.v1", "reliable", lambda c: c.odom_max_hz, _encode_odom),
    ChannelDef(
        "global_costmap",
        "costmap.zlib.v1",
        "latest",
        lambda c: c.costmap_max_hz,
        _encode_costmap,
        resend_on_subscribe=True,
    ),
)

# The tx (viewer->robot) counterpart of CHANNELS: stream -> (encoding,
# delivery). Every entry needs a matching `Out` on the module and a handler
# in _supervise; it is also the delivery source for tx channels in authored
# manifests (dimos/web/cockpit.py).
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
    registry = {cd.ch: (cd.encoding, cd.delivery) for cd in CHANNELS}
    tx_registry = {ch: (encoding, delivery) for ch, encoding, delivery in TX_CHANNELS}
    return build_manifest_data(
        layout,
        (),
        registry=registry,
        rx_streams=frozenset(registry),
        tx_streams=frozenset(tx_registry),
        tx_registry=tx_registry,
        extra_channels=tuple(
            ChannelRequest(cd.ch, "rx", cd.encoding, cd.max_hz(config))
            for cd in CHANNELS
            if cd.ch in present
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
        self._robot_info: RobotInfo | None = None
        self._manifest: RobotManifest | None = None
        self._channel_defs: tuple[ChannelDef, ...] = ()
        self._min_interval: dict[str, float] = {}
        self._last_input: dict[str, float] = {}
        # Newest raw message (+ arrival wall time, the replay's frame ts) per
        # resend_on_subscribe channel; written on the transport thread, read
        # on the loop (GIL-atomic dict swap), and kept across sessions so a
        # reconnect replays too. Pins the full grid (MBs, one per channel);
        # encoding stays lazy.
        self._last_msg: dict[str, tuple[Any, float]] = {}
        # Resolved at start from the manifest's jpeg channel params.
        self._jpeg_quality: int = self.config.jpeg_quality
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
        self.encoded: dict[str, int] = {cd.ch: 0 for cd in CHANNELS}

    async def main(self) -> AsyncIterator[None]:
        supervisor: asyncio.Task[None] | None = None
        try:
            self._robot_info = resolve_robot_info(self.config)
            manifest_data = self.config.manifest
            if manifest_data is None:
                allowed = (
                    None
                    if self.config.available_channels is None
                    else frozenset(self.config.available_channels)
                )
                available = tuple(
                    cd.ch
                    for cd in CHANNELS
                    if (allowed is None or cd.ch in allowed)
                    and self.inputs[cd.ch].transport is not None
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
            by_ch = {cd.ch: cd for cd in CHANNELS}
            rx_specs = [spec for spec in manifest.channels if spec.dir == "rx"]
            for spec in rx_specs:
                encoder = by_ch.get(spec.ch)
                if (
                    encoder is None
                    or encoder.encoding != spec.encoding
                    or encoder.delivery != spec.delivery
                ):
                    raise RuntimeError(
                        f"manifest channel {spec.ch!r} ({spec.encoding}/{spec.delivery}) has no "
                        f"matching encoder; this bridge supports: {sorted(by_ch)}"
                    )
            self._channel_defs = tuple(by_ch[spec.ch] for spec in rx_specs)
            self._min_interval = {spec.ch: 1.0 / spec.maxHz for spec in rx_specs}
            self._jpeg_quality = self._resolve_jpeg_quality(rx_specs)
            by_tx = {ch: (encoding, delivery) for ch, encoding, delivery in TX_CHANNELS}
            for spec in manifest.channels:
                if spec.dir != "tx":
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
            unwired = sorted(spec.ch for spec in rx_specs if self.inputs[spec.ch].transport is None)
            if unwired:
                logger.info(
                    f"relay bridge: channels {unwired} advertised without a wired input; "
                    "their panels will wait for data"
                )
            for cd in self._channel_defs:
                if cd.resend_on_subscribe and self.inputs[cd.ch].transport is not None:
                    # Always-on raw cache: grids published before the first
                    # viewer, or while nobody watches, must still be
                    # replayable on the next 0->1 subscribe.
                    self.register_disposable(
                        Disposable(
                            self.inputs[cd.ch].subscribe(functools.partial(self._cache_input, cd))
                        )
                    )
            self._manifest = manifest.model_dump()
            self._url = self.config.relay_url or self.config.g.relay_url
            if self._url is None:
                # Probe before the (expensive) build: a start that will lose
                # the port must not rewrite the dist a running relay serves.
                _probe_local_port(self.config.local_port)
                if self.config.cockpit_build:
                    try:
                        await self._build_cockpit()
                    except Exception:
                        logger.exception("cockpit build failed; continuing with the relay only")
                self._url = await _blocking_call(self._spawn_relay, self.config.open_browser)
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

    async def _build_cockpit(self) -> None:
        """Build the Cockpit dist (checkouts only) before spawning the relay.

        The blocking build runs in a thread but stays cancellable: on
        cancellation (or module close, via _close_module) the build child is
        killed and the worker reaped within a bounded grace, so teardown
        never waits out the 600 s build timeout.
        """
        cancel = threading.Event()
        self._build_cancel = cancel
        work = asyncio.create_task(
            asyncio.to_thread(ensure_cockpit_dist, find_web_dir(), cancel=cancel)
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
        # main(): an in-flight cockpit build must die now, not at its timeout.
        cancel = self._build_cancel
        if cancel is not None:
            cancel.set()
        super()._close_module()

    def _resolve_jpeg_quality(self, rx_specs: list[ChannelSpec]) -> int:
        # A single int is honest while CHANNELS has exactly one jpeg entry;
        # revisit if a second camera channel ever appears.
        quality = self.config.jpeg_quality
        for spec in rx_specs:
            if spec.encoding != "jpeg.v1":
                continue
            candidate = spec.params.get("quality", quality)
            if (
                isinstance(candidate, bool)
                or not isinstance(candidate, int)
                or not 0 <= candidate <= 100
            ):
                raise RuntimeError(
                    f"manifest channel {spec.ch!r} quality must be an int in 0..100, "
                    f"got {candidate!r}"
                )
            quality = candidate
        return quality

    def _spawn_relay(self, open_browser: bool) -> str:
        """Start a fresh local relay child (blocking; run via to_thread)."""
        _probe_local_port(self.config.local_port)
        self._relay = RelayProcess(port=self.config.local_port)
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
        for cd in self._channel_defs:
            if cd.delivery == "latest":
                senders[cd.ch] = client.latest_writer(cd.ch).offer
            else:
                senders[cd.ch] = functools.partial(self._send_reliable, client, cd.ch)
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
                    self._url = await _blocking_call(self._spawn_relay, False)
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
        for cd in self._channel_defs:
            active = cd.ch in session.unsubs
            should = cd.ch in want
            if should and not active:
                if self.inputs[cd.ch].transport is None:
                    # Advertised but unwired (manifest-authored): nothing to
                    # subscribe; the panel shows "waiting for data".
                    continue
                cached = self._last_msg.get(cd.ch)
                if cached is not None:
                    # Replay precedes the subscribe: this offer runs
                    # synchronously on the loop, so a live frame - possible
                    # only once subscribed - always queues behind it and wins
                    # the 1-slot mailbox. Fires on 0->1 transitions only: the
                    # relay reports sub-set changes and stays cache-free, so
                    # an extra viewer on an already-active channel waits for
                    # the next publish (review issue 2, deferred).
                    msg, recv_ts = cached
                    try:
                        encoded = cd.encode(self, msg)
                    except Exception:
                        logger.exception(f"relay bridge: replaying {cd.ch} failed")
                        encoded = None
                    if encoded is not None:
                        # self.encoded counts live-path encodes only; the
                        # arrival ts keeps a stale replay honest about its age.
                        self._offer(session, session.senders[cd.ch], *encoded, recv_ts)
                session.unsubs[cd.ch] = self.inputs[cd.ch].subscribe(
                    functools.partial(self._on_input, session, cd, session.senders[cd.ch])
                )
                logger.info(f"relay bridge: viewer subscribed to {cd.ch}; encoding started")
            elif active and not should:
                unsubscribe = session.unsubs[cd.ch]
                unsubscribe()
                del session.unsubs[cd.ch]
                logger.info(f"relay bridge: no viewers on {cd.ch}; encoding stopped")
        unknown = want - {cd.ch for cd in self._channel_defs}
        if unknown:
            logger.debug(f"relay bridge: ignoring unknown channels {sorted(unknown)}")

    def _on_input(self, session: _Session, cd: ChannelDef, sender: _Sender, msg: Any) -> None:
        """Transport-thread callback: maxHz gate, encode, hand to the loop."""
        if session.retired.is_set():
            return
        now = time.monotonic()
        if not _passes_rate_gate(self._last_input, cd.ch, now, self._min_interval[cd.ch]):
            return
        try:
            encoded = cd.encode(self, msg)
        except Exception:
            logger.exception(f"relay bridge: encoding {cd.ch} failed")
            return
        if encoded is None:
            return
        payload, meta = encoded
        self.encoded[cd.ch] += 1
        loop = self._loop
        if loop is not None and loop.is_running():
            loop.call_soon_threadsafe(self._offer, session, sender, payload, meta)

    def _cache_input(self, cd: ChannelDef, msg: Any) -> None:
        """Transport-thread callback: remember the newest raw message so a
        0->1 subscribe can replay it (its arrival time becomes the frame ts)."""
        self._last_msg[cd.ch] = (msg, time.time())

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
    # relay bridge. Preserve that atom rather than overriding its kwargs.
    if any(atom.module is RelayBridgeModule for atom in blueprint.blueprints):
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
        for channel in CHANNELS
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
