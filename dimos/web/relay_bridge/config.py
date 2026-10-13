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

"""RelayBridgeModule configuration and what the bridge derives from it: the
robot identity and the availability-driven default manifest."""

from __future__ import annotations

from collections.abc import Collection
import socket
from typing import Any

from pydantic import Field

from dimos.core.module import ModuleConfig

# No import cycle: cockpit.py only imports the bridge lazily inside
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
from dimos.web.relay_bridge.channels import BUILTIN_CHANNELS, TX_CHANNELS, RuntimeChannelSpec
from dimos.web.relay_bridge.protocol import RobotInfo


class RelayBridgeConfig(ModuleConfig):
    relay_url: str | None = None
    """HTTP URL of a relay started elsewhere (e.g. http://localhost:7780); its
    WebTransport endpoint is discovered through /api/info on every connect.
    None: spawn a local one."""
    relay_ca: str | None = None
    """PEM CA bundle that signed the relay_url relay's certificate (mkcert, a
    private CA). It replaces the default trust stores for both the /api/info
    fetch and QUIC, so leave it unset for a relay with a public certificate."""
    relay_key: str | None = None
    """Robot key for a relay_url relay started with --auth-file (bound to
    robot_id there), sent in hello. Falls back to GlobalConfig.relay_key
    (RELAY_KEY)."""
    rtc: bool = True
    """Deliver jpeg.v1 video channels as WebRTC tracks (video.webrtc.v1)
    through the relay's Cloudflare SFU when the relay advertises it and
    aiortc (the webrtc extra) is installed; False keeps JPEG frames through
    the relay."""
    rtc_file: str | None = None
    """Cloudflare configuration for the spawned local relay (its --rtc-file:
    {"appId", "appSecret", "turnKeyId"?, "turnToken"?}). Local relay only:
    an external relay (relay_url) carries its own."""
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
