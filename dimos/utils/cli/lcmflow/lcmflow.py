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

"""lcmflow — model layer for the LCM packet-highway visualization.

Every LCM packet becomes a *vehicle* driving down a per-topic *lane*.
Packet size picks the vehicle class: tiny telemetry messages (cmd_vel,
tf) are fast little dots, raw images and point clouds are long, slower
trucks. Packets that arrive faster than the lane can fit them merge into
a single vehicle with a passenger count (``xN``).

The model is deliberately renderer-agnostic so the Textual TUI and any
future web frontend share the same physics.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math
import time
from typing import Any

from dimos.utils.cli import theme
from dimos.utils.cli.lcmspy.lcmspy import LCMSpy, LCMSpyConfig, Topic as SpyTopic

# Cells per second a speed-1.0 vehicle covers. Frequency shows up as
# vehicle density, size as vehicle length and a small speed handicap.
BASE_SPEED = 24.0

# Free cells required behind the previous vehicle before a new one may
# spawn; arrivals during that window coalesce into the previous vehicle.
SPAWN_GAP = 2

TRAIL_LEN = 3


@dataclass(frozen=True)
class SizeClass:
    """A vehicle class for a packet-size band."""

    name: str
    max_bytes: int  # inclusive upper bound for this class
    length: int  # body length in cells
    speed: float  # multiplier on BASE_SPEED


# Ordered smallest to largest. Calibrated against typical robot streams:
# Twist/tf ~100 B, odom/camera_info ~1 KiB, laser scans ~10 KiB,
# costmaps/JPEG ~100 KiB, raw images and point clouds 0.5 MiB+.
SIZE_CLASSES: tuple[SizeClass, ...] = (
    SizeClass("nano", 256, 1, 1.5),
    SizeClass("small", 2_048, 2, 1.25),
    SizeClass("medium", 32_768, 4, 1.0),
    SizeClass("large", 524_288, 7, 0.8),
    SizeClass("mega", 2**63 - 1, 11, 0.65),
)


def size_class(n_bytes: int) -> SizeClass:
    """Pick the vehicle class for a packet of *n_bytes*."""
    for cls in SIZE_CLASSES:
        if n_bytes <= cls.max_bytes:
            return cls
    return SIZE_CLASSES[-1]


# Colors keyed by LCM message type name (the part after the last '.'
# in a '/topic#pkg.Type' channel), falling back to package, falling
# back to a stable hash into the palette.
TYPE_COLORS: dict[str, str] = {
    "Image": theme.YELLOW,
    "CompressedImage": theme.BRIGHT_YELLOW,
    "CameraInfo": theme.BRIGHT_YELLOW,
    "PointCloud2": theme.PURPLE,
    "LaserScan": theme.BRIGHT_PURPLE,
    "OccupancyGrid": theme.BLUE,
    "Path": theme.BRIGHT_BLUE,
    "Odometry": theme.CYAN,
    "TFMessage": theme.AGENT,
}

PKG_COLORS: dict[str, str] = {
    "sensor_msgs": theme.YELLOW,
    "geometry_msgs": theme.CYAN,
    "nav_msgs": theme.BLUE,
    "tf2_msgs": theme.AGENT,
    "vision_msgs": theme.BRIGHT_PURPLE,
    "std_msgs": theme.WHITE,
}

FALLBACK_PALETTE: tuple[str, ...] = (
    theme.CYAN,
    theme.YELLOW,
    theme.BLUE,
    theme.PURPLE,
    theme.AGENT,
    theme.BRIGHT_YELLOW,
    theme.BRIGHT_BLUE,
    theme.BRIGHT_PURPLE,
)


def split_channel(channel: str) -> tuple[str, str]:
    """Split an LCM channel into (topic, type) at the '#' marker.

    '/odom#geometry_msgs.PoseStamped' -> ('/odom', 'geometry_msgs.PoseStamped')
    """
    if "#" in channel:
        topic, _, type_name = channel.partition("#")
        return topic, type_name
    return channel, ""


def color_for_channel(channel: str) -> str:
    """Stable display color for an LCM channel."""
    _, type_name = split_channel(channel)
    if type_name:
        msg_name = type_name.rsplit(".", 1)[-1]
        if msg_name in TYPE_COLORS:
            return TYPE_COLORS[msg_name]
        pkg = type_name.split(".", 1)[0]
        if pkg in PKG_COLORS:
            return PKG_COLORS[pkg]
    if channel.startswith("/rpc"):
        return theme.BRIGHT_BLUE
    # Stable (non-salted) hash so colors survive restarts.
    digest = sum(channel.encode())
    return FALLBACK_PALETTE[digest % len(FALLBACK_PALETTE)]


@dataclass
class Vehicle:
    """One packet (or a coalesced burst of packets) on the road."""

    t0: float  # lane-clock time the vehicle spawned
    cls: SizeClass
    n_bytes: int
    count: int = 1

    @property
    def speed(self) -> float:
        return BASE_SPEED * self.cls.speed

    @property
    def length(self) -> int:
        # Coalesced bursts stretch a little so dense streams read as
        # convoys rather than a single flickering car.
        stretch = min(4, int(math.log2(self.count)) if self.count > 1 else 0)
        return self.cls.length + stretch

    def head(self, now: float) -> float:
        """Head (front bumper) position in cells at lane-clock *now*."""
        return (now - self.t0) * self.speed

    def absorb(self, n_bytes: int) -> None:
        """Merge a packet that arrived before this vehicle cleared the on-ramp."""
        self.count += 1
        self.n_bytes += n_bytes
        cls = size_class(n_bytes)
        if cls.length > self.cls.length:
            self.cls = cls


@dataclass
class Lane:
    """A single topic's lane: live vehicles plus spawn bookkeeping."""

    channel: str
    color: str = ""
    vehicles: deque[Vehicle] = field(default_factory=deque)
    topic: str = field(init=False, default="")
    type_name: str = field(init=False, default="")

    def __post_init__(self) -> None:
        if not self.color:
            self.color = color_for_channel(self.channel)
        self.topic, self.type_name = split_channel(self.channel)

    def spawn(self, n_bytes: int, now: float) -> Vehicle:
        """Add a packet at lane-clock *now*, coalescing into the newest
        vehicle if it has not yet cleared the on-ramp."""
        last = self.vehicles[-1] if self.vehicles else None
        if last is not None and last.head(now) < last.length + SPAWN_GAP:
            last.absorb(n_bytes)
            return last
        vehicle = Vehicle(t0=now, cls=size_class(n_bytes), n_bytes=n_bytes)
        self.vehicles.append(vehicle)
        return vehicle

    def prune(self, now: float, road_width: int) -> None:
        """Drop vehicles whose tail (and trail) left the visible road."""
        while self.vehicles:
            v = self.vehicles[0]
            if v.head(now) - v.length - TRAIL_LEN > road_width:
                self.vehicles.popleft()
            else:
                break


class PacketSpyConfig(LCMSpyConfig):
    max_pending_packets: int = 100_000


class PacketSpy(LCMSpy):
    """LCMSpy that additionally queues every packet arrival for the renderer.

    The LCM handle thread appends; the UI thread drains via
    :meth:`drain`. ``deque`` keeps that lock-free and bounded.
    """

    config: PacketSpyConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.pending: deque[tuple[str, int]] = deque(maxlen=self.config.max_pending_packets)

    def msg(self, topic, data) -> None:  # type: ignore[no-untyped-def, override]
        super().msg(topic, data)
        self.pending.append((topic, len(data)))

    def topics(self) -> dict[str, SpyTopic]:
        """Typed accessor for the per-channel statistics."""
        return self.topic  # type: ignore[return-value]

    def drain(self) -> list[tuple[str, int]]:
        """Return and clear all packets queued since the last drain."""
        drained: list[tuple[str, int]] = []
        while True:
            try:
                drained.append(self.pending.popleft())
            except IndexError:
                return drained


class Highway:
    """All lanes plus the animation clock (pausable)."""

    def __init__(self, spy: PacketSpy | None = None) -> None:
        self.spy = spy if spy is not None else PacketSpy()
        self.lanes: dict[str, Lane] = {}
        self.paused = False
        self._clock = 0.0
        self._last_tick = time.monotonic()

    def start(self) -> None:
        self.spy.start()

    def stop(self) -> None:
        self.spy.stop()

    @property
    def clock(self) -> float:
        return self._clock

    def toggle_pause(self) -> None:
        self.paused = not self.paused

    def tick(self, road_width: int) -> None:
        """Advance the clock, spawn queued packets, prune off-road vehicles."""
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now
        if not self.paused:
            self._clock += dt

        packets = self.spy.drain()
        if self.paused:
            return
        for channel, n_bytes in packets:
            lane = self.lanes.get(channel)
            if lane is None:
                lane = Lane(channel)
                self.lanes[channel] = lane
            lane.spawn(n_bytes, self._clock)

        for lane in self.lanes.values():
            lane.prune(self._clock, road_width)
