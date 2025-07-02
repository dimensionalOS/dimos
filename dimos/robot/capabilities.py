# Copyright 2025 Dimensional Inc.
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

"""Capability protocols for DIMOS robots.

Each protocol describes *one* orthogonal capability. Concrete connection
classes do *not* need to inherit from these protocols – they satisfy a
protocol simply by implementing methods with compatible signatures.

Runtime checks (``isinstance(obj, Move)``) are enabled via
``@runtime_checkable`` so you can safely branch on capabilities at run
-time without raising ``TypeError``.
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable, Optional
from reactivex.observable import Observable

from dimos.types.vector import Vector


@runtime_checkable
class Move(Protocol):
    """Capability: command body-centric velocity for a duration (or continuously)."""

    def move(self, velocity: Vector, duration: float = 0.0) -> bool: ...
    def stop(self) -> bool: ...


@runtime_checkable
class Stop(Protocol):
    """Capability: halt all motion quickly."""

    def stop(self) -> bool: ...


@runtime_checkable
class Disconnect(Protocol):
    """Capability: close network / transport resources."""

    def disconnect(self) -> None: ...


@runtime_checkable
class Video(Protocol):
    """Capability: provide RGB video frames as an RxPY ``Observable``."""

    def video_stream(self, **kwargs) -> Optional[Observable]: ...


@runtime_checkable
class Lidar(Protocol):
    """Capability: provide LIDAR packets/frames as an RxPY ``Observable``."""

    def lidar_stream(self) -> Observable: ...


@runtime_checkable
class Odometry(Protocol):
    """Capability: provide odometry (pose) updates as an RxPY ``Observable``."""

    def odom_stream(self) -> Observable: ...
    def get_pose(self) -> dict: ...


@runtime_checkable
class Connection(Protocol):
    """Minimal transport-level contract every connection must satisfy."""

    ip: str  # Public network identifier (may be DNS, IP, serial port, etc.)

    def connect(self) -> None: ...
    def disconnect(self) -> None: ...


@runtime_checkable
class WebRTCRequest(Protocol):
    """Capability: ability to enqueue WebRTC API requests via `queue_webrtc_req`."""

    def queue_webrtc_req(
        self,
        api_id: int,
        topic: str | None = None,
        parameter: str | dict | None = "",
        priority: int = 0,
        request_id: str | None = None,
        data=None,
        timeout: float = 1000.0,
    ): ...


def has_capability(obj, proto: type[Protocol]) -> bool:
    """Return ``True`` if *obj* implements *proto* at runtime."""

    return isinstance(obj, proto)


# Cosmetic decorator
def implements(*protocols: type[Protocol]):
    """No-op decorator to annotate which capability protocols a class fulfills."""

    def _decorator(cls):  # type: ignore[missing-return-type-hint]
        cls.__implements__ = tuple(protocols)  # type: ignore[attr-defined]
        return cls

    return _decorator
