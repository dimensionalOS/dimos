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

"""Machine-local session bus — bridges one Cloudflare/broker session across the
worker processes of a single blueprint.

A broker session is one ``RTCPeerConnection`` and therefore lives in exactly one
process (the *owner*). When broker-bound modules are spread across worker
processes, the non-owner workers reach that single session through this bus
instead of each opening their own session. The bus is pure shared memory
(``dimos.protocol.pubsub.impl.shmpubsub``), so it never leaves the host.

Per broker channel there are two subjects, so neither side needs to reason about
direction beyond publish-vs-subscribe:

* ``cf.<session>.<channel>.up``    worker → owner   (owner re-publishes to CF)
* ``cf.<session>.<channel>.down``  owner → worker   (delivered from CF / synthetic)

Reliability mirrors the broker channel itself (see ``broker.py``): the reliable
control-plane channels ride a reliable SHM queue; the lossy channels ride the
default latest-wins SHM channel; video rides a latest-wins frame subject.

The subject namespace keys on the ``BrokerConfig`` identity plus ``DIMOS_RUN_ID``
so co-tenant robots/tools and other runs on the same host never share segments.
"""

from __future__ import annotations

import hashlib
import os
from typing import TYPE_CHECKING, Any

from dimos.protocol.pubsub.impl.shmpubsub import PickleSharedMemory, SharedMemoryPubSubBase
from dimos.protocol.pubsub.shm.ipc_factory import CpuShmQueue

if TYPE_CHECKING:
    from dimos.protocol.pubsub.impl.webrtc.providers.broker import BrokerConfig

# Set per launch by the coordinator; inherited by every forkserver worker of the
# run (see dimos/core/coordination/process_lifecycle.py: DIMOS_RUN_ID_ENV). Kept
# as a bare string to avoid a provider→coordination import dependency.
_RUN_ID_ENV = "DIMOS_RUN_ID"

# Broker channel directions, mirrored from broker.py (kept here so broker_proxy
# needs no import from broker.py — that would be circular, since broker.py
# imports the election entrypoint from broker_proxy). Direction is from the
# robot's point of view.
INBOUND_CHANNELS: tuple[str, ...] = ("cmd_unreliable", "state_reliable")  # operator → robot
OUTBOUND_CHANNELS: tuple[str, ...] = ("state_reliable_back", "map_unreliable")  # robot → operator

# Broker channels that must not silently drop (control plane). Everything else
# (cmd_unreliable, map_unreliable) is lossy by design and matches broker.py's
# _channel_options(). Video is handled separately (frame subject).
RELIABLE_CHANNELS: frozenset[str] = frozenset({"state_reliable", "state_reliable_back"})


def is_reliable(channel: str) -> bool:
    """Whether *channel* needs every-message (reliable) delivery on the bus."""
    return channel in RELIABLE_CHANNELS

# Reliable-queue sizing. Control-plane messages are small and low-rate, so a
# small per-slot capacity keeps slots*capacity modest (unlike the 3.6MB frame
# default). Sized for a burst without unbounded memory.
_RELIABLE_SLOTS = 64
_RELIABLE_CAPACITY = 64 * 1024  # bytes per message


class _ReliableBytesShm(SharedMemoryPubSubBase):
    """Every-message SHM bytes pubsub (queue-backed) for reliable channels."""

    _channel_class = CpuShmQueue
    _channel_kwargs = {"slots": _RELIABLE_SLOTS}

    def __init__(self, **kwargs: Any) -> None:
        kwargs.setdefault("default_capacity", _RELIABLE_CAPACITY)
        super().__init__(**kwargs)


def session_key(config: BrokerConfig) -> str:
    """Stable per-session id: broker identity + run id, hashed.

    Equal ``BrokerConfig`` in the same run ⇒ same key ⇒ same bus subjects, so
    every worker of the blueprint rendezvous on one session; different runs or
    different configs never collide.
    """
    run_id = os.environ.get(_RUN_ID_ENV, "")
    ident = repr(sorted(config.model_dump().items())) + "|" + run_id
    return hashlib.blake2b(ident.encode(), digest_size=6).hexdigest()


def up_subject(session: str, channel: str) -> str:
    """worker → owner subject for a broker channel."""
    return f"cf.{session}.{channel}.up"


def down_subject(session: str, channel: str) -> str:
    """owner → worker subject for a broker channel."""
    return f"cf.{session}.{channel}.down"


def video_subject(session: str) -> str:
    """owner-bound camera-frame subject (single producer)."""
    return f"cf.{session}.video"


def make_reliable_bus() -> SharedMemoryPubSubBase:
    """A started queue-backed (every-message) SHM bytes pubsub for the reliable
    control-plane channels. One instance hosts every reliable subject."""
    bus = _ReliableBytesShm()
    bus.start()
    return bus


def make_lossy_bus() -> SharedMemoryPubSubBase:
    """A started latest-wins SHM bytes pubsub for the lossy channels. One
    instance hosts every lossy subject."""
    bus = SharedMemoryPubSubBase()
    bus.start()
    return bus


def make_video_bus() -> PickleSharedMemory:
    """A started latest-wins SHM pubsub carrying camera ``Image`` objects.

    Pickle (not JPEG) so the owner hands the raw frame straight to the video
    track without a decode/re-encode round-trip; latest-wins matches realtime
    video (a late subscriber wants the newest frame, not a backlog).
    """
    bus = PickleSharedMemory()
    bus.start()
    return bus
