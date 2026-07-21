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

"""One broker session shared across a blueprint's worker processes.

A broker session is a single ``RTCPeerConnection`` and so lives in one process.
When broker-bound modules land in several worker processes, exactly one process
(the *owner*) holds the real :class:`BrokerProvider`; the others resolve to a
:class:`ProxyBrokerProvider` that forwards publish/subscribe/video over the
machine-local :mod:`session_bus` to the owner, where a :class:`SessionRelay`
mirrors those bus subjects onto the real session's CF channels.

Ownership is decided by :func:`elect_session_owner` — a machine-local ``flock``
keyed on the session id. The winner runs the real provider + relay; losers proxy.
No coordinator involvement: the workers self-elect. The lock auto-releases when
the owner process dies, so a restart re-elects cleanly.
"""

from __future__ import annotations

from collections.abc import Callable
import os
import tempfile
import threading
from typing import TYPE_CHECKING, Any

from dimos.protocol.pubsub.impl.webrtc.providers import session_bus
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.protocol.pubsub.impl.shmpubsub import PickleSharedMemory, SharedMemoryPubSubBase

logger = setup_logger()


# ─── Ownership election (machine-local, coordinator-free) ────────────────


def elect_session_owner(session: str) -> tuple[bool, int | None]:
    """Try to become the owner of *session* on this host.

    Races every process of the run for one ``flock``. Returns ``(is_owner,
    lock_fd)``: the winner gets ``(True, fd)`` and MUST keep ``fd`` open for the
    provider's lifetime (closing it — or dying — releases ownership); losers get
    ``(False, None)`` and should proxy.
    """
    import fcntl

    path = os.path.join(tempfile.gettempdir(), f"dimos-cf-{session}.lock")
    fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o644)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        os.close(fd)
        logger.info("CF session %s: another worker owns it — proxying", session)
        return False, None
    logger.info("CF session %s: this worker is the owner", session)
    return True, fd


def release_session_owner(fd: int | None) -> None:
    """Release the ownership lock (closing the fd drops the ``flock``)."""
    if fd is not None:
        try:
            os.close(fd)
        except OSError:
            logger.exception("Failed to release CF ownership lock fd=%s", fd)


# ─── Proxy provider (non-owner workers) ──────────────────────────────────


class ProxyBrokerProvider:
    """Stand-in for ``BrokerProvider`` in a non-owner worker.

    Implements the ``Provider`` protocol (plus ``set_video_frame``) but holds no
    ``RTCPeerConnection``: publish → ``up`` subject, subscribe → ``down`` subject,
    video → the frame subject, all over :mod:`session_bus`. The owner's
    :class:`SessionRelay` bridges these to the one real CF session.
    """

    def __init__(self, session: str) -> None:
        self._session = session
        self._started = False
        self._lock = threading.Lock()
        # No CF session of its own (the owner holds it); present as None so code
        # that reads provider.session_id on a non-owner worker doesn't crash.
        self.session_id: str | None = None
        self._reliable: SharedMemoryPubSubBase | None = None
        self._lossy: SharedMemoryPubSubBase | None = None
        self._video: PickleSharedMemory | None = None
        self._audio_warned = False

    def start(self) -> None:
        with self._lock:
            if self._started:
                return
            self._reliable = session_bus.make_reliable_bus()
            self._lossy = session_bus.make_lossy_bus()
            self._started = True

    def stop(self) -> None:
        with self._lock:
            if not self._started:
                return
            for bus in (self._reliable, self._lossy, self._video):
                if bus is not None:
                    try:
                        bus.stop()
                    except Exception:
                        logger.exception("Error stopping proxy bus")
            self._reliable = self._lossy = self._video = None
            self._started = False

    @property
    def is_connected(self) -> bool:
        return self._started

    def _bytes_bus(self, channel: str) -> SharedMemoryPubSubBase:
        assert self._reliable is not None and self._lossy is not None
        return self._reliable if session_bus.is_reliable(channel) else self._lossy

    def publish(self, topic: str, data: bytes) -> None:
        """Robot → operator: forward to the owner over the ``up`` subject."""
        if not self._started:
            self.start()
        self._bytes_bus(topic).publish(session_bus.up_subject(self._session, topic), data)

    def subscribe(self, topic: str, callback: Callable[[bytes, str], None]) -> Callable[[], None]:
        """Operator → robot: receive from the owner over the ``down`` subject."""
        if not self._started:
            self.start()

        def _adapt(data: bytes, _subject: str) -> None:
            callback(data, topic)

        return self._bytes_bus(topic).subscribe(session_bus.down_subject(self._session, topic), _adapt)

    def set_video_frame(self, img: Any) -> None:
        """Robot → operator video: ship the frame to the owner's video track."""
        with self._lock:
            if self._video is None:
                self._video = session_bus.make_video_bus()
            video = self._video
        video.publish(session_bus.video_subject(self._session), img)

    def set_audio_frame_callback(self, cb: Callable[[bytes, int, int], None] | None) -> None:
        """Operator → robot audio sink. Not bridged across workers in v1: the
        operator's mic frames only reach a sink registered on the OWNER worker.
        Put the audio-consuming module on the owner if you need ``audio_in``."""
        if cb is not None and not self._audio_warned:
            self._audio_warned = True
            logger.warning(
                "operator audio (audio_in) is not bridged to non-owner workers; "
                "place the audio-consuming module on the session-owner worker"
            )


# ─── Relay (owner worker) ────────────────────────────────────────────────


class SessionRelay:
    """Mirrors the machine-local session bus onto the one real CF session.

    Runs in the owner process, bound to the real ``BrokerProvider`` (duck-typed:
    needs ``subscribe``/``publish``/``set_video_frame``). It is one extra
    publisher/subscriber on that provider, so it composes with any on-owner
    modules that bind the same channels directly — nothing special-cases
    local-vs-remote subscribers.
    """

    def __init__(self, session: str, provider: Any) -> None:
        self._session = session
        self._provider = provider
        self._lock = threading.Lock()
        self._started = False
        self._reliable: SharedMemoryPubSubBase | None = None
        self._lossy: SharedMemoryPubSubBase | None = None
        self._video: PickleSharedMemory | None = None
        self._unsubs: list[Callable[[], None]] = []

    def _bytes_bus(self, channel: str) -> SharedMemoryPubSubBase:
        assert self._reliable is not None and self._lossy is not None
        return self._reliable if session_bus.is_reliable(channel) else self._lossy

    def start(self) -> None:
        with self._lock:
            if self._started:
                return
            self._reliable = session_bus.make_reliable_bus()
            self._lossy = session_bus.make_lossy_bus()
            self._video = session_bus.make_video_bus()
            self._started = True

        # Inbound (operator → robot): CF → real provider → publish to `down` so
        # non-owner subscribers receive it. On-owner subscribers already get it
        # directly from the provider; the synthetic operator_lost and the ping
        # fan out here too (they arrive on the state_reliable callback).
        for ch in session_bus.INBOUND_CHANNELS:
            bus = self._bytes_bus(ch)
            subject = session_bus.down_subject(self._session, ch)

            def _to_down(data: bytes, _t: str, bus: Any = bus, subject: str = subject) -> None:
                try:
                    bus.publish(subject, data)
                except Exception:
                    logger.exception("relay: failed to publish inbound to %s", subject)

            self._unsubs.append(self._provider.subscribe(ch, _to_down))

        # Outbound (robot → operator): non-owner publishers → `up` → real
        # provider → CF. On-owner publishers call the provider directly.
        for ch in session_bus.OUTBOUND_CHANNELS:
            bus = self._bytes_bus(ch)
            subject = session_bus.up_subject(self._session, ch)

            def _to_cf(data: bytes, _s: str, ch: str = ch) -> None:
                try:
                    self._provider.publish(ch, data)
                except Exception:
                    logger.exception("relay: failed to publish %s to CF", ch)

            self._unsubs.append(bus.subscribe(subject, _to_cf))

        # Video: non-owner producer → frame subject → real sendonly track.
        assert self._video is not None

        def _to_track(img: Any, _s: str) -> None:
            try:
                self._provider.set_video_frame(img)
            except Exception:
                logger.exception("relay: failed to forward video frame")

        self._unsubs.append(self._video.subscribe(session_bus.video_subject(self._session), _to_track))
        logger.info("SessionRelay started for CF session %s", self._session)

    def stop(self) -> None:
        with self._lock:
            if not self._started:
                return
            self._started = False
        for unsub in self._unsubs:
            try:
                unsub()
            except Exception:
                logger.exception("relay: unsubscribe error")
        self._unsubs.clear()
        for bus in (self._reliable, self._lossy, self._video):
            if bus is not None:
                try:
                    bus.stop()
                except Exception:
                    logger.exception("relay: error stopping bus")
        self._reliable = self._lossy = self._video = None
