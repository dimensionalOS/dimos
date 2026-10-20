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

"""Pub/sub over ZeroMQ, on the LCM-compatible `Topic`.

Wire format is two frames, `[key, payload]`, where `key` is `str(topic)` (for
example `/odom#nav_msgs.Odometry`) followed by a NUL. zmq filters by prefix,
so the NUL is what makes a subscription to `/foo` an exact match that never
sees `/foobar`. Pattern topics and `subscribe_all` subscribe to every key and
match in Python, the way LCM does natively.

zmq subscriptions travel to the publishers asynchronously, so a message sent
right after `subscribe()` returns would normally be lost (the slow-joiner
problem). `subscribe()` therefore publishes a private probe to itself until it
comes back, which proves the subscription queued ahead of it has reached this
process's publisher; the wait is bounded by `settle_timeout`.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
from functools import lru_cache
import itertools
import re
import threading
import time
from typing import Any
import uuid

import zmq

from dimos.protocol.pubsub.encoders import LCMEncoderMixin, PickleEncoderMixin
from dimos.protocol.pubsub.impl.lcmpubsub import Topic
from dimos.protocol.pubsub.spec import AllPubSub
from dimos.protocol.service.zmqservice import ZMQService
from dimos.utils.logging_config import setup_logger

__all__ = ["ZMQ", "PickleZMQ", "Topic", "ZMQPubSubBase"]

logger = setup_logger()

# Ends every wire key, turning zmq's prefix match into an exact one.
_TERMINATOR = b"\x00"
# Starts every probe key. A channel never starts with NUL, so a subscription
# to everything can tell probes (any instance's) from traffic.
_PROBE_MARK = b"\x00"

# How long the receive thread sleeps in poll() before re-checking for work
# that arrived without a doorbell ring.
_POLL_TIMEOUT_MS = 100
# Messages the receive thread drains between looks at the pending ops.
_RECV_BATCH = 64
# Probe publish backoff while waiting for a subscription to settle.
_SETTLE_FIRST_WAIT = 0.001
_SETTLE_MAX_WAIT = 0.02
# Seconds stop() waits for the receive thread.
_STOP_JOIN_TIMEOUT = 5.0

_OP_SUBSCRIBE = b"S"
_OP_UNSUBSCRIBE = b"U"


def _wire_key(topic: Topic | str) -> bytes:
    """The first frame a message on this topic carries."""
    key = str(topic).encode()
    if _TERMINATOR in key:
        raise ValueError(f"ZeroMQ topic {str(topic)!r} must not contain NUL")
    return key + _TERMINATOR


@lru_cache(maxsize=1024)
def _channel_to_topic(channel: str, default_lcm_type: type | None = None) -> Topic:
    """The Topic a received key names. Cached: treat the result as immutable."""
    return Topic.from_channel_str(channel, default_lcm_type)


def _pattern_regex(topic: Topic) -> re.Pattern[str]:
    """Anchored regex for a pattern topic, with LCM's optional `#type` suffix."""
    pattern = str(topic)
    if not pattern.endswith("*"):
        pattern = f"{pattern}(#.*)?"
    return re.compile(pattern)


@dataclass(eq=False)
class _Subscription:
    topic: Topic
    callback: Callable[[bytes, Topic], None]
    # The zmq-level prefix this rides on: the exact key, or b"" for patterns.
    zmq_key: bytes
    matcher: Callable[[str], Any] | None
    alive: bool = True


class ZMQPubSubBase(ZMQService, AllPubSub[Topic, bytes]):
    """Raw bytes pub/sub over ZeroMQ.

    One PUB socket carries every publish, under a lock. One SUB socket is owned
    by the receive thread, which also applies subscription changes: zmq sockets
    are single-threaded, so callers queue the change and ring an inproc doorbell.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._init_runtime()
        self._started_once = False

    def _init_runtime(self) -> None:
        self._iid = uuid.uuid4().hex.encode()
        self._probe_counter = itertools.count()
        self._lock = threading.Lock()
        self._exact: dict[bytes, list[_Subscription]] = {}
        self._patterns: list[_Subscription] = []
        # Live callbacks per zmq-level prefix; the prefix is subscribed while > 0.
        self._refs: dict[bytes, int] = {}
        self._probes: dict[bytes, threading.Event] = {}
        self._ops: deque[tuple[bytes, bytes]] = deque()
        self._pub_lock = threading.Lock()
        self._pub: zmq.Socket[bytes] | None = None
        self._ctl_lock = threading.Lock()
        self._ctl: zmq.Socket[bytes] | None = None
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def __getstate__(self):  # type: ignore[no-untyped-def]
        """Drop sockets, threads and locks on top of the context."""
        state = super().__getstate__()  # type: ignore[no-untyped-call]
        for key in (
            "_iid",
            "_probe_counter",
            "_lock",
            "_exact",
            "_patterns",
            "_refs",
            "_probes",
            "_ops",
            "_pub_lock",
            "_pub",
            "_ctl_lock",
            "_ctl",
            "_stop_event",
            "_thread",
        ):
            state.pop(key, None)
        return state

    def __setstate__(self, state) -> None:  # type: ignore[no-untyped-def]
        super().__setstate__(state)
        self._init_runtime()

    # -- lifecycle ----------------------------------------------------------

    def start(self) -> None:
        with self._ctl_lock:
            if self._thread is not None and self._thread.is_alive():
                return
            super().start()
            context = self.context
            sub = context.socket(zmq.SUB)
            sub.set_hwm(self.config.hwm)
            sub.linger = 0
            sub.connect(self.config.sub_endpoint)
            doorbell = f"inproc://dimos-zmq-doorbell-{self._iid.decode()}"
            ctl_rx = context.socket(zmq.PAIR)
            ctl_rx.set_hwm(0)
            ctl_rx.linger = 0
            ctl_rx.bind(doorbell)
            ctl = context.socket(zmq.PAIR)
            ctl.set_hwm(0)
            ctl.linger = 0
            ctl.connect(doorbell)
            pub = context.socket(zmq.PUB)
            pub.set_hwm(self.config.hwm)
            pub.linger = 0
            pub.connect(self.config.pub_endpoint)
            with self._pub_lock:
                self._pub = pub
            self._ctl = ctl
            self._stop_event = threading.Event()
            # A fresh socket knows no prefixes: rebuild them from the live
            # callbacks rather than replaying ops queued while stopped. zmq
            # counts duplicate subscriptions, so each prefix goes exactly once.
            with self._lock:
                self._ops.clear()
                self._ops.extend((_OP_SUBSCRIBE, key) for key in self._refs)
            self._thread = threading.Thread(
                target=self._recv_loop,
                args=(sub, ctl_rx, self._stop_event),
                name="zmq-recv",
                daemon=True,
            )
            self._thread.start()
            self._started_once = True

    def stop(self) -> None:
        with self._ctl_lock:
            thread = self._thread
            ctl = self._ctl
            self._ctl = None
            self._thread = None
            self._stop_event.set()
            if ctl is not None:
                self._ring(ctl)
        with self._pub_lock:
            if self._pub is not None:
                self._pub.close()
                self._pub = None
        if thread is not None and threading.current_thread() is not thread:
            # A callback may stop us from the receive thread, which exits once
            # the callback returns.
            thread.join(timeout=_STOP_JOIN_TIMEOUT)
            if thread.is_alive():
                logger.warning("ZeroMQ receive thread did not stop in time")
        if ctl is not None:
            ctl.close()
        with self._lock:
            self._ops.clear()
            for event in self._probes.values():
                event.set()
        super().stop()

    @staticmethod
    def _ring(ctl: zmq.Socket[bytes]) -> None:
        """Wake the receive thread. A lost ring costs at most one poll timeout."""
        try:
            ctl.send(b"", zmq.NOBLOCK)
        except zmq.ZMQError:
            pass

    def _queue_op(self, op: bytes, key: bytes) -> None:
        with self._lock:
            self._ops.append((op, key))
        with self._ctl_lock:
            if self._ctl is not None:
                self._ring(self._ctl)

    # -- receive thread -----------------------------------------------------

    def _recv_loop(
        self, sub: zmq.Socket[bytes], ctl_rx: zmq.Socket[bytes], stop: threading.Event
    ) -> None:
        poller = zmq.Poller()
        poller.register(sub, zmq.POLLIN)
        poller.register(ctl_rx, zmq.POLLIN)
        try:
            while not stop.is_set():
                events = dict(poller.poll(_POLL_TIMEOUT_MS))
                if ctl_rx in events:
                    while True:
                        try:
                            ctl_rx.recv(zmq.NOBLOCK)
                        except zmq.Again:
                            break
                self._apply_ops(sub)
                if stop.is_set() or sub not in events:
                    continue
                for _ in range(_RECV_BATCH):
                    try:
                        frames = sub.recv_multipart(zmq.NOBLOCK)
                    except zmq.Again:
                        break
                    self._dispatch(frames)
                    if stop.is_set():
                        break
        except zmq.ZMQError as e:
            if e.errno != zmq.ETERM:
                logger.error("ZeroMQ receive loop stopped on error", exc_info=True)
        except Exception:
            logger.error("ZeroMQ receive loop stopped on error", exc_info=True)
        finally:
            sub.close()
            ctl_rx.close()

    def _apply_ops(self, sub: zmq.Socket[bytes]) -> None:
        with self._lock:
            ops = list(self._ops)
            self._ops.clear()
        for op, key in ops:
            if op == _OP_SUBSCRIBE:
                sub.setsockopt(zmq.SUBSCRIBE, key)
            else:
                sub.setsockopt(zmq.UNSUBSCRIBE, key)

    def _dispatch(self, frames: list[bytes]) -> None:
        if len(frames) != 2:
            logger.warning("Dropping malformed ZeroMQ message", frames=len(frames))
            return
        key, payload = frames
        if key.startswith(_PROBE_MARK):
            with self._lock:
                probe = self._probes.get(key)
            if probe is not None:
                probe.set()
            return
        with self._lock:
            exact = list(self._exact.get(key, ()))
            patterns = list(self._patterns)
        for sub in exact:
            if sub.alive:
                self._deliver(sub, payload, sub.topic)
        if not patterns:
            return
        channel = key.removesuffix(_TERMINATOR).decode(errors="replace")
        for sub in patterns:
            assert sub.matcher is not None
            if sub.alive and sub.matcher(channel):
                self._deliver(sub, payload, _channel_to_topic(channel, sub.topic.lcm_type))

    @staticmethod
    def _deliver(sub: _Subscription, payload: bytes, topic: Topic) -> None:
        try:
            sub.callback(payload, topic)
        except Exception:
            logger.error(f"Callback error on topic {topic}", exc_info=True)

    # -- publish ------------------------------------------------------------

    def publish(self, topic: Topic | str, message: bytes) -> None:
        """Publish bytes on a topic. Nobody listening is not an error."""
        self._publish_raw(_wire_key(topic), message)

    def _publish_raw(self, key: bytes, payload: bytes) -> None:
        with self._pub_lock:
            if self._pub is None:
                if not self._started_once:
                    raise RuntimeError("ZeroMQ pub/sub not started. Call start() first.")
                logger.error("Tried to publish after ZeroMQ pub/sub was stopped")
                return
            try:
                # Copying keeps pyzmq's zero-copy garbage thread out of the process;
                # a memcpy is noise next to the socket hop even for camera frames.
                self._pub.send_multipart([key, payload])
            except zmq.ZMQError:
                logger.error(f"Error publishing to {key!r}", exc_info=True)

    # -- subscribe ----------------------------------------------------------

    def subscribe(
        self, topic: Topic, callback: Callable[[bytes, Topic], None]
    ) -> Callable[[], None]:
        """Subscribe to an exact or pattern topic. Returns the unsubscribe function.

        Returns once the subscription has settled at this process's publisher
        (see the module docstring), or after `settle_timeout`.
        """
        if not self._started_once:
            raise RuntimeError("ZeroMQ pub/sub not started. Call start() first.")
        if topic.is_pattern:
            sub = _Subscription(topic, callback, b"", _pattern_regex(topic).fullmatch)
        else:
            sub = _Subscription(topic, callback, _wire_key(topic), None)
        with self._lock:
            if sub.matcher is None:
                self._exact.setdefault(sub.zmq_key, []).append(sub)
            else:
                self._patterns.append(sub)
            first = self._refs.get(sub.zmq_key, 0) == 0
            self._refs[sub.zmq_key] = self._refs.get(sub.zmq_key, 0) + 1
        if first:
            self._queue_op(_OP_SUBSCRIBE, sub.zmq_key)
            self._settle()

        def unsubscribe() -> None:
            with self._lock:
                if not sub.alive:
                    return
                sub.alive = False
                if sub.matcher is None:
                    subs = self._exact[sub.zmq_key]
                    subs.remove(sub)
                    if not subs:
                        del self._exact[sub.zmq_key]
                else:
                    self._patterns.remove(sub)
                self._refs[sub.zmq_key] -= 1
                last = self._refs[sub.zmq_key] == 0
                if last:
                    del self._refs[sub.zmq_key]
            if last:
                self._queue_op(_OP_UNSUBSCRIBE, sub.zmq_key)

        return unsubscribe

    def subscribe_all(self, callback: Callable[[bytes, Topic], Any]) -> Callable[[], None]:
        return self.subscribe(Topic(re.compile(".*")), callback)

    def _settle(self) -> None:
        """Block until a probe queued behind the latest subscription round-trips.

        Skipped on the receive thread itself (a callback subscribing), which
        would otherwise wait on a probe only it can receive.
        """
        timeout = self.config.settle_timeout
        if timeout <= 0 or threading.current_thread() is self._thread:
            return
        probe = _PROBE_MARK + self._iid + b"/" + str(next(self._probe_counter)).encode()
        probe += _TERMINATOR
        arrived = threading.Event()
        with self._lock:
            self._probes[probe] = arrived
        self._queue_op(_OP_SUBSCRIBE, probe)
        deadline = time.monotonic() + timeout
        wait = _SETTLE_FIRST_WAIT
        try:
            while True:
                with self._pub_lock:
                    if self._pub is None:
                        return
                self._publish_raw(probe, b"")
                if arrived.wait(wait):
                    return
                if time.monotonic() >= deadline:
                    logger.warning(
                        "ZeroMQ subscription did not settle, is a proxy reachable?",
                        pub=self.config.pub_endpoint,
                        sub=self.config.sub_endpoint,
                        timeout=timeout,
                    )
                    return
                wait = min(wait * 2, _SETTLE_MAX_WAIT)
        finally:
            with self._lock:
                self._probes.pop(probe, None)
            self._queue_op(_OP_UNSUBSCRIBE, probe)


class ZMQ(  # type: ignore[misc]
    LCMEncoderMixin,
    ZMQPubSubBase,
):
    """ZeroMQ pub/sub with LCM encoding for typed DimosMsg."""

    ...


class PickleZMQ(
    PickleEncoderMixin,  # type: ignore[type-arg]
    ZMQPubSubBase,
):
    """ZeroMQ pub/sub with pickle encoding for arbitrary Python objects."""

    ...
