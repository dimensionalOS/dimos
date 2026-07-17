#!/usr/bin/env python3
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


from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
import hashlib
import os
import struct
import threading
import time
from typing import TYPE_CHECKING, Any
import uuid

import numpy as np
import numpy.typing as npt

from dimos.protocol.pubsub.encoders import lcm_decode, lcm_encode, pickle_decode, pickle_encode
from dimos.protocol.pubsub.shm.ipc_factory import CpuShmChannel, FrameChannel
from dimos.protocol.pubsub.spec import PubSub
from dimos.protocol.pubsub.topic import Topic
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable

logger = setup_logger()


@dataclass
class SharedMemoryConfig:
    prefer: str = "auto"  # "auto" | "cpu"  (DIMOS_IPC_BACKEND overrides), TODO: "cuda"
    default_capacity: int = 3686400  # payload bytes (excludes 4-byte header)
    close_channels_on_stop: bool = True


class SharedMemoryPubSubBase(PubSub[str, Any]):
    """
    Pub/Sub over SharedMemory/CUDA-IPC, modeled after LCMPubSubBase but self-contained.
    Wire format per topic/frame: [len:uint32_le] + payload bytes (padded to fixed capacity).
    Features ported from Service:
      - start()/stop() lifecycle
      - one frame channel per topic
      - per-topic fanout thread (reads from channel, invokes subscribers)
      - CPU/CUDA backend selection (auto + env override)
      - reconfigure(topic, capacity=...)
      - drop initial empty frame; synchronous local delivery; echo suppression
    """

    # Frame-channel implementation backing each topic. Streaming keeps the
    # default double-buffered CpuShmChannel (latest-wins); subclasses that need
    # every message delivered (e.g. ShmRPC) override this with CpuShmQueue.
    _channel_class: type[FrameChannel] = CpuShmChannel

    # Extra keyword args passed to _channel_class(...) at construction (empty for
    # the streaming channel; ShmRPC sets e.g. {"slots": N}). Folded into the
    # segment name so distinct layouts never mmap over one another.
    _channel_kwargs: dict[str, Any] = {}

    # Per-topic state
    # TODO: implement "is_cuda" below capacity, above cp
    class _TopicState:
        __slots__ = (
            "capacity",
            "channel",
            "cp",
            "dtype",
            "last_local_payload",
            "last_seq",
            "publish_buffer",
            "publish_lock",
            "shape",
            "stop",
            "subs",
            "suppress_counts",
            "thread",
        )

        def __init__(self, channel, capacity: int, cp_mod) -> None:  # type: ignore[no-untyped-def]
            self.channel = channel
            self.capacity = int(capacity)
            self.shape = (self.capacity + 20,)  # +20 for header: length(4) + uuid(16)
            self.dtype = np.uint8
            self.subs: list[Callable[[bytes, str], None]] = []
            self.stop = threading.Event()
            self.thread: threading.Thread | None = None
            self.last_seq = 0  # start at 0 to avoid b"" on first poll
            # TODO: implement an initializer variable for is_cuda once CUDA IPC is in
            self.cp = cp_mod
            self.last_local_payload: bytes | None = None
            self.suppress_counts: dict[bytes, int] = defaultdict(int)  # UUID bytes as key
            # Pre-allocated buffer to avoid allocation on every publish
            self.publish_buffer: npt.NDArray[np.uint8] = np.zeros(self.shape, dtype=self.dtype)
            # Lock for thread-safe publish buffer access
            self.publish_lock = threading.Lock()

    # init / lifecycle

    def __init__(
        self,
        *,
        prefer: str = "auto",
        default_capacity: int = 3686400,
        close_channels_on_stop: bool = True,
        **_: Any,
    ) -> None:
        super().__init__()
        self.config = SharedMemoryConfig(
            prefer=prefer,
            default_capacity=default_capacity,
            close_channels_on_stop=close_channels_on_stop,
        )
        self._topics: dict[str, SharedMemoryPubSubBase._TopicState] = {}
        self._lock = threading.Lock()

    def start(self) -> None:
        pref = self.config.prefer.lower()
        backend = os.getenv("DIMOS_IPC_BACKEND", pref).lower()
        logger.debug(f"SharedMemory PubSub starting (backend={backend})")
        # No global thread needed; per-topic fanout starts on first subscribe.

    def stop(self) -> None:
        with self._lock:
            for _topic, st in list(self._topics.items()):
                if st.thread:
                    st.stop.set()
                    st.thread.join()
                    st.thread = None
                if self.config.close_channels_on_stop:
                    st.channel.close()
            self._topics.clear()
        logger.debug("SharedMemory PubSub stopped.")

    # PubSub API (bytes on the wire)

    def publish(self, topic: str, message: bytes) -> None:
        st = self._ensure_topic(topic)

        # Normalize once
        payload_bytes = bytes(message)
        L = len(payload_bytes)
        if L > st.capacity:
            logger.error(f"Payload too large: {L} > capacity {st.capacity}")
            raise ValueError(f"Payload too large: {L} > capacity {st.capacity}")

        # Create a unique identifier using UUID4
        message_id = uuid.uuid4().bytes  # 16 bytes

        # Mark this message to suppress its echo
        st.suppress_counts[message_id] += 1

        # Synchronous local delivery first (zero extra copies)
        for cb in list(st.subs):
            cb(payload_bytes, topic)

        # Build host frame [len:4] + [uuid:16] + payload and publish
        # We embed the message UUID in the frame for echo suppression
        # Reuse pre-allocated buffer to avoid allocation overhead
        # Lock to prevent concurrent threads from corrupting the shared buffer
        with st.publish_lock:
            host = st.publish_buffer
            # Pack: length(4) + uuid(16) + payload
            header = struct.pack("<I", L + 16)  # L+16 for uuid
            host[:4] = np.frombuffer(header, dtype=np.uint8)
            host[4:20] = np.frombuffer(message_id, dtype=np.uint8)
            if L:
                host[20 : 20 + L] = np.frombuffer(memoryview(payload_bytes), dtype=np.uint8)

            # Only copy actual message size (header + payload) not full capacity
            st.channel.publish(host, length=20 + L)

    def subscribe(self, topic: str, callback: Callable[[bytes, str], Any]) -> Callable[[], None]:
        """Subscribe a callback(message: bytes, topic). Returns unsubscribe."""
        st = self._ensure_topic(topic)
        st.subs.append(callback)
        if st.thread is None:
            st.thread = threading.Thread(target=self._fanout_loop, args=(topic, st), daemon=True)
            st.thread.start()

        def _unsub() -> None:
            st.subs.remove(callback)
            if not st.subs and st.thread:
                st.stop.set()
                st.thread.join()
                st.thread = None
                st.stop.clear()

        return _unsub

    # Capacity mgmt

    def reconfigure(self, topic: str, *, capacity: int) -> dict:  # type: ignore[type-arg]
        """Change payload capacity (bytes) for a topic; returns new descriptor."""
        st = self._ensure_topic(topic)
        new_cap = int(capacity)
        new_shape = (new_cap + 20,)  # +20 for header: length(4) + uuid(16)
        # Lock to ensure no publish is using the buffer while we replace it
        with st.publish_lock:
            desc = st.channel.reconfigure(new_shape, np.uint8)
            st.capacity = new_cap
            st.shape = new_shape
            st.dtype = np.uint8
            st.last_seq = -1
            st.publish_buffer = np.zeros(new_shape, dtype=np.uint8)
        return desc  # type: ignore[no-any-return]

    # Internals

    def _ensure_topic(self, topic: str) -> _TopicState:
        with self._lock:
            st = self._topics.get(topic)
            if st is not None:
                return st
            cap = int(self.config.default_capacity)

            def _names_for_topic(topic: str, capacity: int) -> tuple[str, str]:
                # Python's SharedMemory requires names without a leading '/'.
                # Fold the channel class name AND its layout kwargs (e.g. slots)
                # into the hash so two pubsubs using different layouts (streaming
                # CpuShmChannel vs RPC CpuShmQueue, or two ring sizes) on the same
                # topic get distinct segments rather than mmapping incompatible
                # layouts over each other.
                # Use a short digest to avoid macOS shared memory name length limits.
                layout = f"{self._channel_class.__name__}:{sorted(self._channel_kwargs.items())}"
                h = hashlib.blake2b(
                    f"{topic}:{capacity}:{layout}".encode(), digest_size=8
                ).hexdigest()
                return f"psm_{h}_data", f"psm_{h}_ctrl"

            data_name, ctrl_name = _names_for_topic(topic, cap)
            ch = self._channel_class(
                (cap + 20,),
                np.uint8,
                data_name=data_name,
                ctrl_name=ctrl_name,
                **self._channel_kwargs,
            )
            st = SharedMemoryPubSubBase._TopicState(ch, cap, None)
            self._topics[topic] = st
            return st

    def _fanout_loop(self, topic: str, st: _TopicState) -> None:
        while not st.stop.is_set():
            seq, _ts_ns, view = st.channel.read(last_seq=st.last_seq, require_new=True)
            if view is None:
                time.sleep(0.001)
                continue
            st.last_seq = seq

            host = np.array(view, copy=True)

            L = struct.unpack("<I", host[:4].tobytes())[0]
            if L < 16 or L > st.capacity + 16:
                continue

            message_id = host[4:20].tobytes()
            payload_len = L - 16
            if payload_len == 0:
                continue
            payload = host[20 : 20 + payload_len].tobytes()

            cnt = st.suppress_counts.get(message_id, 0)
            if cnt > 0:
                if cnt == 1:
                    del st.suppress_counts[message_id]
                else:
                    st.suppress_counts[message_id] = cnt - 1
                continue

            for cb in list(st.subs):
                cb(payload, topic)


BytesSharedMemory = SharedMemoryPubSubBase


class PickleSharedMemory(SharedMemoryPubSubBase):
    """SharedMemory pubsub that transports arbitrary Python objects via pickle."""

    def publish(self, topic: str, message: Any) -> None:
        super().publish(topic, pickle_encode(message, topic))

    def subscribe(self, topic: str, callback: Callable[[Any, str], Any]) -> Callable[[], None]:
        return super().subscribe(
            topic, lambda message, name: callback(pickle_decode(message, name), name)
        )


class LCMSharedMemoryPubSubBase(PubSub[Topic, Any]):
    """SharedMemory pubsub that uses LCM Topic type, delegating to SharedMemoryPubSubBase."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__()
        self._shm = SharedMemoryPubSubBase(**kwargs)

    def start(self) -> None:
        self._shm.start()

    def stop(self) -> None:
        self._shm.stop()

    def publish(self, topic: Topic, message: bytes) -> None:
        self._shm.publish(str(topic), message)

    def subscribe(
        self, topic: Topic, callback: Callable[[bytes, Topic], Any]
    ) -> Callable[[], None]:
        def wrapper(msg: bytes, _: str) -> None:
            callback(msg, topic)

        return self._shm.subscribe(str(topic), wrapper)

    def reconfigure(self, topic: Topic, *, capacity: int) -> dict:  # type: ignore[type-arg]
        return self._shm.reconfigure(str(topic), capacity=capacity)


class LCMSharedMemory(LCMSharedMemoryPubSubBase):
    """SharedMemory pubsub that uses LCM binary encoding (no pickle overhead)."""

    def publish(self, topic: Topic, message: Any) -> None:
        super().publish(topic, lcm_encode(message, topic))

    def subscribe(self, topic: Topic, callback: Callable[[Any, Topic], Any]) -> Callable[[], None]:
        return super().subscribe(
            topic, lambda message, name: callback(lcm_decode(message, name), name)
        )
