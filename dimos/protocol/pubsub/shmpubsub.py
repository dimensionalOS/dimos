#!/usr/bin/env python3
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

# ---------------------------------------------------------------------------
# SharedMemory Pub/Sub over unified IPC channels (CPU/CUDA)
# ---------------------------------------------------------------------------

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

from dimos.protocol.pubsub.shm.ipc_factory import CpuShmChannel
from dimos.protocol.pubsub.spec import PickleEncoderMixin, PubSub, PubSubEncoderMixin
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable

logger = setup_logger()


# --------------------------------------------------------------------------------------
# Configuration (kept local to PubSub now that Service is gone)
# --------------------------------------------------------------------------------------


@dataclass
class SharedMemoryConfig:
    prefer: str = "auto"  # "auto" | "cpu"  (DIMOS_IPC_BACKEND overrides), TODO: "cuda"
    default_capacity: int = 3686400  # payload bytes (excludes 4-byte header)
    close_channels_on_stop: bool = True


# --------------------------------------------------------------------------------------
# Core PubSub with integrated SHM/IPC transport (previously the Service logic)
# --------------------------------------------------------------------------------------


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

    # ----- init / lifecycle -------------------------------------------------

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
        pref = (self.config.prefer or "auto").lower()
        backend = os.getenv("DIMOS_IPC_BACKEND", pref).lower()
        logger.info(f"SharedMemory PubSub starting (backend={backend})")
        # No global thread needed; per-topic fanout starts on first subscribe.

    def stop(self) -> None:
        with self._lock:
            for _topic, st in list(self._topics.items()):
                # stop fanout
                try:
                    if st.thread:
                        st.stop.set()
                        st.thread.join(timeout=0.5)
                        st.thread = None
                except Exception:
                    pass
                # close/unlink channels if configured
                if self.config.close_channels_on_stop:
                    try:
                        st.channel.close()
                    except Exception:
                        pass
            self._topics.clear()
        logger.info("SharedMemory PubSub stopped.")

    # ----- PubSub API (bytes on the wire) ----------------------------------

    def publish(self, topic: str, message: bytes) -> None:
        if not isinstance(message, bytes | bytearray | memoryview):
            raise TypeError(f"publish expects bytes-like, got {type(message)!r}")

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
            try:
                cb(payload_bytes, topic)
            except Exception:
                logger.warn(f"Payload couldn't be pushed to topic: {topic}")
                pass

        # Build host frame [len:4] + [uuid:16] + payload and publish
        # We embed the message UUID in the frame for echo suppression
        host = np.zeros(st.shape, dtype=st.dtype)
        # Pack: length(4) + uuid(16) + payload
        header = struct.pack("<I", L + 16)  # L+16 for uuid
        host[:4] = np.frombuffer(header, dtype=np.uint8)
        host[4:20] = np.frombuffer(message_id, dtype=np.uint8)
        if L:
            host[20 : 20 + L] = np.frombuffer(memoryview(payload_bytes), dtype=np.uint8)

        st.channel.publish(host)

    def subscribe(self, topic: str, callback: Callable[[bytes, str], Any]) -> Callable[[], None]:
        """Subscribe a callback(message: bytes, topic). Returns unsubscribe."""
        st = self._ensure_topic(topic)
        st.subs.append(callback)
        if st.thread is None:
            st.thread = threading.Thread(target=self._fanout_loop, args=(topic, st), daemon=True)
            st.thread.start()

        def _unsub() -> None:
            try:
                st.subs.remove(callback)
            except ValueError:
                pass
            if not st.subs and st.thread:
                st.stop.set()
                st.thread.join(timeout=0.5)
                st.thread = None
                st.stop.clear()

        return _unsub

    # ----- Capacity mgmt ----------------------------------------------------

    def reconfigure(self, topic: str, *, capacity: int) -> dict:  # type: ignore[type-arg]
        """Change payload capacity (bytes) for a topic; returns new descriptor."""
        st = self._ensure_topic(topic)
        new_cap = int(capacity)
        new_shape = (new_cap + 20,)  # +20 for header: length(4) + uuid(16)
        desc = st.channel.reconfigure(new_shape, np.uint8)
        st.capacity = new_cap
        st.shape = new_shape
        st.dtype = np.uint8
        st.last_seq = -1
        return desc  # type: ignore[no-any-return]

    # ----- Internals --------------------------------------------------------

    def _ensure_topic(self, topic: str) -> _TopicState:
        with self._lock:
            st = self._topics.get(topic)
            if st is not None:
                return st
            cap = int(self.config.default_capacity)

            def _names_for_topic(topic: str, capacity: int) -> tuple[str, str]:
                # Python's SharedMemory requires names without a leading '/'
                # Use shorter digest to avoid macOS shared memory name length limits
                h = hashlib.blake2b(f"{topic}:{capacity}".encode(), digest_size=8).hexdigest()
                return f"psm_{h}_data", f"psm_{h}_ctrl"

            data_name, ctrl_name = _names_for_topic(topic, cap)
            ch = CpuShmChannel((cap + 20,), np.uint8, data_name=data_name, ctrl_name=ctrl_name)
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

            try:
                # Read header: length(4) + uuid(16)
                L = struct.unpack("<I", host[:4].tobytes())[0]

                if L < 16 or L > st.capacity + 16:
                    continue

                # Extract UUID
                message_id = host[4:20].tobytes()

                # Extract actual payload (after removing the 16 bytes for uuid)
                payload_len = L - 16
                if payload_len > 0:
                    payload = host[20 : 20 + payload_len].tobytes()
                else:
                    continue

                # Drop exactly the number of local echoes we created
                cnt = st.suppress_counts.get(message_id, 0)
                if cnt > 0:
                    if cnt == 1:
                        del st.suppress_counts[message_id]
                    else:
                        st.suppress_counts[message_id] = cnt - 1
                    continue  # suppressed

            except Exception:
                continue

            for cb in list(st.subs):
                try:
                    cb(payload, topic)
                except Exception:
                    pass


# --------------------------------------------------------------------------------------
# Encoders + concrete PubSub classes
# --------------------------------------------------------------------------------------


class SharedMemoryBytesEncoderMixin(PubSubEncoderMixin[str, bytes]):
    """Identity encoder for raw bytes."""

    def encode(self, msg: bytes, _: str) -> bytes:
        if isinstance(msg, bytes | bytearray | memoryview):
            return bytes(msg)
        raise TypeError(f"SharedMemory expects bytes-like, got {type(msg)!r}")

    def decode(self, msg: bytes, _: str) -> bytes:
        return msg


class SharedMemory(
    SharedMemoryBytesEncoderMixin,
    SharedMemoryPubSubBase,
):
    """SharedMemory pubsub that transports raw bytes."""

    ...


class PickleSharedMemory(
    PickleEncoderMixin[str, Any],
    SharedMemoryPubSubBase,
):
    """SharedMemory pubsub that transports arbitrary Python objects via pickle."""

    ...
