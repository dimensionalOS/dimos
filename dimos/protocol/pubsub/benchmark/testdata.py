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

from collections.abc import Callable, Generator
from contextlib import contextmanager
from dataclasses import dataclass
import os
from typing import TYPE_CHECKING, Any

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.protocol.pubsub.benchmark.type import Case
from dimos.protocol.pubsub.impl.zenohpubsub import Topic as ZenohTopic, Zenoh
from dimos.protocol.pubsub.spec import PubSub
from dimos.protocol.service.zenohservice import ZenohSessionPool
from dimos.utils.testing.waiting import wait_until

try:
    import cyclonedds as _cyclonedds  # noqa: F401

    DDS_AVAILABLE = True
except ImportError:
    DDS_AVAILABLE = False
from dimos.protocol.pubsub.impl.lcmpubsub import LCM, LCMPubSubBase, Topic as LCMTopic
from dimos.protocol.pubsub.impl.memory import Memory
from dimos.protocol.pubsub.impl.shmpubsub import (
    BytesSharedMemory,
    LCMSharedMemory,
    PickleSharedMemory,
)


def make_data_bytes(size: int) -> bytes:
    """Generate random bytes of given size."""
    return bytes(i % 256 for i in range(size))


def make_data_image(size: int) -> Image:
    """Generate an RGB Image with approximately `size` bytes of data."""
    raw_data = np.frombuffer(make_data_bytes(size), dtype=np.uint8).reshape(-1)
    # Pad to make it divisible by 3 for RGB
    padded_size = ((len(raw_data) + 2) // 3) * 3
    padded_data = np.pad(raw_data, (0, padded_size - len(raw_data)))
    pixels = len(padded_data) // 3
    # Find reasonable dimensions
    height = max(1, int(pixels**0.5))
    width = pixels // height
    data = padded_data[: height * width * 3].reshape(height, width, 3)
    return Image(data=data, format=ImageFormat.RGB)


testcases: list[Case[Any, Any]] = []


@contextmanager
def lcm_pubsub_channel() -> Generator[LCM, None, None]:
    lcm_pubsub = LCM()
    lcm_pubsub.start()
    yield lcm_pubsub
    lcm_pubsub.stop()


def lcm_msggen(size: int) -> tuple[LCMTopic, Image]:
    topic = LCMTopic(topic="benchmark/lcm", lcm_type=Image)
    return (topic, make_data_image(size))


testcases.append(
    Case(
        pubsub_context=lcm_pubsub_channel,
        msg_gen=lcm_msggen,
    )
)


@contextmanager
def udp_bytes_pubsub_channel() -> Generator[LCMPubSubBase, None, None]:
    """LCM with raw bytes - no encoding overhead."""
    lcm_pubsub = LCMPubSubBase()
    lcm_pubsub.start()
    yield lcm_pubsub
    lcm_pubsub.stop()


def udp_bytes_msggen(size: int) -> tuple[LCMTopic, bytes]:
    """Generate raw bytes for LCM transport benchmark."""
    topic = LCMTopic(topic="benchmark/lcm_raw")
    return (topic, make_data_bytes(size))


testcases.append(
    Case(
        pubsub_context=udp_bytes_pubsub_channel,
        msg_gen=udp_bytes_msggen,
    )
)


@contextmanager
def memory_pubsub_channel() -> Generator[Memory, None, None]:
    """Context manager for Memory PubSub implementation."""
    yield Memory()


def memory_msggen(size: int) -> tuple[str, Any]:
    return ("benchmark/memory", make_data_image(size))


# testcases.append(
#     Case(
#         pubsub_context=memory_pubsub_channel,
#         msg_gen=memory_msggen,
#     )
# )


@contextmanager
def shm_pickle_pubsub_channel() -> Generator[PickleSharedMemory, None, None]:
    # 12MB capacity to handle benchmark sizes up to 10MB
    shm_pubsub = PickleSharedMemory(prefer="cpu", default_capacity=12 * 1024 * 1024)
    shm_pubsub.start()
    yield shm_pubsub
    shm_pubsub.stop()


def shm_msggen(size: int) -> tuple[str, Any]:
    """Generate message for SharedMemory pubsub benchmark."""
    return ("benchmark/shm", make_data_image(size))


testcases.append(
    Case(
        pubsub_context=shm_pickle_pubsub_channel,
        msg_gen=shm_msggen,
    )
)


@contextmanager
def shm_bytes_pubsub_channel() -> Generator[BytesSharedMemory, None, None]:
    """SharedMemory with raw bytes - no pickle overhead."""
    shm_pubsub = BytesSharedMemory(prefer="cpu", default_capacity=12 * 1024 * 1024)
    shm_pubsub.start()
    yield shm_pubsub
    shm_pubsub.stop()


def shm_bytes_msggen(size: int) -> tuple[str, bytes]:
    """Generate raw bytes for SharedMemory transport benchmark."""
    return ("benchmark/shm_bytes", make_data_bytes(size))


testcases.append(
    Case(
        pubsub_context=shm_bytes_pubsub_channel,
        msg_gen=shm_bytes_msggen,
    )
)


@contextmanager
def shm_lcm_pubsub_channel() -> Generator[LCMSharedMemory, None, None]:
    """SharedMemory with LCM binary encoding - no pickle overhead."""
    shm_pubsub = LCMSharedMemory(prefer="cpu", default_capacity=12 * 1024 * 1024)
    shm_pubsub.start()
    yield shm_pubsub
    shm_pubsub.stop()


testcases.append(
    Case(
        pubsub_context=shm_lcm_pubsub_channel,
        msg_gen=lcm_msggen,  # Reuse the LCM message generator
    )
)

if DDS_AVAILABLE:
    from cyclonedds.idl import IdlStruct
    from cyclonedds.idl.types import sequence, uint8
    from cyclonedds.qos import Policy, Qos

    from dimos.protocol.pubsub.impl.ddspubsub import (
        DDS,
        Topic as DDSTopic,
    )

    @dataclass
    class DDSBenchmarkData(IdlStruct):  # type: ignore[misc]
        """DDS message type for benchmarking with variable-size byte payload."""

        data: sequence[uint8]  # type: ignore[type-arg]

    @contextmanager
    def dds_high_throughput_pubsub_channel() -> Generator[DDS, None, None]:
        """DDS with high-throughput QoS preset."""
        HIGH_THROUGHPUT_QOS = Qos(
            Policy.Reliability.BestEffort,
            Policy.History.KeepLast(depth=1),
            Policy.Durability.Volatile,
        )
        dds_pubsub = DDS(qos=HIGH_THROUGHPUT_QOS)
        dds_pubsub.start()
        yield dds_pubsub
        dds_pubsub.stop()

    @contextmanager
    def dds_reliable_pubsub_channel() -> Generator[DDS, None, None]:
        """DDS with reliable QoS preset."""
        RELIABLE_QOS = Qos(
            Policy.Reliability.Reliable(max_blocking_time=0),
            Policy.History.KeepLast(depth=5000),
            Policy.Durability.Volatile,
        )
        dds_pubsub = DDS(qos=RELIABLE_QOS)
        dds_pubsub.start()
        yield dds_pubsub
        dds_pubsub.stop()

    def dds_msggen(size: int) -> tuple[DDSTopic, DDSBenchmarkData]:
        """Generate DDS message for benchmark."""
        topic = DDSTopic(name="benchmark/dds", data_type=DDSBenchmarkData)
        return (topic, DDSBenchmarkData(data=list(make_data_bytes(size))))  # type: ignore[arg-type]

    testcases.append(
        Case(
            pubsub_context=dds_high_throughput_pubsub_channel,
            msg_gen=dds_msggen,
        )
    )

    testcases.append(
        Case(
            pubsub_context=dds_reliable_pubsub_channel,
            msg_gen=dds_msggen,
        )
    )


try:
    from dimos.protocol.pubsub.impl.redispubsub import Redis

    @contextmanager
    def redis_pubsub_channel() -> Generator[Redis, None, None]:
        redis_pubsub = Redis()
        redis_pubsub.start()
        yield redis_pubsub
        redis_pubsub.stop()

    def redis_msggen(size: int) -> tuple[str, Any]:
        # Redis uses JSON serialization, so use a simple dict with base64-encoded data
        import base64

        data = base64.b64encode(make_data_bytes(size)).decode("ascii")
        return ("benchmark/redis", {"data": data, "size": size})

    testcases.append(
        Case(
            pubsub_context=redis_pubsub_channel,
            msg_gen=redis_msggen,
        )
    )

except (ConnectionError, ImportError):
    # either redis is not installed or the server is not running
    print("Redis not available")


@contextmanager
def zenoh_pubsub_channel() -> Generator[Zenoh, None, None]:
    """Publisher and subscriber on one shared session (co-located modules).

    This exercises zenoh's intra-session local routing, which is markedly
    slower than the link path for messages >=256KiB; see zenoh_peers below.
    """
    pool = ZenohSessionPool()
    zenoh_pubsub = Zenoh(session_pool=pool)
    zenoh_pubsub.start()
    yield zenoh_pubsub
    zenoh_pubsub.stop()
    pool.close_all()


def zenoh_msggen(size: int) -> tuple[ZenohTopic, Image]:
    return (ZenohTopic("dimos/benchmark/zenoh", Image), make_data_image(size))


testcases.append(
    Case(
        pubsub_context=zenoh_pubsub_channel,
        msg_gen=zenoh_msggen,
    )
)


@dataclass
class _SplitPubSub(PubSub[Any, Any]):
    """Route publish and subscribe to different pubsub instances."""

    pub_side: Zenoh
    sub_side: Zenoh

    def publish(self, topic: Any, msg: Any) -> None:
        self.pub_side.publish(topic, msg)

    def subscribe(self, topic: Any, callback: Any) -> Callable[[], None]:
        return self.sub_side.subscribe(topic, callback)


@contextmanager
def zenoh_peers_pubsub_channel() -> Generator[_SplitPubSub, None, None]:
    """Publisher and subscriber on separate peer sessions (cross-process shape).

    Modules in different workers talk session-to-session over a link, which is
    zenoh's fast path. Sessions live in one process here, but the wire path is
    the same as between processes.
    """
    pub_pool, sub_pool = ZenohSessionPool(), ZenohSessionPool()
    pub_side = Zenoh(session_pool=pub_pool)
    sub_side = Zenoh(session_pool=sub_pool)
    pub_side.start()
    sub_side.start()
    # Multicast scouting takes a moment; publishing before the peers connect
    # would count as loss.
    wait_until(
        lambda: len(pub_side.session.info.peers_zid()) > 0,
        timeout=5.0,
        message="Zenoh peer sessions did not discover each other",
    )
    yield _SplitPubSub(pub_side=pub_side, sub_side=sub_side)
    pub_side.stop()
    sub_side.stop()
    pub_pool.close_all()
    sub_pool.close_all()


testcases.append(
    Case(
        pubsub_context=zenoh_peers_pubsub_channel,
        msg_gen=zenoh_msggen,
    )
)


from dimos.protocol.pubsub.impl.rospubsub import (
    ROS_AVAILABLE,
    DimosROS,
    RawROS,
    RawROSTopic,
    ROSTopic,
)

if TYPE_CHECKING:
    from numpy.typing import NDArray

if ROS_AVAILABLE:
    from rclpy.qos import (
        QoSDurabilityPolicy,
        QoSHistoryPolicy,
        QoSProfile,
        QoSReliabilityPolicy,
    )
    from sensor_msgs.msg import Image as ROSImage

    @contextmanager
    def ros_best_effort_pubsub_channel() -> Generator[RawROS, None, None]:
        qos = QoSProfile(  # type: ignore[no-untyped-call]
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=5000,
        )
        ros_pubsub = RawROS(node_name="benchmark_ros_best_effort", qos=qos)
        ros_pubsub.start()
        yield ros_pubsub
        ros_pubsub.stop()

    @contextmanager
    def ros_reliable_pubsub_channel() -> Generator[RawROS, None, None]:
        qos = QoSProfile(  # type: ignore[no-untyped-call]
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=5000,
        )
        ros_pubsub = RawROS(node_name="benchmark_ros_reliable", qos=qos)
        ros_pubsub.start()
        yield ros_pubsub
        ros_pubsub.stop()

    def ros_msggen(size: int) -> tuple[RawROSTopic, ROSImage]:
        import numpy as np

        # Create image data
        raw_data: NDArray[np.uint8] = np.frombuffer(make_data_bytes(size), dtype=np.uint8)
        padded_size = ((len(raw_data) + 2) // 3) * 3
        padded_data: NDArray[np.uint8] = np.pad(raw_data, (0, padded_size - len(raw_data)))
        pixels = len(padded_data) // 3
        height = max(1, int(pixels**0.5))
        width = pixels // height
        final_data: NDArray[np.uint8] = padded_data[: height * width * 3]

        # Create ROS Image message
        msg = ROSImage()
        msg.height = height
        msg.width = width
        msg.encoding = "rgb8"
        msg.step = width * 3
        msg.data = bytes(final_data)

        topic = RawROSTopic(topic="/benchmark/ros", ros_type=ROSImage)
        return (topic, msg)

    testcases.append(
        Case(
            pubsub_context=ros_best_effort_pubsub_channel,
            msg_gen=ros_msggen,
        )
    )

    testcases.append(
        Case(
            pubsub_context=ros_reliable_pubsub_channel,
            msg_gen=ros_msggen,
        )
    )

    @contextmanager
    def dimos_ros_best_effort_pubsub_channel() -> Generator[DimosROS, None, None]:
        qos = QoSProfile(  # type: ignore[no-untyped-call]
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=5000,
        )
        ros_pubsub = DimosROS(node_name="benchmark_dimos_ros_best_effort", qos=qos)
        ros_pubsub.start()
        yield ros_pubsub
        ros_pubsub.stop()

    @contextmanager
    def dimos_ros_reliable_pubsub_channel() -> Generator[DimosROS, None, None]:
        qos = QoSProfile(  # type: ignore[no-untyped-call]
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=5000,
        )
        ros_pubsub = DimosROS(node_name="benchmark_dimos_ros_reliable", qos=qos)
        ros_pubsub.start()
        yield ros_pubsub
        ros_pubsub.stop()

    def dimos_ros_msggen(size: int) -> tuple[ROSTopic, Image]:
        topic = ROSTopic(topic="/benchmark/dimos_ros", msg_type=Image)
        return (topic, make_data_image(size))

    # commented to save benchmarking time,
    # since reliable and best effort are very similar in performance for local pubsub
    # testcases.append(
    #     Case(
    #         pubsub_context=dimos_ros_best_effort_pubsub_channel,
    #         msg_gen=dimos_ros_msggen,
    #     )
    # )

    testcases.append(
        Case(
            pubsub_context=dimos_ros_reliable_pubsub_channel,
            msg_gen=dimos_ros_msggen,
        )
    )


# WebRTC over the live Cloudflare Realtime SFU. Needs network + credentials,
# so it only registers when CF_TELEOP_APP_ID/SECRET are set (CI skips it).
try:
    from dimos.protocol.pubsub.impl.webrtc.providers.cloudflare import (
        MAX_MSG_SIZE,
        CloudflareConfig,
        CloudflareProvider,
    )
    from dimos.protocol.pubsub.impl.webrtc.providers.spec import WEBRTC_AVAILABLE
    from dimos.protocol.pubsub.impl.webrtc.webrtcpubsub import WebRTCPubSub
except ImportError:
    WEBRTC_AVAILABLE = False

if WEBRTC_AVAILABLE:
    # WebRTC pubsub stack over an in-memory loopback provider: measures the
    # WebRTCPubSub/provider dispatch overhead with no network or SCTP — the
    # keyless upper bound to read the WebrtcCloudflare row against.

    class _LoopbackProvider:
        def __init__(self) -> None:
            self._subs: dict[str, list[Callable[[bytes, str], None]]] = {}
            self._started = False

        @property
        def is_connected(self) -> bool:
            return self._started

        def start(self) -> None:
            self._started = True

        def stop(self) -> None:
            self._started = False

        def publish(self, topic: str, data: bytes) -> None:
            for cb in self._subs.get(topic, ()):
                cb(data, topic)

        def subscribe(
            self, topic: str, callback: Callable[[bytes, str], None]
        ) -> Callable[[], None]:
            self._subs.setdefault(topic, []).append(callback)

            def _unsub() -> None:
                if callback in self._subs.get(topic, ()):
                    self._subs[topic].remove(callback)

            return _unsub

    @contextmanager
    def webrtc_loopback_pubsub_channel() -> Generator[WebRTCPubSub, None, None]:
        pubsub = WebRTCPubSub(provider=_LoopbackProvider())
        pubsub.start()
        try:
            yield pubsub
        finally:
            pubsub.stop()

    def webrtc_loopback_msggen(size: int) -> tuple[str, bytes]:
        return ("benchmark/webrtc_local", make_data_bytes(size))

    testcases.append(
        Case(
            pubsub_context=webrtc_loopback_pubsub_channel,
            msg_gen=webrtc_loopback_msggen,
        )
    )


if WEBRTC_AVAILABLE and os.environ.get("CF_TELEOP_APP_ID"):

    @contextmanager
    def webrtc_cloudflare_pubsub_channel() -> Generator[WebRTCPubSub, None, None]:
        config = CloudflareConfig(
            app_id=os.environ["CF_TELEOP_APP_ID"],
            app_secret=os.environ["CF_TELEOP_APP_SECRET"],
        )
        pubsub = WebRTCPubSub(provider=CloudflareProvider(config))
        pubsub.start()
        try:
            yield pubsub
        finally:
            pubsub.stop()

    def webrtc_msggen(size: int) -> tuple[str, bytes]:
        if size > MAX_MSG_SIZE:
            pytest.skip(f"{size}B exceeds the SCTP/CF DataChannel message limit")
        return ("benchmark/webrtc", make_data_bytes(size))

    testcases.append(
        Case(
            pubsub_context=webrtc_cloudflare_pubsub_channel,
            msg_gen=webrtc_msggen,
        )
    )
