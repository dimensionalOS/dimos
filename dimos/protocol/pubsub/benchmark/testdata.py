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

from collections.abc import Generator
from contextlib import contextmanager
from typing import Any

import numpy as np

from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.protocol.pubsub.benchmark.type import Case
from dimos.protocol.pubsub.lcmpubsub import LCM, LCMPubSubBase, Topic as LCMTopic
from dimos.protocol.pubsub.memory import Memory
from dimos.protocol.pubsub.shmpubsub import BytesSharedMemory, LCMSharedMemory, PickleSharedMemory


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
    lcm_pubsub = LCM(autoconf=True)
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
    lcm_pubsub = LCMPubSubBase(autoconf=True)
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


try:
    from dimos.protocol.pubsub.redispubsub import Redis

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


from dimos.protocol.pubsub.rospubsub import ROS_AVAILABLE, RawROS, RawROSTopic

if ROS_AVAILABLE:
    from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from sensor_msgs.msg import Image as ROSImage

    @contextmanager
    def ros_best_effort_pubsub_channel() -> Generator[RawROS, None, None]:
        qos = QoSProfile(
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
        qos = QoSProfile(
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
        data = np.frombuffer(make_data_bytes(size), dtype=np.uint8).reshape(-1)
        padded_size = ((len(data) + 2) // 3) * 3
        data = np.pad(data, (0, padded_size - len(data)))
        pixels = len(data) // 3
        height = max(1, int(pixels**0.5))
        width = pixels // height
        data = data[: height * width * 3]

        # Create ROS Image message
        msg = ROSImage()
        msg.height = height
        msg.width = width
        msg.encoding = "rgb8"
        msg.step = width * 3
        msg.data = data.tobytes()

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
