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

import math
import threading
import time

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos_generated.std_msgs.msg import Header
from dimos_generated.tf2_msgs.msg import TFMessage
import pytest

from dimos.core.transport_factory import make_transport
from dimos.memory.store.memory import MemoryStore
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.tf import StreamTF
from dimos.msgs.geometry import quaternion_from_euler
from dimos.msgs.time import time_from_seconds, to_seconds
from dimos.protocol.tf.tf import TF, MultiTBuffer, TBuffer, TFLookup


# from https://foxglove.dev/blog/understanding-ros-transforms
def test_tf_ros_example() -> None:
    tf = TF()

    base_link_to_arm = TransformStamped(
        header=Header(frame_id="base_link", stamp=time_from_seconds(time.time())),
        child_frame_id="arm",
        transform=Transform(
            translation=Vector3(x=1.0, y=-1.0, z=0.0),
            rotation=quaternion_from_euler(0, 0, math.pi / 6),
        ),
    )

    arm_to_end = TransformStamped(
        header=Header(frame_id="arm", stamp=time_from_seconds(time.time())),
        child_frame_id="end_effector",
        transform=Transform(
            translation=Vector3(x=1.0, y=1.0, z=0.0),
            rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )

    tf.publish(base_link_to_arm, arm_to_end)
    time.sleep(0.2)

    end_effector_global_pose = tf.get("base_link", "end_effector")
    assert end_effector_global_pose is not None

    assert end_effector_global_pose.transform.translation.x == pytest.approx(1.366, abs=1e-3)
    assert end_effector_global_pose.transform.translation.y == pytest.approx(0.366, abs=1e-3)

    tf.dispose()


def test_tf_main() -> None:
    """Test TF broadcasting and querying between two TF instances.
    If you run rerun-bridge this will show up in the UI"""

    # here we create broadcasting and receiving TF instance.
    # this is to verify that comms work multiprocess, normally
    # you'd use only one instance in your module
    broadcaster_transport = make_transport("/tf", TFMessage)
    querier_transport = make_transport("/tf", TFMessage)
    broadcaster = TF(broadcaster_transport)
    querier = TF(querier_transport)

    # Create a transform from world to robot
    current_time = time.time()

    world_to_charger = TransformStamped(
        header=Header(frame_id="world", stamp=time_from_seconds(current_time)),
        child_frame_id="charger",
        transform=Transform(
            translation=Vector3(x=2.0, y=-2.0, z=0.0), rotation=quaternion_from_euler(0, 0, 2)
        ),
    )

    world_to_robot = TransformStamped(
        header=Header(frame_id="world", stamp=time_from_seconds(current_time)),
        child_frame_id="robot",
        transform=Transform(
            translation=Vector3(x=1.0, y=2.0, z=3.0),
            rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )

    # Broadcast the transform
    broadcaster.publish(world_to_robot)
    broadcaster.publish(world_to_charger)
    # Give time for the message to propagate
    time.sleep(0.05)

    # Verify frames are available
    frames = querier.get_frames()
    assert "world" in frames
    assert "robot" in frames

    # Add another transform in the chain
    robot_to_sensor = TransformStamped(
        header=Header(frame_id="robot", stamp=time_from_seconds(current_time)),
        child_frame_id="sensor",
        transform=Transform(
            translation=Vector3(x=0.5, y=0.0, z=0.2),
            rotation=Quaternion(x=0.0, y=0.0, z=0.707107, w=0.707107),
        ),
    )

    broadcaster.publish(robot_to_sensor)

    time.sleep(0.05)

    # we can now query (from a separate process given we use querier) the transform tree
    chain_transform = querier.get("world", "sensor")

    # broadcaster will agree with us
    assert broadcaster.get("world", "sensor") == chain_transform

    # The chain should compose: world->robot (1,2,3) + robot->sensor (0.5,0,0.2)
    # Expected translation: (1.5, 2.0, 3.2)
    assert chain_transform is not None
    assert abs(chain_transform.transform.translation.x - 1.5) < 0.001
    assert abs(chain_transform.transform.translation.y - 2.0) < 0.001
    assert abs(chain_transform.transform.translation.z - 3.2) < 0.001

    # A perceived object is expressed as a stamped sensor -> object transform.
    random_t = TransformStamped(
        header=Header(frame_id="sensor", stamp=time_from_seconds(time.time())),
        child_frame_id="random_object",
        transform=Transform(translation=Vector3(x=1, y=0, z=0)),
    )

    # we broadcast our object location
    broadcaster.publish(random_t)

    ## we could also publish world -> random_object if we wanted to
    # broadcaster.publish(
    #    broadcaster.get("world", "sensor") + random_object_in_view.new_transform("sensor").inverse()
    # )
    ## (this would mess with the transform system because it expects trees not graphs)
    ## and our random_object would get re-connected to world from sensor

    print(broadcaster)

    # Give time for the message to propagate
    time.sleep(0.05)

    # we know where the object is in the world frame now
    world_object = broadcaster.get("world", "random_object")

    # both instances agree
    assert querier.get("world", "random_object") == world_object

    print("world object", world_object)

    # if you have "diagon" https://diagon.arthursonzogni.com/ installed you can draw a graph
    print(broadcaster.graph())

    assert world_object is not None
    assert abs(world_object.transform.translation.x - 1.5) < 0.001
    assert abs(world_object.transform.translation.y - 3.0) < 0.001
    assert abs(world_object.transform.translation.z - 3.2) < 0.001

    # this doesn't work atm
    robot_to_charger = broadcaster.get("robot", "charger")
    assert robot_to_charger is not None

    # Expected: robot->world->charger
    print(f"robot_to_charger translation: {robot_to_charger.transform.translation}")
    print(f"robot_to_charger rotation: {robot_to_charger.transform.rotation}")

    assert abs(robot_to_charger.transform.translation.x - 1.0) < 0.001
    assert abs(robot_to_charger.transform.translation.y - (-4.0)) < 0.001
    assert abs(robot_to_charger.transform.translation.z - (-3.0)) < 0.001

    # Stop services (they were autostarted but don't know how to autostop)
    broadcaster.dispose()
    querier.dispose()
    broadcaster_transport.stop()
    querier_transport.stop()


class TestTBuffer:
    def test_add_transform(self) -> None:
        buffer = TBuffer(buffer_size=10.0)
        transform = TransformStamped(
            header=Header(frame_id="world", stamp=time_from_seconds(time.time())),
            child_frame_id="robot",
            transform=Transform(
                translation=Vector3(x=1.0, y=2.0, z=3.0),
                rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        )

        buffer.add(transform)
        assert len(buffer) == 1
        assert buffer.first() == transform

    def test_get(self) -> None:
        buffer = TBuffer()
        base_time = time.time()

        # Add transforms at different times
        for i in range(3):
            transform = TransformStamped(
                header=Header(frame_id="world", stamp=time_from_seconds(base_time + i * 0.5)),
                child_frame_id="robot",
                transform=Transform(translation=Vector3(x=float(i), y=0.0, z=0.0)),
            )
            buffer.add(transform)

        # Test getting latest transform
        latest = buffer.get()
        assert latest is not None
        assert latest.transform.translation.x == 2.0

        # Test getting transform at specific time
        middle = buffer.get(time_point=base_time + 0.75)
        assert middle is not None
        assert middle.transform.translation.x == 2.0  # Closest to i=1

        # Test time tolerance
        result = buffer.get(time_point=base_time + 10.0, time_tolerance=0.1)
        assert result is None  # Outside tolerance

    def test_buffer_pruning(self) -> None:
        buffer = TBuffer(buffer_size=1.0)  # 1 second buffer

        # Add old transform
        old_time = time.time() - 2.0
        old_transform = TransformStamped(
            header=Header(frame_id="world", stamp=time_from_seconds(old_time)),
            child_frame_id="robot",
            transform=Transform(translation=Vector3(x=1.0, y=0.0, z=0.0)),
        )
        buffer.add(old_transform)

        # Add recent transform
        recent_transform = TransformStamped(
            header=Header(frame_id="world", stamp=time_from_seconds(time.time())),
            child_frame_id="robot",
            transform=Transform(translation=Vector3(x=2.0, y=0.0, z=0.0)),
        )
        buffer.add(recent_transform)

        # Old transform should be pruned
        assert len(buffer) == 1
        first = buffer.first()
        assert first is not None
        assert first.transform.translation.x == 2.0


class TestMultiTBuffer:
    def test_multiple_frame_pairs(self) -> None:
        ttbuffer = MultiTBuffer(buffer_size=10.0)

        # Add transforms for different frame pairs
        transform1 = TransformStamped(
            header=Header(frame_id="world", stamp=time_from_seconds(time.time())),
            child_frame_id="robot1",
            transform=Transform(translation=Vector3(x=1.0, y=0.0, z=0.0)),
        )

        transform2 = TransformStamped(
            header=Header(frame_id="world", stamp=time_from_seconds(time.time())),
            child_frame_id="robot2",
            transform=Transform(translation=Vector3(x=2.0, y=0.0, z=0.0)),
        )

        ttbuffer.receive_transform(transform1, transform2)

        # Should have two separate buffers
        assert len(ttbuffer.buffers) == 2
        assert ("world", "robot1") in ttbuffer.buffers
        assert ("world", "robot2") in ttbuffer.buffers

    def test_graph(self) -> None:
        ttbuffer = MultiTBuffer(buffer_size=10.0)

        # Add transforms for different frame pairs
        transform1 = TransformStamped(
            header=Header(frame_id="world", stamp=time_from_seconds(time.time())),
            child_frame_id="robot1",
            transform=Transform(translation=Vector3(x=1.0, y=0.0, z=0.0)),
        )

        transform2 = TransformStamped(
            header=Header(frame_id="world", stamp=time_from_seconds(time.time())),
            child_frame_id="robot2",
            transform=Transform(translation=Vector3(x=2.0, y=0.0, z=0.0)),
        )

        ttbuffer.receive_transform(transform1, transform2)

        print(ttbuffer.graph())

    def test_forward_tolerance_returns_when_buffer_fills(self) -> None:
        ttbuffer = MultiTBuffer()
        base_time = time.time()

        def publish_after_delay() -> None:
            time.sleep(0.05)
            ttbuffer.receive_transform(
                TransformStamped(
                    header=Header(frame_id="world", stamp=time_from_seconds(base_time)),
                    child_frame_id="robot",
                    transform=Transform(translation=Vector3(x=1.0, y=0.0, z=0.0)),
                )
            )

        publisher = threading.Thread(target=publish_after_delay)
        publisher.start()

        t0 = time.monotonic()
        result = ttbuffer.get(
            "world", "robot", time_point=base_time, time_tolerance=0.1, forward_tolerance=1.0
        )
        elapsed = time.monotonic() - t0
        publisher.join()

        assert result is not None
        assert result.transform.translation.x == 1.0
        assert elapsed < 0.5

    def test_forward_tolerance_times_out(self) -> None:
        ttbuffer = MultiTBuffer()
        t0 = time.monotonic()
        result = ttbuffer.get("world", "robot", time_point=time.time(), forward_tolerance=0.1)
        elapsed = time.monotonic() - t0
        assert result is None
        assert 0.08 < elapsed < 1.0

    def test_forward_tolerance_fast_path_when_already_available(self) -> None:
        ttbuffer = MultiTBuffer()
        base_time = time.time()
        ttbuffer.receive_transform(
            TransformStamped(
                header=Header(frame_id="world", stamp=time_from_seconds(base_time)),
                child_frame_id="robot",
                transform=Transform(translation=Vector3(x=2.0, y=0.0, z=0.0)),
            )
        )

        t0 = time.monotonic()
        result = ttbuffer.get("world", "robot", time_point=base_time, forward_tolerance=10.0)
        elapsed = time.monotonic() - t0

        assert result is not None
        assert result.transform.translation.x == 2.0
        assert elapsed < 0.05

    def test_forward_tolerance_wakes_on_chain_completion(self) -> None:
        ttbuffer = MultiTBuffer()
        base_time = time.time()

        ttbuffer.receive_transform(
            TransformStamped(
                header=Header(frame_id="world", stamp=time_from_seconds(base_time)),
                child_frame_id="robot",
                transform=Transform(translation=Vector3(x=1.0, y=0.0, z=0.0)),
            )
        )

        def publish_after_delay() -> None:
            time.sleep(0.05)
            ttbuffer.receive_transform(
                TransformStamped(
                    header=Header(frame_id="robot", stamp=time_from_seconds(base_time)),
                    child_frame_id="sensor",
                    transform=Transform(translation=Vector3(x=0.0, y=2.0, z=0.0)),
                )
            )

        publisher = threading.Thread(target=publish_after_delay)
        publisher.start()

        result = ttbuffer.get(
            "world", "sensor", time_point=base_time, time_tolerance=0.1, forward_tolerance=1.0
        )
        publisher.join()

        assert result is not None
        assert result.transform.translation.x == 1.0
        assert result.transform.translation.y == 2.0

    def test_get_transform_search_direct(self) -> None:
        ttbuffer = MultiTBuffer()
        base_time = time.time()

        # Add direct transform
        transform = TransformStamped(
            header=Header(frame_id="world", stamp=time_from_seconds(base_time)),
            child_frame_id="robot",
            transform=Transform(translation=Vector3(x=1.0, y=0.0, z=0.0)),
        )
        ttbuffer.receive_transform(transform)

        # Search should return single transform
        result = ttbuffer.get_transform_search("world", "robot")
        assert result is not None
        assert len(result) == 1
        assert result[0].transform.translation.x == 1.0

    def test_get_transform_search_chain(self) -> None:
        ttbuffer = MultiTBuffer()
        base_time = time.time()

        # Create transform chain: world -> robot -> sensor
        transform1 = TransformStamped(
            header=Header(frame_id="world", stamp=time_from_seconds(base_time)),
            child_frame_id="robot",
            transform=Transform(translation=Vector3(x=1.0, y=0.0, z=0.0)),
        )
        transform2 = TransformStamped(
            header=Header(frame_id="robot", stamp=time_from_seconds(base_time)),
            child_frame_id="sensor",
            transform=Transform(translation=Vector3(x=0.0, y=2.0, z=0.0)),
        )
        ttbuffer.receive_transform(transform1, transform2)

        # Search should find chain
        result = ttbuffer.get_transform_search("world", "sensor")
        assert result is not None
        assert len(result) == 2
        assert result[0].transform.translation.x == 1.0  # world -> robot
        assert result[1].transform.translation.y == 2.0  # robot -> sensor

    def test_get_transform_search_complex_chain(self) -> None:
        ttbuffer = MultiTBuffer()
        base_time = time.time()

        # Create more complex graph:
        # world -> base -> arm -> hand
        #      \-> robot -> sensor
        transforms = [
            TransformStamped(
                header=Header(frame_id="world", stamp=time_from_seconds(base_time)),
                child_frame_id="base",
                transform=Transform(translation=Vector3(x=1.0, y=0.0, z=0.0)),
            ),
            TransformStamped(
                header=Header(frame_id="base", stamp=time_from_seconds(base_time)),
                child_frame_id="arm",
                transform=Transform(translation=Vector3(x=0.0, y=1.0, z=0.0)),
            ),
            TransformStamped(
                header=Header(frame_id="arm", stamp=time_from_seconds(base_time)),
                child_frame_id="hand",
                transform=Transform(translation=Vector3(x=0.0, y=0.0, z=1.0)),
            ),
            TransformStamped(
                header=Header(frame_id="world", stamp=time_from_seconds(base_time)),
                child_frame_id="robot",
                transform=Transform(translation=Vector3(x=2.0, y=0.0, z=0.0)),
            ),
            TransformStamped(
                header=Header(frame_id="robot", stamp=time_from_seconds(base_time)),
                child_frame_id="sensor",
                transform=Transform(translation=Vector3(x=0.0, y=2.0, z=0.0)),
            ),
        ]

        for t in transforms:
            ttbuffer.receive_transform(t)

        # Find path world -> hand (should go through base -> arm)
        result = ttbuffer.get_transform_search("world", "hand")
        assert result is not None
        assert len(result) == 3
        assert result[0].child_frame_id == "base"
        assert result[1].child_frame_id == "arm"
        assert result[2].child_frame_id == "hand"

    def test_get_transform_search_no_path(self) -> None:
        ttbuffer = MultiTBuffer()
        base_time = time.time()

        # Create disconnected transforms
        transform1 = TransformStamped(
            header=Header(frame_id="world", stamp=time_from_seconds(base_time)),
            child_frame_id="robot",
            transform=Transform(),
        )
        transform2 = TransformStamped(
            header=Header(frame_id="base", stamp=time_from_seconds(base_time)),
            child_frame_id="sensor",
            transform=Transform(),
        )
        ttbuffer.receive_transform(transform1, transform2)

        # No path exists
        result = ttbuffer.get_transform_search("world", "sensor")
        assert result is None

    def test_get_transform_search_with_time(self) -> None:
        ttbuffer = MultiTBuffer()
        base_time = time.time()

        # Add transforms at different times
        old_transform = TransformStamped(
            header=Header(frame_id="world", stamp=time_from_seconds(base_time - 10.0)),
            child_frame_id="robot",
            transform=Transform(translation=Vector3(x=1.0, y=0.0, z=0.0)),
        )
        new_transform = TransformStamped(
            header=Header(frame_id="world", stamp=time_from_seconds(base_time)),
            child_frame_id="robot",
            transform=Transform(translation=Vector3(x=2.0, y=0.0, z=0.0)),
        )
        ttbuffer.receive_transform(old_transform, new_transform)

        # Search at specific time
        result = ttbuffer.get_transform_search("world", "robot", time_point=base_time)
        assert result is not None
        assert result[0].transform.translation.x == 2.0

        # Search with time tolerance
        result = ttbuffer.get_transform_search(
            "world", "robot", time_point=base_time + 1.0, time_tolerance=0.1
        )
        assert result is None  # Outside tolerance

    def test_get_transform_search_shortest_path(self) -> None:
        ttbuffer = MultiTBuffer()
        base_time = time.time()

        # Create graph with multiple paths:
        # world -> A -> B -> target (3 hops)
        # world -> target (direct, 1 hop)
        transforms = [
            TransformStamped(
                header=Header(frame_id="world", stamp=time_from_seconds(base_time)),
                child_frame_id="A",
                transform=Transform(),
            ),
            TransformStamped(
                header=Header(frame_id="A", stamp=time_from_seconds(base_time)),
                child_frame_id="B",
                transform=Transform(),
            ),
            TransformStamped(
                header=Header(frame_id="B", stamp=time_from_seconds(base_time)),
                child_frame_id="target",
                transform=Transform(),
            ),
            TransformStamped(
                header=Header(frame_id="world", stamp=time_from_seconds(base_time)),
                child_frame_id="target",
                transform=Transform(),
            ),
        ]

        for t in transforms:
            ttbuffer.receive_transform(t)

        # BFS should find the direct path (shortest)
        result = ttbuffer.get_transform_search("world", "target")
        assert result is not None
        assert len(result) == 1  # Direct path, not the 3-hop path
        assert result[0].child_frame_id == "target"

    def test_string_representations(self) -> None:
        # Test empty buffers
        empty_buffer = TBuffer()
        assert str(empty_buffer) == "TBuffer(empty)"

        empty_ttbuffer = MultiTBuffer()
        assert str(empty_ttbuffer) == "MultiTBuffer(empty)"

        # Test TBuffer with data
        buffer = TBuffer()
        base_time = time.time()
        for i in range(3):
            transform = TransformStamped(
                header=Header(frame_id="world", stamp=time_from_seconds(base_time + i * 0.1)),
                child_frame_id="robot",
                transform=Transform(translation=Vector3(x=float(i), y=0.0, z=0.0)),
            )
            buffer.add(transform)

        buffer_str = str(buffer)
        assert "3 msgs" in buffer_str
        assert "world -> robot" in buffer_str
        assert "0.20s" in buffer_str  # duration

        # Test MultiTBuffer with multiple frame pairs
        ttbuffer = MultiTBuffer()
        transforms = [
            TransformStamped(
                header=Header(frame_id="world", stamp=time_from_seconds(base_time)),
                child_frame_id="robot1",
                transform=Transform(),
            ),
            TransformStamped(
                header=Header(frame_id="world", stamp=time_from_seconds(base_time + 0.5)),
                child_frame_id="robot2",
                transform=Transform(),
            ),
            TransformStamped(
                header=Header(frame_id="robot1", stamp=time_from_seconds(base_time + 1.0)),
                child_frame_id="sensor",
                transform=Transform(),
            ),
        ]

        for t in transforms:
            ttbuffer.receive_transform(t)

        ttbuffer_str = str(ttbuffer)
        print("\nMultiTBuffer string representation:")
        print(ttbuffer_str)

        assert "MultiTBuffer(3 buffers):" in ttbuffer_str
        assert "TBuffer(world -> robot1, 1 msgs" in ttbuffer_str
        assert "TBuffer(world -> robot2, 1 msgs" in ttbuffer_str
        assert "TBuffer(robot1 -> sensor, 1 msgs" in ttbuffer_str


# --- Grid tests: every get() scenario below runs against the live service and
# --- against StreamTF replaying a recorded tf stream (memory- and sqlite-backed).

T0 = 1_700_000_000.0


def _t(parent: str, child: str, x: float, ts: float) -> TransformStamped:
    return TransformStamped(
        header=Header(frame_id=parent, stamp=time_from_seconds(ts)),
        child_frame_id=child,
        transform=Transform(translation=Vector3(x=x, y=0.0, z=0.0)),
    )


@pytest.fixture(
    params=[
        "live",
        "stream_memory",
        # sqlite-vec ships a 32-bit binary in the aarch64 wheel and fails to
        # load on macOS CI (same guard as memory/conftest.py).
        pytest.param("stream_sqlite", marks=[pytest.mark.skipif_aarch64, pytest.mark.skipif_macos]),
    ]
)
def make_tf(request, tmp_path):  # type: ignore[no-untyped-def]
    """Builder fixture: feed it transforms, get back a TFLookup over them."""
    stores = []

    def build(*transforms: TransformStamped) -> TFLookup:
        if request.param == "live":
            buf = MultiTBuffer()
            buf.receive_transform(*transforms)
            return buf
        store = (
            MemoryStore()
            if request.param == "stream_memory"
            else SqliteStore(path=str(tmp_path / "tf.db"))
        )
        stores.append(store)
        stream = store.stream("tf", TFMessage)
        for t in transforms:
            stream.append(TFMessage(transforms=[t]), ts=to_seconds(t.header.stamp), pose=None)
        return StreamTF(store.stream("tf", TFMessage))

    yield build
    for store in stores:
        store.stop()


class TestLookupGrid:
    """get() scenarios that must answer identically live and over a recording."""

    def test_latest(self, make_tf) -> None:  # type: ignore[no-untyped-def]
        tf = make_tf(*(_t("world", "robot", float(i), T0 + i * 0.1) for i in range(3)))
        got = tf.get("world", "robot")
        assert got is not None
        assert got.transform.translation.x == 2.0

    def test_nearest_in_time(self, make_tf) -> None:  # type: ignore[no-untyped-def]
        tf = make_tf(*(_t("world", "robot", float(i), T0 + i * 0.5) for i in range(5)))
        got = tf.get("world", "robot", time_point=T0 + 1.25)
        assert got is not None
        # Equidistant between i=2 (t=1.0) and i=3 (t=1.5) — the later one wins.
        assert got.transform.translation.x == 3.0

    def test_inverse(self, make_tf) -> None:  # type: ignore[no-untyped-def]
        tf = make_tf(_t("world", "robot", 5.0, T0))
        got = tf.get("robot", "world", time_point=T0)
        assert got is not None
        assert got.transform.translation.x == pytest.approx(-5.0)

    def test_time_tolerance(self, make_tf) -> None:  # type: ignore[no-untyped-def]
        tf = make_tf(_t("world", "robot", 1.0, T0))
        assert tf.get("world", "robot", time_point=T0 + 0.1, time_tolerance=0.2) is not None
        assert tf.get("world", "robot", time_point=T0 + 0.5, time_tolerance=0.1) is None

    def test_same_frame_identity(self, make_tf) -> None:  # type: ignore[no-untyped-def]
        tf = make_tf(_t("world", "robot", 1.0, T0))
        got = tf.get("world", "world", time_point=T0)
        assert got is not None
        assert got.header.frame_id == "world"
        assert got.child_frame_id == "world"
        assert got.transform.translation.x == 0.0
        assert got.transform.rotation.w == 1.0

    def test_unknown_frame(self, make_tf) -> None:  # type: ignore[no-untyped-def]
        tf = make_tf(_t("world", "robot", 1.0, T0))
        assert tf.get("foo", "bar", time_point=T0) is None

    def test_chain_composition(self, make_tf) -> None:  # type: ignore[no-untyped-def]
        # world -> robot: translate (1, 0, 0); robot -> sensor: translate
        # (0, 2, 0) and rotate 90° around Z.
        tf = make_tf(
            TransformStamped(
                header=Header(frame_id="world", stamp=time_from_seconds(T0)),
                child_frame_id="robot",
                transform=Transform(
                    translation=Vector3(x=1.0, y=0.0, z=0.0),
                    rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
            ),
            TransformStamped(
                header=Header(frame_id="robot", stamp=time_from_seconds(T0)),
                child_frame_id="sensor",
                transform=Transform(
                    translation=Vector3(x=0.0, y=2.0, z=0.0),
                    rotation=Quaternion(
                        x=0.0, y=0.0, z=math.sin(math.pi / 4), w=math.cos(math.pi / 4)
                    ),
                ),
            ),
        )
        result = tf.get("world", "sensor", time_point=T0)
        assert result is not None
        assert result.transform.translation.x == pytest.approx(1.0)
        assert result.transform.translation.y == pytest.approx(2.0)
        assert result.transform.translation.z == pytest.approx(0.0)
        assert result.transform.rotation.z == pytest.approx(math.sin(math.pi / 4))
        assert result.transform.rotation.w == pytest.approx(math.cos(math.pi / 4))
        assert result.header.frame_id == "world"
        assert result.child_frame_id == "sensor"

    def test_chain_with_sparse_static_edge(self, make_tf) -> None:  # type: ignore[no-untyped-def]
        # A static edge published once at startup composes with dynamic data
        # arriving ten seconds later (default lookup reach = buffer_size).
        tf = make_tf(
            _t("world", "map", 100.0, T0),
            *(_t("map", "base", i / 10, T0 + i / 10) for i in range(100)),
        )
        got = tf.get("world", "base", time_point=T0 + 9.0)
        assert got is not None
        assert got.transform.translation.x == pytest.approx(109.0)

    def test_conforms_to_tf_lookup(self, make_tf) -> None:  # type: ignore[no-untyped-def]
        assert isinstance(make_tf(_t("world", "robot", 1.0, T0)), TFLookup)


class TestStreamTF:
    """Replay-specific surface: construction, bounded caching, and lookahead."""

    @pytest.fixture
    def store(self):  # type: ignore[no-untyped-def]
        store = MemoryStore()
        stream = store.stream("tf", TFMessage)
        # Startup burst: a static stamped exactly at the stream start (regression:
        # strict `ts >` range queries used to drop it), then dynamic map→base.
        stream.append(TFMessage(transforms=[_t("world", "map", 100.0, T0)]), ts=T0, pose=None)
        for i in range(100):
            ts = T0 + i / 10
            stream.append(TFMessage(transforms=[_t("map", "base", i / 10, ts)]), ts=ts, pose=None)
        yield store
        store.stop()

    def test_from_store(self, store) -> None:  # type: ignore[no-untyped-def]
        assert StreamTF.from_store(store) is not None
        assert StreamTF.from_store(store, "nope") is None

    def test_missing_stream(self) -> None:
        with pytest.raises(ValueError, match="Stream configuration"):
            StreamTF()

    def test_empty_stream(self) -> None:
        store = MemoryStore()
        store.stream("tf", TFMessage)
        try:
            tf = StreamTF(store.stream("tf", TFMessage))
            assert tf.get("world", "base", time_point=T0) is None
        finally:
            store.stop()

    def test_get_frames(self, store) -> None:  # type: ignore[no-untyped-def]
        tf = StreamTF(store.stream("tf", TFMessage))
        # Nothing is cached until a lookup pulls a window in.
        assert tf.get_frames() == set()
        tf.get("map", "base", time_point=T0 + 5.0)
        assert tf.get_frames() == {"world", "map", "base"}

    def test_read_only(self, store) -> None:  # type: ignore[no-untyped-def]
        tf = StreamTF(store.stream("tf", TFMessage))
        with pytest.raises(NotImplementedError):
            tf.publish(_t("map", "base", 0.0, T0))

    def test_get_pose(self, store) -> None:  # type: ignore[no-untyped-def]
        tf = StreamTF(store.stream("tf", TFMessage))
        pose = tf.get_pose("map", "base", time_point=T0 + 5.0)
        assert pose is not None
        assert pose.pose.position.x == pytest.approx(5.0)

    def test_cache_prefetch_and_eviction(self) -> None:
        # 40 s of data with a small cache_span: a miss caches the query window
        # plus cache_span ahead, follow-ups inside it are pure cache hits, and
        # the first query past it evicts everything and re-caches.
        store = MemoryStore()
        stream = store.stream("tf", TFMessage)
        for i in range(400):
            ts = T0 + i / 10
            stream.append(TFMessage(transforms=[_t("map", "base", i / 10, ts)]), ts=ts, pose=None)
        try:
            tf = StreamTF(store.stream("tf", TFMessage), cache_span=2.0)
            got = tf.get("map", "base", time_point=T0 + 35.0, time_tolerance=0.5)
            assert got is not None
            assert got.transform.translation.x == pytest.approx(35.0)
            covered = tf._covered
            assert covered == pytest.approx((T0 + 34.5, T0 + 37.5))
            # Inside the prefetched span: served from cache, no re-query.
            got = tf.get("map", "base", time_point=T0 + 37.0, time_tolerance=0.5)
            assert got is not None
            assert got.transform.translation.x == pytest.approx(37.0)
            assert tf._covered == covered
            # Past the span: evict and re-cache around the new query.
            got = tf.get("map", "base", time_point=T0 + 39.0, time_tolerance=0.5)
            assert got is not None
            assert got.transform.translation.x == pytest.approx(39.0)
            assert tf._covered == pytest.approx((T0 + 38.5, T0 + 41.5))
            # Bounded: the buffer holds the re-cached span, not the stream.
            assert len(tf.buffers[("map", "base")]) < 100
        finally:
            store.stop()


def test_buffer_retains_distinct_nanoseconds_and_owns_its_snapshots():
    buffer = TBuffer()
    first = TransformStamped(
        header=Header(frame_id="a", stamp=Time(sec=1700000000, nanosec=123456789)),
        child_frame_id="b",
    )
    second = TransformStamped(
        header=Header(frame_id="a", stamp=Time(sec=1700000000, nanosec=123456790)),
        child_frame_id="b",
    )
    buffer.add(first)
    buffer.add(second)
    first.header.stamp.nanosec = 0
    assert len(buffer) == 2
    stored_first = buffer.first()
    assert stored_first is not None
    assert stored_first.header.stamp.nanosec == 123456789
    stored_first.header.stamp.nanosec = 1
    unchanged = buffer.first()
    assert unchanged is not None
    assert unchanged.header.stamp.nanosec == 123456789
    latest = buffer.get()
    assert latest is not None
    assert latest.header.stamp.nanosec == 123456790


def test_buffer_drops_out_of_order_data_older_than_retained_window():
    buffer = TBuffer(buffer_size=1)
    for sec in (10, 11, 1):
        buffer.add(TransformStamped(header=Header(stamp=Time(sec=sec))))
    assert len(buffer) == 2
    first = buffer.first()
    assert first is not None
    assert first.header.stamp.sec == 10


def test_buffer_preserves_negative_stamp():
    buffer = TBuffer()
    buffer.add(TransformStamped(header=Header(stamp=Time(sec=-1, nanosec=500000000))))
    result = buffer.get(time_point=-0.5, time_tolerance=0)
    assert result is not None
    assert result.header.stamp == Time(sec=-1, nanosec=500000000)
