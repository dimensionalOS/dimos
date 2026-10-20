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

import asyncio
from contextlib import ExitStack
import json
from threading import Event

import numpy as np
import pytest

from dimos.core.cdr_transport import CdrTransport
from dimos.mapping.odometry_hist import OdometryHist, path_at_true_height
from dimos.messages.geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    PoseWithCovariance,
    Quaternion,
    Twist,
    TwistWithCovariance,
    Vector3,
)
from dimos.messages.nav_msgs.msg import Odometry, Path
from dimos.messages.std_msgs.msg import Header
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped as LegacyPoseStamped
from dimos.msgs.nav_msgs.Path import Path as LegacyPath
from dimos.msgs.time import time_from_nanoseconds, to_nanoseconds
from dimos.protocol.rpc.pubsubrpc import LCMRPC
from dimos.visualization.rerun.cdr import generated_archetypes, path_archetype


class MemoryRPC(LCMRPC):
    """Use real RPC and lifecycle code without an external network."""

    def __init__(self, **kwargs):
        super().__init__(url="memq://", **kwargs)


def odometry(ns, x):
    return Odometry(
        header=Header(frame_id="odom", stamp=time_from_nanoseconds(ns)),
        child_frame_id="base_link",
        pose=PoseWithCovariance(
            pose=Pose(position=Point(x=x, y=2, z=3), orientation=Quaternion(w=1, x=0, y=0, z=0)),
            covariance=np.zeros(36, dtype=np.float64),
        ),
        twist=TwistWithCovariance(
            twist=Twist(linear=Vector3(x=0, y=0, z=0), angular=Vector3(x=0, y=0, z=0)),
            covariance=np.zeros(36, dtype=np.float64),
        ),
    )


def _send(module, message):
    assert module._loop is not None
    asyncio.run_coroutine_threadsafe(module.handle_odometry(message), module._loop).result(
        timeout=5
    )


@pytest.fixture
def history_factory():
    with ExitStack() as cleanup:

        def create(**kwargs):
            wire = CdrTransport("odometry_history", Path, backend="lcm", url="memq://")
            cleanup.callback(wire.stop)
            module = OdometryHist(rpc_transport=MemoryRPC, **kwargs)
            cleanup.callback(module.stop)
            module.odom_hist.transport = wire
            ready = Event()
            received = []

            def receive(value):
                received.append(value)
                ready.set()

            cleanup.callback(wire.subscribe(receive))
            return module, received, ready

        yield create, cleanup


@pytest.mark.parametrize("base", [0, 1700000000123456789])
def test_interval_queue_pose_stamps_replay_restart_and_bound_are_preserved(history_factory, base):
    create, _ = history_factory
    module, received, ready = create(max_poses=3)

    _send(module, odometry(base, 0))
    assert ready.wait(5)
    assert to_nanoseconds(received[-1].header.stamp) == base
    ready.clear()
    _send(module, odometry(base + 50000000, 0.1))
    assert len(received) == 1
    _send(module, odometry(base + 100000000, 0.1))
    assert ready.wait(5)
    assert len(received) == 2
    assert to_nanoseconds(received[-1].header.stamp) == base + 100000000
    assert [to_nanoseconds(value.header.stamp) for value in received[-1].poses] == [
        base,
        base + 50000000,
    ]
    ready.clear()
    _send(module, odometry(base + 100000001, 0.2))
    assert len(received) == 2
    _send(module, odometry(base - 1, 0.3))
    assert ready.wait(5)
    assert len(received) == 3
    assert len(received[-1].poses) == 3
    assert to_nanoseconds(received[-1].header.stamp) == base - 1
    assert [to_nanoseconds(value.header.stamp) for value in received[-1].poses] == [
        base + 50000000,
        base + 100000001,
        base - 1,
    ]


@pytest.mark.parametrize("override,expected", [("", "odom"), ("map", "map")])
def test_snapshot_ownership_step_threshold_and_frame_override(history_factory, override, expected):
    create, _ = history_factory
    module, received, ready = create(frame_id=override, min_publish_interval_seconds=0)
    original = odometry(1700000000123456789, 0)

    _send(module, original)
    assert ready.wait(5)
    first = received[-1]
    first.poses[0].pose.position.x = 99
    original.pose.pose.position.x = 88
    ready.clear()
    _send(module, odometry(1700000000123456790, 0.01))
    assert len(received) == 1
    _send(module, odometry(1700000000123456791, 0.1))
    assert ready.wait(5)

    assert len(received) == 2
    assert [pose.pose.position.x for pose in received[-1].poses] == [0, 0.1]
    assert received[-1].header.frame_id == expected
    assert [pose.header.frame_id for pose in received[-1].poses] == [expected, expected]
    assert to_nanoseconds(received[-1].header.stamp) == 1700000000123456791


def test_real_input_port_dispatches_generated_odometry_into_cdr_path(history_factory):
    create, cleanup = history_factory
    incoming = CdrTransport("history_input", Odometry, backend="lcm", url="memq://")
    cleanup.callback(incoming.stop)
    module, received, ready = create()
    module.odometry.transport = incoming
    assert module.odometry.type is Odometry
    assert module.odom_hist.type is Path
    module.start()
    source = odometry(1700000000123456789, 1)

    incoming.broadcast(None, source)

    assert ready.wait(5)
    assert len(received) == 1
    assert received[0].header == source.header
    assert received[0].poses[0].pose == source.pose.pose


def test_generated_path_rendering_retains_geometry_frame_and_exact_fields():
    header = Header(frame_id="map", stamp=time_from_nanoseconds(1700000000123456789))
    path = Path(
        header=header,
        poses=[
            PoseStamped(
                header=header,
                pose=Pose(
                    position=Point(x=1, y=2, z=3), orientation=Quaternion(w=1, x=0, y=0, z=0)
                ),
            )
        ],
    )

    entries = generated_archetypes(path, "world/path")

    assert [name for name, _ in entries] == ["world/path", "world/path", "world/path/message"]
    line, frame, text = [value for _, value in entries]
    assert line.strips.as_arrow_array().to_pylist() == [[[1.0, 2.0, 3.5]]]
    assert frame.parent_frame.as_arrow_array().to_pylist() == ["tf#/map"]
    fields = json.loads(text.text.as_arrow_array().to_pylist()[0])
    assert fields["header"]["stamp"]["nanosec"] == 123456789
    assert fields["poses"][0]["pose"]["position"] == {"x": 1, "y": 2, "z": 3}
    assert path_at_true_height(path).strips.as_arrow_array().to_pylist() == [[[1.0, 2.0, 3.0]]]
    legacy = LegacyPath(
        ts=0,
        frame_id="map",
        poses=[
            LegacyPoseStamped(position=[1, 2, 3], orientation=[0, 0, 0, 1], ts=0, frame_id="map")
        ],
    )
    assert path_at_true_height(legacy).strips.as_arrow_array().to_pylist() == [[[1.0, 2.0, 3.0]]]
    assert path_archetype(Path(header=header, poses=[])).strips.as_arrow_array().to_pylist() == []
