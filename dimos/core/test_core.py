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

from pathlib import Path
import runpy
import time

import pytest
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import IO, In, Out
from dimos.core.testing import MockRobotClient
from dimos.core.transport import LCMTransport, pLCMTransport
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.encode.cbor import decode, encode
from dimos.robot.unitree.type.odometry import Odometry


class Navigation(Module):
    mov: Out[Vector3]
    lidar: In[PointCloud2]
    target_position: In[Vector3]
    odometry: In[Odometry]

    odom_msg_count = 0
    lidar_msg_count = 0

    @rpc
    def navigate_to(self, target: Vector3) -> bool: ...

    @rpc
    def start(self) -> None:
        def _odom(msg) -> None:
            self.odom_msg_count += 1
            print("RCV:", msg)
            self.mov.publish(msg.position)

        unsub = self.odometry.subscribe(_odom)
        self.register_disposable(Disposable(unsub))

        def _lidar(msg) -> None:
            self.lidar_msg_count += 1
            print("RCV:", msg)

        unsub = self.lidar.subscribe(_lidar)
        self.register_disposable(Disposable(unsub))


@pytest.fixture
def navigation():
    nav = Navigation()
    try:
        yield nav
    finally:
        nav.stop()


@pytest.mark.parametrize("port", [In, Out, IO])
def test_stream_declaration_requires_a_message_type(port: type) -> None:
    with pytest.raises(TypeError, match=rf"Broken\.frames.*{port.__name__}\["):
        type("Broken", (Module,), {"__annotations__": {"frames": port}})


@pytest.mark.parametrize("reference", ['"Output"', "ports.Out"])
def test_bare_stream_with_an_unresolved_peer(tmp_path: Path, reference: str) -> None:
    source = tmp_path / "reader.py"
    source.write_text(
        "from __future__ import annotations\n"
        "from dimos.core import stream as ports\n"
        "from dimos.core.module import Module\n"
        "from dimos.core.stream import Out as Output\n"
        "class Broken(Module):\n"
        "    peer: Later\n"
        f"    frames: {reference}\n"
        "class Later: pass\n"
    )
    with pytest.raises(TypeError, match=r"Broken\.frames needs a message type: use Out\["):
        runpy.run_path(str(source))


@pytest.mark.parametrize("peer", ["int", "DefinedLater"])
def test_class_local_bare_stream_shadows_global(monkeypatch: pytest.MonkeyPatch, peer: str) -> None:
    monkeypatch.setitem(globals(), "Port", int)
    with pytest.raises(TypeError, match=r"Broken\.frames.*Out\["):

        class Broken(Module):
            Port = Out
            frames: "Port"
            other: peer


class DelayedWriter(Module):
    Output = Out
    peer: "DelayedPeer"
    frames: "Output[int]"


class DelayedPeer:
    pass


def test_typed_output_with_a_delayed_peer() -> None:
    (port,) = DelayedWriter.blueprint().blueprints[0].streams
    assert (port.name, port.direction, port.type) == ("frames", "out", int)

    writer = DelayedWriter()
    try:
        assert isinstance(writer.frames, Out) and writer.frames.type is int
    finally:
        writer.stop()


def test_classmethods(navigation) -> None:
    # Test class property access
    class_rpcs = Navigation.rpcs
    # Test instance property access
    instance_rpcs = navigation.rpcs

    # Assertions
    assert isinstance(class_rpcs, dict), "Class rpcs should be a dictionary"
    assert isinstance(instance_rpcs, dict), "Instance rpcs should be a dictionary"
    assert class_rpcs == instance_rpcs, "Class and instance rpcs should be identical"

    # Check that we have the expected RPC methods
    assert "navigate_to" in class_rpcs, "navigate_to should be in rpcs"
    assert "start" in class_rpcs, "start should be in rpcs"
    assert "get_stream_descriptors" in class_rpcs

    # Check that the values are callable
    assert callable(class_rpcs["navigate_to"]), "navigate_to should be callable"
    assert callable(class_rpcs["start"]), "start should be callable"

    # Check that they have the __rpc__ attribute
    assert hasattr(class_rpcs["navigate_to"], "__rpc__"), (
        "navigate_to should have __rpc__ attribute"
    )
    assert hasattr(class_rpcs["start"], "__rpc__"), "start should have __rpc__ attribute"


def test_rpc_signatures_keep_plain_and_message_types(navigation: Navigation) -> None:
    infos = {info.name: info for info in navigation.get_rpc_signatures()}

    navigate_to = infos["navigate_to"]
    assert [(param.name, param.annotation) for param in navigate_to.params] == [("target", Vector3)]
    assert navigate_to.return_annotation is bool
    assert infos["start"].return_annotation is None
    # Any becomes object.
    assert infos["peek_stream"].return_annotation is object
    # The second hint cannot be resolved, and the first still is.
    assert [param.annotation for param in infos["set_module_ref"].params] == [str, object]
    assert decode(encode(navigate_to)) == navigate_to


@pytest.mark.self_hosted
def test_basic_deployment(dimos) -> None:
    robot = dimos.deploy(MockRobotClient)

    print("\n")
    print("lidar stream", robot.lidar)
    print("odom stream", robot.odometry)

    nav = dimos.deploy(Navigation)

    # this one encodes proper LCM messages
    robot.lidar.transport = LCMTransport("/lidar", PointCloud2)

    # odometry & mov on the untyped (CBOR) LCM transport
    robot.odometry.transport = pLCMTransport("/odom")
    nav.mov.transport = pLCMTransport("/mov")

    nav.lidar.connect(robot.lidar)
    nav.odometry.connect(robot.odometry)
    robot.mov.connect(nav.mov)

    robot.start()
    nav.start()

    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline:
        if robot.mov_msg_count >= 8 and nav.odom_msg_count >= 8 and nav.lidar_msg_count >= 8:
            break
        time.sleep(0.1)

    assert robot.mov_msg_count >= 8
    assert nav.odom_msg_count >= 8
    assert nav.lidar_msg_count >= 8
