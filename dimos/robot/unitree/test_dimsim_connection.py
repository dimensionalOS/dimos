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

from contextlib import ExitStack
import threading
import time
import uuid

import numpy as np
import pytest

from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.transport import LCMTransport
from dimos.core.transport_factory import make_transport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.unitree import dimsim_connection
from dimos.robot.unitree.dimsim_connection import DimSimConnection
from dimos.robot.unitree.go2.connection import GO2Connection


@pytest.fixture
def simulator(mocker):
    return mocker.patch.object(dimsim_connection, "DimSimProcess").return_value


@pytest.mark.parametrize("backend", ["lcm", "zenoh"])
@pytest.mark.parametrize("sensors", [True, False])
def test_module_outputs_and_commands(backend, sensors, simulator, monkeypatch):
    """Use actual GO2Connection outputs, remappings and transport serialization."""
    monkeypatch.setattr(global_config, "transport", backend)
    monkeypatch.setattr(global_config, "robot_ip", None)
    monkeypatch.setattr(global_config, "robot_ips", None)
    monkeypatch.setattr(global_config, "zenoh_connect", "")
    cfg = GlobalConfig(transport=backend, simulation="dimsim", robot_ip="")
    module = GO2Connection(
        g=cfg, camera=sensors, lidar=sensors, frame_id_prefix="test_robot", publish_tf=sensors
    )
    connection = module.connection
    prefix = f"/dt_{uuid.uuid4().hex[:8]}"
    got, wire_commands = {}, []
    changed = threading.Condition()

    def collect(name, msg):
        with changed:
            got[name] = msg
            changed.notify_all()

    def collect_command(msg):
        with changed:
            wire_commands.append(msg)
            changed.notify_all()

    with ExitStack() as stack:
        # Public streams are deliberately remapped away from simulator wire names.
        for name, kind in [
            ("color_image", Image),
            ("lidar", PointCloud2),
            ("odom", PoseStamped),
            ("tf", TFMessage),
            ("camera_info", CameraInfo),
            ("cmd_vel", Twist),
        ]:
            public = make_transport(f"{prefix}/{name}", kind, g=cfg)
            public.start()
            stack.callback(public.stop)
            module.set_transport(name, public)
            if name != "cmd_vel":
                stack.callback(public.subscribe(lambda msg, name=name: collect(name, msg)))
        messages = {
            "/color_image": Image.from_numpy(
                np.full((8, 8, 3), 30, dtype=np.uint8), ts=42, frame_id="camera_optical"
            ),
            "/lidar": PointCloud2.from_numpy(
                np.array([[1, 2, 3]], dtype=np.float32), timestamp=42, frame_id="world"
            ),
            "/odom": PoseStamped(ts=42, frame_id="world", position=[1, 2, 0.5]),
        }
        sources = {}
        for topic, msg in messages.items():
            transport = LCMTransport(topic, type(msg), url=connection.wire_url)
            transport.start()
            stack.callback(transport.stop)
            sources[topic] = transport
        wire = LCMTransport("/cmd_vel", Twist, url=connection.wire_url)
        wire.start()
        stack.callback(wire.stop)
        stack.callback(wire.subscribe(collect_command))
        # Subscriptions must be installed before simulator startup emits frames.
        simulator.start.side_effect = lambda: connection.video_stream().on_next(
            messages["/color_image"]
        )
        stack.callback(module.stop)
        module.start()
        expected = {"odom", "color_image", "lidar", "tf", "camera_info"} if sensors else {"odom"}
        deadline = time.monotonic() + 5
        with changed:
            while time.monotonic() < deadline and not expected <= got.keys():
                for topic, msg in messages.items():
                    sources[topic].publish(msg)
                changed.wait_for(lambda: expected <= got.keys(), timeout=0.05)
            assert expected <= got.keys()
            assert tuple(got["odom"].position) == pytest.approx((1, 2, 0.5))
            assert got["odom"].frame_id == "world"
            if sensors:
                assert got["color_image"].frame_id == "test_robot/camera_optical"
                assert messages["/color_image"].frame_id == "camera_optical"
                assert got["lidar"].frame_id == "world"
                np.testing.assert_array_equal(got["lidar"].points_f32(), [[1, 2, 3]])
                assert got["camera_info"].frame_id == "test_robot/camera_optical"
                assert {t.child_frame_id for t in got["tf"].transforms} == {
                    f"test_robot/{n}"
                    for n in ("base_link", "camera_link", "camera_optical", "lidar_link")
                }
            else:
                assert not changed.wait_for(lambda: len(got) > 1, timeout=0.1)
        module.cmd_vel.transport.publish(Twist(linear=[0.3, 0, 0]))
        with changed:
            assert changed.wait_for(lambda: bool(wire_commands), timeout=3)
            assert wire_commands[0].linear.x == pytest.approx(0.3)
            assert not changed.wait_for(lambda: len(wire_commands) > 1, timeout=0.1)
        module.stop_movement()
        with changed:
            assert changed.wait_for(lambda: len(wire_commands) == 2, timeout=3)
            assert tuple(wire_commands[-1].linear) == (0, 0, 0)
        module.stop()
        assert not connection.move(Twist())
        with changed:
            got.clear()
            sources["/odom"].publish(messages["/odom"])
            assert not changed.wait_for(lambda: "odom" in got, timeout=0.1)
        if sensors:
            assert not module._camera_info_thread.is_alive()


def test_private_connections_are_isolated(simulator):
    first, second = [DimSimConnection(GlobalConfig()) for _ in range(2)]
    assert first.wire_url != second.wire_url
    seen, other = threading.Event(), threading.Event()
    with ExitStack() as stack:
        stack.callback(first.stop)
        stack.callback(second.stop)
        stack.callback(first.odom_stream().subscribe(lambda _: seen.set()).dispose)
        stack.callback(second.odom_stream().subscribe(lambda _: other.set()).dispose)
        first.start()
        second.start()
        source = LCMTransport("/odom", PoseStamped, url=first.wire_url)
        stack.callback(source.stop)
        source.publish(PoseStamped(frame_id="world"))
        assert seen.wait(3)
        assert not other.wait(0.1)


def test_stop_waits_for_startup_then_closes_everything(simulator, mocker):
    mocker.patch.object(dimsim_connection, "LCMTransport")
    entered, release = threading.Event(), threading.Event()

    def starting():
        entered.set()
        assert release.wait(5)

    simulator.start.side_effect = starting
    conn = DimSimConnection(GlobalConfig())
    start = threading.Thread(target=conn.start)
    stop = threading.Thread(target=conn.stop)
    try:
        start.start()
        assert entered.wait(3)
        stop.start()
        release.set()
        start.join(5)
        stop.join(5)
        assert not start.is_alive() and not stop.is_alive()
        simulator.stop.assert_called_once()
        assert not conn.move(Twist())
    finally:
        release.set()
        start.join(5)
        if stop.ident is not None:
            stop.join(5)
        conn.stop()


def test_concurrent_start_stop_and_retry(simulator, mocker):
    transports = []

    def make(*args, **kwargs):
        transport = mocker.Mock()
        transports.append(transport)
        return transport

    mocker.patch.object(dimsim_connection, "LCMTransport", side_effect=make)
    conn = DimSimConnection(GlobalConfig())
    simulator.start.side_effect = RuntimeError("failed")
    with pytest.raises(RuntimeError, match="failed"):
        conn.start()
    assert not conn.move(Twist())
    simulator.start.side_effect = None
    for method in (conn.start, conn.stop):
        threads = [threading.Thread(target=method) for _ in range(4)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(5)
            assert not thread.is_alive()
    assert simulator.start.call_count == simulator.stop.call_count == 2
    for transport in transports:
        assert transport.start.call_count == transport.stop.call_count == 2
        if transport.subscribe.called:
            assert transport.subscribe.return_value.call_count == 2


def test_non_world_odometry_rejected(simulator):
    module = GO2Connection(g=GlobalConfig(simulation="dimsim", robot_ip=""), odom_frame_id="other")
    try:
        with pytest.raises(ValueError, match="world-registered"):
            module.start()
        simulator.start.assert_not_called()
    finally:
        module.stop()
