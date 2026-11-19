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

from functools import partial
import signal
import subprocess
import warnings

import numpy as np
import psutil
import pytest
import rerun as rr
from rerun.experimental import RrdReader

from dimos.mapping.ray_tracing.viz import render_map_region
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.pubsub.impl.zenohpubsub import Topic
from dimos.protocol.pubsub.spec import SubscribeAllCapable
from dimos.utils.executable import find_executable
from dimos.utils.testing.waiting import wait_until
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.static_log import StaticLog, memory_limit_bytes


def cloud(points):
    return rr.Points3D(np.zeros((points, 3), dtype=np.float32))


def logged(log):
    return [
        (call.args[0], [type(a).__name__ for a in call.args[1:]]) for call in log.call_args_list
    ]


@pytest.mark.parametrize(
    ("limit", "expected"),
    [("1GiB", 2**30), ("256MB", 256_000_000), ("25%", 2 * 2**30)],
)
def test_memory_limit_bytes_reads_sizes_and_shares_of_ram(mocker, limit, expected):
    mocker.patch.object(psutil, "virtual_memory").return_value.total = 8 * 2**30

    assert memory_limit_bytes(limit) == expected


def test_logs_everything_as_static(mocker):
    log = mocker.patch.object(rr, "log")
    points = cloud(1)

    StaticLog("1GiB").log("world/map", points)

    log.assert_called_once_with("world/map", points, static=True)


def test_replays_what_was_logged_once_after_half_a_buffer_of_static_data(mocker):
    log = mocker.patch.object(rr, "log")
    static_log = StaticLog("100KiB")
    static_log.log("world/lidar", rr.Transform3D(parent_frame="tf#/lidar"), once=True)
    static_log.log("world/robot", rr.Boxes3D(half_sizes=[1, 1, 1]), once=True)
    log.reset_mock()

    # A region counts as 14 kB. Three stay under the 50 KiB half, the fourth passes it.
    for region in range(3):
        static_log.log(f"world/map/{region}", cloud(1000))
    assert len(log.call_args_list) == 3
    static_log.log("world/map/3", cloud(1000))

    assert logged(log)[4:] == [("world/lidar", ["Transform3D"]), ("world/robot", ["Boxes3D"])]
    assert all(call.kwargs == {"static": True} for call in log.call_args_list)


def test_replays_only_the_latest_value_of_an_archetype(mocker):
    log = mocker.patch.object(rr, "log")
    static_log = StaticLog("100KiB")
    old = rr.Transform3D(parent_frame="tf#/old")
    new = rr.Transform3D(parent_frame="tf#/new")
    pinhole = rr.Pinhole(focal_length=1.0, width=2, height=2)
    static_log.log("world/camera", old, once=True)
    static_log.log("world/camera", pinhole, once=True)
    static_log.log("world/camera", new, once=True)
    log.reset_mock()

    static_log.log("world/map", cloud(5000))

    assert log.call_args_list[1] == mocker.call("world/camera", new, pinhole, static=True)


@pytest.fixture
def bridge(request, mocker, unused_tcp_port):
    """A bridge on a real proxy whose buffer is far smaller than what the test logs."""
    previous = rr.get_global_data_recording()
    source = mocker.Mock(spec=SubscribeAllCapable)
    source.subscribe_all.return_value = lambda: None
    uri = f"rerun+http://127.0.0.1:{unused_tcp_port}/proxy"
    module = RerunBridgeModule(
        pubsubs=[source],
        blueprint=None,
        rerun_open="none",
        connect_url=uri,
        server_memory_limit="256KiB",
        static={"world/robot": lambda rerun: rerun.Boxes3D(half_sizes=[0.3, 0.2, 0.1])},
        visual_override={
            "world/map_regions": partial(
                render_map_region, voxel_size=0.1, height_range=(0.0, 1.0), static=request.param
            ),
        },
    )
    try:
        module.start()
        yield module, uri
    finally:
        module.stop()
        rr.disconnect()
        # Without this the next test's rr.log calls pile up in the disconnected recording.
        rr.bindings.set_global_data_recording(previous.to_native() if previous else None)


@pytest.fixture
def late_client(tmp_path):
    """Save what a viewer that connects now is sent. Returns the columns it has so far."""
    clients = []

    def connect(uri):
        path = tmp_path / "late.rrd"
        clients.append(
            subprocess.Popen(
                # It also serves gRPC itself, so keep it off the default port.
                [find_executable("rerun"), "--save", str(path), "--port", "auto", uri],
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        )

        def columns():
            if not path.exists() or path.stat().st_size == 0:
                return set()
            # The file has no footer while the client is still writing it.
            with warnings.catch_warnings():
                warnings.filterwarnings("ignore", message="RRD file has no footer/manifest")
                return set(RrdReader(path).stream().collect().schema().column_names())

        return columns

    yield connect
    for client in clients:
        client.send_signal(signal.SIGINT)
        client.wait(timeout=10)


@pytest.mark.parametrize("bridge", [False, True], indirect=True, ids=["temporal", "static"])
def test_late_viewer_gets_what_the_bridge_logged_once(bridge, late_client):
    module, uri = bridge
    recording = rr.get_global_data_recording()
    rng = np.random.default_rng(42)
    lidar = PointCloud2.from_numpy(np.array([[1, 2, 3]], dtype=np.float32), frame_id="lidar")
    module._on_message(lidar, Topic(topic="dimos/lidar", lcm_type=PointCloud2))

    # 1.2 MB of map regions through a 256 KiB buffer, as timeline or as static data.
    for seq in range(20):
        region = PointCloud2.from_numpy(rng.random((4000, 3), dtype=np.float32), frame_id="odom")
        region.seq = seq
        module._on_message(region, Topic(topic="dimos/map_regions", lcm_type=PointCloud2))
        recording.flush()

    columns = late_client(uri)
    expected = {
        "/world/lidar:Transform3D:parent_frame",
        "/world/robot:Boxes3D:half_sizes",
        "/:AnnotationContext:context",
    }
    try:
        wait_until(lambda: expected <= columns(), timeout=10)
    except TimeoutError:
        pytest.fail(f"The late viewer never got {expected - columns()}")
