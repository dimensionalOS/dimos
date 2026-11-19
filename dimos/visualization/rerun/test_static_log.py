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


@pytest.mark.parametrize(
    ("limit", "expected"),
    [("1GiB", 2**30), ("256MB", 256_000_000), ("25%", 2 * 2**30)],
)
def test_memory_limit_bytes_reads_sizes_and_shares_of_ram(mocker, limit, expected):
    mocker.patch.object(psutil, "virtual_memory").return_value.total = 8 * 2**30

    assert memory_limit_bytes(limit) == expected


@pytest.mark.parametrize("limit", ["0", "0%", "unlimited", "none", "max", "∞"])
def test_disabled_or_unlimited_history_sends_each_update_once(mocker, limit):
    log = mocker.patch.object(rr, "log")
    static_log = StaticLog(limit)
    robot, region = cloud(1), cloud(5000)
    static_log.log("world/robot", robot, once=True)
    static_log.log("world/map", region)

    assert log.call_args_list == [
        mocker.call("world/robot", robot, static=True),
        mocker.call("world/map", region, static=True),
    ]


def test_refresh_replays_updated_positions_and_partial_colors(mocker):
    log = mocker.patch.object(rr, "log")
    static_log = StaticLog("100KiB")
    static_log.log("world/robot", cloud(1), once=True)
    static_log.log("world/robot", rr.Points3D.from_fields(colors=[[0, 255, 0]]), once=True)
    static_log.log("/world/robot", rr.Points3D([[7, 8, 9]]))
    static_log.log("world/robot", rr.Boxes3D(half_sizes=[1, 1, 1]))
    log.reset_mock()

    static_log.log("world/map", cloud(5000))

    assert [call.args[0] for call in log.call_args_list] == [
        "world/map",
        "world/robot",
        "world/robot",
    ]
    assert all(call.kwargs == {"static": True} for call in log.call_args_list)
    assert {
        str(batch.component_descriptor()): batch.as_arrow_array().to_pylist()
        for call in log.call_args_list[1:]
        for batch in call.args[1]
    } == {"Points3D:positions": [[7, 8, 9]], "Points3D:colors": [0x00FF00FF]}


def test_each_cloud_records_its_frame_and_the_latest_frame_is_available_as_a_fallback(tmp_path):
    path = tmp_path / "frames.rrd"
    topic = Topic(topic="dimos/lidar", lcm_type=PointCloud2)
    module = RerunBridgeModule(pubsubs=[])
    try:
        with rr.RecordingStream("frame-history") as recording:
            recording.save(path)
            for step, frame in enumerate(["base_link", "base_link", "odom"]):
                recording.set_time("step", sequence=step)
                msg = PointCloud2.from_numpy(
                    np.array([[1, 2, 3]], dtype=np.float32), frame_id=frame
                )
                module._on_message(msg, topic)
    finally:
        module.stop()

    samples = []
    fallback = []
    for chunk in RrdReader(path).stream():
        data = chunk.to_record_batch().to_pydict()
        if chunk.entity_path == "/world/lidar":
            assert not chunk.is_static
            assert "Points3D:positions" in data
            samples.extend(zip(data["step"], data["Transform3D:parent_frame"], strict=True))
        if chunk.entity_path == "/frame_attachments/world/lidar":
            assert chunk.is_static
            fallback.extend(
                zip(data["Transform3D:child_frame"], data["Transform3D:parent_frame"], strict=True)
            )
    assert sorted(samples) == [(0, ["tf#/base_link"]), (1, ["tf#/base_link"]), (2, ["tf#/odom"])]
    assert fallback[-1] == (["tf#/world/lidar"], ["tf#/odom"])


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
        static={
            "world/robot": lambda rerun: rerun.Boxes3D(half_sizes=[0.3, 0.2, 0.1]),
            "world/scene": lambda rerun: rerun.Asset3D(
                contents=np.random.default_rng(1).bytes(170 * 1024)
            ),
        },
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


@pytest.mark.parametrize(
    ("bridge", "regions", "points"),
    [(False, 20, 4000), (True, 20, 4000), (True, 5, 1500)],
    indirect=["bridge"],
    ids=["temporal", "static", "large-asset-with-five-small-updates"],
)
def test_late_viewer_gets_what_the_bridge_logged_once(bridge, late_client, regions, points):
    module, uri = bridge
    recording = rr.get_global_data_recording()
    rng = np.random.default_rng(42)
    lidar = PointCloud2.from_numpy(np.array([[1, 2, 3]], dtype=np.float32), frame_id="lidar")
    module._on_message(lidar, Topic(topic="dimos/lidar", lcm_type=PointCloud2))

    # Five small updates fit in half the buffer, but exceed the space left by the scene.
    for seq in range(regions):
        region = PointCloud2.from_numpy(rng.random((points, 3), dtype=np.float32), frame_id="odom")
        region.seq = seq
        module._on_message(region, Topic(topic="dimos/map_regions", lcm_type=PointCloud2))
        recording.flush()

    columns = late_client(uri)
    expected = {
        "/frame_attachments/world/lidar:Transform3D:parent_frame",
        "/world/scene:Asset3D:blob",
        "/world/robot:Boxes3D:half_sizes",
        "/:AnnotationContext:context",
    }
    try:
        wait_until(lambda: expected <= columns(), timeout=10)
    except TimeoutError:
        pytest.fail(f"The late viewer never got {expected - columns()}")
