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

"""Late clients recover scene state through the real, overflowing Rerun proxy."""

from contextlib import contextmanager
from functools import partial
import os
import shutil
import signal
import subprocess
import warnings

import numpy as np
import pytest
import rerun as rr
from rerun.experimental import RrdReader
import rerun_bindings

from dimos.mapping.ray_tracing.viz import render_map_region
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.pubsub.impl.zenohpubsub import Topic
from dimos.protocol.pubsub.spec import SubscribeAllCapable
from dimos.utils.testing.waiting import wait_until
from dimos.visualization.rerun.bridge import RerunBridgeModule


def robot(rerun):
    return rerun.Boxes3D(half_sizes=[0.3, 0.2, 0.1])


@pytest.fixture
def bridge(request, mocker, unused_tcp_port):
    previous = rr.get_global_data_recording()
    source = mocker.Mock(spec=SubscribeAllCapable)
    source.subscribe_all.return_value = lambda: None
    uri = f"rerun+http://127.0.0.1:{unused_tcp_port}/proxy"
    value = RerunBridgeModule(
        pubsubs=[source],
        blueprint=None,
        rerun_open="none",
        connect_url=uri,
        server_memory_limit="16KiB",
        scene_refresh_interval=0.1,
        retained_topics=["loaded_map"],
        static={"world/robot": robot},
        visual_override={
            "world/map_regions": partial(
                render_map_region,
                voxel_size=0.1,
                height_range=(0.0, 1.0),
                static=request.param,
            ),
        },
    )
    try:
        value.start()
        yield value, uri
    finally:
        value.stop()
        rr.disconnect()
        rerun_bindings.set_global_data_recording(previous.to_native() if previous else None)


@pytest.fixture
def capture(tmp_path):
    cli = shutil.which("rerun")
    assert cli is not None

    @contextmanager
    def start(uri):
        path = tmp_path / "late.rrd"
        client = subprocess.Popen(
            [cli, "--save", str(path), "--bind", "127.0.0.1", "--port", "auto", uri],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            env={**os.environ, "RERUN_ANALYTICS_ENABLED": "false"},
        )
        try:
            yield path
        finally:
            if client.poll() is None:
                client.send_signal(signal.SIGINT)
            try:
                client.wait(timeout=5)
            except subprocess.TimeoutExpired:
                client.kill()
                client.wait(timeout=5)
            path.unlink(missing_ok=True)

    return start


def columns(path):
    if not path.exists() or path.stat().st_size == 0:
        return set()
    # A live --save stream has no footer yet; RrdReader can scan complete chunks.
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", message="RRD file has no footer/manifest")
        return set(RrdReader(path).stream().collect().schema().column_names())


@pytest.mark.parametrize("bridge", [False, True], indirect=True)
def test_late_and_reconnected_clients_recover_scene_after_eviction(bridge, capture):
    module, uri = bridge
    lidar = Topic(topic="dimos/lidar", lcm_type=PointCloud2)
    loaded_map = Topic(topic="dimos/loaded_map", lcm_type=PointCloud2)
    regions = Topic(topic="dimos/map_regions", lcm_type=PointCloud2)
    cloud = PointCloud2.from_numpy(np.array([[1, 2, 3]], dtype=np.float32), frame_id="lidar")
    module._on_message(cloud, lidar)
    module._on_message(cloud, loaded_map)
    recording = rr.get_global_data_recording()
    rng = np.random.default_rng(42)

    for _ in range(8):
        region = PointCloud2.from_numpy(rng.random((4000, 3), dtype=np.float32), frame_id="odom")
        module._on_message(region, regions)
        recording.flush()
    module._on_message(cloud, lidar)
    recording.flush()

    expected = {
        "/world/lidar:Transform3D:parent_frame",
        "/world/loaded_map:Points3D:positions",
        "/world/loaded_map:Transform3D:parent_frame",
        "/world/robot:Boxes3D:half_sizes",
        "/:AnnotationContext:context",
    }
    # No sensor messages are published after connecting: the bridge must repair
    # the scene itself, including a second client after further buffer eviction.
    for _ in range(2):
        with capture(uri) as path:
            try:
                wait_until(lambda path=path: expected <= columns(path), timeout=10)
            except TimeoutError:
                pytest.fail(f"Missing scene components: {expected - columns(path)}")
            assert expected <= columns(path)
