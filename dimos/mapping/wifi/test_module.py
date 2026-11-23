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
from contextlib import aclosing

import numpy as np
import pytest

from dimos.hardware.sensors.wifi.module import WifiMonitor
from dimos.mapping.wifi.module import WifiCoverage
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.sensor_msgs.WifiScan import WifiAccessPoint, WifiScan


@pytest.fixture
def coverage(mocker):
    module = WifiCoverage(cell_size=1)
    publish = mocker.patch.object(module.wifi_coverage, "publish")
    yield module, publish
    module.stop()


@pytest.mark.asyncio
async def test_streams_produce_wire_compatible_coloured_coverage_and_rpc_selection(coverage):
    module, publish = coverage
    await module.handle_odometry(Odometry(ts=10, frame_id="map", pose=Pose(position=[2, 3, 0])))
    await module.handle_wifi_scan(
        WifiScan(
            10,
            "wifi",
            [
                WifiAccessPoint(10, "aa:bb:cc:dd:ee:01", "office", 5500, -90),
                WifiAccessPoint(10, "aa:bb:cc:dd:ee:02", "office", 5640, -40),
            ],
        )
    )
    cloud = publish.call_args.args[0]
    decoded = PointCloud2.lcm_decode(cloud.lcm_encode())
    points, colors = decoded.as_numpy()
    np.testing.assert_allclose(points, [[2.5, 3.5, 0.5]])
    np.testing.assert_allclose(colors, [[0, 1, 0]])
    assert decoded.frame_id == "map"
    assert len(module.get_access_points()) == 2
    module.select_access_point("AA:BB:CC:DD:EE:01")
    np.testing.assert_allclose(publish.call_args.args[0].as_numpy()[1], [[1, 0, 0]])
    assert module.get_cells()[0]["mean_dbm"] == -90
    module.reset()
    assert module.get_cells() == []
    assert publish.call_args.args[0].as_numpy()[0].shape == (0, 3)


@pytest.mark.asyncio
async def test_monitor_default_does_not_trigger_scans_and_stops_polling(mocker):
    module = WifiMonitor(interface="wifi")
    mocker.patch.object(module, "check_dependencies")
    scan = WifiScan(10, "wifi", [])
    read = mocker.patch("dimos.hardware.sensors.wifi.module.read_scan", return_value=scan)
    trigger = mocker.patch("dimos.hardware.sensors.wifi.module.run_iw")
    received = asyncio.Event()
    publish = mocker.patch.object(module.wifi_scan, "publish", side_effect=lambda _: received.set())
    try:
        async with aclosing(module.main()) as lifecycle:
            await anext(lifecycle)
            await asyncio.wait_for(received.wait(), timeout=1)
        publish.assert_called_once_with(scan)
        read.assert_awaited_once_with("wifi", 5.0)
        trigger.assert_not_called()
    finally:
        module.stop()
