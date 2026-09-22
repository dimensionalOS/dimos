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

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Point, Pose, Quaternion
from dimos_generated.nav_msgs.msg import Odometry, Path
from dimos_generated.std_msgs.msg import Header
import pytest

from dimos.mapping.odometry_hist import OdometryHist, path_at_true_height


def odometry(x: float, nanosec: int, sec: int = 1700000000) -> Odometry:
    message = Odometry(header=Header(stamp=Time(sec=sec, nanosec=nanosec), frame_id="odom"))
    message.pose.pose = Pose(position=Point(x=x, y=2, z=3), orientation=Quaternion(w=1))
    return Odometry.decode(message.encode())


@pytest.mark.asyncio
async def test_history_retains_exact_stamp_and_one_nanosecond_interval(request):
    module = OdometryHist(min_publish_interval_seconds=1e-9)
    request.addfinalizer(module.stop)
    paths = []
    module.odom_hist.subscribe(lambda path: paths.append(Path.decode(path.encode())))
    source = odometry(1, 123456789)
    await module.handle_odometry(source)
    source.pose.pose.position.x = 100
    await module.handle_odometry(odometry(2, 123456790))
    assert len(paths) == 2
    assert paths[-1].header.stamp.nanosec == 123456790
    assert [p.header.stamp.nanosec for p in paths[-1].poses] == [123456789, 123456790]
    assert paths[-1].poses[0].pose.position.x == 1
    assert paths[0].poses[0].pose.position.x == 1
    archetype = path_at_true_height(paths[-1])
    assert archetype.strips.as_arrow_array().to_pylist() == [[[1, 2, 3], [2, 2, 3]]]


@pytest.mark.asyncio
async def test_history_throttles_bounds_and_restarts_at_zero(request):
    module = OdometryHist(max_poses=2, frame_id="world", min_publish_interval_seconds=0.1)
    request.addfinalizer(module.stop)
    paths = []
    module.odom_hist.subscribe(paths.append)
    await module.handle_odometry(odometry(1, 0))
    await module.handle_odometry(odometry(2, 50000000))
    assert len(paths) == 1
    await module.handle_odometry(odometry(3, 100000000))
    assert len(paths) == 2 and len(paths[-1].poses) == 2
    assert paths[-1].header.frame_id == "world"
    assert all(p.header.frame_id == "world" for p in paths[-1].poses)
    await module.handle_odometry(odometry(4, 0, sec=0))
    assert len(paths) == 3 and paths[-1].header.stamp == Time()
