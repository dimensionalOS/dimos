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

"""GO2L1Lidar's core, synthetic and over a 13 s clip: a knock, its bad scans, a stall and its late cloud.

The clip (``go2_l1_knock.db``) holds what the module would receive live: GO2DDS's
``l1_lidar_raw``/``l1_lidar_imu`` (header stamps) and body ``go2_imu`` (50 Hz), and Point-LIO's
``odometry``, each observation at its receive time.
"""

import math

import numpy as np
import pytest
from scipy.spatial import cKDTree

from dimos.memory.store.sqlite import SqliteStore
from dimos.message_utils import pointcloud, time as message_time
from dimos.messages.geometry_msgs.msg import (
    Point,
    Pose,
    PoseWithCovariance,
    Quaternion,
    Transform,
    TransformStamped,
    Twist,
    TwistWithCovariance,
    Vector3,
)
from dimos.messages.nav_msgs.msg import Odometry
from dimos.messages.std_msgs.msg import Header
from dimos.protocol.tf.static_tf_publisher import frames_to_edge_transforms
from dimos.protocol.tf.tf import MultiTBuffer
from dimos.robot.unitree.go2.base import Go2BaseConfig
from dimos.robot.unitree.go2.dds.module import GO2L1LidarConfig, L1Deskew
from dimos.robot.unitree.go2.dds.pointcloud import sensor_cloud
from dimos.utils.data import get_data

# The first L1 cloud of across_the_box; the clip starts 3.5 s after it.
T0 = 1791993294.5419347


def _mount(**overrides):
    """mid360_link <- utlidar_lidar and base_link <- utlidar_lidar, as GO2DDS's mount tree has them."""
    c = Go2BaseConfig(mid360_mount="ATHENS", **overrides)

    def rad(deg):
        return tuple(math.radians(a) for a in deg)

    buffer = MultiTBuffer()
    buffer.receive_transform(
        *frames_to_edge_transforms(
            [
                ("base_link", None, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
                ("front_camera", "base_link", c.camera_xyz, (0.0, 0.0, 0.0)),
                ("mid360_link", "front_camera", c.mid360_xyz, rad(c.mid360_mount)),
                ("utlidar_lidar", "base_link", c.l1_xyz, rad(c.l1_mount)),
            ]
        )
    )
    return buffer.get("mid360_link", "utlidar_lidar"), buffer.get("base_link", "utlidar_lidar")


def _replay(**mount):
    store = SqliteStore(path=str(get_data("go2_l1_knock.db")))
    store.start()
    feed = []
    for name in ("l1_lidar_raw", "l1_lidar_imu", "go2_imu", "odometry"):
        feed += [(obs.ts, name, obs.data) for obs in store.stream(name)]
    feed.sort(key=lambda f: f[0])
    core = L1Deskew(GO2L1LidarConfig(), *_mount(**mount))
    out = []
    for ts, name, msg in feed:
        if name == "l1_lidar_raw":
            out += core.on_cloud(msg, ts)
        elif name == "l1_lidar_imu":
            core.on_l1_imu(msg)
        elif name == "go2_imu":
            core.on_body_imu(msg)
        else:
            out += core.on_odometry(msg)
    store.stop()
    return core, out


def _odom(ts, x):
    return Odometry(
        Header(message_time.time_from_seconds(ts), "odom"),
        "mid360_link",
        PoseWithCovariance(Pose(Point(x, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)), np.zeros(36)),
        TwistWithCovariance(Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), np.zeros(36)),
    )


def test_a_scan_waits_for_odometry_and_deskews_per_point():
    core = L1Deskew(
        GO2L1LidarConfig(),
        TransformStamped(
            message_time.header_now("mid360_link"),
            "utlidar_lidar",
            Transform(Vector3(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        ),
        TransformStamped(
            message_time.header_now("base_link"),
            "utlidar_lidar",
            Transform(Vector3(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        ),
    )
    cloud = sensor_cloud(
        np.array([[1.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.5, 0.0, 0.0], [3.0, 0.0, 0.0]]),
        header=Header(message_time.time_from_seconds(10.0), "utlidar_lidar"),
        offset_times=np.array([0, 50_000_000, 0, 0]),
    )
    core.on_odometry(_odom(9.9, 0.0))
    assert core.on_cloud(cloud, arrival=10.08) == []
    assert core.on_odometry(_odom(10.02, 0.6)) == []  # the scan ends at 10.05
    (out,) = core.on_odometry(_odom(10.1, 1.0))
    # Odometry moves 5 m/s: each point lands by the pose at its own time; the body
    # (0.5 m) and out-of-range (3 m) returns are gone.
    np.testing.assert_allclose(pointcloud.xyz(out)[:, 0], [1.5, 1.75], atol=1e-5)
    assert (out.header.frame_id, message_time.to_seconds(out.header.stamp)) == ("odom", 10.0)


def test_a_late_scan_is_dropped():
    core = L1Deskew(
        GO2L1LidarConfig(),
        TransformStamped(
            message_time.header_now("mid360_link"),
            "utlidar_lidar",
            Transform(Vector3(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        ),
        TransformStamped(
            message_time.header_now("base_link"),
            "utlidar_lidar",
            Transform(Vector3(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        ),
    )
    cloud = sensor_cloud(
        np.array([[1.0, 0.0, 0.0]]), header=Header(message_time.time_from_seconds(10.0), "world")
    )
    core.on_odometry(_odom(9.9, 0.0))
    assert core.on_cloud(cloud, arrival=11.5) == []
    assert core.on_odometry(_odom(10.1, 1.0)) == []
    assert core.dropped_late == 1


@pytest.fixture(scope="module")
def replay():
    return _replay()


def _consistency(clouds):
    """Fraction of each clean scan's points within 5 cm of the scans before it."""
    clean = [c for c in clouds if message_time.to_seconds(c.header.stamp) < T0 + 5.4]
    seen = pointcloud.xyz(clean[0])
    hits = []
    for cloud in clean[1:]:
        pts = pointcloud.xyz(cloud)
        dist, _ = cKDTree(seen).query(pts)
        hits.append(np.mean(dist < 0.05))
        seen = np.vstack([seen, pts])
    return float(np.mean(hits))


@pytest.mark.self_hosted
def test_knock_holds_off_the_bad_stretch(replay):
    core, out = replay
    (start, end), *_ = core.knocks
    # The knock lands ~5.6 s in and the bad scans run 5.9 to 9.1 s; the trailing RMS
    # window crosses at ~5.93 s, a quarter second after a centred one would.
    assert T0 + 5.5 < start < T0 + 6.0
    assert end > T0 + 9.1
    assert core.dropped_knock > 20
    assert not any(
        a <= message_time.to_seconds(c.header.stamp) < b for c in out for a, b in core.knocks
    )


@pytest.mark.self_hosted
def test_the_stalled_cloud_is_dropped(replay):
    core, out = replay
    # Stamped 10.05 s in, delivered 6 s later; placed by its stamp it would smear the map.
    assert core.dropped_late >= 1
    assert all(
        message_time.to_seconds(c.header.stamp) < T0 + 10.1
        or message_time.to_seconds(c.header.stamp) > T0 + 15.0
        for c in out
    )


@pytest.mark.self_hosted
def test_scans_land_in_odom_filtered(replay):
    _, out = replay
    assert len(out) > 20
    assert {c.header.frame_id for c in out} == {"odom"}
    assert all(len(pointcloud.xyz(c)) > 100 for c in out)


@pytest.mark.self_hosted
def test_clean_scans_agree_with_each_other(replay):
    _, out = replay
    good = _consistency(out)
    # The radar_joint-like yaw error (~120 deg) the fitted mount replaced: scans disagree.
    _, wrong = _replay(l1_mount=(-167.877, -7.907, 60.0))
    print(f"\nclean-scan consistency: fitted {good:.2f}, yawed mount {_consistency(wrong):.2f}")
    assert good > 0.8
    assert _consistency(wrong) < good - 0.15
