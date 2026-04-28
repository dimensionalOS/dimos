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

import sys
from types import SimpleNamespace

import numpy as np

from dimos.robot.deeprobotics.m20.blueprints.nav import m20_rerun


class _FakeRerun:
    class Boxes3D:
        def __init__(self, **kwargs):
            self.kwargs = kwargs

    class Transform3D:
        def __init__(self, **kwargs):
            self.kwargs = kwargs

    class Quaternion:
        def __init__(self, **kwargs):
            self.kwargs = kwargs

    class Points3D:
        def __init__(self, *args, **kwargs):
            self.args = args
            self.kwargs = kwargs


class _FakeSubscription:
    def __init__(self):
        self.queue_capacity = None

    def set_queue_capacity(self, capacity):
        self.queue_capacity = capacity


class _FakeLCM:
    def __init__(self):
        self.channels = []
        self.unsubscribed = []

    def subscribe(self, channel, handler):
        sub = _FakeSubscription()
        self.channels.append(channel)
        return sub

    def unsubscribe(self, sub):
        self.unsubscribed.append(sub)


def test_m20_rerun_lcm_subscribes_to_pgo_and_planner_topics(monkeypatch):
    pubsub = object.__new__(m20_rerun.M20RerunLCM)
    pubsub.l = _FakeLCM()

    pubsub.subscribe_all(lambda _msg, _topic: None)

    assert set(pubsub.l.channels) == {
        spec.channel for spec in m20_rerun.M20_RERUN_TOPIC_SPECS
    }
    assert set(pubsub.l.channels) >= {
        "/corrected_odometry#nav_msgs.Odometry",
        "/global_map_pgo#sensor_msgs.PointCloud2",
        "/terrain_map#sensor_msgs.PointCloud2",
        "/terrain_map_ext#sensor_msgs.PointCloud2",
        "/obstacle_cloud#sensor_msgs.PointCloud2",
        "/costmap_cloud#sensor_msgs.PointCloud2",
        "/path#nav_msgs.Path",
        "/goal_path#nav_msgs.Path",
        "/clicked_point#geometry_msgs.PointStamped",
        "/goal#geometry_msgs.PointStamped",
        "/way_point#geometry_msgs.PointStamped",
    }


def test_m20_rerun_topics_are_declared_in_one_typed_table():
    channels = {
        spec.channel: spec.throttle_group for spec in m20_rerun.M20_RERUN_TOPIC_SPECS
    }

    assert channels["/registered_scan#sensor_msgs.PointCloud2"] == "registered_scan"
    assert channels["/terrain_map#sensor_msgs.PointCloud2"] == "debug_cloud"
    assert channels["/global_map_pgo#sensor_msgs.PointCloud2"] == "debug_cloud"
    assert channels["/clicked_point#geometry_msgs.PointStamped"] is None
    assert channels["/goal#geometry_msgs.PointStamped"] is None
    assert channels["/way_point#geometry_msgs.PointStamped"] is None
    assert "/goal#geometry_msgs.PoseStamped" not in channels
    assert len(channels) == len(m20_rerun.M20_RERUN_TOPIC_SPECS)


def test_static_robot_attaches_to_sensor_tf():
    entries = m20_rerun.static_robot(_FakeRerun)

    transform = entries[-1]
    assert transform.kwargs["parent_frame"] == "tf#/sensor"


def test_odometry_tf_override_publishes_sensor_frame(monkeypatch):
    monkeypatch.setitem(__import__("sys").modules, "rerun", _FakeRerun)
    odom = SimpleNamespace(
        x=1.0,
        y=2.0,
        z=0.3,
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.5, w=0.866),
    )

    entries = m20_rerun.m20_odometry_tf_override(odom)

    assert len(entries) == 1
    path, transform = entries[0]
    assert path == "tf#/sensor"
    assert transform.kwargs["translation"] == [1.0, 2.0, 0.3]
    assert transform.kwargs["parent_frame"] == "tf#/map"
    assert transform.kwargs["child_frame"] == "tf#/sensor"


def test_raw_points_override_hides_raw_lidar():
    assert m20_rerun.raw_points_override(object()) is None


def test_registered_scan_override_samples_points(monkeypatch):
    class Cloud:
        def __init__(self):
            self.points = np.column_stack(
                (
                    np.arange(16_000, dtype=np.float32),
                    np.zeros(16_000, dtype=np.float32),
                    np.ones(16_000, dtype=np.float32),
                )
            )

        def points_f32(self):
            return self.points

    monkeypatch.setitem(sys.modules, "rerun", _FakeRerun)
    cloud = Cloud()

    points = m20_rerun.registered_scan_override(cloud)

    assert len(points.kwargs["positions"]) <= 8_000
    assert len(points.kwargs["class_ids"]) == len(points.kwargs["positions"])


def test_registered_scan_override_honors_max_points_env(monkeypatch):
    class Cloud:
        def __init__(self):
            self.points = np.column_stack(
                (
                    np.arange(16_000, dtype=np.float32),
                    np.zeros(16_000, dtype=np.float32),
                    np.ones(16_000, dtype=np.float32),
                )
            )

        def points_f32(self):
            return self.points

    monkeypatch.setitem(sys.modules, "rerun", _FakeRerun)
    monkeypatch.setenv("M20_RERUN_REGISTERED_SCAN_MAX_POINTS", "1000")

    points = m20_rerun.registered_scan_override(Cloud())

    assert len(points.kwargs["positions"]) <= 1_000


def test_registered_scan_period_can_be_disabled(monkeypatch):
    monkeypatch.setenv("M20_RERUN_REGISTERED_SCAN_HZ", "0")

    assert m20_rerun.registered_scan_period_sec() is None


def test_registered_scan_period_uses_env_hz(monkeypatch):
    monkeypatch.setenv("M20_RERUN_REGISTERED_SCAN_HZ", "0.5")

    assert m20_rerun.registered_scan_period_sec() == 2.0
