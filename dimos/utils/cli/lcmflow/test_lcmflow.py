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

from dimos.utils.cli import theme
from dimos.utils.cli.lcmflow.lcmflow import (
    BASE_SPEED,
    SIZE_CLASSES,
    SPAWN_GAP,
    Highway,
    Lane,
    PacketSpy,
    color_for_channel,
    size_class,
    split_channel,
)


def test_size_classes_ordered() -> None:
    bounds = [cls.max_bytes for cls in SIZE_CLASSES]
    assert bounds == sorted(bounds)
    lengths = [cls.length for cls in SIZE_CLASSES]
    assert lengths == sorted(lengths)
    # Bigger packets are slower.
    speeds = [cls.speed for cls in SIZE_CLASSES]
    assert speeds == sorted(speeds, reverse=True)


def test_size_class_selection() -> None:
    assert size_class(60).name == "nano"  # Twist
    assert size_class(1_000).name == "small"  # odom
    assert size_class(10_000).name == "medium"  # laser scan
    assert size_class(100_000).name == "large"  # costmap
    assert size_class(2_764_800).name == "mega"  # raw image
    assert size_class(2**40).name == "mega"


def test_split_channel() -> None:
    assert split_channel("/odom#geometry_msgs.PoseStamped") == (
        "/odom",
        "geometry_msgs.PoseStamped",
    )
    assert split_channel("/plain") == ("/plain", "")


def test_color_for_channel() -> None:
    assert color_for_channel("/color_image#sensor_msgs.Image") == theme.YELLOW
    assert color_for_channel("/lidar#sensor_msgs.PointCloud2") == theme.PURPLE
    assert color_for_channel("/cmd_vel#geometry_msgs.Twist") == theme.CYAN
    assert color_for_channel("/rpc/foo") == theme.BRIGHT_BLUE
    # Unknown channels get a stable palette color.
    assert color_for_channel("/mystery") == color_for_channel("/mystery")


def test_vehicle_position_and_speed() -> None:
    lane = Lane("/cmd_vel#geometry_msgs.Twist")
    vehicle = lane.spawn(60, now=0.0)
    speed = BASE_SPEED * vehicle.cls.speed
    assert vehicle.head(0.0) == 0.0
    assert vehicle.head(1.0) == speed
    # Nano packets outrun mega packets.
    mega = Lane("/img#sensor_msgs.Image").spawn(2_000_000, now=0.0)
    assert vehicle.head(1.0) > mega.head(1.0)


def test_lane_coalescing() -> None:
    lane = Lane("/tf#tf2_msgs.TFMessage")
    v1 = lane.spawn(300, now=0.0)
    # Arrives before v1 cleared the on-ramp: coalesce.
    v2 = lane.spawn(300, now=0.001)
    assert v2 is v1
    assert v1.count == 2
    assert v1.n_bytes == 600
    # Arrives after v1 cleared (length + gap cells): new vehicle.
    clear_time = (v1.length + SPAWN_GAP + 1) / v1.speed
    v3 = lane.spawn(300, now=clear_time)
    assert v3 is not v1
    assert len(lane.vehicles) == 2


def test_coalesced_burst_upgrades_class() -> None:
    lane = Lane("/mixed")
    v = lane.spawn(100, now=0.0)
    assert v.cls.name == "nano"
    lane.spawn(1_000_000, now=0.001)
    assert v.cls.name == "mega"


def test_coalesced_burst_stretches() -> None:
    lane = Lane("/tf#tf2_msgs.TFMessage")
    v = lane.spawn(300, now=0.0)
    base_length = v.length
    for _ in range(7):
        lane.spawn(300, now=0.001)
    assert v.count == 8
    assert base_length < v.length <= v.cls.length + 4


def test_lane_prune() -> None:
    lane = Lane("/odom#nav_msgs.Odometry")
    lane.spawn(500, now=0.0)
    lane.prune(now=0.1, road_width=100)
    assert len(lane.vehicles) == 1
    lane.prune(now=1_000.0, road_width=100)
    assert len(lane.vehicles) == 0


def test_packet_spy_drain() -> None:
    spy = PacketSpy()
    spy.msg("/a", b"xx")
    spy.msg("/b", b"yyyy")
    drained = spy.drain()
    assert drained == [("/a", 2), ("/b", 4)]
    assert spy.drain() == []
    # Stats side stays intact (LCMSpy behavior).
    assert spy.topic["/a"].total_traffic() == 2


def test_highway_tick_spawns_and_pauses() -> None:
    highway = Highway()
    highway.spy.msg("/a#std_msgs.String", b"x" * 100)
    highway.tick(road_width=100)
    assert "/a#std_msgs.String" in highway.lanes
    assert len(highway.lanes["/a#std_msgs.String"].vehicles) == 1

    clock_before = highway.clock
    highway.toggle_pause()
    highway.spy.msg("/a#std_msgs.String", b"x" * 100)
    highway.tick(road_width=100)
    # Paused: clock frozen, packets dropped, no new vehicles.
    assert highway.clock == clock_before
    assert len(highway.lanes["/a#std_msgs.String"].vehicles) == 1
