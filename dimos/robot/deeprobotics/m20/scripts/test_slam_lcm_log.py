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

from pathlib import Path

import pytest

from dimos.robot.deeprobotics.m20.scripts import slam_lcm_log


def test_record_topics_select_front_airy_imu_by_default() -> None:
    topics = slam_lcm_log.record_topics(lidar_source="front")

    assert "/raw_points#sensor_msgs.PointCloud2" in topics
    assert "/airy_imu_front#sensor_msgs.Imu" in topics
    assert "/airy_imu_rear#sensor_msgs.Imu" not in topics
    assert "/odometry#nav_msgs.Odometry" in topics
    assert "/registered_scan#sensor_msgs.PointCloud2" in topics
    assert "/global_map_fastlio#sensor_msgs.PointCloud2" in topics


def test_record_topics_select_rear_airy_imu() -> None:
    topics = slam_lcm_log.record_topics(lidar_source="rear")

    assert "/airy_imu_rear#sensor_msgs.Imu" in topics
    assert "/airy_imu_front#sensor_msgs.Imu" not in topics


def test_record_topics_can_include_nav_debug_without_commands() -> None:
    topics = slam_lcm_log.record_topics(lidar_source="front", include_nav_debug=True)

    assert "/terrain_map#sensor_msgs.PointCloud2" in topics
    assert "/costmap_cloud#sensor_msgs.PointCloud2" in topics
    assert "/cmd_vel#geometry_msgs.Twist" not in topics
    assert "/nav_cmd_vel#geometry_msgs.Twist" not in topics


def test_lcm_channel_regex_is_anchored_and_escapes_message_type_dots() -> None:
    regex = slam_lcm_log.channel_regex(
        [
            "/raw_points#sensor_msgs.PointCloud2",
            "/airy_imu_front#sensor_msgs.Imu",
        ]
    )

    assert regex == ("^(/raw_points#sensor_msgs\\.PointCloud2|/airy_imu_front#sensor_msgs\\.Imu)$")


def test_build_logger_command_records_exact_topic_regex() -> None:
    command = slam_lcm_log.build_logger_command(
        log_path=Path("/tmp/m20_slam/session.lcm"),
        topics=[
            "/raw_points#sensor_msgs.PointCloud2",
            "/airy_imu_front#sensor_msgs.Imu",
        ],
        lcm_url="udpm://239.255.76.67:7667?ttl=1",
        force=True,
        flush_interval_ms=25,
        max_unwritten_mb=512,
    )

    assert command == [
        "lcm-logger",
        "-f",
        "--flush-interval=25",
        "--max-unwritten-mb=512",
        "--lcm-url=udpm://239.255.76.67:7667?ttl=1",
        "-c",
        "^(/raw_points#sensor_msgs\\.PointCloud2|/airy_imu_front#sensor_msgs\\.Imu)$",
        "/tmp/m20_slam/session.lcm",
    ]


def test_build_logplayer_command_replays_inputs_only_by_default() -> None:
    command = slam_lcm_log.build_logplayer_command(
        log_path=Path("/tmp/m20_slam/session.lcm"),
        lidar_source="front",
        speed=0.5,
    )

    assert command == [
        "lcm-logplayer",
        "--speed=0.5",
        "--regexp=^(/raw_points#sensor_msgs\\.PointCloud2|/airy_imu_front#sensor_msgs\\.Imu)$",
        "/tmp/m20_slam/session.lcm",
    ]


def test_build_logplayer_command_rejects_invalid_lidar_source() -> None:
    with pytest.raises(ValueError, match="lidar_source"):
        slam_lcm_log.build_logplayer_command(
            log_path=Path("/tmp/m20_slam/session.lcm"),
            lidar_source="side",
        )


def test_timed_record_treats_signal_terminated_logger_as_success() -> None:
    assert slam_lcm_log.record_exit_code(returncode=-15, timed_stop=True) == 0
    assert slam_lcm_log.record_exit_code(returncode=-2, timed_stop=True) == 0
    assert slam_lcm_log.record_exit_code(returncode=3, timed_stop=True) == 3
    assert slam_lcm_log.record_exit_code(returncode=-15, timed_stop=False) == -15
