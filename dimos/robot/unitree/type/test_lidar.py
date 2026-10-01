#!/usr/bin/env python3
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

import itertools
from typing import cast

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.sensor_msgs.msg import PointCloud2
from dimos_generated.std_msgs.msg import Header
import numpy as np
import pytest
import reactivex as rx

from dimos.msgs.pointcloud import pointcloud_xyz
from dimos.msgs.time import time_from_seconds, to_seconds
from dimos.robot.unitree.type.lidar import (
    RawLidarMsg,
    pointcloud2_from_webrtc_lidar,
    repair_stale_ts,
)
from dimos.utils.testing.replay import SensorReplay


def _stamped(seconds: float) -> PointCloud2:
    return PointCloud2(header=Header(stamp=time_from_seconds(seconds)))


@pytest.mark.self_hosted
def test_init() -> None:
    lidar = SensorReplay("office_lidar")

    for raw_frame in itertools.islice(lidar.iterate(), 5):
        assert isinstance(raw_frame, dict)
        frame = pointcloud2_from_webrtc_lidar(cast("RawLidarMsg", raw_frame))
        assert isinstance(frame, PointCloud2)


def test_repair_stale_ts_extrapolates_non_monotonic_stamp() -> None:
    # input: two healthy frames, one stale, one healthy
    raw = [_stamped(0.000), _stamped(0.130), _stamped(-6.354), _stamped(0.260)]
    out: list[float] = []
    rx.from_iterable(raw).pipe(repair_stale_ts(default_period=0.130)).subscribe(
        on_next=lambda item: out.append(to_seconds(item.header.stamp))
    )
    assert out == [0.000, 0.130, 0.260, 0.390]


def test_repair_stale_ts_passes_monotonic_unchanged() -> None:
    raw = [_stamped(0.0), _stamped(0.130), _stamped(0.260)]
    out: list[float] = []
    rx.from_iterable(raw).pipe(repair_stale_ts()).subscribe(
        on_next=lambda item: out.append(to_seconds(item.header.stamp))
    )
    assert out == [0.0, 0.130, 0.260]


def test_repair_stale_ts_handles_consecutive_bad_frames() -> None:
    # two stale-stamp frames in a row → each forward-extrapolated by default_period
    raw = [_stamped(0.0), _stamped(-6.354), _stamped(-6.354), _stamped(0.500)]
    out: list[float] = []
    rx.from_iterable(raw).pipe(repair_stale_ts(default_period=0.130)).subscribe(
        on_next=lambda item: out.append(to_seconds(item.header.stamp))
    )
    assert out == [0.0, 0.130, 0.260, 0.500]


def test_repair_stale_ts_old_firmware_uses_system_time_after_calibration() -> None:
    raw = [_stamped(100.0) for _ in range(13)]
    out: list[float] = []
    rx.from_iterable(raw).pipe(
        repair_stale_ts(default_period=0.130, calibration_frames=10, now=lambda: 999.0)
    ).subscribe(on_next=lambda item: out.append(to_seconds(item.header.stamp)))
    assert out[0] == 100.0
    assert out[1:10] == pytest.approx([100.0 + 0.130 * i for i in range(1, 10)])
    assert out[10:] == [999.0, 999.0, 999.0]


def test_repair_stale_ts_new_firmware_repair_persists_after_calibration() -> None:
    raw = [_stamped(i * 0.130) for i in range(11)] + [_stamped(-6.354), _stamped(2.0)]
    out: list[float] = []
    rx.from_iterable(raw).pipe(
        repair_stale_ts(default_period=0.130, calibration_frames=10, now=lambda: 999.0)
    ).subscribe(on_next=lambda item: out.append(to_seconds(item.header.stamp)))
    assert out[:11] == pytest.approx([i * 0.130 for i in range(11)])
    assert out[11] == pytest.approx(10 * 0.130 + 0.130)
    assert out[12] == pytest.approx(2.0)


def test_repair_stale_ts_calibration_boundary_one_differs() -> None:
    raw = [_stamped(5.0) for _ in range(9)] + [
        _stamped(5.5),
        _stamped(5.6),
        _stamped(5.7),
    ]
    out: list[float] = []
    rx.from_iterable(raw).pipe(
        repair_stale_ts(default_period=0.130, calibration_frames=10, now=lambda: 999.0)
    ).subscribe(on_next=lambda item: out.append(to_seconds(item.header.stamp)))
    assert 999.0 not in out


@pytest.mark.parametrize("override", [None, Time(sec=1700000000, nanosec=123456789)])
def test_webrtc_xyz_conversion_preserves_points_and_explicit_time(override):
    raw = {"data": {"stamp": -0.5, "data": {"points": np.array([[1.25, 2.5, 3.75]])}}}
    cloud = pointcloud2_from_webrtc_lidar(cast("RawLidarMsg", raw), stamp=override)
    decoded = PointCloud2.decode(cloud.encode())
    np.testing.assert_array_equal(pointcloud_xyz(decoded), [[1.25, 2.5, 3.75]])
    assert decoded.header.frame_id == "world"
    assert decoded.header.stamp == (
        Time(sec=-1, nanosec=500000000) if override is None else override
    )
