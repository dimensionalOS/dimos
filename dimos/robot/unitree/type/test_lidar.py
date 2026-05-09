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

import pytest
import reactivex as rx

from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.type.lidar import (
    RawLidarMsg,
    pointcloud2_from_webrtc_lidar,
    repair_stale_ts,
)
from dimos.types.timestamped import Timestamped
from dimos.utils.testing.replay import SensorReplay


def test_init() -> None:
    lidar = SensorReplay("office_lidar")

    for raw_frame in itertools.islice(lidar.iterate(), 5):
        assert isinstance(raw_frame, dict)
        frame = pointcloud2_from_webrtc_lidar(cast("RawLidarMsg", raw_frame))
        assert isinstance(frame, PointCloud2)


def test_repair_stale_ts_extrapolates_non_monotonic_stamp() -> None:
    # input: two healthy frames, one stale, one healthy
    raw = [Timestamped(0.000), Timestamped(0.130), Timestamped(-6.354), Timestamped(0.260)]
    out: list[float] = []
    rx.from_iterable(raw).pipe(repair_stale_ts(default_period=0.130)).subscribe(
        on_next=lambda item: out.append(item.ts)
    )
    assert out == [0.000, 0.130, 0.260, 0.390]


def test_repair_stale_ts_passes_monotonic_unchanged() -> None:
    raw = [Timestamped(0.0), Timestamped(0.130), Timestamped(0.260)]
    out: list[float] = []
    rx.from_iterable(raw).pipe(repair_stale_ts()).subscribe(
        on_next=lambda item: out.append(item.ts)
    )
    assert out == [0.0, 0.130, 0.260]


def test_repair_stale_ts_handles_consecutive_bad_frames() -> None:
    # two stale-stamp frames in a row → each forward-extrapolated by default_period
    raw = [Timestamped(0.0), Timestamped(-6.354), Timestamped(-6.354), Timestamped(0.500)]
    out: list[float] = []
    rx.from_iterable(raw).pipe(repair_stale_ts(default_period=0.130)).subscribe(
        on_next=lambda item: out.append(item.ts)
    )
    assert out == [0.0, 0.130, 0.260, 0.500]


def test_repair_stale_ts_old_firmware_uses_system_time_after_calibration() -> None:
    raw = [Timestamped(100.0) for _ in range(13)]
    out: list[float] = []
    rx.from_iterable(raw).pipe(
        repair_stale_ts(default_period=0.130, calibration_frames=10, now=lambda: 999.0)
    ).subscribe(on_next=lambda item: out.append(item.ts))
    assert out[0] == 100.0
    assert out[1:10] == pytest.approx([100.0 + 0.130 * i for i in range(1, 10)])
    assert out[10:] == [999.0, 999.0, 999.0]


def test_repair_stale_ts_new_firmware_repair_persists_after_calibration() -> None:
    raw = [Timestamped(i * 0.130) for i in range(11)] + [Timestamped(-6.354), Timestamped(2.0)]
    out: list[float] = []
    rx.from_iterable(raw).pipe(
        repair_stale_ts(default_period=0.130, calibration_frames=10, now=lambda: 999.0)
    ).subscribe(on_next=lambda item: out.append(item.ts))
    assert out[:11] == pytest.approx([i * 0.130 for i in range(11)])
    assert out[11] == pytest.approx(10 * 0.130 + 0.130)
    assert out[12] == pytest.approx(2.0)


def test_repair_stale_ts_calibration_boundary_one_differs() -> None:
    raw = [Timestamped(5.0) for _ in range(9)] + [
        Timestamped(5.5),
        Timestamped(5.6),
        Timestamped(5.7),
    ]
    out: list[float] = []
    rx.from_iterable(raw).pipe(
        repair_stale_ts(default_period=0.130, calibration_frames=10, now=lambda: 999.0)
    ).subscribe(on_next=lambda item: out.append(item.ts))
    assert 999.0 not in out
