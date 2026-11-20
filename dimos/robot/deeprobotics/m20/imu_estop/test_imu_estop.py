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

import csv
import gzip
import json
import math
import os
from pathlib import Path
import struct

import pytest

from dimos.robot.deeprobotics.m20.imu_estop.imu_estop import (
    BodyImuRelay,
    BodyImuSniffer,
    Estop,
    EstopConfig,
    HighGLatch,
    parse_airy_imu,
    parse_relay_line,
    soft_estop_frame,
)

STILL = ((0.0, 0.0, 1.0), (0.0, 0.0, 0.0))
SPIKE = ((0.0, 0.0, 5.0), (0.0, 0.0, 0.0))


def _walk(n: int) -> list:
    """A synthetic 2 Hz gait: 1 g +- 1.2 g vertical, 1 rad/s yaw swing, at 200 Hz."""
    return [
        ((0.0, 0.0, 1.0 + 1.2 * math.sin(i / 200 * 2 * math.pi * 2)), (0.0, 0.0, math.sin(i / 50)))
        for i in range(n)
    ]


def test_walking_stays_below_threshold() -> None:
    latch = HighGLatch(EstopConfig())
    assert all(latch.update("rear_lidar", *s) is None for s in _walk(2000))
    assert latch.tripped is None


def test_short_spike_does_not_trip_but_a_run_does() -> None:
    config = EstopConfig()
    latch = HighGLatch(config)
    for _ in range(config.consecutive_samples - 1):
        assert latch.update("rear_lidar", *SPIKE) is None
    assert latch.update("rear_lidar", *STILL) is None
    events = [latch.update("rear_lidar", *SPIKE) for _ in range(config.consecutive_samples)]
    assert events[-1] is not None and events[-1]["first"]
    assert latch.tripped["accel_g"] == 5.0


def test_gyro_trips_and_can_be_turned_off() -> None:
    spin = ((0.0, 0.0, 1.0), (0.0, 0.0, 6.0))
    for limit, trips in ((4.0, True), (None, False)):
        latch = HighGLatch(EstopConfig(gyro_threshold_rad_s=limit, consecutive_samples=1))
        assert (latch.update("front_lidar", *spin) is not None) == trips


def test_latch_holds_until_reset(tmp_path: Path) -> None:
    config = EstopConfig(consecutive_samples=1, state_dir=str(tmp_path))
    estop = Estop(config)
    estop.on_sample("rear_lidar", *SPIKE)
    for sample in _walk(500):
        estop.on_sample("rear_lidar", *sample)
    assert estop.latch.tripped is not None
    # a restart comes back latched from the file
    assert Estop(config).latch.tripped is not None
    estop.reset()
    assert estop.latch.tripped is None and not Path(config.latch_file).exists()
    events = [json.loads(line)["event"] for line in Path(config.event_log).read_text().splitlines()]
    assert events.count("estop") == 1 and events[-1] == "reset"


def test_unarmed_trip_takes_no_action(tmp_path: Path, monkeypatch) -> None:
    estop = Estop(EstopConfig(consecutive_samples=1, state_dir=str(tmp_path)))
    monkeypatch.setattr(
        estop, "trip", lambda: (_ for _ in ()).throw(AssertionError("acted while unarmed"))
    )
    estop.on_sample("rear_lidar", *SPIKE)
    assert estop.latch.tripped is not None


def test_recorded_walk_and_stand_up_never_trip() -> None:
    """The 2026-11-20 session's body IMU (vendor /IMU, 200 Hz) around every peak: two stand-ups, ~85 s of RL walking."""
    latch = HighGLatch(EstopConfig())
    previous_t = None
    peak = 0.0
    with gzip.open(Path(__file__).parent / "walk_peaks_20261120.csv.gz", "rt") as f:
        for row in csv.DictReader(f):
            t = float(row["t"])
            # a gap in the excerpt is calm data: a still sample keeps runs from spanning it
            if previous_t is not None and t - previous_t > 0.006:
                latch.update("body", *STILL)
            previous_t = t
            accel = tuple(float(row[k]) for k in ("ax_g", "ay_g", "az_g"))
            peak = max(peak, math.sqrt(sum(v * v for v in accel)))
            assert (
                latch.update("body", accel, tuple(float(row[k]) for k in ("gx", "gy", "gz")))
                is None
            ), row
    assert peak > 4.4


def test_parse_airy_imu_scales_like_rs_driver() -> None:
    # accel fsr code 1 = +-4 g, gyro fsr code 1 = +-500 deg/s, as the M20's Airys report
    packet = bytearray(51)
    packet[:4] = bytes([0xAA, 0x55, 0x5A, 0x05])
    packet[14:38] = struct.pack(">3i3i", 0, 0, 8192, 0, 0, 3277)
    packet[43], packet[44] = 1, 1
    accel, gyro = parse_airy_imu(bytes(packet))
    assert accel == (0.0, 0.0, 1.0)
    assert abs(gyro[2] - math.radians(50.0)) < 1e-3
    assert parse_airy_imu(bytes(50)) is None


def test_soft_estop_frame_is_motion_state_two() -> None:
    frame = soft_estop_frame(7)
    magic, length, message_id, encoding = struct.unpack("<4sHHB", frame[:9])
    assert (magic.hex(), message_id, encoding, length) == ("eb91eb90", 7, 1, len(frame) - 16)
    device = json.loads(frame[16:])["PatrolDevice"]
    assert (device["Type"], device["Command"], device["Items"]) == (2, 22, {"MotionParam": 2})


def test_body_imu_sniffer_decodes_the_vendor_sample_once() -> None:
    """A real yesense_node SPDP announcement and Imu DATA captured on AOS loopback (2026-11-20)."""
    here = Path(__file__).parent
    spdp, imu = (
        (here / "vendor_yesense_spdp_rtps.bin").read_bytes(),
        (here / "vendor_imu_rtps.bin").read_bytes(),
    )
    sniffer = BodyImuSniffer("yesense")
    assert list(sniffer.feed(imu)) == []
    assert list(sniffer.feed(spdp)) == []
    [(accel_g, gyro)] = list(sniffer.feed(imu))
    assert abs(math.sqrt(sum(v * v for v in accel_g)) - 1.0) < 0.02
    assert max(abs(v) for v in gyro) < 0.05
    assert list(sniffer.feed(imu)) == []


def test_relay_lines_become_samples_in_g() -> None:
    accel, gyro = parse_relay_line(b"0.0 0.0 9.80665 0.1 -0.2 0.3")
    assert accel == pytest.approx((0.0, 0.0, 1.0)) and gyro == pytest.approx((0.1, -0.2, 0.3))
    assert parse_relay_line(b"garbage") is None
    assert parse_relay_line(b"1 2 3") is None


def test_the_relay_feeds_the_latch_and_a_dead_relay_is_restarted_up_to_its_limit(tmp_path) -> None:
    lines = "".join("0 0 49.0 0 0 0\n" for _ in range(6))
    config = EstopConfig(
        state_dir=str(tmp_path),
        body_imu_relay_cmd=["python3", "-c", f"import sys; sys.stdout.write({lines!r})"],
        body_imu_relay_restarts=1,
    )
    estop = Estop(config)
    relay = BodyImuRelay(config, estop)
    samples = []
    for _ in range(config.body_imu_relay_restarts + 2):
        fd = relay.start()
        if fd is None:
            break
        os.set_blocking(fd, True)
        while (batch := relay.read()) is not None:
            samples.extend(batch)
    # it ran 1 + body_imu_relay_restarts times, then gave up
    assert relay.starts == config.body_imu_relay_restarts + 1
    assert len(samples) == 6 * relay.starts
    for sample in samples[: config.consecutive_samples]:
        estop.on_sample("body_dimos", *sample)
    assert estop.latch.tripped is not None and estop.latch.tripped["stream"] == "body_dimos"
