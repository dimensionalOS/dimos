#!/usr/bin/env python3
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

from dimos.msgs.sensor_msgs.ImuInfo import ImuInfo, _packed_fingerprint


def test_lcm_encode_decode() -> None:
    """LCM encode/decode preserves every field."""
    original = ImuInfo(
        gyro_noise_density=2.0e-4,
        gyro_random_walk=1.0e-5,
        accel_noise_density=1.8e-3,
        accel_random_walk=1.0e-4,
        frequency=400.0,
        frame_id="camera_accel_optical_frame",
        ts=123.456,
    )

    decoded = ImuInfo.lcm_decode(original.lcm_encode())

    assert decoded.gyro_noise_density == original.gyro_noise_density
    assert decoded.gyro_random_walk == original.gyro_random_walk
    assert decoded.accel_noise_density == original.accel_noise_density
    assert decoded.accel_random_walk == original.accel_random_walk
    assert decoded.frequency == original.frequency
    assert decoded.frame_id == original.frame_id
    assert abs(decoded.ts - original.ts) < 1e-6


def test_fingerprint_pinned() -> None:
    """The wire fingerprint is shared with the C++ bindings in dimSLAM.

    A change here means the schema changed: regenerate every consumer from the
    .lcm definition in the module docstring, or wire compatibility silently dies.
    """
    assert _packed_fingerprint().hex() == "e6aa5563f2a33280"


def test_with_ts_and_frame() -> None:
    info = ImuInfo(accel_random_walk=1.0e-4, frame_id="a", ts=1.0)
    restamped = info.with_ts(2.0)
    assert restamped.ts == 2.0
    assert restamped.accel_random_walk == 1.0e-4
    assert restamped.frame_id == "a"
    reframed = info.with_frame_id("b")
    assert reframed.frame_id == "b"
    assert info.frame_id == "a"
