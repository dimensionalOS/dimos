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

"""Spot-specific storage policy tests.

The default image codec is selected by payload type, so stream overrides are
the observable contract that distinguishes lossless grayscale from depth.
"""

from __future__ import annotations

from dimos.experimental.robot.bosdyn.spot.config import CAMERA_STREAM_SUFFIXES
from dimos.experimental.robot.bosdyn.spot.recorder import GRAYSCALE_CODEC, SpotRecorderConfig


def test_spot_recorder_keeps_grayscale_streams_on_lossless_lz4(tmp_path) -> None:
    config = SpotRecorderConfig(db_path=tmp_path / "spot.db")

    expected = {f"grayscale_image_{suffix}": GRAYSCALE_CODEC for suffix in CAMERA_STREAM_SUFFIXES}
    assert config.stream_codecs == expected


def test_spot_recorder_leaves_depth_streams_on_default_image_codec(tmp_path) -> None:
    config = SpotRecorderConfig(db_path=tmp_path / "spot.db")

    depth_streams = {f"depth_image_{suffix}" for suffix in CAMERA_STREAM_SUFFIXES}
    assert depth_streams.isdisjoint(config.stream_codecs)
