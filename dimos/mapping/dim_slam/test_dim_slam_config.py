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

"""DimSlamConfig refuses fields it does not have, so an old or misspelled setting fails loudly."""

from pydantic import ValidationError
import pytest

from dimos.mapping.dim_slam.dim_slam import (
    DimSlamConfig,
    ImuConfig,
    LidarConfig,
    MapConfig,
    SourceConfig,
)


@pytest.mark.parametrize(
    ("field", "hint"),
    [("publish_rate", "publish_every"), ("replay_buffer_seconds", "carried forward")],
)
def test_a_removed_field_names_what_replaced_it(field: str, hint: str) -> None:
    with pytest.raises(ValidationError, match=f"{field} was removed in dimSLAM v1.0.*{hint}"):
        DimSlamConfig(camera_mode="stereo", **{field: 1.0})


def test_an_unknown_top_level_field_is_refused() -> None:
    with pytest.raises(ValidationError, match="publish_hz"):
        DimSlamConfig(camera_mode="stereo", publish_hz=10.0)


@pytest.mark.parametrize(
    "build",
    [
        lambda: ImuConfig(frame_id="imu", gyro_noise=0.1),
        lambda: LidarConfig(frame_id="lidar", voxel=0.1),
        lambda: SourceConfig(parent_frame_id="a", child_frame_id="b", variance=1.0),
        lambda: MapConfig(size=1.0),
        lambda: DimSlamConfig(lidars=[{"frame_id": "lidar", "keep_every": 3, "bogus": 1}]),
    ],
)
def test_an_unknown_nested_field_is_refused(build) -> None:
    with pytest.raises(ValidationError, match="Extra inputs are not permitted"):
        build()


def test_a_valid_config_still_builds() -> None:
    config = DimSlamConfig(
        lidars=[LidarConfig(frame_id="front_lidar")],
        imus=[ImuConfig(frame_id="front_lidar", gyro_variance=1e-6)],
        publish_every=4,
    )
    assert config.to_config_dict()["camera_mode"] == ""
