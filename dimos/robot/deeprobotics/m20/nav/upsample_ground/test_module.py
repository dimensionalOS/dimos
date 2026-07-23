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

from collections.abc import Iterator

from pydantic import ValidationError
import pytest

from dimos.robot.deeprobotics.m20.nav.upsample_ground.module import (
    UpsampleGround,
    UpsampleGroundConfig,
)


@pytest.fixture
def upsample_ground() -> Iterator[UpsampleGround]:
    module = UpsampleGround()
    try:
        yield module
    finally:
        module.stop()


def test_upsample_ground_declares_synchronization_streams(
    upsample_ground: UpsampleGround,
) -> None:
    module = upsample_ground

    assert set(module.inputs) == {"current_frame", "global_map", "odometry"}
    assert set(module.outputs) == {"global_map_upsample_ground"}


def test_upsample_ground_uses_independent_queue_capacities() -> None:
    config = UpsampleGroundConfig()

    assert config.current_frame_queue_size == 20
    assert config.global_map_queue_size == 20
    assert config.odometry_queue_size == 200


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("current_frame_queue_size", 0),
        ("global_map_queue_size", 0),
        ("odometry_queue_size", 0),
        ("max_future_sync_delta_s", -0.001),
    ],
)
def test_upsample_ground_rejects_invalid_sync_config(field: str, value: int | float) -> None:
    with pytest.raises(ValidationError):
        UpsampleGroundConfig(**{field: value})
