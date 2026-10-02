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

from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_coordinator import r1pro_control
from dimos.robot.galaxea.r1pro.connection import R1ProConnection, R1ProConnectionConfig


def _connection_kwargs(**overrides: bool) -> dict:
    blueprint = r1pro_control(**overrides)
    return next(a for a in blueprint.active_blueprints if a.module is R1ProConnection).kwargs


def test_unset_knobs_keep_the_connection_defaults() -> None:
    assert _connection_kwargs() == {}


def test_knobs_reach_the_connection() -> None:
    assert _connection_kwargs(publish_odom=False, enable_wrist_color=False) == {
        "publish_odom": False,
        "enable_wrist_color": False,
    }


def test_color_passes_every_camera_frame_by_default() -> None:
    # The head cameras arrive at ~28 Hz; a lower cap would drop frames the depth needs.
    assert R1ProConnectionConfig().color_publish_hz >= 28
