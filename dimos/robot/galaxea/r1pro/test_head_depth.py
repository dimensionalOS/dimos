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

"""The R1's head depth blueprint: its frames and its wiring."""

from dimos.perception.depth2depth_cloud.module import Depth2DepthCloud
from dimos.robot.galaxea.r1pro.head_depth import r1pro_head_depth
from dimos.robot.galaxea.r1pro.lio import BASE_FRAME


def _kwargs(blueprint):
    return next(
        atom for atom in blueprint.active_blueprints if atom.module is Depth2DepthCloud
    ).kwargs


def test_a_height_band_is_measured_from_base_link() -> None:
    kwargs = _kwargs(r1pro_head_depth(min_height_m=-0.15, max_height_m=0.35))
    assert (kwargs["height_frame"], kwargs["min_height_m"], kwargs["max_height_m"]) == (
        BASE_FRAME,
        -0.15,
        0.35,
    )


def test_the_camera_and_the_lidar_are_the_connections_and_point_lios_streams() -> None:
    blueprint = r1pro_head_depth()
    key = blueprint._instance_key(Depth2DepthCloud)
    remaps = blueprint.remapping_map
    assert remaps[(key, "image")] == "head_left_color"
    assert remaps[(key, "camera_info")] == "head_left_info"
    assert remaps[(key, "lidar")] == "lidar"
    assert remaps[(key, "depth_cloud")] == "head_cloud"
