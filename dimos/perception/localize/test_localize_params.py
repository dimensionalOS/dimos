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

from __future__ import annotations

from dataclasses import replace

from dimos.perception.localize.dandetect import DanDetector
from dimos.perception.localize.localize_params import (
    DUAL_YAM_PARAMS,
    G1_PARAMS,
    GO2_PARAMS,
    MANIPULATION_PARAMS,
    NAVIGATION_PARAMS,
    PRESETS,
    XARM_PARAMS,
    parse_params,
)


def test_presets_and_detector_score_shifts() -> None:
    assert XARM_PARAMS == MANIPULATION_PARAMS
    assert GO2_PARAMS == NAVIGATION_PARAMS
    assert DUAL_YAM_PARAMS == replace(MANIPULATION_PARAMS, min_camera_range_m=0.07)
    assert G1_PARAMS == replace(NAVIGATION_PARAMS, min_points=30)
    for name in ("manipulation", "navigation", "xarm", "dual_yam", "go2", "g1"):
        assert name in PRESETS

    gdino = DanDetector(detector="gdino")
    omdet = DanDetector(detector="omdet")

    # Both manipulation and navigation presets shift proportionally for gdino and omdet.
    manip_gdino = gdino.shifted(*parse_params("manipulation", MANIPULATION_PARAMS))
    assert (manip_gdino.candidate_floor, manip_gdino.accept_score) == (0.35, 0.55)

    nav_gdino = gdino.shifted(*parse_params("navigation", MANIPULATION_PARAMS))
    assert (nav_gdino.candidate_floor, nav_gdino.accept_score) == (0.28, 0.47)

    manip_omdet = omdet.shifted(*parse_params("xarm", MANIPULATION_PARAMS))
    assert (manip_omdet.candidate_floor, manip_omdet.accept_score) == (0.15, 0.30)

    nav_omdet = omdet.shifted(*parse_params("go2", MANIPULATION_PARAMS))
    assert (nav_omdet.candidate_floor, nav_omdet.accept_score) == (0.08, 0.22)

    # Explicit field overrides win over model shifts.
    overridden = gdino.shifted(
        *parse_params('{"base": "navigation", "accept_score": 0.50}', MANIPULATION_PARAMS)
    )
    assert (overridden.candidate_floor, overridden.accept_score) == (0.28, 0.50)
