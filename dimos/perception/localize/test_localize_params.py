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
from dimos.perception.localize.tools.tool_localize import (
    DUAL_YAM_PARAMS,
    G1_PARAMS,
    GO2_PARAMS,
    PRESETS,
    XARM_PARAMS,
    parse_params,
)
from dimos.perception.localize.types import LocalizeParams


def test_presets_and_detector_score_shifts() -> None:
    assert XARM_PARAMS == LocalizeParams()
    assert DUAL_YAM_PARAMS == replace(LocalizeParams(), min_camera_range_m=0.07)
    assert G1_PARAMS == replace(GO2_PARAMS, min_points=30)
    for name in ("xarm", "dual_yam", "go2", "g1"):
        assert name in PRESETS

    gdino = DanDetector(detector="gdino")
    omdet = DanDetector(detector="omdet")

    # Both desk and walking-robot params shift proportionally for gdino and omdet.
    xarm_gdino = gdino.shifted(*parse_params("xarm", XARM_PARAMS))
    assert (xarm_gdino.candidate_floor, xarm_gdino.accept_score) == (0.35, 0.55)

    go2_gdino = gdino.shifted(*parse_params("go2", XARM_PARAMS))
    assert (go2_gdino.candidate_floor, go2_gdino.accept_score) == (0.28, 0.47)

    xarm_omdet = omdet.shifted(*parse_params("xarm", XARM_PARAMS))
    assert (xarm_omdet.candidate_floor, xarm_omdet.accept_score) == (0.15, 0.30)

    go2_omdet = omdet.shifted(*parse_params("go2", XARM_PARAMS))
    assert (go2_omdet.candidate_floor, go2_omdet.accept_score) == (0.08, 0.22)

    # Explicit field overrides win over model shifts.
    overridden = gdino.shifted(*parse_params('{"base": "go2", "accept_score": 0.50}', XARM_PARAMS))
    assert (overridden.candidate_floor, overridden.accept_score) == (0.28, 0.50)
