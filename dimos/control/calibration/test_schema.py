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

from typing import Any

import pytest

from dimos.control.calibration._calibration_test_helpers import CREATED, calibration
from dimos.control.calibration.schema import TRACKING_CHECK, ArmConfigPatch, merge_patch

FLOAT = {"limit_mm": 2.0}
TRACKING = {"rest_rms_deg": {"shoulder": 0.2}}
RETRACKED = {"rest_rms_deg": {"shoulder": 0.1}}


@pytest.mark.parametrize(
    "patch,checks",
    [
        ({"gains": {"shoulder": {"kp": 60.0, "kd": 5.0}}}, {"float": FLOAT}),
        (
            {"gains": {"shoulder": {"kp": 80.0, "kd": 5.0}}},
            {"float": FLOAT, TRACKING_CHECK: TRACKING},
        ),
        (
            {"gains": {"shoulder": {"kp": 60.0, "kd": 5.0}}, "checks": {TRACKING_CHECK: RETRACKED}},
            {"float": FLOAT, TRACKING_CHECK: RETRACKED},
        ),
    ],
)
def test_the_stored_tracking_check_stays_only_while_the_gains_do(
    patch: dict[str, Any], checks: dict[str, Any]
) -> None:
    stored = calibration(checks={"float": FLOAT, TRACKING_CHECK: TRACKING})
    merged = merge_patch(stored, ArmConfigPatch.model_validate(patch), created=CREATED)
    assert merged.checks == checks
