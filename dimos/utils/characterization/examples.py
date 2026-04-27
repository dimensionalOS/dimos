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

"""Sample recipes to smoke-test the harness. Defining a recipe takes ~5 lines.

Reference these via ``--recipe``::

    python -m dimos.utils.characterization.scripts.run_session --recipe dimos.utils.characterization.examples:step_vx_1
"""

from __future__ import annotations

from dimos.utils.characterization.recipes import TestRecipe, ramp, step

# A small forward-velocity step: 0 → 1.0 m/s for 3 seconds.
step_vx_1 = TestRecipe(
    name="step_vx_1.0",
    test_type="step",
    duration_s=3.0,
    signal_fn=step(amplitude=1.0, channel="vx"),
)

# A modest angular-velocity step: 0 → 1.0 rad/s for 3 seconds.
step_wz_1 = TestRecipe(
    name="step_wz_1.0",
    test_type="step",
    duration_s=3.0,
    signal_fn=step(amplitude=1.0, channel="wz"),
)

# A slow ramp to 1.5 m/s — useful for finding the saturation point.
ramp_vx_0_to_1p5 = TestRecipe(
    name="ramp_vx_0_to_1.5",
    test_type="ramp",
    duration_s=10.0,
    signal_fn=ramp(start=0.0, end=1.5, duration=10.0, channel="vx"),
)


__all__ = ["ramp_vx_0_to_1p5", "step_vx_1", "step_wz_1"]
