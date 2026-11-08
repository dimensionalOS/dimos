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

"""The G1's model, as the USD vendored beside it."""

from __future__ import annotations

from pathlib import Path

from dimos.robot.assets.model import UsdModel

# The one description: dimos poses tf from it, dim-controller draws it, and MuJoCo simulates it.
# Converted once from the menagerie MJCF -- see scripts/mjcf_to_usd.py for how, and why by hand.
G1_USD = Path(__file__).parent / "assets" / "g1.usda"
G1_MODEL = UsdModel.from_file(G1_USD)
