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

"""Types shared by hardware adapter specifications."""

from collections.abc import Sequence
from dataclasses import dataclass


@dataclass
class JointLimits:
    """Limits in adapter joint order and each joint's local coordinate.

    ``None`` represents a limit the adapter does not know. Arrays still contain
    one entry per adapter joint so values can be resolved by joint index.
    """

    position_lower: Sequence[float | None]
    position_upper: Sequence[float | None]
    velocity_max: Sequence[float | None]
