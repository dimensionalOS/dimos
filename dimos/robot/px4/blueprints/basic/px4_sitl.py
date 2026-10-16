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

"""``px4`` against PX4 SITL (``make px4_sitl gz_x500``).

The RC enable switch is faked: ``sitl_enable(True)`` in ``dimos shell`` turns it on.

Usage:
    dimos run px4-sitl
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.px4.blueprints.basic.px4 import px4_control, px4_visualization
from dimos.robot.px4.config import SITL_MAV_URL

px4_sitl = autoconnect(px4_visualization(), px4_control(mav_url=SITL_MAV_URL, sitl=True))
