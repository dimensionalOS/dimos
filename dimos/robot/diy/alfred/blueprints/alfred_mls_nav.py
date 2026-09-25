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

"""Alfred running MLS planning off the D455 alone, with no Mid-360.

dimos run alfred-mls-nav
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.diy.alfred.blueprints.alfred_hardware import _alfred_hardware
from dimos.robot.diy.alfred.blueprints.vis_nav import vis_nav

alfred_mls_nav = autoconnect(_alfred_hardware, vis_nav()).global_config(
    n_workers=11, robot_model="alfred"
)
