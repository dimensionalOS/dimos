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

"""Alfred's vision-only navigation stack fed from a recording instead of the robot.

    dimos run alfred-replay --alfredreplay.db-path ~/datasets/alfred/drive.db

``AlfredReplay`` publishes the recorded camera, IMU and wheel odometry streams on the
same names ``RealSenseCamera`` and ``AlfredHighLevel`` use live, so everything
downstream is ``vis_nav`` unchanged.
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.diy.alfred.blueprints.vis_nav import vis_nav
from dimos.robot.diy.alfred.replay import AlfredReplay

alfred_replay = autoconnect(
    AlfredReplay.blueprint(),
    vis_nav,
).global_config(n_workers=10, robot_model="alfred")
