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

"""The smart go2 with an authored cockpit: video left (2/3), costmap+pose
right. Edit the layout (swap Row for Col, change shares, drop a panel) and
rerun - the browser rearranges and dropped channels leave the wire entirely.

Separate file on purpose: cockpit() needs the [web] extra at import time,
and `unitree-go2` itself must stay importable without it.
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2
from dimos.web.cockpit import Col, Map2D, Row, Video, cockpit

unitree_go2_cockpit = autoconnect(
    unitree_go2,
    cockpit(
        layout=Row(
            Video("color_image"),
            Col(Map2D(costmap="global_costmap", pose="odom")),
            shares=[2, 1],
        ),
    ),
).global_config(n_workers=11, robot_model="unitree_go2")
