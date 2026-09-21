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
over keyboard teleop right. Edit the layout (swap Row for Col, change
shares, drop a panel) and rerun - the browser rearranges and dropped
channels leave the wire entirely.

Separate file on purpose: cockpit() needs the [web] extra at import time,
and `unitree-go2` itself must stay importable without it.
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2
from dimos.robot.unitree.go2.operator_controls import (
    Go2OperatorControls,
    OperatorCommand,
    OperatorResult,
    OperatorState,
)
from dimos.web.cockpit import Channel, Col, Map2D, Row, Stats, Teleop, Video, cockpit

unitree_go2_cockpit = autoconnect(
    unitree_go2,
    Go2OperatorControls.blueprint(),
    cockpit(
        pages=[Stats()],
        channels=[
            Channel(
                "go2_operator_command",
                OperatorCommand,
                dir="tx",
                encoding="go2.operator.json.v1",
                publish="shared",
                max_hz=2.0,
            ),
            Channel("go2_operator_state", OperatorState, max_hz=1.0),
            Channel("go2_operator_result", OperatorResult, max_hz=10.0),
        ],
        layout=Row(
            Video("color_image"),
            Col(
                Map2D(path="path", click="clicked_point", stop="stop_movement"),
                Teleop(),
                shares=[3, 1],
            ),
            shares=[2, 1],
        ),
    ),
).global_config(n_workers=12, robot_model="unitree_go2")
