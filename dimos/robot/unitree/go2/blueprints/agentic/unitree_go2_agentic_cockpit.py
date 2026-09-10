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

"""The agentic go2 with an authored cockpit: video left, costmap+pose over
keyboard teleop in the middle, the agent chat right (the humancli
conversation on McpClient's human_input/agent/agent_idle streams, with the
composer's push-to-talk mic feeding VoiceInput -> the shared Whisper
pipeline), plus a full-page camera view on its own tab. The cockpit
replaces the legacy :5555 WebInput page, so that module is disabled here
rather than started alongside.

Separate file on purpose: cockpit() needs the [web] extra at import time,
and `unitree-go2-agentic` itself must stay importable without it.
"""

from dimos.agents.voice_input import VoiceInput
from dimos.agents.web_human_input import WebInput
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.go2.blueprints.agentic.unitree_go2_agentic import unitree_go2_agentic
from dimos.stream.audio.decode import ffmpeg_requirement
from dimos.web.cockpit import Chat, Col, Map2D, Row, Teleop, Video, cockpit

unitree_go2_agentic_cockpit = (
    autoconnect(
        unitree_go2_agentic,
        cockpit(
            layout=Row(
                Video("color_image", title="Front camera"),
                Col(
                    Map2D(costmap="global_costmap", pose="odom", title="Map"),
                    Teleop(title="Keyboard teleop"),
                    shares=[3, 1],
                ),
                Chat(title="Agent chat"),
                shares=[2, 1, 1],
            ),
            pages=[Video("color_image", title="Front camera")],
        ),
        VoiceInput.blueprint(),
    )
    .disabled_modules(WebInput)
    .requirements(ffmpeg_requirement)
    .global_config(n_workers=9, robot_model="unitree_go2")
)
