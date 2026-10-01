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

from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_replay import (
    Go2Replay,
    unitree_go2_replay,
)
from dimos.visualization.rerun.bridge import RerunBridgeModule


def test_replay_uses_the_go2_viewer_config() -> None:
    bridges = [a for a in unitree_go2_replay.blueprints if a.module is RerunBridgeModule]
    assert len(bridges) == 1
    for key in ("blueprint", "visual_override", "static"):
        assert bridges[0].kwargs[key] is rerun_config[key]


def test_importing_the_blueprint_opens_no_recording() -> None:
    assert Go2Replay.__annotations__ == {}
    assert Go2Replay.stream_types == {}
