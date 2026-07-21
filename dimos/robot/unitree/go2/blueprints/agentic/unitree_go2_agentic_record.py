#!/usr/bin/env python3
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

"""Go2 agentic blueprint with agent-triggered episode recording.

Layers three modules onto `unitree_go2_agentic`:
- `Go2CollectionRecorder` — records camera/odom/cmd_vel/status to a session DB.
- `EpisodeMonitorModule` — the start/save/discard state machine (also satisfies
  `EpisodeControlSpec` for the skill below).
- `EpisodeRecordingSkillContainer` — exposes start/stop/discard recording to the
  agent over MCP.

Recording is opt-in: the default `unitree-go2-agentic` blueprint is unchanged.
Export the recorded session DB afterwards with `dimos dataprep build`.
"""

from dimos.agents.skills.episode_recording import EpisodeRecordingSkillContainer
from dimos.core.coordination.blueprints import autoconnect
from dimos.learning.collection.blueprint import _session_db
from dimos.learning.collection.episode_monitor import EpisodeMonitorModule
from dimos.learning.collection.go2_recorder import Go2CollectionRecorder
from dimos.robot.unitree.go2.blueprints.agentic.unitree_go2_agentic import unitree_go2_agentic

unitree_go2_agentic_record = autoconnect(
    unitree_go2_agentic,
    Go2CollectionRecorder.blueprint(db_path=_session_db("go2")),
    EpisodeMonitorModule.blueprint(),
    EpisodeRecordingSkillContainer.blueprint(),
)
