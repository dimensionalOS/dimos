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

"""``dimos replay <memory.db>``: the recording's streams back on the bus, with the viewer."""

import os
from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.memory.replay_module import replay_module, rerun_layout
from dimos.visualization.vis_module import vis_module

_DATASET = os.environ.get("REPLAY_DB", "")

Replay = replay_module(_DATASET, os.environ.get("REPLAY_TOPICS", "*"))


def _layout() -> Any:
    return rerun_layout(Replay.stream_types)


replay = autoconnect(
    vis_module(global_config.viewer, rerun_config={"blueprint": _layout}),
    Replay.blueprint(dataset=_DATASET),
)
