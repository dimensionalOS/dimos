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

"""``dimos --replay-db <memory.db> run replay``: every recorded stream back on the bus, with the viewer.

``--replay-db`` must be a path to a recording; the ports are read from it when this
module is imported. Anywhere the value is not a file (the blueprint registry, workers,
tests) ``Replay`` has no class-level ports; instances add them from ``dataset``.
"""

from pathlib import Path
from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.memory.replay_module import replay_module, rerun_layout, stream_types_of
from dimos.visualization.vis_module import vis_module

_DATASET = global_config.replay_db if Path(global_config.replay_db).is_file() else ""

Replay = replay_module(_DATASET)


def _layout() -> Any:
    # Runs in the bridge worker at start(), where the global config is already applied.
    return rerun_layout(stream_types_of(global_config.replay_db))


replay = autoconnect(
    vis_module(global_config.viewer, rerun_config={"blueprint": _layout}),
    Replay.blueprint(dataset=_DATASET),
)
