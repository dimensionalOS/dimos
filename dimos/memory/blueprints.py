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

``--replay-db`` is a recording path or a bare dataset name (data/, LFS), like ``--replay``.
The ports are read from it when this module is imported by ``dimos run replay``. Elsewhere
(the blueprint registry, workers, tests) ``Replay`` has no class-level ports; instances add
them from ``dataset``.
"""

import sys
from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.memory.replay_module import (
    dataset_path,
    recorded_rerun_config,
    replay_module,
    rerun_layout,
    stream_types_of,
)
from dimos.visualization.vis_module import vis_module

# Resolve a bare dataset name only when this process is `dimos ... run replay`. Importing
# the module anywhere else (blueprint registry, tests, forkserver workers) must not touch a
# database or pull from LFS; workers get the resolved path through the blueprint kwargs.
_DATASET = dataset_path(global_config.replay_db, explicit="replay" in sys.argv[1:])

Replay = replay_module(_DATASET)


def _layout() -> Any:
    # Fallback when the recording did not come from a registered blueprint.
    # Runs in the bridge worker at start(), where the global config is already applied.
    return rerun_layout(stream_types_of(global_config.replay_db))


replay = autoconnect(
    vis_module(
        global_config.viewer,
        rerun_config={"blueprint": _layout, **recorded_rerun_config(_DATASET)},
    ),
    Replay.blueprint(dataset=_DATASET),
)
