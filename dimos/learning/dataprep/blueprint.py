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

"""Dataset-build blueprints.

Wraps `DataPrepModule` so users can run::

    dimos run learning-dataprep
    dimos run learning-dataprep -o dataprepmodule.source=data/recordings/foo.db \\
                                -o dataprepmodule.output.path=data/datasets/foo

The defaults below target the included pickplace_001 demo. Episodes are
always segmented from the recording (the `episode_status` stream or
explicit `ranges`) — we never collapse a session into a single episode.
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.learning.dataprep.core import (
    EpisodeExtractor,
    OutputConfig,
    StreamField,
    SyncConfig,
)
from dimos.learning.dataprep.module import DataPrepModule

learning_dataprep = autoconnect(
    DataPrepModule.blueprint(
        source="data/recordings/pickplace_001.db",
        episodes=EpisodeExtractor(),
        observation={
            "image": StreamField(stream="color_image", field="data"),
            "joint_state": StreamField(stream="joint_state", field="position"),
        },
        action={
            "joint_target": StreamField(stream="joint_state", field="position"),
        },
        sync=SyncConfig(anchor="image", rate_hz=14.0, tolerance_ms=80.0),
        output=OutputConfig(
            format="lerobot",
            path="data/datasets/pickplace_001",
            metadata={"fps": 14, "robot": "xarm7", "default_task_label": "pick_and_place"},
        ),
        auto_run=True,
    ),
).transports({})


__all__ = ["learning_dataprep"]
