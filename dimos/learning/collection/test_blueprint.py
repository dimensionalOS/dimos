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

"""Regression tests for collection blueprint module configuration."""

from dimos.learning.collection.blueprint import learning_collect_quest_piper_rerun
from dimos.learning.collection.episode_monitor import (
    EpisodeMonitorModule,
    EpisodeMonitorModuleConfig,
)
from dimos.learning.collection.recorder import CollectionRecorder, CollectionRecorderConfig


def test_piper_rerun_collection_uses_flat_typed_module_config_kwargs() -> None:
    """Dedicated collection kwargs must instantiate each module's config directly."""
    atoms = learning_collect_quest_piper_rerun.blueprints
    monitor_kwargs = next(atom.kwargs for atom in atoms if atom.module is EpisodeMonitorModule)
    recorder_kwargs = next(atom.kwargs for atom in atoms if atom.module is CollectionRecorder)

    monitor_config = EpisodeMonitorModuleConfig(**monitor_kwargs)
    recorder_config = CollectionRecorderConfig(**recorder_kwargs)

    assert monitor_config.default_task_label == "pick_and_place"
    assert recorder_config.task_label == "pick_and_place"
    assert "recordings/session_piper_rerun_" in str(recorder_config.db_path)
