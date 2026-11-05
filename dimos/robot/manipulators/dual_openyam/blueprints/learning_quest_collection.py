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

"""Quest-guided dual OpenYAM collection."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.recorder import collect
from dimos.robot.manipulators.dual_openyam.blueprints.collection import (
    dual_openyam_collection_cameras,
    dual_openyam_collection_profile,
    teleop_webxr_dual_openyam_collection,
)

dual_openyam_quest_collection = autoconnect(
    collect(profile=dual_openyam_collection_profile("dual-openyam-quest")),
    EpisodeMonitorModule.blueprint(instance_name="episodes"),
    teleop_webxr_dual_openyam_collection,
    *dual_openyam_collection_cameras(),
)
