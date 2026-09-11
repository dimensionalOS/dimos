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

"""Collection modules and compatible exports of the standalone policy profiles."""

from dimos.imitation.collection.native_recorder import declare_recorder
from dimos.robot.manipulators.openyam.learning_profile import (
    OPENYAM_CAMERA_SHAPE as OPENYAM_CAMERA_SHAPE,
    OPENYAM_FPS as OPENYAM_FPS,
    OPENYAM_QUEST_IO as OPENYAM_QUEST_IO,
    OPENYAM_TEACH_IO as OPENYAM_TEACH_IO,
)

OpenYamQuestRecorder = declare_recorder(
    "OpenYamQuestRecorder",
    __name__,
    OPENYAM_QUEST_IO,
)

OpenYamTeachRecorder = declare_recorder(
    "OpenYamTeachRecorder",
    __name__,
    OPENYAM_TEACH_IO,
)
