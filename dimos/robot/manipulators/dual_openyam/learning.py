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
from dimos.imitation.collection.recorder import declare_python_recorder
from dimos.robot.manipulators.dual_openyam.learning_profile import (
    ABC_JOINTS as ABC_JOINTS,
    DUAL_OPENYAM_ABC_IO as DUAL_OPENYAM_ABC_IO,
    DUAL_OPENYAM_CAMERA_SHAPE as DUAL_OPENYAM_CAMERA_SHAPE,
    DUAL_OPENYAM_FPS as DUAL_OPENYAM_FPS,
    DUAL_OPENYAM_LEROBOT_IO as DUAL_OPENYAM_LEROBOT_IO,
    DUAL_OPENYAM_SIM_CAMERA_SHAPE as DUAL_OPENYAM_SIM_CAMERA_SHAPE,
    DUAL_OPENYAM_SIM_CAPTURE_FPS as DUAL_OPENYAM_SIM_CAPTURE_FPS,
    DUAL_OPENYAM_SIM_FPS as DUAL_OPENYAM_SIM_FPS,
    DUAL_OPENYAM_SIM_TASK as DUAL_OPENYAM_SIM_TASK,
    DUAL_OPENYAM_TWO_WRIST_IO as DUAL_OPENYAM_TWO_WRIST_IO,
)

DualOpenYamQuestRecorder = declare_recorder(
    "DualOpenYamQuestRecorder",
    __name__,
    DUAL_OPENYAM_TWO_WRIST_IO,
)

DualOpenYamSimRecorder = declare_python_recorder(
    "DualOpenYamSimRecorder", __name__, DUAL_OPENYAM_LEROBOT_IO
)
