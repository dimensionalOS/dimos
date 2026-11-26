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

"""The example world's USD and the frames that accept attached hardware."""

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.hardware_module import Mounts
from dimos.robot.assets.model import UsdModel
from dimos.sim2.hardware import SimHardwareConfig, SimHardwareModule


class BasicSceneConfig(SimHardwareConfig):
    usd: UsdModel = UsdModel.from_file(DIMOS_PROJECT_ROOT / "assets/sim2_basic/scene.usda")


class BasicScene(SimHardwareModule):
    config: BasicSceneConfig
    mounts = Mounts(
        *(f"world/assets/base/tool/{kind}_mount" for kind in ("camera", "lidar", "imu"))
    )
