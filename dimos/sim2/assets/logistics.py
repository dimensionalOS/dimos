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

from pathlib import Path

from dimos.core.hardware_module import Mounts
from dimos.robot.assets.model import UsdModel
from dimos.sim2.scene import SimScene, SimSceneConfig

LOGISTICS = Path(__file__).parent / "logistics.usda"
# The one robot spot, its root body's height above the floor.
GO2_SPOT = (0.0, 0.0, 0.35)


class LogisticsConfig(SimSceneConfig):
    usd: UsdModel = UsdModel.from_file(LOGISTICS).with_frames(
        {"go2": ("logistics", GO2_SPOT, (0.0, 0.0, 0.0))}
    )


class Logistics(SimScene):
    """A logistics room with one spot, `go2`."""

    config: LogisticsConfig
    mounts = Mounts("logistics/go2")
