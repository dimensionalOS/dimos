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

"""Official OpenArm v2.0 bimanual robot model."""

from __future__ import annotations

from dimos.robot.assets.model import RobotModel
from dimos.robot.assets.source import RobotDescriptionSource

OPENARM_DESCRIPTION_URL = "https://github.com/enactic/openarm_description"
OPENARM_DESCRIPTION_REF = "1fba2cbc05001f05b4514120b70130b4ac06f409"

OPENARM_DESCRIPTION_SOURCE = RobotDescriptionSource(
    url=OPENARM_DESCRIPTION_URL,
    ref=OPENARM_DESCRIPTION_REF,
)
OPENARM_DESCRIPTION_ROOT = OPENARM_DESCRIPTION_SOURCE / "."
OPENARM_BIMANUAL_XACRO = (
    OPENARM_DESCRIPTION_SOURCE
    / "assets"
    / "robot"
    / "openarm_v2.0"
    / "urdf"
    / "openarm_v20.urdf.xacro"
)

OPENARM_BIMANUAL_MODEL = RobotModel.from_file(
    OPENARM_BIMANUAL_XACRO,
    package_paths={"openarm_description": OPENARM_DESCRIPTION_ROOT},
    xacro_args={
        "robot_preset": "default_bimanual",
        "emit_grasp_frame": "true",
    },
)
