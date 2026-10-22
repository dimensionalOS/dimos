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

"""Piper position servos and a gripper measured as metres between fingers."""

from dimos.sim2.sensors.camera.spec import Camera
from dimos.sim2.spec import Joint, RobotConfig
from dimos.utils.data import LfsPath

PIPER_HOME = (
    0.793,
    1.568186214614724,
    -1.0290351975897356,
    0.0008456548489068756,
    0.9771515619106422,
    -0.13286819850920156,
)


def piper_definition(
    *,
    source: str = "arm",
    gripper: bool = True,
) -> RobotConfig:
    return RobotConfig(
        model=LfsPath("piper/piper.xml"),
        root_body="base_link",
        joints=(
            *(
                Joint(f"{source}/joint{i}", f"joint{i}", f"joint{i}", mode="position")
                for i in range(1, 7)
            ),
            *(
                (
                    Joint(
                        f"{source}/gripper",
                        "joint7",
                        "gripper",
                        mode="position",
                        scale=0.5,
                        ctrl_scale=0.5,
                        lower=0.0,
                        upper=0.07,
                    ),
                )
                if gripper
                else ()
            ),
        ),
        # The asset's moving spotlight shadow crashes Apple's offscreen GL worker.
        sensors=(Camera("wrist_camera", "wrist_camera", shadows=False),),
    )
