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

"""xArm6/7 native servos, SDK gripper units and wrist RGB-D camera."""

from dimos.sim2.sensors.camera.spec import Camera
from dimos.sim2.spec import Joint, RobotConfig
from dimos.utils.data import LfsPath

XARM7_HOME = (0.0, -0.247, 0.0, 0.909, 0.0, 1.15644, 0.0)
XARM6_HOME = (0.0, -0.247, -0.909, 0.0, 1.15644, 0.0)


def xarm_definition(
    dof: int,
    *,
    source: str = "arm",
    gripper: bool = True,
    camera_pointcloud: bool = False,
) -> RobotConfig:
    if dof not in (6, 7):
        raise ValueError("xArm simulation supports 6 or 7 arm joints")
    return RobotConfig(
        model=LfsPath(f"xarm{dof}/xarm{dof}.xml"),
        root_body="link_base",
        joints=(
            *(
                Joint(f"{source}/joint{i}", f"joint{i}", f"act{i}", mode="position")
                for i in range(1, dof + 1)
            ),
            *(
                (
                    Joint(
                        f"{source}/gripper",
                        "left_driver_joint",
                        "gripper",
                        mode="position",
                        scale=-0.001,
                        offset=0.85,
                        ctrl_scale=-0.3,
                        ctrl_offset=255.0,
                        lower=0.0,
                        upper=850.0,
                    ),
                )
                if gripper
                else ()
            ),
        ),
        sensors=(Camera("wrist_camera", "wrist_camera", pointcloud=camera_pointcloud),),
    )
