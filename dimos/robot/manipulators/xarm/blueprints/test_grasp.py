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

from dimos.hardware.sensors.camera.realsense.camera import RealSenseCameraConfig
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.manipulators.xarm.blueprints.grasp import (
    XARM_WRIST_CAMERA_TRANSFORM,
    XArmWristCameraTf,
    xarm_grasp,
)


def test_wrist_camera_edge_joins_the_camera_subtree_to_the_arm() -> None:
    """RealSenseCamera publishes only below camera_link.

    Without a parent for it nothing resolves into world and every cloud the
    camera produces is silently unusable, so the edge has to be composed in and
    its parent has to be a link the planning model actually publishes.
    """
    manipulation = next(
        atom.kwargs
        for atom in xarm_grasp.active_blueprints
        if issubclass(atom.module, ManipulationModule)
    )

    assert any(issubclass(atom.module, XArmWristCameraTf) for atom in xarm_grasp.active_blueprints)
    assert XARM_WRIST_CAMERA_TRANSFORM.child_frame_id == RealSenseCameraConfig().frame_id
    assert XARM_WRIST_CAMERA_TRANSFORM.frame_id in manipulation["model"].tf_extra_links
