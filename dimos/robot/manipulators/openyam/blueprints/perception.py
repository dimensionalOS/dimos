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

"""OpenYAM pick-and-place blueprints.

``openyam_pickplace`` provides the planning + skills half of the perception
stack (PickAndPlaceModule: pick/place/scan/go_home skills, meshcat planning
visualization). Compose it with a coordinator at run time:

    dimos run openyam-pickplace coordinator-openyam-can

The camera + ObjectSceneRegistrationModule half (which feeds the ``objects``
input that pick() consumes) mirrors
``dimos/robot/manipulators/xarm/blueprints/perception.py`` and lands once the
kit camera's driver story is settled. It needs:

- a DepthCameraHardware module publishing color_image/depth_image/
  camera_info/tf (RealSense pattern; no macOS RGB-D driver exists in-tree),
- ``base_frame_id=OPENYAM_WRIST_LINK`` and a hand-eye calibrated
  ``base_transform`` (placeholder below; calibrate with
  ``dimos cameracalibrate`` + an AprilTag rig),
- ObjectSceneRegistrationModule(target_frame="world").
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.manipulators.openyam.config import (
    OPENYAM_WRIST_LINK,
    make_openyam_model_config,
)

# PLACEHOLDER hand-eye transform (wrist link -> camera mount). Calibrate on
# hardware before trusting perception-derived grasps; the xArm equivalent
# (XARM_PERCEPTION_CAMERA_TRANSFORM) was measured on its physical rig.
OPENYAM_CAMERA_TRANSFORM = Transform(
    translation=Vector3(x=0.05, y=0.0, z=0.02),
    rotation=Quaternion(0.0, 0.0, 0.0, 1.0),  # xyzw
)

openyam_pickplace = autoconnect(
    PickAndPlaceModule.blueprint(
        robots=[
            make_openyam_model_config(
                name="arm",
                tf_extra_links=[OPENYAM_WRIST_LINK],
            )
        ],
        planning_timeout=10.0,
        visualization={"backend": "meshcat"},
        floor_z=-0.02,
    ),
)
