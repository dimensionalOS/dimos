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

"""The xArm6 with its two calibrated cameras on tf, and nothing else.

``dimos run xarm6-cameras --xarmconnection.ip <ip> --realsensecamera.serial-number <wrist>
--chest_realsensecamera.serial-number <chest>`` brings up the planner with its viser view,
the coordinator and both RealSenses. The wrist camera hangs on ``link6`` and the chest camera
stands in the world, each from its hand-eye calibration (``cameras.py``), so both point clouds
land in the arm's frame and overlap on the table. The chest camera's topics and frames carry
the ``chest/`` prefix: ``chest/color_image``, ``chest/pointcloud``, ``chest/camera_link``.

This is the base to compose further work on: autoconnect it with the modules that consume
``pointcloud``, ``chest/pointcloud`` or the images, as ``grasp.py`` does for one camera.
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.manipulators.xarm.blueprints.basic import XARM_REMAPS, xarm_coordinator
from dimos.robot.manipulators.xarm.cameras import (
    CHEST_NAMESPACE,
    xarm6_chest_camera,
    xarm6_wrist_camera,
)
from dimos.robot.manipulators.xarm.config import XARM_ARMS, make_xarm_model_config
from dimos.visualization.vis_module import vis_module

_dof, _robot_type = XARM_ARMS["xarm6"]

xarm6_cameras = autoconnect(
    planner(
        model=make_xarm_model_config(
            _dof,
            robot_type=_robot_type,
            robot="arm",
            gripper_hardware_id="arm",
            base_pose=PoseStamped(frame_id="world"),
            # the planner publishes link6, which the wrist camera's mount hangs on
            tf_extra_links=[f"link{_dof}"],
        ),
        # fixed mounts, published with the arm's links at the same stamp
        static_transforms=[xarm6_wrist_camera(), xarm6_chest_camera()],
        visualization={"backend": "viser"},
    ),
    xarm_coordinator("xarm6"),
    RealSenseCamera.blueprint(enable_pointcloud=True),
    # tf stays global so the chest camera's own imager frames join the one tree
    RealSenseCamera.blueprint(enable_pointcloud=True).namespace(CHEST_NAMESPACE, expose={"tf"}),
    vis_module(global_config.viewer),
).remappings(XARM_REMAPS)
