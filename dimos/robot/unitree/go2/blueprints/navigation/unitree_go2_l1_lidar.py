#!/usr/bin/env python3
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

"""Go2 onboard (L1) lidar re-expressed in the Mid-360 frame.

Sibling to ``unitree_go2_nav_3d``. Instead of the physical Mid-360 + Point-LIO,
this runs ``SimpleLidar``: it pulls the Go2's onboard lidar over WebRTC, undoes
the onboard world transform back to ``base_link``, then applies the static
``base_link -> mid360_link`` transform (inverted) so the cloud looks like it was
captured by the Mid-360. Downstream consumers calibrated for the Mid-360 mount
therefore see the L1 lidar in the frame they expect.
"""

import numpy as np

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.lidar.pointlio.mount_correction import base_to_frame_matrix
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config
from dimos.robot.unitree.go2.config import mid360_urdf_path
from dimos.robot.unitree.go2.simple_lidar import SimpleLidar
from dimos.robot.urdf_loader import UrdfLoader
from dimos.visualization.vis_module import vis_module

MID360_FRAME = "mid360_link"

# The Go2 lidar lands in base_link after SimpleLidar undoes the onboard transform.
# Re-express it in the Mid-360 frame: inv(base_link -> mid360_link) maps base_link
# coordinates into mid360_link, and the cloud is stamped mid360_link to match.
_base_to_mid360 = base_to_frame_matrix(
    UrdfLoader(name="go2_mid360", model_path=mid360_urdf_path), MID360_FRAME
)
_lidar_in_mid360 = [float(value) for value in np.linalg.inv(_base_to_mid360).flatten()]

unitree_go2_l1_lidar = autoconnect(
    vis_module(viewer_backend=global_config.viewer, rerun_config=rerun_config),
    SimpleLidar.blueprint(transform=_lidar_in_mid360, output_frame=MID360_FRAME),
).global_config(n_workers=4, robot_model="unitree_go2")
