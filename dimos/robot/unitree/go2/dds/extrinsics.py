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

"""Go2 L1 lidar extrinsic (base_link <- lidar) and camera mount.

The L1 is mounted nearly upside-down: raw lidar +z points at the floor. The
official Unitree config (go2_l1_lidar.yaml) encodes this as base->lidar
``[0.28216, 0, -0.02467, roll=0, pitch=2.88, yaw=0]`` plus a separate
``rotate_yaw_bias`` (~-123 deg, calibrated to the front-leg position). EXT_R below
is that flip, leveled to the averaged ground normal over several stationary windows
(floor tilt ~1-2 deg, floor below the robot) and yawed so the map heading matches
the trajectory. Validated against the official "default imu reading"
[yaw -57.9, pitch -8.1, roll -167.3] (agrees to a few degrees).
"""

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos_generated.std_msgs.msg import Header
import numpy as np
from scipy.spatial.transform import Rotation

from dimos.msgs.geometry import quaternion_from_matrix

# base_link <- lidar rotation (lidar points -> base frame: p_base = EXT_R @ p_lidar + EXT_T)
EXT_R = np.array(
    [
        [0.504486, -0.843018, 0.186588],
        [-0.853668, -0.519391, -0.038544],
        [0.129405, -0.139840, -0.981682],
    ],
    dtype=np.float64,
)
EXT_T = np.array([0.28216, 0.0, -0.02467], dtype=np.float64)

# base_link -> camera_optical (from dimos GO2 connection BASE_TO_OPTICAL):
# translate 0.3m forward, then rotate into the optical frame.
CAM_T = np.array([0.30, 0.0, 0.0], dtype=np.float64)
CAM_Q = np.array([-0.5, 0.5, -0.5, 0.5], dtype=np.float64)  # xyzw

# Normalize the measured six-decimal rotation with the same SciPy conversion
# as the previous numeric helper, then publish generated frame-labelled values.
LIDAR_TO_BASE = TransformStamped(
    header=Header(frame_id="base_link", stamp=Time(sec=0, nanosec=0)),
    child_frame_id="lidar",
    transform=Transform(
        translation=Vector3(x=EXT_T[0], y=EXT_T[1], z=EXT_T[2]),
        rotation=quaternion_from_matrix(Rotation.from_matrix(EXT_R).as_matrix()),
    ),
)
BASE_TO_CAMERA = TransformStamped(
    header=Header(frame_id="base_link", stamp=Time(sec=0, nanosec=0)),
    child_frame_id="camera_optical",
    transform=Transform(
        translation=Vector3(x=CAM_T[0], y=CAM_T[1], z=CAM_T[2]),
        rotation=Quaternion(x=CAM_Q[0], y=CAM_Q[1], z=CAM_Q[2], w=CAM_Q[3]),
    ),
)
