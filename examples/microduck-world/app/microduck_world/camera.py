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

"""Use the robot's exported head-camera mount and its forward/left/up frame."""

import mujoco
import numpy as np

HEAD_CAMERA = "head_camera"


def configure_head_camera(spec: mujoco.MjSpec) -> None:
    """Convert the CAD camera site frame to MuJoCo's right/up/back camera axes."""
    camera = spec.camera(HEAD_CAMERA)
    mount = spec.site(HEAD_CAMERA)
    if camera is None or mount is None:
        raise ValueError("The robot must provide its head camera and mounting site")
    # Site: X forward, Y left, Z up. Camera: X right, Y up, -Z forward.
    optical = np.array([0.5, 0.5, -0.5, -0.5])
    rotation = np.empty(4)
    mujoco.mju_mulQuat(rotation, np.asarray(mount.quat), optical)
    camera.pos = list(mount.pos)
    camera.quat = rotation.tolist()


def configure_clipping(model: mujoco.MjModel) -> None:
    """Keep optical near/far distances metric as the world extent changes."""
    model.vis.map.znear = 0.005 / model.stat.extent
    model.vis.map.zfar = 30.0 / model.stat.extent
