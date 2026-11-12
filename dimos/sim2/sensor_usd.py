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


import math

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.robot.assets.model import RobotModel, UsdModel

_APERTURE_MM = 24.0


def frame_usd(frame_id: str, *, name: str) -> list[str]:
    return ["", UsdModel.from_robot_model(RobotModel.from_root_link(frame_id, name=name)).usd]


def camera_usd(frame_id: str, *, name: str, fovy: float) -> list[str]:
    from dimos.robot.assets.usd_utils.frames import link_prim_paths
    from dimos.robot.assets.usd_utils.pxr import layer_from, load_pxr, prim_identifier, usd_matrix

    pxr = load_pxr()
    root, usd = frame_usd(frame_id, name=name)
    layer = layer_from(pxr, usd, name, root)
    stage = pxr.Usd.Stage.Open(layer)
    mount = link_prim_paths(stage)[frame_id]
    camera = pxr.UsdGeom.Camera.Define(
        stage, pxr.Sdf.Path(mount).AppendChild(prim_identifier(pxr, f"{frame_id}_optics"))
    )
    camera.CreateVerticalApertureAttr(_APERTURE_MM)
    camera.CreateHorizontalApertureAttr(_APERTURE_MM)
    camera.CreateFocalLengthAttr(_APERTURE_MM / 2 / math.tan(math.radians(fovy) / 2))
    facing = np.eye(4)
    facing[:3, :3] = Rotation.from_euler("x", math.pi).as_matrix()
    camera.AddTransformOp().Set(usd_matrix(pxr, facing))
    return [root, str(layer.ExportToString())]
