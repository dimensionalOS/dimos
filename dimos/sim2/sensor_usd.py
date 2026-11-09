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

"""What a sensor answers `get_usd` with when nobody has measured its case.

Everything mountable describes itself in USD, so a UsdManager can graft it onto the frame it is
bolted to and a viewer can place it from tf. A sensor that someone has measured says so with its
real dimensions -- the MID-360 is a 65 x 65 x 60 mm box, and `mid360_usd` is that box. For the
rest, the honest description is **the frame and nothing else**: a camera's optical frame is a pose,
not a shape, and a layer claiming a case nobody measured would put a made-up box in the physics and
on the screen.

A frame is also most of what a consumer needs from these. Nothing collides with a camera; what
asks is asking where it is -- and, for a camera, what it can see from there, which is a USD
camera's job to say.
"""

import math

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.robot.assets.model import RobotModel, UsdModel

# A 35 mm film back: USD states optics as a focal length against an aperture, and one of the two
# has to be chosen before the other means anything.
_APERTURE_MM = 24.0


def frame_usd(frame_id: str, *, name: str) -> list[str]:
    """``[asset_root, usd]``: one link called ``frame_id``, under a layer called ``name``.

    The asset root is empty because the layer names no files. Saying so is not the same as
    saying "wherever you like": a manager rebases a subtree onto the root its module reported,
    and an empty one means there is nothing to rebase.
    """
    return ["", UsdModel.from_robot_model(RobotModel.from_root_link(frame_id, name=name)).usd]


def camera_usd(frame_id: str, *, name: str, fovy: float) -> list[str]:
    """``[asset_root, usd]``: the camera's optical frame, with a USD camera on it saying its field of view.

    The frame is the pose and the camera prim is the optics, which is where USD keeps them. A
    consumer that wants to draw what this camera can see has it from the layer rather than from
    this module's config, and nothing has to be told twice.

    USD states optics as a focal length against an aperture rather than as an angle, so the
    vertical aperture is fixed at a film back's 24 mm and the focal length is whatever gives
    ``fovy``. It is also turned to face the way USD faces: a camera prim looks down -z with +y
    up, and an optical frame is z forward with y down, which is the same 180 degrees about x
    that a MuJoCo camera needs.
    """
    from dimos.robot.assets.usd import _identifier, _layer_from, _link_prim_paths, _matrix, _pxr

    pxr = _pxr()
    root, usd = frame_usd(frame_id, name=name)
    layer = _layer_from(pxr, usd, name)
    stage = pxr.Usd.Stage.Open(layer)
    mount = _link_prim_paths(stage)[frame_id]
    camera = pxr.UsdGeom.Camera.Define(
        stage, pxr.Sdf.Path(mount).AppendChild(_identifier(pxr, f"{frame_id}_optics"))
    )
    camera.CreateVerticalApertureAttr(_APERTURE_MM)
    camera.CreateHorizontalApertureAttr(_APERTURE_MM)
    camera.CreateFocalLengthAttr(_APERTURE_MM / 2 / math.tan(math.radians(fovy) / 2))
    facing = np.eye(4)
    facing[:3, :3] = Rotation.from_euler("x", math.pi).as_matrix()
    camera.AddTransformOp().Set(_matrix(pxr, facing))
    return [root, str(layer.ExportToString())]
