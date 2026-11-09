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

A frame is also all a consumer actually needs from these. Nothing collides with a camera; what
asks is asking where it is.
"""

from dimos.robot.assets.model import RobotModel, UsdModel


def frame_usd(frame_id: str, *, name: str) -> list[str]:
    """``[asset_root, usd]``: one link called ``frame_id``, under a layer called ``name``.

    The asset root is empty because the layer names no files. Saying so is not the same as
    saying "wherever you like": a manager rebases a subtree onto the root its module reported,
    and an empty one means there is nothing to rebase.
    """
    return ["", UsdModel.from_robot_model(RobotModel.from_root_link(frame_id, name=name)).usd]
