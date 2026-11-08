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

"""The G1 serving its own description, for a UsdManager that mounts it into a scene."""

from __future__ import annotations

from pathlib import Path

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.robot.unitree.g1.model import G1_MODEL, G1_USD


class G1UsdSource(Module):
    """Answers ``get_usd`` with the G1's layer, in this instance's own frames.

    A robot in a scene is a mount: the UsdManager holds the scene and asks each mounted module
    for its subtree. Deploying this namespaced is what makes three G1s three G1s rather than one
    drawn three times -- the namespace is already on every frame this module stamps, so the
    manager's graft keeps them apart with nothing extra said here.
    """

    @rpc
    def get_usd(self) -> list[str]:
        """``[asset_root, usd]``: the vendored G1 layer and the directory its meshes sit in.

        The root is this robot's assets directory rather than a path baked at build time: that
        one is written on a build machine, whose paths mean nothing on the robot.
        """
        return [str(Path(G1_USD).parent), G1_MODEL.usd]
