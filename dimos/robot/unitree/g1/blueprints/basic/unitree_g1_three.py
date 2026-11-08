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

"""Three G1s in one scene: a root one and two namespaced, served as a single USD."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.g1.usd_source import G1UsdSource
from dimos.robot.usd_manager import UsdManager
from dimos.simulation.scenes.three_g1 import THREE_G1_SCENE

# The frames the scene offers a robot to stand on; see scenes/three_g1/scene.usda.
MAIN, LEFT, RIGHT = "stand_main", "stand_left", "stand_right"

unitree_g1_three = autoconnect(
    # The manager holds the scene, not a robot. Each G1 is a mount: it answers with its own
    # description and the manager grafts it onto the frame it stands on, so what goes out on
    # render_file_request is one layer with three robots in it.
    UsdManager.blueprint(
        model=THREE_G1_SCENE,
        mounts={
            MAIN: G1UsdSource,
            LEFT: G1UsdSource.blueprint().namespace("left"),
            RIGHT: G1UsdSource.blueprint().namespace("right"),
        },
    ),
    # The root one takes no namespace, so its frames keep the names a controller already drives.
    G1UsdSource.blueprint(),
    G1UsdSource.blueprint().namespace("left"),
    G1UsdSource.blueprint().namespace("right"),
)
