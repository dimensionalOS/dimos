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

"""Five ACT picks followed by physical tray delivery through the house."""

from dimos.constants import RECORDINGS_DIR
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.galaxea.r1pro.home_coordinator import R1ProHomeCoordinator
from dimos.robot.galaxea.r1pro.home_sim import R1ProHomeDemo, R1ProHomeSim
from dimos.robot.galaxea.r1pro.navigation_blueprint import build_r1pro_packing_navigation

r1pro_home_sim = autoconnect(
    build_r1pro_packing_navigation(
        # Replaced with a unique generated scene by the sim/coordinator build step.
        scene_path=RECORDINGS_DIR / "r1pro-home-sim" / "scene.xml",
        artifact=str(RECORDINGS_DIR / "r1pro-act-task" / "policy-packing-augmented"),
        simulator=R1ProHomeSim,
        coordinator_type=R1ProHomeCoordinator,
        prepare_scene_on_build=True,
    ),
    R1ProHomeDemo.blueprint(),
).global_config(transport="zenoh", viewer="none", simulation="mujoco", n_workers=4)
