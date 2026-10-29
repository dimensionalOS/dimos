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

"""Run robosuite's unmodified Panda Lift task through ordinary DimOS streams."""

from robosuite.environments.base import MujocoEnv

from dimos.sim2.module import SimulationModule
from dimos.sim2.rendering import make_environment


def panda_lift() -> MujocoEnv:
    """The factory executes inside the simulator worker, on its owning thread."""
    return make_environment(
        "Lift",
        robots="Panda",
        has_renderer=False,
        has_offscreen_renderer=False,
        use_camera_obs=False,
        control_freq=20,
        horizon=1000,
    )


robosuite_lift = SimulationModule.blueprint(environment=panda_lift)
