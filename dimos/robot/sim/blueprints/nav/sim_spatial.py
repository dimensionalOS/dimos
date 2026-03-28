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

"""DimSim spatial blueprint — nav + spatial memory."""

from dimos.core.blueprints import autoconnect
from dimos.perception.spatial_perception import spatial_memory
from dimos.robot.sim.blueprints.nav.sim_nav import sim_nav

sim_spatial = autoconnect(
    sim_nav,
    spatial_memory(),
).global_config(n_workers=8)

__all__ = ["sim_spatial"]
