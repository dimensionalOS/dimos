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

"""One benchmark case driven against the M20 navigation stack in the simulated world."""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.navigation.bench.driver import EpisodeDriver
from dimos.robot.deeprobotics.m20.blueprints.m20_sim_nav import m20_sim_nav
from dimos.simulation.m20_sim.world import SimM20World

m20_sim_nav_episode = autoconnect(
    m20_sim_nav, SimM20World.blueprint(wait_for_scene=True), EpisodeDriver.blueprint()
).global_config(n_workers=9)
