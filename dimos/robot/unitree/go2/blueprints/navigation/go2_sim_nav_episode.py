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

"""One benchmark case driven against the Go2 navigation stack in the simulated world."""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.navigation.sim_eval.driver import EpisodeDriver
from dimos.robot.unitree.go2.blueprints.navigation.go2_sim_nav import go2_sim_nav

go2_sim_nav_episode = autoconnect(go2_sim_nav, EpisodeDriver.blueprint()).global_config(
    n_workers=10
)
