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


"""go2-sim-nav with named map locations that outlive the run, kept by the premap relocalizer.

A new map, where the locations get named (the run records into its own ``memory.db``)::

    dtk run go2-sim-nav-named-locations --record sqlite --record-topics 'tf,global_map,named_locations'

Build and name a map out of that recording in the Map Editor, then come back to it::

    dtk run go2-sim-nav-named-locations --localmaprelocalization.map-file <that recording>.db

The stack's own cloud is on ``global_map``, so loading a recording that has no named map in it needs
``--localmaprelocalization.cloud-stream global_map``.
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.go2.blueprints.navigation.go2_sim_nav import go2_sim_nav
from dimos.robot.unitree.go2.nav_3d_config import relocalization

go2_sim_nav_named_locations = autoconnect(
    go2_sim_nav,
    # loaded_map is a never-drop channel under zenoh, so no republish
    relocalization(
        republish_loaded_map=0.0,
        new_map=True,
        tf_interval=1.0,
    ),
).global_config(n_workers=11)
