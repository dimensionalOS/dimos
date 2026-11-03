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

A new map, where the locations get named::

    dtk run go2-sim-nav-named-locations --record walk.db

Back on that map, relocalized against the recording's map_cloud, with the locations named in it::

    dtk run go2-sim-nav-named-locations --localmaprelocalization.map-file walk.db
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
        save_locations_to="named_locations.db",
        tf_interval=1.0,
    ),
).global_config(n_workers=11)
