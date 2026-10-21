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

from dimos.core.coordination.blueprints import autoconnect
from dimos.hosted.tags import GPU_MEM_GB
from dimos.mapping.experimental.mesh import MESH_ENTITY, MeshColours, MeshModule
from dimos.mapping.ray_tracing.viz import MAP_REGIONS_ENTITY
from dimos.navigation.global_planner.mls_planner.viz import SURFACE_MAP_ENTITY
from dimos.robot.unitree.go2.dds.blueprints import (
    VIEW_Z_BAND,
    VIEWER_GLOBAL_CONFIG,
    _go2_nav_stack,
    nav_viewer,
)
from dimos.robot.unitree.go2.nav_3d_config import voxel_size

# go2-nav-viewer meshing map_regions on this machine's GPU; the mesh stands in for
# the region cloud and surface map, which stay tickable.
_viewer = nav_viewer(
    extra_topics=("mesh",),
    visual_override={MESH_ENTITY: MeshColours(alpha=1.0)},
    hidden=(MAP_REGIONS_ENTITY, SURFACE_MAP_ENTITY),
)
go2_nav_viewer_mesh = autoconnect(
    _viewer, MeshModule.blueprint(voxel_size=voxel_size, z_band=VIEW_Z_BAND)
).global_config(n_workers=4, **VIEWER_GLOBAL_CONFIG)

# go2-nav on the go2 Host, the mesher on the GPU Host with the most memory, the
# viewer here. Run with `dimos run go2-nav-hosted`. This machine's Host reaches the
# robot's by HOST__CONNECT=tcp/<robot-ip>:7447, or by scouting where the LAN carries multicast.
go2_nav_hosted = autoconnect(
    _go2_nav_stack.hosted(tags={"go2"}),
    MeshModule.blueprint(voxel_size=voxel_size, z_band=VIEW_Z_BAND).hosted(
        tags={"gpu"}, prefer=GPU_MEM_GB
    ),
    _viewer,
).global_config(transport="zenoh", robot_model="unitree_go2")
