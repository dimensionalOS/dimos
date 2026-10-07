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
from dimos.mapping.experimental.mesh import MESH_ENTITY, MeshColours, MeshModule
from dimos.mapping.ray_tracing.viz import MAP_REGIONS_ENTITY
from dimos.navigation.global_planner.mls_planner.viz import SURFACE_MAP_ENTITY
from dimos.robot.unitree.go2.dds.blueprints import (
    VIEW_Z_BAND,
    VIEWER_GLOBAL_CONFIG,
    go2_nav_stack,
    nav_viewer,
)
from dimos.robot.unitree.go2.nav_3d_config import voxel_size

# go2-dds-nav-viewer meshing map_regions on this machine's GPU; the mesh stands in for
# the region cloud and surface map, which stay tickable.
_viewer_mesh = autoconnect(
    nav_viewer(
        extra_topics=("mesh",),
        visual_override={MESH_ENTITY: MeshColours(alpha=0.4)},
        hidden=(MAP_REGIONS_ENTITY, SURFACE_MAP_ENTITY),
    ),
    MeshModule.blueprint(voxel_size=voxel_size, z_band=VIEW_Z_BAND),
)
go2_dds_nav_viewer_mesh = _viewer_mesh.global_config(n_workers=4, **VIEWER_GLOBAL_CONFIG)

# go2-dds-nav on the robot's Host plus the meshing viewer here, deployed by
# `dimos host deploy go2-dds-nav-hosted`. Each Host's daemon is its zenoh router and every
# process is its client, so mesh stays on this machine and only subscribed topics cross.
go2_dds_nav_hosted = autoconnect(
    go2_nav_stack(session=None).hosted(tags={"go2"}),
    _viewer_mesh,
).global_config(transport="zenoh", n_workers=11, robot_model="unitree_go2")
