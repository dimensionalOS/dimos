# Copyright 2025-2026 Dimensional Inc.
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

"""Render the loaded MuJoCo meshes in Viser, without a second robot model."""

import numpy as np
from scipy.spatial.transform import Rotation


class R1ProMujocoViewer:
    def __init__(self, plant, host, port):
        import viser

        self.server = viser.ViserServer(host=host, port=port)
        self.server.scene.add_grid("/ground", width=8.0, height=8.0, plane="xy")
        self.meshes = []
        m = plant.model
        for i in range(m.ngeom):
            if m.geom_group[i] != 1 or m.geom_type[i] != plant.mj.mjtGeom.mjGEOM_MESH:
                continue
            mesh = m.geom_dataid[i]
            v, f = m.mesh_vertadr[mesh], m.mesh_faceadr[mesh]
            handle = self.server.scene.add_mesh_simple(
                f"/r1pro/{i}",
                m.mesh_vert[v : v + m.mesh_vertnum[mesh]].copy(),
                m.mesh_face[f : f + m.mesh_facenum[mesh]].copy(),
                color=tuple((255 * m.geom_rgba[i, :3]).astype(int)),
            )
            self.meshes.append((i, handle))
        self.update(plant)

    def update(self, plant):
        d = plant.data
        with self.server.atomic():
            for i, handle in self.meshes:
                handle.position = d.geom_xpos[i].copy()
                handle.wxyz = np.roll(
                    Rotation.from_matrix(d.geom_xmat[i].reshape(3, 3)).as_quat(), 1
                )

    def stop(self):
        self.server.stop()
