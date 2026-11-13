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

"""MuJoCo geometry-only adapter shared by native render and ray queries."""

from dimos_lcm.sim_msgs import WorldState
import mujoco
import numpy as np

from dimos.sim2.geometry.poses import pose_array, poses_by_id


class MujocoGeometry:
    def __init__(self, model: mujoco.MjModel, revision: str) -> None:
        self.model = model
        self.data = mujoco.MjData(model)
        self.revision = revision
        self.body_ids = tuple(model.body(i).name for i in range(1, model.nbody))
        self.mocap_ids = np.asarray([model.body(n).mocapid[0] for n in self.body_ids], dtype=int)
        if model.nq or model.nv or model.nu or np.any(self.mocap_ids < 0):
            raise ValueError("query geometry must not contain dynamics coordinates")

    def select_surfaces(self, excluded_bodies: tuple[str, ...] = ()) -> None:
        """Optical consumers use visual surfaces instead of collision envelopes."""
        model = self.model
        excluded = {model.body(name).id for name in excluded_bodies}
        visual_bodies = {
            int(model.geom_bodyid[i])
            for i in range(model.ngeom)
            if model.geom_contype[i] == model.geom_conaffinity[i] == 0
        }
        for i in range(model.ngeom):
            body = int(model.geom_bodyid[i])
            hidden = (
                model.geom_group[i] >= 3
                or body in excluded
                or (
                    body != 0
                    and body in visual_bodies
                    and (model.geom_contype[i] or model.geom_conaffinity[i])
                )
            )
            model.geom_group[i] = 3 if hidden else 0

    def set_poses(self, sample: WorldState) -> None:
        poses = poses_by_id(sample)
        if sample.model_revision != self.revision or set(poses) != set(self.body_ids):
            raise ValueError("body pose sample does not match the resolved scene")
        values = np.asarray([pose_array(poses[n]) for n in self.body_ids])
        self.data.mocap_pos[self.mocap_ids] = values[:, :3]
        self.data.mocap_quat[self.mocap_ids] = values[:, [6, 3, 4, 5]]
        self.data.time = sample.header.time_ns / 1e9
        mujoco.mj_kinematics(self.model, self.data)
        mujoco.mj_comPos(self.model, self.data)
        mujoco.mj_camlight(self.model, self.data)
