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

"""Continuous DimOS planar control exposed through Habitat's task-action seam."""

from habitat.core.embodied_task import SimulatorTaskAction
from habitat.core.registry import registry
import numpy as np
import quaternion

from .motion import integrate_planar


@registry.register_task_action
class DimosPlanarAction(SimulatorTaskAction):
    """Apply one bounded planar period while keeping Habitat in charge of stepping."""

    name = "DIMOS_PLANAR"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.requested_position = None
        self.accepted_position = None

    def reset(self, *args, **kwargs):
        self.requested_position = None
        self.accepted_position = None

    def step(self, linear_x, linear_y, angular_z, period_seconds, **_kwargs):
        state = self._sim.get_agent_state()
        start = np.asarray(state.position, dtype=np.float64)
        requested, rotation_xyzw = integrate_planar(
            start,
            [state.rotation.x, state.rotation.y, state.rotation.z, state.rotation.w],
            linear_x,
            linear_y,
            angular_z,
            period_seconds,
        )
        accepted = np.asarray(self._sim.pathfinder.try_step(start, requested))
        rotation = quaternion.quaternion(
            rotation_xyzw[3],
            rotation_xyzw[0],
            rotation_xyzw[1],
            rotation_xyzw[2],
        )
        observations = self._sim.get_observations_at(
            accepted.tolist(),
            rotation,
            keep_agent_at_new_pose=True,
        )
        if observations is None:
            raise RuntimeError("Habitat rejected a collision-filtered planar pose")
        self.requested_position = requested
        self.accepted_position = accepted
        return observations
