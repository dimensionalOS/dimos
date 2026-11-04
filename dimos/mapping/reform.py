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

"""The loop-closure warp config shared by RayTracingVoxelMap and MLSPlannerNative."""

from __future__ import annotations

from pydantic import BaseModel, ConfigDict


class ReformConfig(BaseModel):
    """How a module warps itself through a pose-graph optimizer's loop closures (its pose_graph and loop_closure_event inputs)."""

    model_config = ConfigDict(extra="forbid")

    # warp through each loop_closure_event; off ignores pose_graph and loop_closure_event
    enabled: bool = False
    # spatial blend width between keyframe corrections, m (0 = twice the mean keyframe spacing)
    blend_sigma_m: float = 0.0
    # temporal blend width, s, keeping a revisited place's passes apart (0 = twice the mean keyframe interval)
    blend_time_sigma_s: float = 5.0
    # a keyframe moving its surroundings less than this waits for a later closure (half a voxel moves none)
    min_shift_m: float = 0.04
    # a warp blends the one keyframe nearest each cell of this size, m (0 = the blend width, negative = every keyframe)
    thin_cell_m: float = 0.0
    # blend widths past which a keyframe's weight counts as nothing (exp(-3**2 / 2), about 1%)
    reach_sigmas: float = 3.0
