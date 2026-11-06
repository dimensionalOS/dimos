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

from dimos.core.global_config import GlobalConfig
from dimos.navigation.go2.loop_closure.module import PGOVoxelMapperConfig, _pgo_config
from dimos.navigation.go2.loop_closure.pgo import PGOConfig


def test_loop_closure_is_on_for_a_real_robot_and_off_in_simulation() -> None:
    real = PGOVoxelMapperConfig(g=GlobalConfig())
    sim = PGOVoxelMapperConfig(g=GlobalConfig(simulation="dimsim"))

    assert _pgo_config(real).loop_closure is True
    assert _pgo_config(sim).loop_closure is False
    # the rest of the PGO settings are kept
    assert _pgo_config(sim).key_pose_delta_deg == sim.pgo.key_pose_delta_deg


def test_loop_closure_flag_overrides_the_simulation_default() -> None:
    sim_on = PGOVoxelMapperConfig(
        g=GlobalConfig(simulation="dimsim"), pgo=PGOConfig(loop_closure=True)
    )
    real_off = PGOVoxelMapperConfig(g=GlobalConfig(), pgo=PGOConfig(loop_closure=False))

    assert _pgo_config(sim_on) is sim_on.pgo
    assert _pgo_config(real_off) is real_off.pgo
