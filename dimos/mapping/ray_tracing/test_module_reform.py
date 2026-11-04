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
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap, RayTracingVoxelMapConfig
from dimos.mapping.reform import ReformConfig
from dimos.navigation.global_planner.mls_planner.mls_planner_native import (
    MLSPlannerNative,
    MLSPlannerNativeConfig,
)


def test_both_modules_take_the_closure_inputs_with_reform_off_by_default():
    for module, config in [
        (RayTracingVoxelMap, RayTracingVoxelMapConfig()),
        (MLSPlannerNative, MLSPlannerNativeConfig()),
    ]:
        assert {"pose_graph", "loop_closure_event"} <= set(module.__annotations__)
        assert config.to_config_dict()["reform"] == ReformConfig().model_dump()
        assert config.reform.enabled is False


def test_the_planner_keeps_its_fallback_and_limits_by_default():
    config = MLSPlannerNativeConfig()
    assert config.require_full_route is False
    assert config.reform_rekey_radius_m == 3.0
    assert config.max_region_columns == 10_000_000
    assert config.max_closing_box_cells == 250_000_000
