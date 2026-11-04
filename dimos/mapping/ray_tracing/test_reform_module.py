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

from dimos.mapping.ray_tracing.reform_module import (
    ReformRayTracingVoxelMap,
    ReformRayTracingVoxelMapConfig,
)


def test_the_module_declares_the_closure_inputs() -> None:
    ports = ReformRayTracingVoxelMap.__annotations__
    assert {"pose_graph", "loop_closure_event"} <= set(ports)
    config = ReformRayTracingVoxelMapConfig()
    assert config.executable.endswith("reform_ray_tracing")
    assert {
        "reform_blend_sigma_m",
        "reform_blend_time_sigma_s",
        "reform_min_shift_m",
    } <= set(config.to_config_dict())
