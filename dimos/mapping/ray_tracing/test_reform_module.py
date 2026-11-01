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

from pathlib import Path

from dimos.mapping.ray_tracing.reform_module import (
    ReformRayTracingVoxelMap,
    ReformRayTracingVoxelMapConfig,
)

REPO = Path(__file__).resolve().parents[3]


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


def test_the_warp_is_the_planners() -> None:
    # Each crate builds alone under nix, so the ray tracer keeps its own copy.
    ours = REPO / "dimos/mapping/ray_tracing/rust/src/reform.rs"
    planners = REPO / "dimos/navigation/global_planner/mls_planner/rust/src/reform.rs"
    assert ours.read_text() == planners.read_text()
