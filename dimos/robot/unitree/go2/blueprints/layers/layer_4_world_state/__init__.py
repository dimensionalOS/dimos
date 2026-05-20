#!/usr/bin/env python3
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

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.perception.spatial_perception import SpatialMemory
from dimos.robot.unitree.go2.blueprints.layers.layer_4_world_state.semantic_temporal_map import (
    _Go2SemanticTemporalMap,
)
from dimos.robot.unitree.go2.blueprints.layers.layer_4_world_state.structured_world_state import (
    _Go2StructuredWorldState,
)

_go2_spatial_world_state = autoconnect(
    SpatialMemory.blueprint(),
    _Go2SemanticTemporalMap.blueprint(),
    _Go2StructuredWorldState.blueprint(),
)


def _go2_temporal_memory_world_state() -> Blueprint:
    """Layer 4: temporal memory, imported lazily after CLI config is resolved."""
    from dimos.core.global_config import global_config
    from dimos.perception.experimental.temporal_memory.temporal_memory import (
        TemporalMemory,
        TemporalMemoryConfig,
    )

    return autoconnect(
        TemporalMemory.blueprint(
            config=TemporalMemoryConfig(new_memory=global_config.new_memory)
        ),
    )


__all__ = ["_go2_spatial_world_state", "_go2_temporal_memory_world_state"]
