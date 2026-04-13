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

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.perception.experimental.temporal_memory.temporal_memory import (
    TemporalMemory,
    TemporalMemoryConfig,
)
from dimos.robot.unitree.go2.blueprints.agentic.unitree_go2_agentic import unitree_go2_agentic

# This module is imported lazily by `get_by_name()` in the CLI run command,
# AFTER global_config.update() has applied CLI flags like --new-memory.
unitree_go2_temporal_memory = autoconnect(
    unitree_go2_agentic,
    TemporalMemory.blueprint(config=TemporalMemoryConfig(new_memory=global_config.new_memory)),
)

__all__ = ["unitree_go2_temporal_memory"]
