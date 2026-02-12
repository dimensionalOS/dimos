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

"""Cascaded GO2 blueprints split into focused modules."""

import lazy_loader as lazy

__getattr__, __dir__, __all__ = lazy.attach(
    __name__,
    submod_attrs={
        "agentic._common_agentic": ["_common_agentic"],
        "agentic.unitree_go2_agentic": ["unitree_go2_agentic"],
        "agentic.unitree_go2_agentic_huggingface": ["unitree_go2_agentic_huggingface"],
        "agentic.unitree_go2_agentic_mcp": ["unitree_go2_agentic_mcp"],
        "agentic.unitree_go2_agentic_ollama": ["unitree_go2_agentic_ollama"],
        "agentic.unitree_go2_temporal_memory": ["unitree_go2_temporal_memory"],
        "basic.unitree_go2_basic": ["_linux", "_mac", "unitree_go2_basic"],
        "smart._with_jpeg": ["_with_jpeglcm"],
        "smart.unitree_go2": ["unitree_go2"],
        "smart.unitree_go2_detection": ["unitree_go2_detection"],
        "smart.unitree_go2_ros": ["unitree_go2_ros"],
        "smart.unitree_go2_spatial": ["unitree_go2_spatial"],
        "smart.unitree_go2_vlm_stream_test": ["unitree_go2_vlm_stream_test"],
    },
)
