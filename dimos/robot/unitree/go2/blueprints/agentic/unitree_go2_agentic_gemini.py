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

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.skills.gemini_speak_skill import GeminiSpeakSkill
from dimos.agents.skills.speak_skill import SpeakSkill
from dimos.core.coordination.blueprints import autoconnect
from dimos.experimental.security_demo.security_module import SecurityModule
from dimos.perception.spatial_perception import SpatialMemory
from dimos.robot.unitree.go2.blueprints.agentic._common_agentic import _common_agentic
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2_spatial import unitree_go2_spatial

# Disabled for a no-OpenAI / non-CUDA setup:
#  - SecurityModule needs a CUDA GPU (EdgeTAM) -> won't boot on Apple Silicon.
#  - SpeakSkill (TTS) hardcodes OpenAI -> needs OPENAI_API_KEY; replaced below
#    by GeminiSpeakSkill, which uses the Gemini TTS API (reuses GOOGLE_API_KEY,
#    no GPU, cross-platform so it can also run on the robot).
# SpatialMemory is re-declared to use remote Gemini embeddings instead of the
# local CLIP model (which fails on Apple CoreML); the later atom overrides the
# one inside unitree_go2_spatial.
unitree_go2_agentic_gemini = autoconnect(
    unitree_go2_spatial,
    SpatialMemory.blueprint(embedding_model="gemini", embedding_dimensions=768),
    McpServer.blueprint(),
    McpClient.blueprint(model="google_genai:gemini-2.5-flash"),
    _common_agentic,
    GeminiSpeakSkill.blueprint(),
).disabled_modules(SecurityModule, SpeakSkill)

__all__ = ["unitree_go2_agentic_gemini"]
