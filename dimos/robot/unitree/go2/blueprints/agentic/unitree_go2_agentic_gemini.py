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
from dimos.agents.skills.navigation import NavigationSkillContainer
from dimos.agents.skills.person_follow import PersonFollowSkillContainer
from dimos.agents.skills.map_uploader import MapUploader
from dimos.agents.skills.speak_skill import SpeakSkill
from dimos.agents.skills.take_picture_skill import TakePictureSkill
from dimos.core.coordination.blueprints import autoconnect
from dimos.experimental.security_demo.security_module import SecurityModule
from dimos.perception.spatial_perception import SpatialMemory
from dimos.robot.unitree.go2.blueprints.agentic._common_agentic import _common_agentic
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2_spatial import unitree_go2_spatial
from dimos.robot.unitree.go2.connection import GO2Connection

# Disabled for a no-OpenAI / non-CUDA setup:
#  - SecurityModule needs a CUDA GPU (EdgeTAM) -> won't boot on Apple Silicon.
#  - SpeakSkill (TTS) hardcodes OpenAI -> needs OPENAI_API_KEY; replaced below
#    by GeminiSpeakSkill, which uses the Gemini TTS API (reuses GOOGLE_API_KEY,
#    no GPU, cross-platform so it can also run on the robot).
# SpatialMemory is re-declared to use remote Gemini embeddings instead of the
# local CLIP model (which fails on Apple CoreML); the later atom overrides the
# one inside unitree_go2_spatial.
# Detection VL is kept LOCAL (Moondream) where it makes sense:
#  - look_out_for uses the global detection_model (default "moondream").
#  - PersonFollowSkillContainer is re-declared with vl_model_name="moondream";
#    tracking_mode="auto" resolves to periodic re-detection on non-CUDA machines
#    (no EdgeTAM/CUDA) and EdgeTAM on GPU.
# NavigationSkillContainer is re-declared to use Gemini for detection instead of
# Qwen, which would otherwise require ALIBABA_API_KEY.
unitree_go2_agentic_gemini = autoconnect(
    unitree_go2_spatial,
    SpatialMemory.blueprint(embedding_model="gemini", embedding_dimensions=768),
    McpServer.blueprint(),
    McpClient.blueprint(model="google_genai:gemini-2.5-flash"),
    _common_agentic,
    NavigationSkillContainer.blueprint(vl_model_name="gemini"),
    PersonFollowSkillContainer.blueprint(
        camera_info=GO2Connection.camera_info_static,
        vl_model_name="moondream",
    ),
    GeminiSpeakSkill.blueprint(),
    TakePictureSkill.blueprint(),
    MapUploader.blueprint(),
).disabled_modules(SecurityModule, SpeakSkill)

__all__ = ["unitree_go2_agentic_gemini"]
