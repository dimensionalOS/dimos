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

"""PACK MIND sim backbone — step 1: VLM perception proof (CUDA-free).

Replaces EdgeTAM with a VLM that *describes* the camera frame, exactly like the
robot-telephone reference (ouazmourad/dimos-21-days-sprint). Runs one G1 MuJoCo
sim (no perception stack, so no CUDA) plus a VLM describer, then asks the VLM to
describe what the simulated robot sees. If this prints a sensible description,
the VLM-perception -> semantic-memory bridge works on this machine.

macOS needs mjpython for MuJoCo. Needs OPENAI_API_KEY + internet.

    mjpython dimos/experimental/pack_mind/sim_perception.py
"""

from __future__ import annotations

import time

from dimos.agents.vlm_agent import VLMAgent
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_basic_sim import (
    unitree_g1_basic_sim,
)

CAMERA_WARMUP_S = 20.0


class DescriberVLM(VLMAgent):
    """VLMAgent + a visual_query RPC that describes the latest camera frame.

    Local VLMAgent has query() (text-only) and query_image(img, q); it lacks the
    fork's visual_query, so we add it here: query using the last frame received
    on the color_image stream.
    """

    @rpc
    def visual_query(self, query: str) -> str:
        if self._latest_image is None:
            return "No image yet — camera not streaming."
        response = self._invoke_image(self._latest_image, query)
        content = response.content
        return content if isinstance(content, str) else str(content)


def main() -> None:
    global_config.update(simulation="mujoco")
    blueprint = autoconnect(unitree_g1_basic_sim, DescriberVLM.blueprint(model="gpt-4o"))
    coordinator = ModuleCoordinator.build(blueprint)
    try:
        print(f"[pack-mind] sim up; warming camera {CAMERA_WARMUP_S:.0f}s...", flush=True)
        time.sleep(CAMERA_WARMUP_S)
        vlm = coordinator.get_instance(DescriberVLM)
        prompt = (
            "Look at the scene and pick the most prominent object. Describe its "
            "color, shape, size, and position relative to other objects in 2-3 "
            "sentences. Do not name it directly — only its visual properties."
        )
        print("[pack-mind] querying VLM with live sim frame...\n", flush=True)
        print("VLM DESCRIPTION:\n" + vlm.visual_query(prompt), flush=True)
    finally:
        print("\n[pack-mind] shutting down sim...", flush=True)
        coordinator.stop()


if __name__ == "__main__":
    main()
