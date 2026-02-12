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

"""
Deployment helpers for TemporalMemory module.
"""

import os
from typing import TYPE_CHECKING

from dimos.core._dask_exports import DimosCluster
from dimos.models.vl.base import VlModel
from dimos.spec import Camera as CameraSpec

from .temporal_memory import TemporalMemory, TemporalMemoryConfig

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs import Image


def deploy(
    dimos: DimosCluster,
    camera: CameraSpec,
    vlm: VlModel | None = None,
    config: TemporalMemoryConfig | None = None,
) -> TemporalMemory:
    """Deploy TemporalMemory with a camera.

    Args:
        dimos: Dimos cluster instance
        camera: Camera module to connect to
        vlm: Optional VLM instance (creates OpenAI VLM if None)
        config: Optional temporal memory configuration
    """
    if vlm is None:
        from dimos.models.vl.openai import OpenAIVlModel

        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            raise ValueError("OPENAI_API_KEY environment variable not set")
        vlm = OpenAIVlModel(api_key=api_key)

    temporal_memory = dimos.deploy(TemporalMemory, vlm=vlm, config=config)  # type: ignore[attr-defined]

    if camera.color_image.transport is None:
        from dimos.core.transport import JpegShmTransport

        transport: JpegShmTransport[Image] = JpegShmTransport("/temporal_memory/color_image")
        camera.color_image.transport = transport

    temporal_memory.color_image.connect(camera.color_image)
    temporal_memory.start()
    return temporal_memory  # type: ignore[return-value,no-any-return]
