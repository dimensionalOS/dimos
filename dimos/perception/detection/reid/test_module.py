# Copyright 2025 Dimensional Inc.
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
import time

import pytest
import torch

from dimos.core import LCMTransport, start
from dimos.models.embedding import TorchReIDModel
from dimos.msgs.foxglove_msgs import ImageAnnotations
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.perception.detection.reid.module import ReidModule
from dimos.perception.detection.reid.embedding_id_system import EmbeddingIDSystem


def test_reid_ingress():
    # Create TorchReID-based IDSystem for testing
    reid_model = TorchReIDModel(model_name="osnet_x1_0")
    reid_model.warmup()
    # idsystem = EmbeddingIDSystem(
    #     model=lambda: reid_model,
    #     padding=20,
    #     similarity_threshold=0.75,
    # )

    # reid_module = ReidModule(idsystem=idsystem, warmup=False)
    # print("Processing detections through ReidModule...")
    # reid_module.annotations._transport = LCMTransport("/annotations", ImageAnnotations)
    # reid_module.ingress(imageDetections2d)
    # reid_module._close_module()
    # print("✓ ReidModule ingress test completed successfully")
