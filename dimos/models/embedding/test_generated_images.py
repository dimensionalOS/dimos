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

from unittest.mock import PropertyMock

import numpy as np
import pytest
import torch
from transformers.feature_extraction_utils import BatchFeature
from transformers.modeling_outputs import BaseModelOutputWithPooling

from dimos.message_utils.image import image_from_array, image_view
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.std_msgs.msg import Header
from dimos.models.embedding.clip import CLIPModel
from dimos.models.embedding.dino import DINOModel
from dimos.models.embedding.siglip import SigLIPModel
from dimos.models.embedding.siglip2 import SigLIP2Model


@pytest.mark.parametrize("model_type", [CLIPModel, DINOModel, SigLIPModel, SigLIP2Model])
def test_generated_embeddings_keep_zero_source_time_and_convert_rgb(mocker, model_type):
    processor, network = mocker.Mock(), mocker.Mock()
    features = torch.tensor([[3.0, 4.0]])
    processor.return_value = BatchFeature(data={"pixel_values": torch.zeros(1, 3, 4, 8)})
    network.get_image_features.return_value = features
    network.return_value = BaseModelOutputWithPooling(
        last_hidden_state=features[:, None, :], pooler_output=features
    )
    mocker.patch.object(model_type, "_processor", new_callable=PropertyMock, return_value=processor)
    mocker.patch.object(model_type, "_model", new_callable=PropertyMock, return_value=network)
    image = image_from_array(
        np.full((4, 8, 3), [0, 0, 255], dtype=np.uint8),
        encoding="bgr8",
        header=Header(frame_id="camera", stamp=Time(sec=0, nanosec=0)),
    )
    model = model_type(device="cpu")
    result = model.embed(image)
    assert result.timestamp == 0.0
    torch.testing.assert_close(result.vector, torch.tensor([0.6, 0.8]))
    assert processor.call_args.kwargs["images"][0].getpixel((0, 0)) == (255, 0, 0)
    np.testing.assert_array_equal(image_view(image)[0, 0], [0, 0, 255])
