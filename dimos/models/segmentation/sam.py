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

from transformers import SamModel, SamProcessor
import torch
import numpy as np

class SAM:
    def __init__(self, model_name="facebook/sam-vit-huge", device="cuda"):
        self.device = device
        self.sam_model = SamModel.from_pretrained(model_name).to(self.device)
        self.sam_processor = SamProcessor.from_pretrained(model_name)

    def run_inference_from_points(self, image, points):
        sam_inputs = self.sam_processor(image, input_points=points, return_tensors="pt").to(self.device)
        with torch.no_grad():
            sam_outputs = self.sam_model(**sam_inputs)
        return self.sam_processor.image_processor.post_process_masks(sam_outputs.pred_masks.cpu(), sam_inputs["original_sizes"].cpu(), sam_inputs["reshaped_input_sizes"].cpu())
