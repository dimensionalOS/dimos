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

from transformers import AutoProcessor, CLIPSegForImageSegmentation
import torch
import numpy as np

class CLIPSeg:
    def __init__(self, model_name="CIDAS/clipseg-rd64-refined"):
        self.clipseg_processor = AutoProcessor.from_pretrained(model_name)
        self.clipseg_model = CLIPSegForImageSegmentation.from_pretrained(model_name)

    def run_inference(self, image, text_descriptions):
        inputs = self.clipseg_processor(text=text_descriptions, images=[image] * len(text_descriptions), padding=True, return_tensors="pt")
        outputs = self.clipseg_model(**inputs)
        logits = outputs.logits
        return logits.detach().unsqueeze(1)