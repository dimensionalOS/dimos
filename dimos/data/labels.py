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

import os
from dimos.models.labels.llava_34b import Llava
from PIL import Image
from dimos.types.label import LabelType

class LabelProcessor:
    def __init__(self, debug: bool = False):
        self.model = None
        self.prompt = 'Create a JSON representation where each entry consists of a key "object" with a numerical suffix starting from 1, and a corresponding "description" key with a value that is a concise, up to six-word sentence describing each main, distinct object or person in the image. Each pair should uniquely describe one element without repeating keys. An example: {"object1": { "description": "Man in red hat walking." },"object2": { "description": "Wooden pallet with boxes." },"object3": { "description": "Cardboard boxes stacked." },"object4": { "description": "Man in green vest standing." }}'
        self.debug = debug

    def _initialize_model(self):
        if self.model is None:
            from dimos.models.labels.llava_34b import Llava
            self.model = Llava(mmproj=f"{os.getcwd()}/models/mmproj-model-f16.gguf", model_path=f"{os.getcwd()}/models/llava-v1.6-34b.Q4_K_M.gguf", gpu=True)
            if self.debug:
                print("Llava model initialized.")

    def caption_image_data(self, frame):
        self._initialize_model()
        try:
            output = self.model.run_inference(frame, self.prompt, return_json=True)
            if self.debug:
                print("Output:", output)
            return LabelType(labels=output, metadata={"frame_id": frame.id})
        except Exception as e:
            print(f"Error in captioning image: {e}")
            return LabelType(labels={}, metadata={"error": str(e)})