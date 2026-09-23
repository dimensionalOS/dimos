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

"""Inspect the JPEG request produced from a generated image, without model inference."""

import base64
from pathlib import Path
from unittest.mock import Mock, patch

from dimos_generated.sensor_msgs.msg import Image
from langchain_core.messages import AIMessage
import numpy as np

from dimos.agents.vlm_agent import VLMAgent
from dimos.msgs.image import image_from_array


def main() -> None:
    pixels = np.zeros((120, 160, 3), dtype=np.uint8)
    pixels[:, :80, 0] = 255
    pixels[:, 80:, 1] = 255
    image = image_from_array(pixels, encoding="rgb8")
    model = Mock()
    model.invoke.return_value = AIMessage(content="Demo response (no inference)")
    with patch("langchain.chat_models.init_chat_model", return_value=model):
        agent = VLMAgent()
    try:
        result = agent.query_image(Image.decode(image.encode()), "Describe the colors")
        request = model.invoke.call_args.args[0][-1]
        encoded = request.content[1]["image_url"]["url"].split(",", 1)[1]
        destination = Path("build/message-codegen/demo/evidence/vlm-request.jpg")
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(base64.b64decode(encoded))
        assert result == "Demo response (no inference)"
        print(f"Generated Image → CDR → VLM JPEG request: {destination}")
        print("Expected image: red left half, green right half, 160 x 120 pixels")
        print("Model invocation stubbed; no credentials or network inference used.")
    finally:
        agent.stop()


if __name__ == "__main__":
    main()
