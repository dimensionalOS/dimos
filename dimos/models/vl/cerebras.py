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

from functools import cached_property
import os

from openai import OpenAI

from dimos.models.vl.openai import OpenAIVlModel, OpenAIVlModelConfig


class CerebrasVlModelConfig(OpenAIVlModelConfig):
    model_name: str = "gemma-4-31b"
    base_url: str = "https://api.cerebras.ai/v1"


class CerebrasVlModel(OpenAIVlModel):
    """Cerebras vision using the shared image, caption and detection interface."""

    config: CerebrasVlModelConfig

    @cached_property
    def _client(self) -> OpenAI:
        api_key = self.config.api_key or os.environ.get("CEREBRAS_API_KEY")
        if not api_key:
            raise ValueError("Set CEREBRAS_API_KEY to use the Cerebras vision model")
        return OpenAI(api_key=api_key, base_url=self.config.base_url)
