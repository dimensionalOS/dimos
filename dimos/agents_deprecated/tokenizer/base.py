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

from abc import ABC, abstractmethod

# TODO: Add a class for specific tokenizer exceptions
# TODO: Build out testing and logging
# TODO: Create proper doc strings after multiple tokenizers are implemented


class AbstractTokenizer(ABC):
    @abstractmethod
    def tokenize_text(self, text: str):  # type: ignore[no-untyped-def]
        pass

    @abstractmethod
    def detokenize_text(self, tokenized_text):  # type: ignore[no-untyped-def]
        pass

    @abstractmethod
    def token_count(self, text: str):  # type: ignore[no-untyped-def]
        pass

    @abstractmethod
    def image_token_count(self, image_width, image_height, image_detail: str = "low"):  # type: ignore[no-untyped-def]
        pass
