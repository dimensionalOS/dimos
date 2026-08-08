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

from dimos.agents.utils import _display_message_content


def test_display_message_content_omits_encrypted_reasoning() -> None:
    content = [
        {"type": "reasoning", "encrypted_content": "secret"},
        {"type": "function_call", "name": "go_home"},
        {"type": "text", "text": "Reached home."},
    ]

    assert _display_message_content(content) == "Reached home."
