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

from dimos.models.vl.base import VlModel
from dimos.models.vl.types import VlModelName

__all__ = ["VlModelName", "create"]


def create(name: VlModelName) -> VlModel:
    # This uses inline imports to only import what's needed.
    match name:
        case "qwen":
            from dimos.models.vl.qwen import QwenVlModel

            return QwenVlModel()
        case "moondream":
            from dimos.models.vl.moondream import MoondreamVlModel

            return MoondreamVlModel()
