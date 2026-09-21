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

"""The message definitions and native types shipped in the DimOS wheel."""

from importlib import import_module
from pathlib import Path
from typing import Any

from dimos.message_codegen.definitions import Definitions


def schema_root() -> Path:
    return Path(__file__).with_name("schemas")


def message_types() -> dict[str, Any]:
    extension = import_module("dimos_generated")
    return {
        message.name: getattr(getattr(extension, message.package).msg, message.short_name)
        for message in Definitions([]).resolve()
    }
