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

"""Discover installed generated types after checking their definition closure."""

from typing import Any

from dimos.message_codegen.definitions import Definitions
from dimos.message_codegen.providers import providers, schema_roots


def message_types() -> dict[str, Any]:
    Definitions(schema_roots()).resolve()
    result: dict[str, Any] = {}
    for provider in providers():
        for name, message_type in provider.message_types().items():
            result.setdefault(name, message_type)
    return result
