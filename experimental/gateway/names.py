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

from typing import Any

from experimental.gateway.errors import ApiError


def check_name(name: Any) -> str:
    """Validate a catalog or launch name, including one read from a saved request."""
    if (
        not isinstance(name, str)
        or not name
        or name.startswith("-")
        or any(char.isspace() for char in name)
    ):
        raise ApiError(400, f"bad name: {name}")
    return name
