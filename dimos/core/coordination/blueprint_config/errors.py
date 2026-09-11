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

"""Blueprint configuration error type and error formatting."""

from __future__ import annotations

from pydantic import ValidationError


class BlueprintConfigError(ValueError):
    """A user-correctable blueprint configuration error."""


def format_validation_error(root: str, error: ValidationError) -> str:
    details = []
    for item in error.errors(include_url=False):
        location = ".".join(str(part) for part in item["loc"])
        qualified = f"{root}.{location}" if location else root
        details.append(f"{qualified}: {item['msg']}")
    return "Invalid blueprint configuration: " + "; ".join(details)
