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

"""Shared helpers for hardware registries."""

from __future__ import annotations


def normalize_adapter_name(name: str) -> str:
    """Normalize adapter names consistently for registration and lookup."""
    key = name.strip().lower()
    if not key:
        raise ValueError("Adapter name must be non-empty")
    return key
