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

"""Shared helpers for control task registries and extension registration."""

from __future__ import annotations


def normalize_task_name(name: str) -> str:
    """Normalize task type names consistently for registration and lookup."""
    key = name.strip().lower()
    if not key:
        raise ValueError("Task type must be non-empty")
    return key


def validate_task_factory_path(
    factory_path: str,
    *,
    label: str = "task factory path",
) -> None:
    """Validate a lazy factory path of the form ``module:function``."""
    module_name, separator, attr = factory_path.partition(":")
    if not factory_path.strip() or separator != ":" or not module_name or not attr:
        raise ValueError(f"Invalid {label}: {factory_path!r}")


__all__ = ["normalize_task_name", "validate_task_factory_path"]
