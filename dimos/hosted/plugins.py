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

"""Plugins are the modules of a package, one file each: drop a file in to add one."""

from __future__ import annotations

import importlib
import pkgutil
from types import ModuleType


def load(package: str) -> list[ModuleType]:
    """Every non-private submodule of ``package``, by name."""
    pkg = importlib.import_module(package)
    return [
        importlib.import_module(f"{package}.{info.name}")
        for info in sorted(pkgutil.iter_modules(pkg.__path__), key=lambda info: info.name)
        if not info.name.startswith("_")
    ]
