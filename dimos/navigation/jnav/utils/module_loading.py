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

"""Dynamically load a Module class by file path and filter its config.

Lets the loop-closure eval drivers swap in any module-under-test from a
``--module-path``/``--module-name`` pair without importing it statically.
"""

from __future__ import annotations

import importlib
from pathlib import Path
from typing import Any, get_type_hints


def load_module_class(module_path: Path, module_name: str) -> type:
    """Import the class `module_name` from a python file.

    Resolved through the normal package system (path -> dotted name rooted at
    the last `dimos` directory) so the class pickles/deploys into workers."""
    parts = module_path.resolve().with_suffix("").parts
    if "dimos" not in parts:
        raise SystemExit(f"--module-path must live inside the dimos package tree: {module_path}")
    package_root = len(parts) - 1 - parts[::-1].index("dimos")
    dotted = ".".join(parts[package_root:])
    module = importlib.import_module(dotted)
    if not hasattr(module, module_name):
        raise SystemExit(f"no class {module_name!r} in {dotted} ({module_path})")
    return getattr(module, module_name)  # type: ignore[no-any-return]


def filter_config_for_module(module_class: type, config: dict[str, Any]) -> dict[str, Any]:
    """Drop config keys the module's Config class doesn't declare.

    DEFAULT_PGO_CONFIG carries cmu-specific knobs (use_scan_context, ...);
    other loop-closure modules shouldn't crash on them."""
    try:
        config_class = get_type_hints(module_class)["config"]
        known = set(config_class.model_fields)
    except Exception:
        return config
    dropped = sorted(set(config) - known)
    if dropped:
        print(f"config keys not in {config_class.__name__} (dropped): {dropped}")
    return {key: value for key, value in config.items() if key in known}
