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

"""Let static API extraction visit DimOS's implicit namespace subpackages."""

from pathlib import Path
from typing import Any

from griffe import Extension, Module


def add_namespace_children(parent: Module, directory: Path) -> None:
    """Register source-only namespace directories through Griffe's public API."""
    for child in directory.iterdir():
        if (
            child.is_dir()
            and child.name.isidentifier()
            and not child.name.startswith("__")
            and not (child / "__init__.py").exists()
        ):
            namespace = Module(child.name, filepath=[child], parent=parent)
            parent.set_member(child.name, namespace)
            add_namespace_children(namespace, child)


class DimosNamespaces(Extension):
    """Keep canonical DimOS names without changing runtime package files."""

    def on_module_instance(self, *, mod: Module, **kwargs: Any) -> None:
        if (
            (mod.path == "dimos" or mod.path.startswith("dimos."))
            and isinstance(mod.filepath, Path)
            and mod.filepath.name == "__init__.py"
        ):
            add_namespace_children(mod, mod.filepath.parent)
