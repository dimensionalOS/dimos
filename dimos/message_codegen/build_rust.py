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

"""Cargo entry point using only this source package and Python's standard library."""

import importlib
import importlib.util
from pathlib import Path
import shutil
import sys
from types import ModuleType


def main() -> None:
    root = Path(__file__).resolve().parent
    # Cargo packages do not preserve the repository's parent directory names.
    # Bind the bundled codegen package explicitly, without importing installed DimOS.
    parent = ModuleType("dimos")
    parent.__path__ = []
    sys.modules["dimos"] = parent
    spec = importlib.util.spec_from_file_location(
        "dimos.message_codegen", root / "__init__.py", submodule_search_locations=[str(root)]
    )
    assert spec is not None and spec.loader is not None
    package = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = package
    spec.loader.exec_module(package)
    definitions_module = importlib.import_module("dimos.message_codegen.definitions")
    rust_module = importlib.import_module("dimos.message_codegen.rust")
    definitions = definitions_module.Definitions([])
    messages = definitions.resolve()
    output = Path(sys.argv[1])
    output.mkdir(parents=True, exist_ok=True)
    (output / "messages.rs").write_text(rust_module.generate(messages, definitions))
    shutil.copyfile(root / "templates/codec.rs", output / "codec.rs")


if __name__ == "__main__":
    main()
