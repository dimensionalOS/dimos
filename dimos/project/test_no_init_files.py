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

from dimos.constants import DIMOS_PROJECT_ROOT


def test_no_init_files():
    dimos_dir = DIMOS_PROJECT_ROOT / "dimos"
    init_files = sorted(dimos_dir.rglob("__init__.py"))
    # The root dimos/__init__.py is allowed for the porcelain lazy import.
    init_files = [f for f in init_files if f != dimos_dir / "__init__.py"]
    if init_files:
        listing = "\n".join(f"  - {f.relative_to(dimos_dir)}" for f in init_files)
        raise AssertionError(
            f"Found __init__.py files in dimos/:\n{listing}\n\n"
            "__init__.py files are not allowed because they lead to unnecessary "
            "extraneous imports. Everything should be imported straight from the "
            "source module."
        )
