# Copyright 2025-2026 Dimensional Inc.
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


import pathlib

import pytest

from dimos.utils.cli.graph import main


def test_file_not_found() -> None:
    with pytest.raises(FileNotFoundError):
        main("/nonexistent/path.py")


def test_no_blueprints(tmp_path: pathlib.Path) -> None:
    p = tmp_path / "empty.py"
    p.write_text("x = 42\n")
    with pytest.raises(RuntimeError, match="No Blueprint instances"):
        main(str(p))


def test_module_load_failure(tmp_path: pathlib.Path) -> None:
    p = tmp_path / "bad.py"
    p.write_text("raise ImportError('boom')\n")
    with pytest.raises(ImportError, match="boom"):
        main(str(p))
