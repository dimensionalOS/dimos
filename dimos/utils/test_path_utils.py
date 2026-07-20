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
from pathlib import Path

from dimos.utils.path_utils import get_project_root


def test_get_project_root_is_absolute_and_contains_dimos_package() -> None:
    root = get_project_root()
    assert isinstance(root, Path)
    assert root.is_absolute()
    assert root.is_dir()
    # path_utils.py lives at <root>/dimos/utils/path_utils.py, so the
    # returned project root must contain the dimos package and this file.
    assert (root / "dimos" / "utils" / "path_utils.py").exists()
