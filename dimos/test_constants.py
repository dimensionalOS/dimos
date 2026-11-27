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

import importlib.util
from pathlib import Path
import shutil
from types import ModuleType

from dimos import constants


def _constants_in(root: Path) -> ModuleType:
    """dimos/constants.py as it evaluates when the package folder is in ``root``."""
    copy = root / "dimos" / "constants.py"
    copy.parent.mkdir()
    shutil.copy(constants.__file__, copy)
    spec = importlib.util.spec_from_file_location("constants_copy", copy)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_a_checkout_writes_into_its_own_folder(tmp_path: Path) -> None:
    (tmp_path / ".git").mkdir()

    checkout = _constants_in(tmp_path)

    assert checkout.IS_CHECKOUT
    assert checkout.WRITE_ROOT == tmp_path
    assert checkout.LOG_DIR == tmp_path / "logs"
    assert checkout.RECORDINGS_DIR == tmp_path / "recordings"
    assert checkout.DOWNLOADS_DIR == tmp_path / "downloads"


def test_an_installed_package_writes_into_the_state_directory(tmp_path: Path) -> None:
    installed = _constants_in(tmp_path)

    assert not installed.IS_CHECKOUT
    assert installed.WRITE_ROOT == installed.STATE_DIR
    assert installed.LOG_DIR == installed.STATE_DIR / "logs"
    assert installed.RECORDINGS_DIR == installed.STATE_DIR / "recordings"
    assert installed.DOWNLOADS_DIR == installed.STATE_DIR / "downloads"
