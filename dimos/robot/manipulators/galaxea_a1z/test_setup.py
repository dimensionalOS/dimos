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

from pathlib import Path
import subprocess

import tomllib

REPOSITORY_ROOT = Path(__file__).parents[4]
PYPROJECT_PATH = REPOSITORY_ROOT / "pyproject.toml"
SETUP_SCRIPT = Path(__file__).parent / "scripts" / "setup_a1z.sh"


def test_locked_a1z_group_contains_vendor_and_macos_transport() -> None:
    project = tomllib.loads(PYPROJECT_PATH.read_text())

    dependencies = set(project["dependency-groups"]["galaxea-a1z"])

    assert dependencies == {
        "a1z @ git+https://github.com/userguide-galaxea/GALAXEA-A1Z.git@e931ecd0e25ad35df251097ba42921b3d2fa7224",
        "gs-usb==0.3.1; sys_platform == 'darwin'",
        "pyusb==1.3.1; sys_platform == 'darwin'",
    }


def test_setup_help_documents_locked_runtime_and_lerobot_option() -> None:
    result = subprocess.run(
        [str(SETUP_SCRIPT), "--help"],
        check=True,
        capture_output=True,
        text=True,
    )

    assert "--sdk-only" in result.stdout
    assert "--with-lerobot" in result.stdout
    assert "Synchronize the locked Galaxea A1Z runtime" in result.stdout


def test_macos_setup_opens_can_transport_without_transmitting() -> None:
    setup_source = SETUP_SCRIPT.read_text()

    assert "GsUsbMacBus(listen_only=True)" in setup_source
