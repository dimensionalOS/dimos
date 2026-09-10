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

import subprocess
import sys

import pytest

from dimup.process import Runner, SetupError
from dimup.setup import supported_platform


@pytest.mark.parametrize("version", ["22.04", "24.04"])
def test_ubuntu_versions(monkeypatch, version):
    monkeypatch.setattr("platform.system", lambda: "Linux")
    monkeypatch.setattr("platform.machine", lambda: "x86_64")
    monkeypatch.setattr(
        "platform.freedesktop_os_release", lambda: {"ID": "ubuntu", "VERSION_ID": version}
    )
    assert supported_platform() == "ubuntu"


def test_rejects_intel_mac(monkeypatch):
    monkeypatch.setattr("platform.system", lambda: "Darwin")
    monkeypatch.setattr("platform.machine", lambda: "x86_64")
    with pytest.raises(SetupError, match="Supported platforms"):
        supported_platform()


def test_apple_silicon(monkeypatch):
    monkeypatch.setattr("platform.system", lambda: "Darwin")
    monkeypatch.setattr("platform.machine", lambda: "arm64")
    monkeypatch.setattr("platform.mac_ver", lambda: ("14.0", (), ""))
    assert supported_platform() == "macos"


def test_failed_command_preserves_log(tmp_path):
    log = tmp_path / "setup.log"
    with pytest.raises(SetupError, match="Failed: example") as failure:
        Runner(log).run(
            "example", [sys.executable, "-c", "import sys; print('detail'); sys.exit(7)"]
        )
    assert "detail" in log.read_text()
    assert str(log) in str(failure.value)


def test_runner_uses_argument_boundaries(tmp_path):
    output = Runner(tmp_path / "setup.log").run(
        "echo",
        [sys.executable, "-c", "import sys; print(sys.argv[1])", "hello; $(false)"],
    )
    assert output == "hello; $(false)"


def test_setup_is_required():
    result = subprocess.run([sys.executable, "-m", "dimup.cli"], capture_output=True, check=False)
    assert result.returncode == 2
