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

import getpass
import subprocess

from dimos.hosted import service

description = "Lingering is on, so the systemd unit starts at boot without a login"
warning = True


def check() -> bool:
    if not service.installed():
        return True
    user = getpass.getuser()
    out = subprocess.run(
        ["loginctl", "show-user", user, "-p", "Linger"], capture_output=True, text=True
    ).stdout
    if "Linger=yes" in out:
        return True
    raise RuntimeError(f"run: sudo loginctl enable-linger {user}")
