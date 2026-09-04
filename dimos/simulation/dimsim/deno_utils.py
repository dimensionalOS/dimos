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

import subprocess

from dimos.utils.deno import DENO_VERSION, ensure_deno  # noqa: F401


def ensure_playwright_chromium(deno_path: str) -> None:
    subprocess.run(
        [deno_path, "run", "--allow-all", "npm:playwright@1.58.2", "install", "chromium"],
        check=True,
    )
