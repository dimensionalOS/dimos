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
import subprocess

from dimos.utils.deno import DENO_VERSION, ensure_deno  # noqa: F401

# Keep in step with misc/DimSim/cli/deno.json: the launcher there must find the
# Chromium revision installed here. 1.61.0 is the first release with builds for
# Ubuntu 26.04.
_PLAYWRIGHT_PACKAGE = "npm:playwright@1.61.0"


def ensure_playwright_chromium(deno_path: str, *, with_deps: bool = False) -> None:
    """Install the pinned Chromium and, when requested, its host dependencies."""
    # Ask Playwright so platform revisions and PLAYWRIGHT_BROWSERS_PATH are
    # respected. DimSim launches full Chromium, even in headless mode.
    executable = subprocess.check_output(
        [
            deno_path,
            "eval",
            "--quiet",
            f'import {{ chromium }} from "{_PLAYWRIGHT_PACKAGE}"; '
            "console.log(chromium.executablePath());",
        ],
        text=True,
    ).strip()
    if not with_deps and Path(executable).is_file():
        return

    options = ["--with-deps"] if with_deps else []
    subprocess.run(
        [deno_path, "run", "--allow-all", _PLAYWRIGHT_PACKAGE, "install", *options, "chromium"],
        check=True,
    )
