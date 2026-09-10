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

"""Build-time helpers that do not import the DimOS runtime."""

from pathlib import Path
import shutil
import subprocess


def ensure_web_dist(root: Path) -> None:
    """Build the web assets for Git installs and source distributions."""
    for project, artifact in (("sdk", "sdk.js"), ("cockpit", "index.html")):
        source = root / "web" / project
        if (source / "dist" / artifact).is_file():
            continue
        deno = shutil.which("deno")
        if deno is None:
            raise RuntimeError(
                "Deno is required to build DimOS web assets from source. "
                "Run dimup setup, then retry dependency installation."
            )
        subprocess.run([deno, "task", "--cwd", str(source), "build"], check=True)
        if not (source / "dist" / artifact).is_file():
            raise RuntimeError(f"Deno build did not produce {source / 'dist' / artifact}")
