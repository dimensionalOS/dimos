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

"""Locate the Deno relay sources (repo checkout or the copy packaged in the wheel)."""

import os
from pathlib import Path

WEB_DIR_ENV_VAR = "DIMOS_WEB_DIR"

# Written into the wheel by the build_py hook in setup.py; absent in checkouts.
_PACKAGED_DIR_NAME = "_relay_dist"


def find_web_dir() -> Path:
    """Return the directory holding the relay's deno.json (the repo's web/)."""
    tried = []

    env = os.environ.get(WEB_DIR_ENV_VAR)
    if env:
        env_dir = Path(env)
        if _is_web_dir(env_dir):
            return env_dir
        tried.append(f"{WEB_DIR_ENV_VAR}={env_dir}")

    checkout = Path(__file__).resolve().parents[3] / "web"
    if _is_web_dir(checkout):
        return checkout
    tried.append(str(checkout))

    packaged = Path(__file__).resolve().parent / _PACKAGED_DIR_NAME
    if _is_web_dir(packaged):
        return packaged
    tried.append(str(packaged))

    raise RuntimeError(
        f"DimOS relay sources not found (tried: {', '.join(tried)}). Run from a dimos "
        "repo checkout or reinstall the dimos wheel (the relay ships inside it); "
        f"{WEB_DIR_ENV_VAR} overrides the location."
    )


def relay_run_cmd(deno: str, web_dir: Path, *args: str) -> list[str]:
    """Build the argv that runs the relay with the pinned config and least permissions."""
    return [
        deno,
        "run",
        "--frozen",
        f"--allow-read={web_dir}",
        "--allow-net",
        "--config",
        str(web_dir / "deno.json"),
        str(web_dir / "relay" / "main.ts"),
        *args,
    ]


def _is_web_dir(path: Path) -> bool:
    return (path / "deno.json").is_file() and (path / "relay" / "main.ts").is_file()
