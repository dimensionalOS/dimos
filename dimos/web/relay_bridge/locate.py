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

# Overrides where the web tree is looked up. An override is served as-is:
# auto-building it would run its build tooling with full permissions, so the
# cockpit build additionally requires the opt-in below (see build_allowed()).
WEB_DIR_ENV_VAR = "DIMOS_WEB_DIR"
WEB_DIR_BUILD_ENV_VAR = "DIMOS_WEB_DIR_BUILD"

# Written into the wheel by the build_py hook in setup.py; absent in checkouts.
_PACKAGED_DIR_NAME = "_relay_dist"


def find_web_dir() -> Path:
    """Return the directory holding the relay's deno.json (the repo's web/)."""
    tried = []

    env = os.environ.get(WEB_DIR_ENV_VAR)
    if env:
        env_dir = Path(env).resolve()
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


def build_allowed(web_dir: Path) -> bool:
    """True when the cockpit auto-build may run in web_dir.

    A tree selected via DIMOS_WEB_DIR gets its build tooling executed with
    `deno run -A`, so it is served as-is unless DIMOS_WEB_DIR_BUILD=1
    acknowledges that; the repo checkout is trusted and always builds.
    """
    env = os.environ.get(WEB_DIR_ENV_VAR)
    if not env or Path(env).resolve() != web_dir.resolve():
        return True
    return os.environ.get(WEB_DIR_BUILD_ENV_VAR) == "1"


def find_cockpit_dist(web_dir: Path) -> Path | None:
    """Built Cockpit app (web/cockpit/dist), canonicalized, or None when not built."""
    dist = web_dir / "cockpit" / "dist"
    return dist.resolve() if (dist / "index.html").is_file() else None


def relay_run_cmd(
    deno: str, web_dir: Path, *args: str, cockpit_dir: Path | None = None
) -> list[str]:
    """Build the argv that runs the relay with the pinned config and least permissions."""
    # Canonical paths: the relay realpath-checks served files against its
    # roots, so a symlinked --allow-read scope (macOS /tmp -> /private/tmp)
    # would deny every read.
    web_dir = web_dir.resolve()
    if cockpit_dir is not None:
        cockpit_dir = cockpit_dir.resolve()
    # --node-modules-dir=none: the workspace root deno.json says "auto" (for
    # the cockpit build tooling), which would make this run materialize
    # node_modules next to the config -- inside site-packages under a wheel.
    allow_read = str(web_dir) if cockpit_dir is None else f"{web_dir},{cockpit_dir}"
    cmd = [
        deno,
        "run",
        "--frozen",
        "--node-modules-dir=none",
        f"--allow-read={allow_read}",
        "--allow-net",
        "--config",
        str(web_dir / "deno.json"),
        str(web_dir / "relay" / "main.ts"),
    ]
    if cockpit_dir is not None:
        cmd += ["--cockpit-dir", str(cockpit_dir)]
    return [*cmd, *args]


def _is_web_dir(path: Path) -> bool:
    return (path / "deno.json").is_file() and (path / "relay" / "main.ts").is_file()
