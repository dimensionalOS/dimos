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

"""Project and installation dotenv settings shared by CLI and agent startup."""

from functools import cache
import os
from pathlib import Path

from dotenv import load_dotenv
from dotenv.main import DotEnv


@cache
def _load_project_dotenv() -> tuple[dict[str, str], set[str]]:
    """Load project values before installation defaults and remember them for the agent filter."""
    if "PYTEST_VERSION" in os.environ:
        return {}, set()

    project = Path.cwd() / ".env"
    paths = [project]
    # find_dotenv starts from cwd instead of the package when a debugger or coverage is active.
    candidates = (parent / ".env" for parent in Path(__file__).parents)
    installed = next((path for path in candidates if path.is_file() or path.is_fifo()), None)
    if installed is not None and installed != project:
        paths.append(installed)

    inherited = os.environ.copy()
    dotenv: dict[str, str] = {}
    for path in paths:
        if not load_dotenv(path):
            continue
        # A re-exec inherits file values. Expand them with the same environment
        # precedence as load_dotenv so the agent still filters interpolated keys.
        for name, value in DotEnv(path, override=False, encoding="utf-8").dict().items():
            if value is not None and inherited.get(name) == value:
                dotenv[name] = value

    injected: set[str] = set()
    for name in os.environ.keys() - inherited.keys():
        if name.upper().startswith("HOST__"):
            # HostConfig reads the machine checkout's own file.
            del os.environ[name]
        else:
            # Keep the actual value after dotenv interpolation and shell overrides.
            dotenv[name] = os.environ[name]
            injected.add(name)
    return dotenv, injected


def load_project_dotenv() -> dict[str, str]:
    """Return values used by the agent's environment filter."""
    return _load_project_dotenv()[0]


def restart_environment() -> dict[str, str]:
    """A new CLI resolves its project from the caller's current shell exports."""
    dotenv, injected = _load_project_dotenv()
    return {
        name: value
        for name, value in os.environ.items()
        if name not in injected or dotenv[name] != value
    }
