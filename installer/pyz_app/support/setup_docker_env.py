#!/usr/bin/env python3
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

# Helper for generating docker-based dev environment assets.
from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

from .bundled_data import DOCKERFILE_TEMPLATE
from .dotenv import setup_dotenv
from .misc import maybe_write

if TYPE_CHECKING:
    from collections.abc import Iterable


_build_script = r"""#!/usr/bin/env bash
set -e

if [ -f ".env" ]; then
    export $(grep -v "^#" .env | xargs)
fi
export DIMOS_ENABLED_FEATURES="${DIMOS_ENABLED_FEATURES:-}"

docker build \
    --build-arg DIMOS_ENABLED_FEATURES="$DIMOS_ENABLED_FEATURES" \
    -t dimos-dev \
    .
"""

_exec_script = r"""#!/usr/bin/env bash
set -e

export DIMOS_ENABLED_FEATURES="${DIMOS_ENABLED_FEATURES:-}"

docker run --rm -it \
    -v "$PWD:/app" \
    -w /app dimos-dev \
    bash -l
"""


def setup_docker_env(project_dir: str | Path, features: Iterable[str]) -> dict[str, Path]:
    """Generate Dockerfile, run_build.sh, run_exec.sh, and .env with features."""
    project_dir = Path(project_dir)
    dockerfile_path = project_dir / "Dockerfile"
    env_path = project_dir / ".env"
    build_script_path = project_dir / "run" / "docker_build"
    exec_script_path = project_dir / "run" / "docker_exec"

    print("")
    setup_dotenv(project_dir, env_path)
    maybe_write(dockerfile_path, DOCKERFILE_TEMPLATE)
    maybe_write(build_script_path, _build_script)
    maybe_write(exec_script_path, _exec_script)

    for script in (build_script_path, exec_script_path):
        try:
            script.chmod(script.stat().st_mode | 0o111)
        except Exception:
            pass

    return {
        "dockerfile": dockerfile_path,
        "env": env_path,
        "build_script": build_script_path,
        "exec_script": exec_script_path,
    }


__all__ = ["DOCKERFILE_TEMPLATE", "setup_docker_env"]
