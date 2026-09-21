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

"""One-time BEHAVIOR environment and licensed asset preparation."""

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess

import requests

from dimos.experimental.isolated_python.module import (
    isolated_python_environment,
    isolated_python_run_command,
    prepare_isolated_python,
)
from dimos.utils.data import get_project_root

PROJECT_DIR = "native/python/behavior"
MARKER = ".behavior-setup.json"


def behavior_project() -> Path:
    return get_project_root() / PROJECT_DIR


def runtime_environment(project: Path) -> dict[str, str]:
    settings = json.loads((project / MARKER).read_text())
    env = isolated_python_environment(project)
    for key in (
        "EXP_PATH",
        "CARB_APP_PATH",
        "ISAAC_PATH",
    ):
        env.pop(key, None)
    env.update(
        OMNI_KIT_ACCEPT_EULA="YES",
        OMNIGIBSON_DATA_PATH=settings["data_path"],
        CUDA_HOME=str(project / ".pixi/envs/default"),
        CUDA_PATH=str(project / ".pixi/envs/default"),
        TMPDIR=str(project / ".downloads"),
        LD_LIBRARY_PATH=os.pathsep.join(
            filter(None, [str(project / ".pixi/envs/default/lib"), env.get("LD_LIBRARY_PATH", "")])
        ),
    )
    return env


def repair_isaac_prebundles(project: Path) -> None:
    """Apply upstream's fix for vendored packages shadowing the locked versions."""
    root = (
        Path(isolated_python_environment(project)["UV_PROJECT_ENVIRONMENT"])
        / "lib/python3.11/site-packages/isaacsim/extscache"
    )
    for archive in root.glob("*/pip_prebundle"):
        for name in ("websockets", "packaging"):
            directory = archive / name
            if directory.is_dir():
                shutil.rmtree(directory)


def install_omnigibson_resources(project: Path) -> None:
    """Install the startup icon omitted by the pinned OmniGibson wheel."""
    target = (
        Path(isolated_python_environment(project)["UV_PROJECT_ENVIRONMENT"])
        / "lib/python3.11/docs/assets/OmniGibson_logo.png"
    )
    response = requests.get(
        "https://raw.githubusercontent.com/StanfordVL/BEHAVIOR-1K/"
        "b1979916ec1549b10a4e65e630bc6504a9af1b00/docs/assets/OmniGibson_logo.png",
        timeout=60,
    )
    response.raise_for_status()
    if not response.content.startswith(b"\x89PNG\r\n\x1a\n"):
        raise ValueError("Upstream startup icon is not a PNG")
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(response.content)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--accept-nvidia-eula", action="store_true")
    parser.add_argument("--accept-dataset-license", action="store_true")
    parser.add_argument("--data-path", type=Path, default=None)
    args = parser.parse_args()
    if not (args.accept_nvidia_eula and args.accept_dataset_license):
        parser.error(
            "Read the NVIDIA Isaac Sim EULA and BEHAVIOR dataset terms linked in "
            "docs/capabilities/simulation/behavior.md, then explicitly pass both acceptance flags."
        )
    project = behavior_project()
    data = (args.data_path or project / ".assets").resolve()
    data.mkdir(parents=True, exist_ok=True)
    marker = project / MARKER
    # Record explicit acceptance before invoking upstream installers, but mark incomplete.
    marker.write_text(json.dumps({"data_path": str(data), "complete": False}))
    (project / ".downloads").mkdir(exist_ok=True)
    env = runtime_environment(project)
    subprocess.run(["pixi", "install", "--locked"], cwd=project, env=env, check=True)
    prepare_isolated_python(project, env)
    repair_isaac_prebundles(project)
    install_omnigibson_resources(project)
    subprocess.run(
        isolated_python_run_command(
            project,
            "python",
            "-m",
            "omnigibson.utils.asset_utils",
            "--download_omnigibson_robot_assets",
            "--download_behavior_1k_assets",
            "--download_2026_challenge_task_instances",
            "--accept_license",
        ),
        cwd=project,
        env=env,
        check=True,
    )
    marker.write_text(json.dumps({"data_path": str(data), "complete": True}))
    print("BEHAVIOR setup complete. Run: dimos run behavior-teleop")


if __name__ == "__main__":
    main()
