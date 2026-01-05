#!/usr/bin/env python3
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

from __future__ import annotations

from pathlib import Path
import time

from ..support import prompt_tools as p
from ..support.bundled_data import PROJECT_TOML
from ..support.constants import PLACEHOLDERS
from ..support.dimos_banner import RenderLogo
from ..support.get_system_analysis import get_system_analysis
from ..support.installer_status import installer_status
from ..support.misc import (
    get_project_directory,
    init_repo_with_gitignore,
    replace_strings_in_directory,
)
from ..support.setup_docker_env import setup_docker_env
from ..support.setup_nix import ensure_flakes_enabled, nix_install, setup_nix_flake
from ..support.shell_tooling import command_exists, run_command


def phase0():
    #
    # provide animation while running system analysis
    #
    fps = 14
    logo = RenderLogo(
        glitchyness=0.45,  # relative quantity of visual artifacting
        stickyness=fps * 0.75,  # how many frames to keep an artifact
        fps=fps,  # at 30fps it flickers a lot in the MacOS stock terminal. Ironically its fine at 30fps in the VS Code terminal
        color_wave_amplitude=10,  # bigger = wider range of colors
        wave_speed=0.01,  # bigger = faster
        wave_freq=0.01,  # smaller = longer streaks of color
        scrollable=True,
    )

    logo.log("- checking system")
    system_analysis = get_system_analysis()
    timeout = 0.7  # wait long enough so users can read what is happening and see logo
    # visually we want cuda to be listed last and os to be first
    cuda = system_analysis["cuda"]
    del system_analysis["cuda"]
    ordered_analysis = {
        "os": system_analysis["os"],
        **system_analysis,
        "cuda": cuda,
    }
    ordered_analysis["cuda"] = cuda

    #
    # print system analysis
    #
    for key, result in ordered_analysis.items():
        name = result.get("name") or key
        exists = result.get("exists", False)
        version = result.get("version", "") or ""
        note = result.get("note", "") or ""
        cross = "\u2718"
        check = "\u2714"
        if not exists:
            logo.log(f"- {p.red(cross)} {name} {note}".strip())
        else:
            logo.log(f"- {p.cyan(check)} {name}: {version} {note}".strip())
        time.sleep(timeout)
    logo.stop()

    if installer_status.get("template_repo"):
        project_dir = get_project_directory()
        project_name = project_dir.name
        # fill out the directory
        replace_strings_in_directory(project_dir, PLACEHOLDERS, project_name)

    optional = PROJECT_TOML["project"].get("optional-dependencies", {})
    features = [f for f in optional.keys() if f not in ["cpu"]]
    p.header("First Phase: Feature Selection")
    selected_features = p.pick_many(
        "Which features do you want? (Pick any number of features)", options=["basics", *features]
    )
    # basics is just a dummy entry to make it more user friendly
    selected_features = [each for each in selected_features if each != "basics"]
    if "sim" in selected_features and "cuda" not in selected_features:
        selected_features.append("cpu")

    #
    # pick install method
    #
    while True:
        choice = p.pick_one(
            "Choose install method",
            options={
                "system": "Typical system install",
                "docker": "Docker container setup",
                "nix": "Nix flake",
                # "nix_venv": "Fast venv setup (nix shell)",
            },
        )
        if choice == "system":
            # continue as normal
            break
        if choice == "docker":
            if not system_analysis.get("docker", {}).get("exists"):
                p.error("Docker is not installed or not detected.")
                print("Download Docker: https://www.docker.com/products/docker-desktop/")
                next_step = p.pick_one(
                    "Docker is required for this option.",
                    options={"back": "Choose a different install method", "exit": "Exit installer"},
                )
                if next_step == "exit":
                    raise SystemExit(1)
                continue
            project_dir = get_project_directory()
            paths = setup_docker_env(project_dir, selected_features)
            p.sub_header("Docker assets created/updated:")
            for key, path in paths.items():
                print(f" - {key}: {path}")
            print(
                f"Use {p.highlight('run/docker_build')} to build the image, and {p.highlight('run/docker_exec')} to start a shell in the container."
            )
            if p.ask_yes_no("Would you like me to build the image now?"):
                run_command([str(paths["build_script"])], check=False)
            if p.ask_yes_no("Would you like me to start a container shell now?"):
                run_command([str(paths["exec_script"])], check=False)
            p.sub_header("Docker setup complete. Exiting installer.")
            raise SystemExit(0)
        if choice == "nix":
            project_dir = get_project_directory()
            example_path = setup_nix_flake(project_dir)
            if not example_path:
                continue

            feat_str = "[" + (",".join(selected_features)) + "]" if selected_features else ""
            if not command_exists("git"):
                print("You need to install git for the flake.nix to work")
                print("Should I install it for you? (y/n)")
                nix_install(["git"]) # this will install nix if needed

            if not Path(project_dir / ".git").exists():
                if p.ask_yes_no("Your project doesn't seem to have a (direct) git repo.\nFlakes require a git repo.\nShould I initialize a new git repo for this flake you? (y/n)"):
                    init_repo_with_gitignore(project_dir)
                else:
                    print("Okay, but make sure to commit the flake.nix changes otherwise you won't be able to run `nix develop`")

            ensure_flakes_enabled()
            print(
                f"Once ready, git commit the flake.nix, run `nix develop`, then run `uv pip install dimos{feat_str}`"
            )
            # TODO: ask if they would like us to setup .envrc for them
            raise SystemExit(0)

    return system_analysis, selected_features


if __name__ == "__main__":
    print(phase0())
