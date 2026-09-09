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

"""Compute an environment delta; the caller evaluates it in Bash or Zsh."""

import json
import os
from pathlib import Path
import shlex
import subprocess
import sys


def changes(before: dict[str, str], after: dict[str, str]) -> str:
    commands = []
    for key in sorted(before.keys() | after.keys()):
        if key in {"_", "SHLVL", "PWD", "OLDPWD"} or before.get(key) == after.get(key):
            continue
        if not key.isascii() or not key.replace("_", "a").isalnum() or key[0].isdigit():
            continue
        commands.append(
            f"export {key}={shlex.quote(after[key])}" if key in after else f"unset {key}"
        )
    return "\n".join(commands)


def main() -> None:
    root = Path(sys.argv[1]).resolve()
    tools = json.loads((root / ".dimos/tools.json").read_text())
    hook = subprocess.check_output(
        [
            tools["pixi"],
            "shell-hook",
            "--shell",
            "bash",
            "--as-is",
            "--manifest-path",
            str(root / ".dimos/pixi.toml"),
        ],
        text=True,
    )
    script = """set -e
eval "$1"
source "$2/.venv/bin/activate"
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PKG_CONFIG_PATH="$CONDA_PREFIX/lib/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
export CPPFLAGS="${CPPFLAGS:+$CPPFLAGS }-idirafter /usr/local/include -idirafter /usr/include"
export PATH="$VIRTUAL_ENV/bin:$3:$4:$PATH"
export NIX_CONFIG="${NIX_CONFIG:-}
extra-experimental-features = nix-command flakes"
exec "$2/.venv/bin/python" -c 'import json, os; print(json.dumps(dict(os.environ)))'
"""
    before = dict(os.environ)
    result = subprocess.check_output(
        [
            "bash",
            "--noprofile",
            "--norc",
            "-c",
            script,
            "activate",
            hook,
            str(root),
            str(Path(tools["uv"]).parent),
            str(Path(tools["nix"]).parent),
        ],
        text=True,
    )
    after = json.loads(result)
    print(changes(before, after))
    print("deactivate() {\n" + changes(after, before) + "\nunset -f deactivate\n}")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        sys.exit(f"Cannot activate workspace: {error}. Run the initializer with --restore.")
