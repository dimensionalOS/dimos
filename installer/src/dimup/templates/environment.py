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

"""Emit a shell environment delta and its inverse for manual deactivation."""

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
    before = dict(os.environ)
    script = """set -e
source "$1/.venv/bin/activate"
export PATH="$VIRTUAL_ENV/bin:$HOME/.cargo/bin:$HOME/.local/bin:$HOME/.nix-profile/bin:/nix/var/nix/profiles/default/bin:/opt/homebrew/bin:$PATH"
export NIX_CONFIG="${NIX_CONFIG:-}
extra-experimental-features = nix-command flakes"
if [ "$(uname -s)" = Darwin ]; then
    export DYLD_FALLBACK_LIBRARY_PATH="/opt/homebrew/opt/jpeg-turbo/lib:/opt/homebrew/lib${DYLD_FALLBACK_LIBRARY_PATH:+:$DYLD_FALLBACK_LIBRARY_PATH}"
    export PKG_CONFIG_PATH="/opt/homebrew/lib/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
fi
exec "$1/.venv/bin/python" -c 'import json, os; print(json.dumps(dict(os.environ)))'
"""
    output = subprocess.check_output(
        ["bash", "--noprofile", "--norc", "-c", script, "activate", str(root)], text=True
    )
    after = json.loads(output)
    print(changes(before, after))
    print("deactivate() {\n" + changes(after, before) + "\nunset -f deactivate\n}")


if __name__ == "__main__":
    main()
