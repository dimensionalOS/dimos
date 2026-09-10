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

"""Persist the Git LFS download policy in the user's configured shell."""

import os
from pathlib import Path
import pwd

from rich.text import Text

from dimup.process import Runner, SetupError


def configure_lfs(runner: Runner) -> None:
    shell = Path(os.environ.get("SHELL") or pwd.getpwuid(os.getuid()).pw_shell).name
    home = Path.home()
    command = "export GIT_LFS_SKIP_SMUDGE=1"
    if shell == "bash":
        profiles = [home / name for name in (".bash_profile", ".bash_login", ".profile")]
        paths = [home / ".bashrc", next((p for p in profiles if p.exists()), profiles[-1])]
    elif shell == "zsh":
        paths = [Path(os.environ.get("ZDOTDIR") or home) / ".zshrc"]
    elif shell == "fish":
        paths = [Path(os.environ.get("XDG_CONFIG_HOME") or home / ".config") / "fish/config.fish"]
        command = "set -gx GIT_LFS_SKIP_SMUDGE 1"
    else:
        runner.console.print(
            Text(
                f"Shell {shell!r} is not configured automatically. Add the equivalent of "
                f"{command!r} to your shell startup configuration."
            )
        )
        return
    block = f"# >>> dimup Git LFS >>>\n{command}\n# <<< dimup Git LFS <<<\n"
    for path in paths:
        try:
            content = path.read_text() if path.exists() else ""
            if block not in content:
                if "# >>> dimup Git LFS >>>" in content:
                    raise SetupError(
                        f"Review the existing dimup Git LFS block in {path} before rerunning setup."
                    )
                path.parent.mkdir(parents=True, exist_ok=True)
                with path.open("a") as output:
                    output.write(("\n" if content and not content.endswith("\n") else "") + block)
            runner.console.print(Text(f"Git LFS configured: {path}"))
        except OSError as error:
            raise SetupError(
                f"Cannot configure Git LFS in {path}: {error}. Check the file permissions and rerun dimup setup."
            ) from error
    runner.console.print(
        Text(
            "Automatic Git LFS downloads are disabled in future terminals for all repositories.\n"
            "Explicit git lfs pull still downloads requested files.\n"
            f"Apply in this terminal: {command}"
        )
    )
