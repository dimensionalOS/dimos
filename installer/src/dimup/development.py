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

"""Clone and prepare DimOS for source contributions."""

import os
from pathlib import Path
import shlex
import shutil
import tempfile

from dimup.process import Runner, SetupError, executable
from dimup.project import write_activation
from dimup.sdk import SDK_URL, desktop_extras
from rich.table import Table
from rich.text import Text
import tomllib

VERIFY_CHECKOUT = """import importlib.metadata as m, json, sys
from pathlib import Path
import dimos
root = Path(sys.argv[1]).resolve()
assert Path(dimos.__file__).resolve() == root / "dimos/__init__.py", dimos.__file__
metadata = json.loads(m.distribution("dimos").read_text("direct_url.json") or "{}")
assert metadata.get("dir_info", {}).get("editable"), metadata
assert metadata.get("url") == root.as_uri(), metadata
print(f"Editable DimOS: {root}")
"""


def clone(root: Path, ref: str, runner: Runner, env: dict[str, str]) -> str:
    git = executable("git")
    runner.run("Clone DimOS", [git, "clone", "--no-checkout", "--", SDK_URL, str(root)], env=env)
    runner.run("Configure Git LFS", [git, "lfs", "install", "--local"], cwd=root, env=env)
    with runner.stage("Select revision"):
        branches = runner.run(
            "Read branches",
            [git, "for-each-ref", "--format=%(refname)", "refs/remotes/origin"],
            cwd=root,
            env=env,
            capture=True,
        ).splitlines()
        if f"refs/remotes/origin/{ref}" in branches:
            runner.run("Check out branch", [git, "switch", ref], cwd=root, env=env, capture=True)
        else:
            runner.run(
                "Fetch revision",
                [git, "fetch", "--", "origin", ref],
                cwd=root,
                env=env,
                capture=True,
            )
            runner.run(
                "Check out revision",
                [git, "switch", "--detach", "FETCH_HEAD"],
                cwd=root,
                env=env,
                capture=True,
            )
            runner.console.print(
                "  Detached checkout; create a branch before committing.", style="dim"
            )
        return runner.run(
            "Read commit", [git, "rev-parse", "HEAD"], cwd=root, env=env, capture=True
        )


def develop(directory: Path, ref: str) -> None:
    root = directory.expanduser().absolute()
    if root.is_symlink() or (root.exists() and (not root.is_dir() or any(root.iterdir()))):
        raise SetupError(f"Destination must be new or empty: {root}")
    with tempfile.TemporaryDirectory(prefix="dimup-dev-") as temporary:
        runner = Runner(Path(temporary) / "setup.log")
        runner.console.print("\n  dimup · Prepare contributor checkout\n", style="bold cyan")
        details = Table.grid(padding=(0, 2))
        details.add_column(style="dim")
        details.add_column()
        details.add_row("  Checkout", Text(str(root)))
        details.add_row("  Revision", Text(ref))
        runner.console.print(details)
        log = root / ".dimos/setup.log"
        try:
            tools = {name: executable(name) for name in ("uv", "git", "cargo", "nix", "deno")}
            env = dict(os.environ)
            env["PATH"] = os.pathsep.join(
                [*(str(Path(p).parent) for p in tools.values()), env.get("PATH", "")]
            )
            env["GIT_LFS_SKIP_SMUDGE"] = "1"
            env["UV_PYTHON_PREFERENCE"] = "only-managed"
            env["UV_PROJECT_ENVIRONMENT"] = str(root / ".venv")
            root.parent.mkdir(parents=True, exist_ok=True)
            sha = clone(root, ref, runner, env)
            runner.console.print(Text(f"  {ref} → {sha[:12]}", style="dim"))
            sdk = tomllib.loads((root / "pyproject.toml").read_text())
            if sdk.get("project", {}).get("name") != "dimos":
                raise SetupError("Selected revision is not a DimOS project.")
            extras = desktop_extras(sdk)
            runner.run(
                "Install development dependencies · uv sync",
                [
                    tools["uv"],
                    "sync",
                    "--locked",
                    "--python",
                    "3.12",
                    "--no-default-groups",
                    "--group",
                    "tests",
                    "--group",
                    "lint",
                    *[arg for extra in extras for arg in ("--extra", extra)],
                ],
                cwd=root,
                env=env,
            )
            with runner.stage("Prepare activation"):
                write_activation(root)
            python = root / ".venv/bin/python"
            runner.run(
                "Install commit hooks",
                [
                    str(python),
                    "-m",
                    "pre_commit",
                    "install",
                    "--hook-type",
                    "pre-commit",
                    "--hook-type",
                    "commit-msg",
                ],
                cwd=root,
                env=env,
            )
            runner.run(
                "Verify contributor checkout",
                [str(python), "-I", "-c", VERIFY_CHECKOUT, str(root)],
                cwd=Path(temporary),
                env=env,
            )
        except KeyboardInterrupt:
            runner.console.print(Text(f"Checkout kept: {root}\nLog: {log}"))
            raise
        except (SetupError, OSError, ValueError, KeyError) as error:
            raise SetupError(
                f"{str(error).replace(str(runner.log), str(log))}\nCheckout kept: {root}\nLog: {log}\n"
                "Fix the problem, then remove this directory or choose a new empty directory before retrying."
            ) from error
        finally:
            log.parent.mkdir(parents=True, exist_ok=True)
            if runner.log.exists():
                shutil.copyfile(runner.log, log)
            else:
                log.touch()
        runner.console.print(Text(f"\n  Ready · {root.name}\n", style="bold green"))
        runner.console.print(
            Text(
                f"  cd {shlex.quote(str(directory.expanduser()))}\n"
                "  source .dimos/activate.sh\n"
                "  dimos doctor\n"
                "  git switch -c feat/my-change\n"
            ),
            soft_wrap=True,
        )
        runner.console.print(
            "  Optional: run direnv allow to activate automatically (including Fish)", style="dim"
        )
        runner.console.print(Text(f"  Log: {log}"), soft_wrap=True)
