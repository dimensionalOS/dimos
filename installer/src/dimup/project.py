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

"""Generate an editable SDK application pinned to one Git revision."""

from importlib.resources import files
import os
from pathlib import Path
import re
import shlex
import tempfile
from typing import Any

from dimup.process import Runner, SetupError, executable
from dimup.sdk import SDK_URL, consumer_policy
from rich.table import Table
from rich.text import Text
import tomli_w
import tomllib


def package_name(directory: Path) -> str:
    name = re.sub(r"[^a-z0-9]+", "-", directory.name.lower()).strip("-")
    if not name or not name[0].isalpha() or name in {"dimos", "dimup"}:
        raise SetupError(
            "Choose a directory name starting with a letter, other than dimos or dimup."
        )
    return name


def resolve_sdk(ref: str, runner: Runner) -> tuple[str, dict[str, Any]]:
    with tempfile.TemporaryDirectory(prefix="dimup-sdk-") as temporary:
        checkout = Path(temporary)
        git = executable("git")
        runner.run("Prepare SDK metadata", [git, "init", "--quiet", str(checkout)], capture=True)
        runner.run(
            "Resolve SDK revision",
            [git, "fetch", "--depth=1", "--filter=blob:none", "--no-tags", "--", SDK_URL, ref],
            cwd=checkout,
            capture=True,
        )
        sha = runner.run(
            "Read SDK commit", [git, "rev-parse", "FETCH_HEAD"], cwd=checkout, capture=True
        )
        source = runner.run(
            "Read SDK dependencies",
            [git, "show", f"{sha}:pyproject.toml"],
            cwd=checkout,
            capture=True,
        )
        return sha, tomllib.loads(source)


def manifest(name: str, sha: str, sdk: dict[str, Any]) -> dict[str, Any]:
    extras, policy = consumer_policy(sdk)
    # System Python builds (notably python.org macOS builds) may lack SQLite
    # extension loading. Keep the interpreter policy when applications are cloned.
    policy["python-preference"] = "only-managed"
    # uv sources apply to direct requirements, not arbitrary transitive packages.
    sourced_dependencies = sorted(policy.get("sources", {}))
    policy["environments"] = [
        "sys_platform == 'linux' and platform_machine == 'x86_64'",
        "sys_platform == 'darwin' and platform_machine == 'arm64'",
    ]
    policy.setdefault("sources", {})["dimos"] = {"git": SDK_URL, "rev": sha}
    return {
        "project": {
            "name": name,
            "version": "0.1.0",
            "requires-python": ">=3.12,<3.13",
            "dependencies": [f"dimos[{','.join(extras)}]", *sourced_dependencies],
            "entry-points": {
                "dimos.blueprints": {"demo": f"{name.replace('-', '_')}.demo:blueprint"}
            },
        },
        "build-system": {"requires": ["setuptools>=70"], "build-backend": "setuptools.build_meta"},
        "dependency-groups": {"dev": ["pytest>=8"]},
        "tool": {
            "uv": policy,
            "setuptools": {"packages": {"find": {"where": ["src"]}}},
            "pytest": {"ini_options": {"testpaths": ["tests"]}},
        },
    }


def write_activation(root: Path) -> None:
    (root / ".dimos").mkdir(exist_ok=True)
    templates = files("dimup") / "templates"
    for template, destination in (
        ("activate.sh", root / ".dimos/activate.sh"),
        ("environment.py", root / ".dimos/environment.py"),
        ("envrc", root / ".envrc"),
    ):
        destination.write_text((templates / template).read_text())


def write_project(root: Path, name: str, sha: str, sdk: dict[str, Any]) -> None:
    module = name.replace("-", "_")
    source = root / "src" / module
    source.mkdir(parents=True)
    (root / "tests").mkdir()
    write_activation(root)
    (root / "pyproject.toml").write_text(tomli_w.dumps(manifest(name, sha, sdk)))
    templates = files("dimup") / "templates"
    for template, destination in (
        ("demo.py", source / "demo.py"),
        ("test_demo.py", root / "tests/test_demo.py"),
        ("README.md", root / "README.md"),
    ):
        text = (
            (templates / template)
            .read_text()
            .replace("APP_MODULE", module)
            .replace("APP_NAME", name)
        )
        destination.write_text(text)
    (source / "__init__.py").touch()
    (root / ".gitignore").write_text(
        ".venv/\n.direnv/\n.dimos/*.log\n.env\n__pycache__/\n*.egg-info/\n.pytest_cache/\nbuild/\ndist/\n"
    )


def create(directory: Path, ref: str) -> None:
    root = directory.expanduser().absolute()
    name = package_name(root)
    if root.is_symlink() or (root.exists() and (not root.is_dir() or any(root.iterdir()))):
        raise SetupError(f"Destination must be new or empty: {root}")
    root.mkdir(parents=True, exist_ok=True)
    runner = Runner(root / ".dimos/setup.log")
    runner.console.print("\n  dimup · Create application\n", style="bold cyan")
    details = Table.grid(padding=(0, 2))
    details.add_column(style="dim")
    details.add_column()
    details.add_row("  Project", Text(str(root)))
    details.add_row("  SDK", Text(ref))
    runner.console.print(details)
    try:
        for tool in ("uv", "git", "cargo", "nix"):
            executable(tool)
        with runner.stage("Resolve SDK"):
            sha, sdk = resolve_sdk(ref, runner)
        if ref != sha:
            runner.console.print(Text(f"  {ref} → {sha[:12]}", style="dim"))
        with runner.stage("Create project files"):
            write_project(root, name, sha, sdk)
        env = dict(os.environ)
        tool_paths = [
            str(Path(executable(tool)).parent) for tool in ("uv", "git", "cargo", "nix")
        ]
        env["PATH"] = os.pathsep.join([*tool_paths, env.get("PATH", "")])
        env["GIT_LFS_SKIP_SMUDGE"] = "1"
        runner.run(
            "Install dependencies · uv sync",
            [executable("uv"), "sync", "--python", "3.12"],
            cwd=root,
            env=env,
        )
        python = root / ".venv/bin/python"
        runner.run(
            "Verify application",
            [
                str(python),
                "-c",
                (
                    "from importlib.metadata import distribution; import sys; "
                    "ep = next(e for e in distribution(sys.argv[1]).entry_points "
                    "if e.group == 'dimos.blueprints' and e.name == 'demo'); ep.load()"
                ),
                name,
            ],
            cwd=root,
            env=env,
        )
    except KeyboardInterrupt:
        runner.console.print(Text(f"Project kept: {root}\nLog: {runner.log}", style="dim"))
        raise
    except (SetupError, OSError, ValueError, KeyError) as error:
        raise SetupError(
            f"{error}\nThe project directory has been kept: {root}\n"
            "Fix the problem, then remove this directory or choose a new empty directory before retrying."
        ) from error
    runner.console.print(Text(f"\n  Ready · {name}\n", style="bold green"))
    runner.console.print(
        Text(
            f"  cd {shlex.quote(str(directory.expanduser()))}\n"
            f"  source .dimos/activate.sh\n"
            f"  dimos run {name}.demo\n"
        ),
        soft_wrap=True,
    )
    runner.console.print("  Optional: run direnv allow to activate automatically", style="dim")
    runner.console.print(Text(f"  Log: {runner.log}", style="dim"), soft_wrap=True)
