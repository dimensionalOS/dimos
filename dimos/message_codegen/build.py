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

"""Language build frontends over the shared message-project generation plan."""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tarfile

from . import backend
from .generate import generate
from .native_build import prepare_cpp, write_cmake_toolchain
from .project import LANGUAGES, Project, prepare


@contextmanager
def project_directory(root: Path) -> Iterator[None]:
    previous = Path.cwd()
    try:
        os.chdir(root)
        yield
    finally:
        os.chdir(previous)


def local_cargo_dependency(text: str, module: str) -> str:
    """Relocate one versioned crate reference inside a local source bundle."""
    crate_name = module.replace("_", "-") + "-messages"

    def replace(match: re.Match[str]) -> str:
        declaration = re.sub(r',\s*path\s*=\s*"[^"]*"', "", match.group(1)).rstrip()
        return declaration + ", path = " + json.dumps("../" + module) + " }"

    return re.sub(
        r"(?m)^(" + re.escape(crate_name) + r"\s*=\s*\{[^}]*?)\s*\}$",
        replace,
        text,
    )


def build_project(
    project: Project,
    languages: tuple[str, ...] | None = None,
    *,
    install: bool = False,
    offline: bool = False,
) -> dict[str, str]:
    selected = languages or project.languages
    if not selected or any(language not in LANGUAGES for language in selected):
        raise ValueError("Select python, cpp and/or rust")
    if install and "python" in selected and sys.prefix == sys.base_prefix:
        raise ValueError(
            "Python --install requires an active virtual environment; no system install is performed"
        )
    for tool in ({"cmake"} if "cpp" in selected else set()) | (
        {"cargo"} if "rust" in selected else set()
    ):
        if shutil.which(tool) is None:
            raise ValueError(
                f"Missing build tool {tool}; prepare the language toolchain before building"
            )
    output = prepare(project, selected)
    dist = project.root / "dist"
    dist.mkdir(exist_ok=True)
    state = output / "artifacts.json"
    artifacts = json.loads(state.read_text()) if state.is_file() else {}
    artifacts.update(
        {
            "schemas": str(output / "schemas.json"),
            "ownership": str(output / "message-package.json"),
        }
    )
    dependencies = project.dependencies()
    if "python" in selected:
        with project_directory(project.root):
            wheel = backend.build_wheel(str(dist))
            backend.build_sdist(str(dist))
        artifacts["python_wheel"] = str(dist / wheel)
        if install:
            subprocess.run(
                [sys.executable, "-m", "pip", "install", "--no-deps", str(dist / wheel)], check=True
            )
    if set(selected) - {"python"}:
        output = prepare(project, selected)
    if "cpp" in selected:
        prefix = prepare_cpp(output, offline=offline)
        artifacts["cmake_prefix"] = str(prefix)
        artifacts["cmake_toolchain"] = str(
            write_cmake_toolchain(prefix, output / "toolchain.cmake")
        )
    if "rust" in selected:
        # A relocatable local source bundle works before any registry publication.
        bundle = output / "cargo-packages"
        lock_path = bundle / project.module / "Cargo.lock"
        previous_lock = lock_path.read_bytes() if lock_path.is_file() else None
        if bundle.exists():
            shutil.rmtree(bundle)
        crates = [(project.module, output), *((dep.module, dep.root) for dep in dependencies)]
        for module, root in crates:
            if not (root / "rust/Cargo.toml").is_file():
                manifest = json.loads((root / "message-package.json").read_text())
                available = {dep.module: dep for dep in dependencies}
                required = set(manifest["dependencies"])
                pending = list(required)
                while pending:
                    dep = available[pending.pop()]
                    children = json.loads((dep.root / "message-package.json").read_text())[
                        "dependencies"
                    ]
                    pending.extend(set(children) - required)
                    required.update(children)
                generated = output / "native-sources" / module
                generate(
                    [root / "schemas"],
                    generated,
                    manifest["owned"],
                    module,
                    version=manifest["version"],
                    shared=True,
                    languages=("rust",),
                    dependencies=tuple(available[name] for name in sorted(required)),
                )
                root = generated
            target = bundle / module
            shutil.copytree(
                root / "rust", target, ignore=shutil.ignore_patterns("target", "Cargo.lock")
            )
            manifest = target / "Cargo.toml"
            text = manifest.read_text()
            for dep in dependencies:
                text = local_cargo_dependency(text, dep.module)
            manifest.write_text(text)
        cargo_manifest = bundle / project.module / "Cargo.toml"
        if previous_lock is not None:
            lock_path.write_bytes(previous_lock)
        subprocess.run(
            [
                "cargo",
                "build",
                "--manifest-path",
                str(cargo_manifest),
                *(["--offline"] if offline else []),
                *(["--locked"] if previous_lock is not None else []),
            ],
            env={**os.environ, "CARGO_TARGET_DIR": str(output / "cargo-target")},
            check=True,
        )
        archive = dist / f"{project.name}-{project.version}-cargo.tar.gz"
        with tarfile.open(archive, "w:gz") as tar:
            tar.add(bundle, arcname=".")
        artifacts["cargo_manifest"] = str(cargo_manifest)
        artifacts["cargo_archive"] = str(archive)
    (output / "artifacts.json").write_text(json.dumps(artifacts, indent=2, sort_keys=True) + "\n")
    return artifacts
