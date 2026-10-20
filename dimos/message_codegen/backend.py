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

"""PEP 517/660 backend for standalone message projects, not the DimOS runtime."""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
from email.message import EmailMessage
from email.policy import default
import gzip
from io import BytesIO
import json
import os
from pathlib import Path
import shutil
import sys
import tarfile
from tempfile import TemporaryDirectory
from typing import Any

from filelock import FileLock
from setuptools import build_meta
import tomli_w

from dimos.message_codegen.project import Project, prepare

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib


def _metadata(project: Project) -> tuple[dict[str, Any], list[Path]]:
    data = tomllib.loads((project.root / "pyproject.toml").read_text())["project"]
    data = dict(data)
    data["dependencies"] = list(
        dict.fromkeys(
            [*data.get("dependencies", []), "dimos-cdrgen==0.1.1", *project.requirements()]
        )
    )
    entry_points = dict(data.get("entry-points", {}))
    providers = dict(entry_points.get("dimos.messages", {}))
    provider = project.python_module + "_schemas.provider"
    if providers.get(project.python_module, provider) != provider:
        raise ValueError("Generated message provider entry point cannot be overridden")
    providers[project.python_module] = provider
    entry_points["dimos.messages"] = providers
    data["entry-points"] = entry_points
    if "dynamic" in data:
        data["dynamic"] = [name for name in data["dynamic"] if name != "dependencies"]
    resources = set()
    for field in ("readme", "license"):
        value = data.get(field)
        filename = (
            value.get("file") if isinstance(value, dict) else value if field == "readme" else None
        )
        if filename:
            resources.add(project.root / filename)
    for pattern in data.get("license-files", []):
        resources.update(project.root.glob(pattern))
    resources.update(
        path for path in (project.root / "README.md", project.root / "LICENSE") if path.is_file()
    )
    for path in resources:
        if (
            path.is_symlink()
            or not path.resolve().is_relative_to(project.root)
            or not path.is_file()
        ):
            raise ValueError(f"Invalid project metadata resource: {path}")
    return data, sorted(resources)


def _native_sources(project: Project) -> list[Path]:
    if not (project.root / "Cargo.toml").is_file():
        return []
    paths = [project.root / "Cargo.toml", project.root / "build.rs"]
    for directory in ("src", "examples", "tests"):
        paths.extend((project.root / directory).rglob("*.rs"))
    for path in paths:
        if (
            not path.is_file()
            or path.is_symlink()
            or not path.resolve().is_relative_to(project.root)
        ):
            raise ValueError(f"Invalid native source artifact: {path}")
    return sorted(paths)


@contextmanager
def python_project() -> Iterator[None]:
    project = Project.load(Path.cwd())
    output = prepare(project, ("python",))
    metadata, resources = _metadata(project)
    # Packaging mutates egg-info/build trees. Keep it separate from immutable
    # generation caches and the stable editable source path.
    package = project.root / "build/message-python"
    with FileLock(str(project.root / "build/.python-package.lock")):
        with TemporaryDirectory(prefix=".python-package-", dir=package.parent) as directory:
            stage = Path(directory) / "python"
            shutil.copytree(output / "python", stage)
            for source in resources:
                target = stage / source.relative_to(project.root)
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(source, target)
            module = project.python_module
            support = module + "_schemas"
            (stage / "setup.py").write_text(
                "from setuptools import find_namespace_packages, setup\n"
                + f"setup(packages=find_namespace_packages(include={[module, module + '.*', support, support + '.*']!r}), "
                + f"package_data={ {module: ['py.typed', '*.pyi', '**/*.pyi'], support: ['schemas.json', 'schemas/**/*', 'package/**/*']}!r})\n"
            )
            native_sources = _native_sources(project)
            if native_sources:
                native = stage / (
                    project.python_module.replace(".", "/") + "_schemas/package/native-project"
                )
                for path in [project.root / "pyproject.toml", *resources, *native_sources]:
                    target = native / path.relative_to(project.root)
                    target.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copyfile(path, target)
                owned = json.loads((output / "message-package.json").read_text())["owned"]
                for name in owned:
                    target = native / project.source.relative_to(project.root) / (name + ".msg")
                    target.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copyfile(output / "schemas" / (name + ".msg"), target)
            (stage / "pyproject.toml").write_text(
                tomli_w.dumps(
                    {
                        "build-system": {
                            "requires": ["setuptools>=77", "wheel"],
                            "build-backend": "setuptools.build_meta",
                        },
                        "project": metadata,
                    }
                )
            )
            previous = Path(directory) / "previous"
            if package.exists():
                package.rename(previous)
            try:
                stage.rename(package)
            except OSError:
                if previous.exists():
                    previous.rename(package)
                raise
        previous_cwd = Path.cwd()
        try:
            os.chdir(package)
            yield
        finally:
            os.chdir(previous_cwd)


def get_requires_for_build_wheel(config_settings: dict[str, Any] | None = None) -> list[str]:
    return [
        "setuptools>=77",
        "wheel",
        *Project.load(Path.cwd()).requirements(),
    ]


get_requires_for_build_editable = get_requires_for_build_wheel


def get_requires_for_build_sdist(config_settings: dict[str, Any] | None = None) -> list[str]:
    return []


def build_wheel(
    wheel_directory: str,
    config_settings: dict[str, Any] | None = None,
    metadata_directory: str | None = None,
) -> str:
    destination = str(Path(wheel_directory).resolve())
    metadata = str(Path(metadata_directory).resolve()) if metadata_directory else None
    with python_project():
        return str(build_meta.build_wheel(destination, config_settings, metadata))


def build_editable(
    wheel_directory: str,
    config_settings: dict[str, Any] | None = None,
    metadata_directory: str | None = None,
) -> str:
    destination = str(Path(wheel_directory).resolve())
    metadata = str(Path(metadata_directory).resolve()) if metadata_directory else None
    with python_project():
        return str(build_meta.build_editable(destination, config_settings, metadata))


def prepare_metadata_for_build_wheel(
    metadata_directory: str, config_settings: dict[str, Any] | None = None
) -> str:
    destination = str(Path(metadata_directory).resolve())
    with python_project():
        return str(build_meta.prepare_metadata_for_build_wheel(destination, config_settings))


prepare_metadata_for_build_editable = prepare_metadata_for_build_wheel


def build_sdist(sdist_directory: str, config_settings: dict[str, Any] | None = None) -> str:
    project = Project.load(Path.cwd())
    if any("path" in spec for spec in project.dependency_specs.values()):
        raise ValueError(
            "Source distributions require installed, version-pinned dependencies; remove development dependency paths"
        )
    name = f"{project.name.replace('-', '_')}-{project.version}"
    destination = Path(sdist_directory).resolve()
    destination.mkdir(parents=True, exist_ok=True)
    paths = [project.root / "pyproject.toml", *sorted(project.source.rglob("*.msg"))]
    if len(paths) == 1:
        raise ValueError("Cannot package a project with no message definitions")
    metadata, resources = _metadata(project)
    paths = sorted({*paths, *resources, *_native_sources(project)})
    for path in paths:
        if path.is_symlink() or not path.resolve().is_relative_to(project.root):
            raise ValueError(f"Source file escapes the project: {path}")
    core_metadata = EmailMessage(policy=default.clone(utf8=True, max_line_length=0))
    core_metadata["Metadata-Version"] = "2.4"
    core_metadata["Name"] = project.name
    core_metadata["Version"] = project.version
    if "requires-python" in metadata:
        core_metadata["Requires-Python"] = metadata["requires-python"]
    if "description" in metadata:
        core_metadata["Summary"] = metadata["description"]
    for requirement in metadata["dependencies"]:
        core_metadata["Requires-Dist"] = requirement
    pkg_info = core_metadata.as_bytes()
    epoch = int(os.environ.get("SOURCE_DATE_EPOCH", "0"))
    with (
        (destination / f"{name}.tar.gz").open("wb") as raw,
        gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=epoch) as compressed,
        tarfile.open(fileobj=compressed, mode="w") as archive,
    ):
        record = tarfile.TarInfo(f"{name}/PKG-INFO")
        record.size = len(pkg_info)
        record.mtime = epoch
        archive.addfile(record, BytesIO(pkg_info))
        for path in paths:
            info = archive.gettarinfo(
                str(path), f"{name}/{path.relative_to(project.root).as_posix()}"
            )
            info.mode = 0o644
            info.uid = info.gid = 0
            info.uname = info.gname = ""
            info.mtime = epoch
            with path.open("rb") as stream:
                archive.addfile(info, stream)
    return f"{name}.tar.gz"
