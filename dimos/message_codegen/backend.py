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
import gzip
import os
from pathlib import Path
import tarfile
from typing import Any

from setuptools import build_meta

from .distribution import write_distribution
from .project import Project, prepare


@contextmanager
def python_project() -> Iterator[None]:
    project = Project.load(Path.cwd())
    output = prepare(project)
    dependencies = project.dependencies()
    write_distribution(
        output,
        project.module,
        (),
        project.version,
        dependencies=dependencies,
        shared=True,
        source_output=output,
    )
    previous = Path.cwd()
    old_prefix = os.environ.get("CMAKE_PREFIX_PATH")
    prefixes = [str(dep.root) for dep in dependencies]
    if old_prefix:
        prefixes.append(old_prefix)
    os.environ["CMAKE_PREFIX_PATH"] = os.pathsep.join(prefixes)
    try:
        os.chdir(output / "python")
        yield
    finally:
        os.chdir(previous)
        if old_prefix is None:
            os.environ.pop("CMAKE_PREFIX_PATH", None)
        else:
            os.environ["CMAKE_PREFIX_PATH"] = old_prefix


def get_requires_for_build_wheel(config_settings: dict[str, Any] | None = None) -> list[str]:
    return [
        "setuptools>=70",
        "wheel",
        "pybind11==3.0.1",
        "cmake>=3.20",
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
    if not project.internal and len(paths) == 1:
        raise ValueError("Cannot package a project with no message definitions")
    paths.extend(
        path for path in (project.root / "README.md", project.root / "LICENSE") if path.is_file()
    )
    epoch = int(os.environ.get("SOURCE_DATE_EPOCH", "0"))
    with (
        (destination / f"{name}.tar.gz").open("wb") as raw,
        gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=epoch) as compressed,
        tarfile.open(fileobj=compressed, mode="w") as archive,
    ):
        for path in paths:
            if path.is_symlink() or not path.resolve().is_relative_to(project.root):
                raise ValueError(f"Source file escapes the project: {path}")
            info = archive.gettarinfo(
                str(path), f"{name}/{path.relative_to(project.root).as_posix()}"
            )
            info.uid = info.gid = 0
            info.uname = info.gname = ""
            info.mtime = epoch
            with path.open("rb") as stream:
                archive.addfile(info, stream)
    return f"{name}.tar.gz"
