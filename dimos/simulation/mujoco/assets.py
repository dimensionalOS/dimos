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

from contextlib import suppress
from importlib.util import find_spec
import os
from pathlib import Path
import subprocess

from dimos.simulation.mujoco import constants
from dimos.utils import data

_BUNDLE_POINTER = (
    b"version https://git-lfs.github.com/spec/v1\n"
    b"oid sha256:5d19265c8aefb6e80b92b803ae164f18d214b827f9f090436eccdf831c5db236\n"
)


def _has_original_meshes(checkout: Path) -> bool:
    directories = ("unitree_go1/assets", "unitree_g1/assets")
    paths = sorted(
        file.relative_to(checkout).as_posix()
        for directory in directories
        for file in (checkout / directory).iterdir()
        if file.is_file()
    )
    if not paths:
        return False

    # Compare the working files with the pinned tree, including sparse or locally edited checkouts.
    tree = subprocess.check_output(
        ["git", "ls-tree", "-r", constants.MENAGERIE_COMMIT, "--", *directories],
        cwd=checkout,
        text=True,
        stderr=subprocess.DEVNULL,
        env={**os.environ, "GIT_ALLOW_PROTOCOL": ""},
    )
    expected = {}
    for entry in tree.splitlines():
        metadata, path = entry.split("\t", 1)
        expected[path] = metadata.split()[2]
    if expected.keys() != set(paths):
        return False
    hashes = subprocess.check_output(
        ["git", "hash-object", "--no-filters", "--", *paths],
        cwd=checkout,
        text=True,
        stderr=subprocess.DEVNULL,
    )
    return hashes.splitlines() == [expected[path] for path in paths]


def unitree_meshes() -> Path:
    data_dir = data.get_data_dir()
    bundled = data_dir / "mujoco_menagerie"
    if bundled.is_dir():
        return bundled

    # Before the first bundled download, an upgrade may already have exactly these meshes.
    spec = find_spec("mujoco_playground")
    # An exact upgrade can remove the Python package and leave its data in a namespace.
    for location in (spec.submodule_search_locations if spec else None) or ():
        checkout = Path(location) / "external_deps" / "mujoco_menagerie"
        head = checkout / ".git" / "HEAD"
        with suppress(OSError, subprocess.CalledProcessError):
            if head.read_bytes().strip() == constants.MENAGERIE_COMMIT.encode():
                archive = data_dir / ".lfs/mujoco_menagerie.tar.gz"
                legacy_bundle = not archive.exists()
                if not legacy_bundle:
                    with archive.open("rb") as stream:
                        legacy_bundle = stream.read(len(_BUNDLE_POINTER)) == _BUNDLE_POINTER
                if legacy_bundle and _has_original_meshes(checkout):
                    return checkout

    return data.get_data("mujoco_menagerie")
