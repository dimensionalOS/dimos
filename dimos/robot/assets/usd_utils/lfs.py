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


from collections.abc import Iterable
from pathlib import Path
import re
import subprocess
from typing import TYPE_CHECKING

from dimos.utils.data import _initialize_git_lfs, _is_lfs_pointer_file, get_project_root

if TYPE_CHECKING:
    from dimos.robot.assets.model import UsdModel

# An asset path as usda writes one.
_ASSET_PATH = re.compile(r"@([^@\n]+)@")


def pull_usd_assets(layers: Iterable[str], root: Path | None = None) -> None:
    """Fetch, in one pull, the files these usda layers name that are Git LFS pointers: absolute names, and relative ones under `root`."""
    # Read off the text, not walked with pxr: a walk opens every layer the stage references, pointers or not.
    named = {Path(asset) for layer in layers for asset in _ASSET_PATH.findall(layer)}
    under = {root / path for path in named if not path.is_absolute()} if root is not None else set()
    pull_lfs_files({path for path in named if path.is_absolute()} | under)


def pull_model_assets(model: "UsdModel") -> None:
    """Fetch the Git LFS pointers a model's own layer file names, before anything opens a stage of it."""
    source = model.source_path
    if source is not None and source.suffix == ".usda":
        pull_usd_assets([source.read_text()], root=source.parent)


def pull_lfs_files(paths: Iterable[Path]) -> None:
    """Fetch, in one pull, every file among `paths` that is a Git LFS pointer (a clone the install docs make skips smudging, so it has them)."""
    pointers = sorted({path.resolve() for path in paths if _is_lfs_pointer_file(path)})
    if not pointers:
        return
    repo_root = get_project_root().resolve()
    fetchable = [
        str(path.relative_to(repo_root)) for path in pointers if path.is_relative_to(repo_root)
    ]
    if fetchable:
        _initialize_git_lfs(repo_root)
        # --exclude= overrides lfs.fetchexclude from .lfsconfig.
        subprocess.run(
            ["git", "lfs", "pull", "--include", ",".join(fetchable), "--exclude="],
            cwd=repo_root,
            check=True,
        )
    left = [str(path) for path in pointers if _is_lfs_pointer_file(path)]
    if left:
        raise RuntimeError(
            f"{len(left)} file(s) are still Git LFS pointers, not their content "
            f"(outside {repo_root}, or not on the LFS server): {left[:5]}"
        )
