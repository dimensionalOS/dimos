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
import subprocess

from dimos.utils.data import _initialize_git_lfs, _is_lfs_pointer_file, get_project_root


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
