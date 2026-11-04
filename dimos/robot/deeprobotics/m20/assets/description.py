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


"""The M20's vendor description, fetched as only the paths it needs (the vendor repo holds every DeepRobotics robot)."""

from __future__ import annotations

from hashlib import sha256
from pathlib import Path
import shutil
import tempfile
from typing import Any

from filelock import FileLock
from git import Repo

from dimos.robot.assets.git_cache import DEFAULT_ROBOT_ASSET_CACHE_ROOT, GitAssetCacheError
from dimos.robot.assets.source import RobotDescriptionSource


class SparseDescriptionSource(RobotDescriptionSource):
    """A description source that shallow-fetches only `sparse_paths` of `ref` (a commit sha)."""

    def __init__(self, url: str, ref: str, sparse_paths: tuple[str, ...]) -> None:
        super().__init__(url, ref)
        self.sparse_paths = tuple(sparse_paths)

    def checkout_path(self) -> Path:
        if self._checkout_path_cache is None:
            self._checkout_path_cache = _sparse_checkout(self.url, self.ref, self.sparse_paths)
        return self._checkout_path_cache

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, SparseDescriptionSource):
            return NotImplemented
        return (self.url, self.ref, self.sparse_paths) == (
            other.url,
            other.ref,
            other.sparse_paths,
        )

    def __hash__(self) -> int:
        return hash((self.url, self.ref, self.sparse_paths))

    def cbor_encode(self) -> dict[str, Any]:  # type: ignore[override]
        return {"url": self.url, "ref": self.ref, "sparse_paths": list(self.sparse_paths)}

    @classmethod
    def cbor_decode(cls, data: dict[str, Any]) -> SparseDescriptionSource:  # type: ignore[override]
        return cls(data["url"], data["ref"], tuple(data["sparse_paths"]))

    def __reduce__(self) -> tuple[Any, ...]:
        return (SparseDescriptionSource, (self.url, self.ref, self.sparse_paths))


def _sparse_checkout(url: str, ref: str, sparse_paths: tuple[str, ...]) -> Path:
    key = sha256("\0".join((url, ref, *sparse_paths)).encode()).hexdigest()[:16]
    checkout_path = DEFAULT_ROBOT_ASSET_CACHE_ROOT / "sparse" / key / Path(url.rstrip("/")).name
    checkout_path.parent.mkdir(parents=True, exist_ok=True)
    with FileLock(str(checkout_path.parent / ".lock")):
        if checkout_path.exists():
            return checkout_path
        temp_path = Path(tempfile.mkdtemp(prefix=".fetch-", dir=checkout_path.parent))
        try:
            repo = Repo.init(temp_path)
            repo.create_remote("origin", url)
            repo.git.sparse_checkout("set", "--no-cone", *sparse_paths)
            # Only the pinned commit, then only the blobs under the sparse paths.
            repo.git.fetch("--depth", "1", "--filter=blob:none", "origin", ref)
            repo.git.checkout("--detach", "FETCH_HEAD")
            temp_path.rename(checkout_path)
        except Exception as exc:
            shutil.rmtree(temp_path, ignore_errors=True)
            raise GitAssetCacheError(
                f"Failed to fetch {', '.join(sparse_paths)} from {url!r} at {ref!r}: {exc}"
            ) from exc
    return checkout_path
