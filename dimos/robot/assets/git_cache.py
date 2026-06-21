# Copyright 2025-2026 Dimensional Inc.
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

"""Git-backed cache for source repositories and other assets."""

from __future__ import annotations

from dataclasses import dataclass
from hashlib import sha256
from pathlib import Path
import shutil
import tempfile
from urllib.parse import urlparse
import warnings

from filelock import FileLock
from git import GitCommandError, InvalidGitRepositoryError, NoSuchPathError, Repo
from platformdirs import user_cache_path

DEFAULT_ROBOT_ASSET_CACHE_ROOT = user_cache_path("dimos", appauthor=False) / "robot_assets"


class GitAssetCacheError(RuntimeError):
    """Raised when an asset source cannot be resolved."""


class GitAssetCacheWarning(RuntimeWarning):
    """Warning emitted when a cached checkout is usable but not fresh."""


@dataclass(frozen=True)
class GitAssetCheckout:
    """Resolved local checkout information."""

    path: Path
    repo_url: str
    ref: str
    updated: bool = False
    used_cached_fallback: bool = False
    skipped_dirty_update: bool = False


class GitAssetCache:
    """Resolve `(repo_url, ref)` pairs into fresh-when-safe cached checkouts.

    Policy:
    - clone when the cache is missing;
    - for clean cached repositories, fetch and check out the declared ref;
    - if fetching/updating a cached repository fails, warn and reuse the cache;
    - if the cached repository has local changes, warn and skip updates.
    """

    def __init__(self, cache_root: Path | str = DEFAULT_ROBOT_ASSET_CACHE_ROOT) -> None:
        self.cache_root = Path(cache_root).expanduser()
        self._sources_root = self.cache_root / "sources"
        self._locks_root = self.cache_root / "locks"

    def resolve(self, repo_url: str, ref: str) -> GitAssetCheckout:
        """Return a local checkout for `repo_url` at `ref`."""
        key = self._source_key(repo_url, ref)
        checkout_path = self._sources_root / key / self._repo_slug(repo_url)
        lock_path = self._locks_root / f"{key}.lock"

        self._sources_root.mkdir(parents=True, exist_ok=True)
        self._locks_root.mkdir(parents=True, exist_ok=True)

        with FileLock(str(lock_path)):
            if not checkout_path.exists():
                return self._clone_missing(repo_url, ref, checkout_path)
            return self._refresh_cached(repo_url, ref, checkout_path)

    @staticmethod
    def _source_key(repo_url: str, ref: str) -> str:
        return sha256(f"{repo_url}\0{ref}".encode()).hexdigest()[:16]

    @staticmethod
    def _repo_slug(repo_url: str) -> str:
        parsed_path = urlparse(repo_url).path or repo_url
        slug = Path(parsed_path.rstrip("/")).name
        if slug.endswith(".git"):
            slug = slug[:-4]
        slug = "".join(char if char.isalnum() or char in {"-", "_", "."} else "-" for char in slug)
        return slug or "checkout"

    def _clone_missing(self, repo_url: str, ref: str, checkout_path: Path) -> GitAssetCheckout:
        temp_parent = checkout_path.parent
        temp_parent.mkdir(parents=True, exist_ok=True)
        temp_path = Path(tempfile.mkdtemp(prefix=f".{checkout_path.name}-", dir=temp_parent))
        try:
            repo = Repo.clone_from(repo_url, temp_path)
            self._checkout_ref(repo, ref)
            temp_path.rename(checkout_path)
            return GitAssetCheckout(path=checkout_path, repo_url=repo_url, ref=ref, updated=True)
        except Exception as exc:
            shutil.rmtree(temp_path, ignore_errors=True)
            raise GitAssetCacheError(
                f"Failed to fetch Git asset source {repo_url!r} at ref {ref!r}: {exc}"
            ) from exc

    def _refresh_cached(self, repo_url: str, ref: str, checkout_path: Path) -> GitAssetCheckout:
        try:
            repo = Repo(checkout_path)
        except (InvalidGitRepositoryError, NoSuchPathError) as exc:
            raise GitAssetCacheError(
                f"Cached asset path {checkout_path} is not a valid Git repository"
            ) from exc

        if self._is_dirty(repo):
            warnings.warn(
                f"Git asset cache {checkout_path} has local changes; skipping upstream update.",
                GitAssetCacheWarning,
                stacklevel=2,
            )
            return GitAssetCheckout(
                path=checkout_path,
                repo_url=repo_url,
                ref=ref,
                skipped_dirty_update=True,
            )

        try:
            repo.remotes.origin.fetch(tags=True)
            before = repo.head.commit.hexsha if repo.head.is_valid() else None
            self._checkout_ref(repo, ref)
            after = repo.head.commit.hexsha if repo.head.is_valid() else None
            return GitAssetCheckout(
                path=checkout_path, repo_url=repo_url, ref=ref, updated=before != after
            )
        except Exception as exc:
            warnings.warn(
                f"Could not update Git asset cache {checkout_path}; using cached checkout: {exc}",
                GitAssetCacheWarning,
                stacklevel=2,
            )
            return GitAssetCheckout(
                path=checkout_path,
                repo_url=repo_url,
                ref=ref,
                used_cached_fallback=True,
            )

    @staticmethod
    def _is_dirty(repo: Repo) -> bool:
        return repo.is_dirty(untracked_files=True)

    @staticmethod
    def _checkout_ref(repo: Repo, ref: str) -> None:
        """Check out a branch, tag, or commit and update clean worktrees safely."""
        remote_ref = f"origin/{ref}"
        remote_refs = {str(r) for r in repo.remotes.origin.refs}

        if remote_ref in remote_refs:
            repo.git.checkout("-B", ref, remote_ref)
            repo.git.reset("--hard", remote_ref)
            return

        try:
            repo.git.checkout(ref)
        except GitCommandError as exc:
            raise GitAssetCacheError(f"Could not check out ref {ref!r}") from exc
