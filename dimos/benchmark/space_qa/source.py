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

"""The pinned SPACE checkout this integration scores against.

SPACE is not vendored and not a wheel: it is a research repository that only
runs from a source tree. It is fetched once into the user cache at a pinned
revision, so a run always grades against the scorer this integration was
written for. Nothing here imports ``space``; the caller puts the returned path
on ``sys.path``.
"""

from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess
import tempfile

from dimos.constants import CACHE_DIR

SPACE_REPO_URL = "https://github.com/apple/ml-space-benchmark.git"
SPACE_REVISION = "564e43932adc84543800dd56b99cee37efaeabd8"
# Points at an existing checkout instead of the cached clone. It still has to
# sit at the pin: the revision, not the path, is what makes a score comparable.
SPACE_SOURCE_ENV = "DIMOS_SPACE_SOURCE"

GIT_TIMEOUT_SECONDS = 900.0

# What to do about a dirty tree depends on whose tree it is: the cached clone
# is disposable, an override is the user's own checkout.
_CACHE_DIRTY_ADVICE = (
    "restore it with `git checkout -- .` and remove what is untracked, or delete the "
    "checkout and let the next run clone it again"
)
_OVERRIDE_DIRTY_ADVICE = (
    "restore the tracked files with `git checkout -- .`, move the untracked ones out of "
    f"the checkout, or unset {SPACE_SOURCE_ENV} to score against the cached clone instead"
)


def space_cache_root() -> Path:
    """Everything SPACE brings with it lands here, outside the repository."""
    return CACHE_DIR / "space-benchmark"


def ensure_space_source() -> Path:
    """Return a SPACE checkout at the pinned revision, cloning it once if needed."""
    override = os.environ.get(SPACE_SOURCE_ENV)
    if override:
        return _use_override(override)
    checkout = space_cache_root() / "src" / "ml-space-benchmark"
    if (checkout / ".git").exists():
        if head_revision(checkout) != SPACE_REVISION:
            # Reached after the pin moves, or if someone worked in the cache.
            _git("-C", str(checkout), "fetch", "--quiet", "origin")
            _git("-C", str(checkout), "checkout", "--quiet", SPACE_REVISION)
        _require_a_clean_checkout(checkout, _CACHE_DIRTY_ADVICE)
        return checkout
    _clone(checkout)
    return checkout


def head_revision(checkout: Path) -> str:
    return _git("-C", str(checkout), "rev-parse", "HEAD")


def _use_override(override: str) -> Path:
    path = Path(override).expanduser()
    # `.git` is a file, not a directory, in a linked worktree.
    if not (path / ".git").exists():
        raise FileNotFoundError(
            f"{SPACE_SOURCE_ENV}={override} is not a git checkout of {SPACE_REPO_URL}"
        )
    head = head_revision(path)
    if head != SPACE_REVISION:
        raise RuntimeError(
            f"{SPACE_SOURCE_ENV}={override} is at {head}, but this integration grades "
            f"against {SPACE_REVISION}; check out the pin or unset {SPACE_SOURCE_ENV}"
        )
    _require_a_clean_checkout(path, _OVERRIDE_DIRTY_ADVICE)
    return path.resolve()


def _require_a_clean_checkout(checkout: Path, advice: str) -> None:
    """A checkout sitting at the pin can still not be the pinned scorer.

    ``HEAD`` names a commit, not the files that will be imported: an edited
    ``qa_agent.py`` or a stray module dropped beside it leaves the revision
    check untouched and still changes how every reply is read. The pin is what
    makes a score comparable, so anything the working tree adds to it is refused
    rather than quietly graded against.
    """
    changes = _git("-C", str(checkout), "status", "--porcelain")
    if not changes:
        return
    raise RuntimeError(
        f"the SPACE checkout at {checkout} is at {SPACE_REVISION} but its working tree has "
        f"been changed, so it is no longer the scorer that pin names:\n{changes}\n{advice}"
    )


def _clone(checkout: Path) -> None:
    """Clone into a sibling and rename, so an interrupted clone is never reused."""
    checkout.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f".{checkout.name}-", dir=checkout.parent))
    try:
        fresh = staging / checkout.name
        _git("clone", "--quiet", SPACE_REPO_URL, str(fresh))
        _git("-C", str(fresh), "checkout", "--quiet", SPACE_REVISION)
        shutil.rmtree(checkout, ignore_errors=True)
        os.replace(fresh, checkout)
    finally:
        shutil.rmtree(staging, ignore_errors=True)


def _git(*args: str) -> str:
    try:
        completed = subprocess.run(
            ["git", *args],
            capture_output=True,
            text=True,
            timeout=GIT_TIMEOUT_SECONDS,
            check=False,
        )
    except OSError as exc:
        raise RuntimeError(f"git is required to fetch SPACE from {SPACE_REPO_URL}: {exc}") from exc
    if completed.returncode != 0:
        detail = completed.stderr.strip() or completed.stdout.strip() or "no output"
        raise RuntimeError(f"`git {' '.join(args)}` failed: {detail}")
    return completed.stdout.strip()
