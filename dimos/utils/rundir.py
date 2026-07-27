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

"""Per-run directory minting: ``NNN_label`` counter dirs + atomic ``latest`` (T16).

Spec: ``dimos/pure/tasks/t16-rundir.md``. Entrypoint mains call
``mint_run_dir(label)`` at startup to claim one ``LOG_DIR/NNN_label/`` dir,
route logs (``set_run_log_dir``) and the debug.db into it, and repoint the
``LOG_DIR/latest`` symlink — one ordered timeline of runs, never cross-mixed.
Library code (``over()``, module constructors, tests) NEVER mints implicitly.
"""

from __future__ import annotations

import os
from pathlib import Path
import re

from dimos.constants import LOG_DIR
from dimos.utils.logging_config import set_run_log_dir

_COUNTER_RE = re.compile(r"^(\d{3,})_")
_SLUG_RE = re.compile(r"[^a-z0-9]+")


def slugify(label: str) -> str:
    """Lowercase to ``[a-z0-9-]``, collapse separators, trim dashes; empty → ``run``."""
    slug = _SLUG_RE.sub("-", label.lower()).strip("-")
    return slug or "run"


def _next_counter(root: Path) -> int:
    """One past the highest existing ``NNN_`` prefix under ``root`` (1 if none)."""
    best = 0
    if root.is_dir():
        for child in root.iterdir():
            m = _COUNTER_RE.match(child.name)
            if m:
                best = max(best, int(m.group(1)))
    return best + 1


def _claim(root: Path, slug: str) -> Path:
    """Claim the next free ``NNN_slug`` dir via ``mkdir(exist_ok=False)`` (race-safe)."""
    n = _next_counter(root)
    while True:
        candidate = root / f"{n:03d}_{slug}"
        try:
            candidate.mkdir(exist_ok=False)
            return candidate
        except FileExistsError:
            n += 1


def _repoint_latest(root: Path, target: Path) -> None:
    """Atomically point ``root/latest`` at ``target`` (symlink + os.replace; no dangle)."""
    try:
        dest = os.path.relpath(target, root)
    except ValueError:  # different drive (Windows) — fall back to absolute
        dest = str(target)
    tmp = root / f".latest.{os.getpid()}.tmp"
    try:
        tmp.unlink()
    except FileNotFoundError:
        pass
    try:
        os.symlink(dest, tmp)
        os.replace(tmp, root / "latest")
    except OSError:
        try:
            tmp.unlink()
        except FileNotFoundError:
            pass


def mint_run_dir(label: str) -> Path:
    """Claim ``LOG_DIR/NNN_label``, route logs into it, repoint ``LOG_DIR/latest``.

    ``GlobalConfig.run_dir`` overrides the counter: an absolute path is used
    verbatim, a relative one resolves under ``LOG_DIR``, and an existing dir is
    appended into (resume). Either way logs + debug.db land there and ``latest``
    is repointed. Returns the run dir.
    """
    from dimos.core.global_config import global_config

    root = LOG_DIR
    root.mkdir(parents=True, exist_ok=True)
    override = global_config.run_dir
    if override:
        p = Path(override)
        run_dir = p if p.is_absolute() else root / p
        run_dir.mkdir(parents=True, exist_ok=True)
    else:
        run_dir = _claim(root, slugify(label))
    set_run_log_dir(run_dir)
    _repoint_latest(root, run_dir)
    return run_dir
