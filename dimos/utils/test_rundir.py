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

"""T16 per-run-dir minting tests (spec: dimos/pure/tasks/t16-rundir.md)."""

from __future__ import annotations

import os
from pathlib import Path
import threading

import pytest

from dimos.utils import logging_config, rundir


@pytest.fixture(autouse=True)
def _isolate(tmp_path, monkeypatch):
    """Pin LOG_DIR to a tmp dir and reset run-log globals/env between tests."""
    monkeypatch.setattr(rundir, "LOG_DIR", tmp_path / "logs")
    monkeypatch.delenv("DIMOS_RUN_LOG_DIR", raising=False)
    monkeypatch.setattr(logging_config, "_RUN_LOG_DIR", None)
    monkeypatch.setattr(logging_config, "_LOG_FILE_PATH", None)
    from dimos.core.global_config import global_config

    monkeypatch.setattr(global_config, "run_dir", None)
    yield


class TestSlugify:
    """slugify() normalizes labels to [a-z0-9-]."""

    def test_lowercases_and_dashes(self):
        assert rundir.slugify("Go2 Nav Pure") == "go2-nav-pure"

    def test_strips_punctuation_and_edges(self):
        assert rundir.slugify("  Unitree_Go2!! ") == "unitree-go2"

    def test_empty_falls_back(self):
        assert rundir.slugify("***") == "run"


class TestCounter:
    """mint_run_dir() claims NNN_label with a global incrementing counter."""

    def test_first_is_001(self):
        d = rundir.mint_run_dir("go2-nav-pure")
        assert d.name == "001_go2-nav-pure"

    def test_increments_globally_across_labels(self):
        a = rundir.mint_run_dir("go2-nav-pure")
        b = rundir.mint_run_dir("unitree-go2")
        assert a.name == "001_go2-nav-pure"
        assert b.name == "002_unitree-go2"

    def test_counter_is_max_plus_one(self, tmp_path):
        (rundir.LOG_DIR).mkdir(parents=True)
        (rundir.LOG_DIR / "007_old").mkdir()
        d = rundir.mint_run_dir("next")
        assert d.name == "008_next"

    def test_rollover_past_999(self):
        rundir.LOG_DIR.mkdir(parents=True)
        (rundir.LOG_DIR / "999_old").mkdir()
        d = rundir.mint_run_dir("big")
        assert d.name == "1000_big"

    def test_sets_run_log_dir_and_env(self):
        d = rundir.mint_run_dir("x")
        assert os.environ["DIMOS_RUN_LOG_DIR"] == str(d)
        assert logging_config.get_run_log_dir() == d


class TestLatestSymlink:
    """LOG_DIR/latest is atomically repointed at every mint."""

    def test_points_at_last_mint(self):
        rundir.mint_run_dir("first")
        second = rundir.mint_run_dir("second")
        link = rundir.LOG_DIR / "latest"
        assert link.is_symlink()
        assert link.resolve() == second.resolve()

    def test_repoint_is_relative(self):
        d = rundir.mint_run_dir("first")
        link = rundir.LOG_DIR / "latest"
        assert os.readlink(link) == d.name  # relative, not absolute

    def test_never_dangles_during_repoint(self):
        rundir.mint_run_dir("first")
        rundir.mint_run_dir("second")
        link = rundir.LOG_DIR / "latest"
        # A repoint that left a window would leave latest dangling; it resolves.
        assert link.exists()


class TestConcurrentClaim:
    """Racing claims get distinct dirs (mkdir(exist_ok=False) retry loop)."""

    def test_threads_race_to_distinct_dirs(self):
        root = rundir.LOG_DIR
        root.mkdir(parents=True)
        results: list[Path] = []
        lock = threading.Lock()
        barrier = threading.Barrier(8)

        def claim() -> None:
            barrier.wait()  # maximize the race
            d = rundir._claim(root, "race")
            with lock:
                results.append(d)

        threads = [threading.Thread(target=claim) for _ in range(8)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert len(results) == 8
        assert len({d.name for d in results}) == 8  # all distinct
        assert all(d.is_dir() for d in results)


class TestRunDirOverride:
    """GlobalConfig.run_dir overrides the counter (abs, relative, resume)."""

    def _set_override(self, monkeypatch, value):
        from dimos.core.global_config import global_config

        monkeypatch.setattr(global_config, "run_dir", value)

    def test_absolute_used_verbatim(self, tmp_path, monkeypatch):
        target = tmp_path / "custom" / "run"
        self._set_override(monkeypatch, str(target))
        d = rundir.mint_run_dir("ignored")
        assert d == target
        assert d.is_dir()
        assert os.environ["DIMOS_RUN_LOG_DIR"] == str(target)

    def test_relative_resolves_under_log_dir(self, monkeypatch):
        self._set_override(monkeypatch, "myrun")
        d = rundir.mint_run_dir("ignored")
        assert d == rundir.LOG_DIR / "myrun"
        assert d.is_dir()

    def test_appends_into_existing_dir(self, monkeypatch):
        self._set_override(monkeypatch, "resume")
        first = rundir.mint_run_dir("ignored")
        (first / "keep.txt").write_text("hi")
        second = rundir.mint_run_dir("ignored")
        assert second == first
        assert (second / "keep.txt").read_text() == "hi"  # not clobbered

    def test_override_repoints_latest(self, monkeypatch):
        self._set_override(monkeypatch, "myrun")
        d = rundir.mint_run_dir("ignored")
        assert (rundir.LOG_DIR / "latest").resolve() == d.resolve()


class TestDebugrecFallbackMints:
    """debugrec._resolve_db mints a run dir when no run dir exists (no shared db)."""

    def test_no_run_dir_mints_labeled_debug(self, monkeypatch):
        from dimos.pure import debugrec

        # No Debug.db, no run_dir arg, no $DIMOS_RUN_LOG_DIR → must mint, not
        # fall back to a shared logs/debug.db.
        db = debugrec._resolve_db(debugrec.Debug(), None)
        db_path = Path(db)
        assert db_path.name == debugrec.DEFAULT_DB_NAME
        run_dir = db_path.parent
        assert run_dir.parent == rundir.LOG_DIR
        assert run_dir.name.endswith("_debug")
        assert run_dir.is_dir()

    def test_explicit_db_is_untouched(self, tmp_path, monkeypatch):
        from dimos.pure import debugrec

        explicit = tmp_path / "mine.db"
        db = debugrec._resolve_db(debugrec.Debug(db=str(explicit)), None)
        assert Path(db) == explicit.resolve()
        assert not (rundir.LOG_DIR / "001_debug").exists()  # no minting

    def test_env_run_dir_is_untouched(self, tmp_path, monkeypatch):
        from dimos.pure import debugrec

        env_dir = tmp_path / "env-run"
        monkeypatch.setenv("DIMOS_RUN_LOG_DIR", str(env_dir))
        db = debugrec._resolve_db(debugrec.Debug(), None)
        assert Path(db) == (env_dir / debugrec.DEFAULT_DB_NAME).resolve()


def _make_db(run_dir: Path) -> None:
    """Create a real debug.db under run_dir (a valid sqlite the reader can open)."""
    from dimos.pure import debugrec

    run_dir.mkdir(parents=True, exist_ok=True)
    writer = debugrec.DebugWriter.open(run_dir / debugrec.DEFAULT_DB_NAME)
    writer.append_row(
        "m.decisions",
        debugrec.TickDecision(0, 0.0, False, "x", {}, None, False, 0),
        ts=0.0,
    )
    writer.close()


class TestLatestReader:
    """debugrec.latest() resolves via the latest symlink first, then highest NNN."""

    def test_prefers_latest_symlink(self, monkeypatch):
        from dimos.pure import debugrec

        # Two run dirs; the symlink points at the OLDER one → latest() follows it.
        root = rundir.LOG_DIR
        old = root / "001_a"
        new = root / "002_b"
        _make_db(old)
        _make_db(new)
        rundir._repoint_latest(root, old)
        run = debugrec.latest(root)
        try:
            assert Path(run.db) == (old / debugrec.DEFAULT_DB_NAME).resolve()
        finally:
            run.close()

    def test_falls_back_to_highest_nnn(self, monkeypatch):
        from dimos.pure import debugrec

        root = rundir.LOG_DIR
        for name in ("001_a", "005_b", "003_c"):
            _make_db(root / name)
        run = debugrec.latest(root)  # no symlink → highest NNN wins
        try:
            assert Path(run.db) == (root / "005_b" / debugrec.DEFAULT_DB_NAME).resolve()
        finally:
            run.close()
