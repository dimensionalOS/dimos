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

from __future__ import annotations

import pytest

from dimos.utils import logging_config
from dimos.utils.logging_config import get_run_log_dir


@pytest.fixture(autouse=True)
def _worker_process(monkeypatch):
    """Module globals as a forkserver worker sees them: set_run_log_dir() never
    ran there, so _RUN_LOG_DIR is None and only the inherited env var remains."""
    monkeypatch.setattr(logging_config, "_RUN_LOG_DIR", None)
    monkeypatch.setattr(logging_config, "_LOG_FILE_PATH", None)
    monkeypatch.delenv("DIMOS_RUN_LOG_DIR", raising=False)


class TestGetRunLogDirEnvFallback:
    """get_run_log_dir() resolves the run directory a worker inherited by env."""

    def test_resolves_from_env_when_global_unset(self, tmp_path, monkeypatch):
        run_dir = tmp_path / "run-worker"
        monkeypatch.setenv("DIMOS_RUN_LOG_DIR", str(run_dir))
        assert get_run_log_dir() == run_dir

    def test_global_wins_over_env(self, tmp_path, monkeypatch):
        monkeypatch.setattr(logging_config, "_RUN_LOG_DIR", tmp_path / "from-global")
        monkeypatch.setenv("DIMOS_RUN_LOG_DIR", str(tmp_path / "from-env"))
        assert get_run_log_dir() == tmp_path / "from-global"

    def test_none_when_neither_is_set(self):
        assert get_run_log_dir() is None

    def test_does_not_create_the_directory(self, tmp_path, monkeypatch):
        """A getter reports; only set_run_log_dir()/_get_log_file_path() mkdir."""
        run_dir = tmp_path / "never-created"
        monkeypatch.setenv("DIMOS_RUN_LOG_DIR", str(run_dir))
        assert get_run_log_dir() == run_dir
        assert not run_dir.exists()

    def test_agrees_with_log_file_path(self, tmp_path, monkeypatch):
        """The two resolvers must land in the same directory -- the split is the
        bug: logging followed the env var while get_run_log_dir() returned None."""
        run_dir = tmp_path / "run-agree"
        monkeypatch.setenv("DIMOS_RUN_LOG_DIR", str(run_dir))
        resolved = get_run_log_dir()
        assert resolved is not None
        assert logging_config._get_log_file_path().parent == resolved


class TestRelocEvalRunLogResolution:
    """RelocEval finds this run's main.jsonl in a worker with no explicit
    run_log_file override -- the case that used to label every fix "unknown"."""

    def test_resolves_without_explicit_override(self, tmp_path, monkeypatch):
        # Imported in-test so dimos.utils tests keep no import-time dependency
        # on dimos.mapping.
        from dimos.mapping.relocalization.eval_module import resolve_run_log

        run_dir = tmp_path / "run-reloceval"
        run_dir.mkdir()
        main_jsonl = run_dir / "main.jsonl"
        main_jsonl.write_text('{"event": "relocalize", "fitness": 0.72}\n')
        monkeypatch.setenv("DIMOS_RUN_LOG_DIR", str(run_dir))

        assert resolve_run_log(None) == main_jsonl

    def test_none_when_run_log_missing(self, tmp_path, monkeypatch):
        """An env var pointing at a run that wrote no log stays unresolved
        rather than handing the parser a nonexistent path."""
        run_dir = tmp_path / "run-empty"
        run_dir.mkdir()
        monkeypatch.setenv("DIMOS_RUN_LOG_DIR", str(run_dir))

        from dimos.mapping.relocalization.eval_module import resolve_run_log

        assert resolve_run_log(None) is None

    def test_explicit_override_still_wins(self, tmp_path, monkeypatch):
        from dimos.mapping.relocalization.eval_module import resolve_run_log

        env_dir = tmp_path / "run-env"
        env_dir.mkdir()
        (env_dir / "main.jsonl").write_text("{}\n")
        override = tmp_path / "explicit.jsonl"
        override.write_text("{}\n")
        monkeypatch.setenv("DIMOS_RUN_LOG_DIR", str(env_dir))

        assert resolve_run_log(str(override)) == override
