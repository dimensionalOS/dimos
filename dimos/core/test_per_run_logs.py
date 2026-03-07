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

import os

import pytest

from dimos.utils import logging_config
from dimos.utils.logging_config import _get_log_file_path, get_run_log_dir, set_run_log_dir


@pytest.fixture(autouse=True)
def _clean_env(monkeypatch):
    """Remove DIMOS_RUN_LOG_DIR from env and reset module globals between tests."""
    monkeypatch.delenv("DIMOS_RUN_LOG_DIR", raising=False)
    monkeypatch.setattr(logging_config, "_RUN_LOG_DIR", None)
    monkeypatch.setattr(logging_config, "_LOG_FILE_PATH", None)


class TestSetRunLogDir:
    """set_run_log_dir() configures per-run logging."""

    def test_creates_directory(self, tmp_path):
        log_dir = tmp_path / "run-001"
        set_run_log_dir(log_dir)
        assert log_dir.is_dir()

    def test_sets_env_var(self, tmp_path):
        log_dir = tmp_path / "run-002"
        set_run_log_dir(log_dir)
        assert os.environ["DIMOS_RUN_LOG_DIR"] == str(log_dir)

    def test_get_run_log_dir_returns_path(self, tmp_path):
        log_dir = tmp_path / "run-003"
        set_run_log_dir(log_dir)
        assert get_run_log_dir() == log_dir


class TestLogFilePathRouting:
    """_get_log_file_path() routes to per-run directory when set."""

    def test_routes_to_run_dir(self, tmp_path):
        log_dir = tmp_path / "run-004"
        set_run_log_dir(log_dir)
        path = _get_log_file_path()
        assert path == log_dir / "main.jsonl"

    def test_routes_via_env_var(self, tmp_path, monkeypatch):
        env_dir = tmp_path / "env-run"
        monkeypatch.setenv("DIMOS_RUN_LOG_DIR", str(env_dir))

        path = logging_config._get_log_file_path()
        assert path == env_dir / "main.jsonl"
        assert env_dir.is_dir()

    def test_falls_back_to_legacy(self):
        path = logging_config._get_log_file_path()
        assert path.name.startswith("dimos_")
        assert path.suffix == ".jsonl"
