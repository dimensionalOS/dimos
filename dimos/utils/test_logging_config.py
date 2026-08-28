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

"""Tests for the compact console log formatter."""

from __future__ import annotations

import pytest

from dimos.utils import logging_config
from dimos.utils.logging_config import _compact_console_processor


def test_module_key_leads_the_kv_tail(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(logging_config, "_CONSOLE_USE_COLORS", False)
    line = _compact_console_processor(
        None,
        "info",
        {
            "timestamp": "2026-08-17T12:00:00.123Z",
            "level": "info",
            "logger": "dimos/utils/thing.py",
            "event": "tick",
            "alpha": 1,
            "module": "planner",
        },
    )
    assert line.endswith("tick module=planner alpha=1")
