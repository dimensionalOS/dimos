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

import pytest

from dimos.core.global_config import GlobalConfig


class TestGlobalConfigSecurityDefaults:
    """Network services must bind to localhost by default (not 0.0.0.0)."""

    def test_listen_host_defaults_to_localhost(self) -> None:
        config = GlobalConfig()
        assert config.listen_host == "127.0.0.1", (
            f"listen_host must default to 127.0.0.1, got {config.listen_host}"
        )


def test_replay_database_can_be_configured_for_existing_command(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    uri = f"dimos-replay://alice/go2-debug/{'a' * 64}?server=https%3A%2F%2Freplays.example"
    monkeypatch.setenv("DIMOS_REPLAY_DB", uri)

    assert GlobalConfig().replay_db == uri
