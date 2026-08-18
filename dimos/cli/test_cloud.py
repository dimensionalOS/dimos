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

import json
from pathlib import Path
import time
from typing import Any

import pytest
import typer

from dimos.cli import cloud
from dimos.core.global_config import global_config


@pytest.fixture
def filestore(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> Path:
    """Force the headless path: no keyring backend, credentials in a temp file."""
    monkeypatch.setattr(cloud, "_keyring", lambda: None)
    cred = tmp_path / "credentials.json"
    monkeypatch.setattr(cloud, "CREDENTIALS_PATH", cred)
    monkeypatch.setattr(global_config, "dimos_api_key", None)
    return cred


def _responses(*rs: dict[str, Any]) -> Any:
    it = iter(rs)
    return lambda path, **kw: next(it)


def test_login_stores_key_owner_only(monkeypatch: pytest.MonkeyPatch, filestore: Path) -> None:
    monkeypatch.setattr(time, "sleep", lambda s: None)
    monkeypatch.setattr(
        cloud,
        "_post",
        _responses(
            {
                "device_code": "dc",
                "user_code": "AAAA-BBBB",
                "verification_uri": "u",
                "interval": 5,
                "expires_in": 900,
            },
            {"status": "authorization_pending"},
            {"status": "ok", "api_key": "dimos_sk_x", "key_id": "dimos_sk_x", "email": "e@x"},
        ),
    )

    cloud.login()

    assert json.loads(filestore.read_text())["api_key"] == "dimos_sk_x"
    assert oct(filestore.stat().st_mode)[-3:] == "600"
    assert cloud.api_key() == "dimos_sk_x"

    cloud.logout()
    assert not filestore.exists() and cloud.api_key() is None


def test_login_denied_exits(monkeypatch: pytest.MonkeyPatch, filestore: Path) -> None:
    monkeypatch.setattr(time, "sleep", lambda s: None)
    monkeypatch.setattr(
        cloud,
        "_post",
        _responses(
            {
                "device_code": "dc",
                "user_code": "AAAA-BBBB",
                "verification_uri": "u",
                "interval": 5,
                "expires_in": 900,
            },
            {"status": "denied"},
        ),
    )
    with pytest.raises(typer.Exit):
        cloud.login()
    assert not filestore.exists()


def test_global_config_key_overrides_stored(
    monkeypatch: pytest.MonkeyPatch, filestore: Path
) -> None:
    filestore.write_text(json.dumps({"api_key": "dimos_sk_stored"}))
    monkeypatch.setattr(global_config, "dimos_api_key", "dimos_sk_env")
    assert cloud.api_key() == "dimos_sk_env"
