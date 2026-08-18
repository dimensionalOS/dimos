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

import pytest
import typer

from dimos.cli import cloud


def test_login_stores_key(monkeypatch, tmp_path):
    cred = tmp_path / "credentials.json"
    monkeypatch.setattr(cloud, "CRED_PATH", cred)
    monkeypatch.setattr(cloud.time, "sleep", lambda s: None)
    responses = iter(
        [
            {
                "device_code": "dc",
                "user_code": "AAAA-BBBB",
                "verification_uri": "u",
                "interval": 5,
                "expires_in": 900,
            },
            {"status": "authorization_pending"},
            {"status": "ok", "api_key": "dimos_sk_x", "key_id": "dimos_sk_x", "email": "e@x"},
        ]
    )
    monkeypatch.setattr(cloud, "_post", lambda path, **kw: next(responses))

    cloud.login()

    assert json.loads(cred.read_text())["api_key"] == "dimos_sk_x"
    assert oct(cred.stat().st_mode)[-3:] == "600"
    assert cloud.api_key() == "dimos_sk_x"


def test_login_denied_exits(monkeypatch, tmp_path):
    monkeypatch.setattr(cloud, "CRED_PATH", tmp_path / "c.json")
    monkeypatch.setattr(cloud.time, "sleep", lambda s: None)
    responses = iter(
        [
            {
                "device_code": "dc",
                "user_code": "AAAA-BBBB",
                "verification_uri": "u",
                "interval": 5,
                "expires_in": 900,
            },
            {"status": "denied"},
        ]
    )
    monkeypatch.setattr(cloud, "_post", lambda path, **kw: next(responses))
    with pytest.raises(typer.Exit):
        cloud.login()


def test_env_var_overrides_stored_key(monkeypatch, tmp_path):
    monkeypatch.setattr(cloud, "CRED_PATH", tmp_path / "missing.json")
    monkeypatch.setenv("DIMOS_API_KEY", "dimos_sk_env")
    assert cloud.api_key() == "dimos_sk_env"
