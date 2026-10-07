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

"""cloud_worker against dimos's real cloud code (only the network is faked): its errors are classified by what dimos
raises, and its login is dimos's own device flow."""

import io
import json
from pathlib import Path
import time
from typing import Any
import urllib.error
import urllib.request

import pytest

from dimos.cli import cloud
from dimos.cloud.cloud_request import HttpCloudRequest
from dimos.cloud.data import DataApi, MultipartBackend
from dimos.core.global_config import global_config
from dimos.gateway import cloud_worker


@pytest.fixture
def no_login(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> Path:
    monkeypatch.setattr(cloud, "_keyring", lambda: None)
    monkeypatch.setattr(cloud, "CREDENTIALS_PATH", tmp_path / "credentials")
    monkeypatch.setattr(global_config, "dimos_api_key", None)
    return tmp_path / "credentials"


def refuse(status: int) -> Any:
    def urlopen(request: Any, timeout: float | None = None) -> Any:
        raise urllib.error.HTTPError(request.full_url, status, "no", {}, io.BytesIO(b"{}"))  # type: ignore[arg-type]

    return urlopen


def raised(call: Any) -> BaseException:
    with pytest.raises(BaseException) as caught:
        call()
    return caught.value


def backend() -> MultipartBackend:
    return MultipartBackend(DataApi(HttpCloudRequest("https://api.x", "k", 1)), "zstd", None, 0)


def test_errors_are_classified_by_what_dimos_raises(
    no_login: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    # no key: refused before dimos is called (its own refusal has only words to tell it by)
    with pytest.raises(cloud_worker.NotLoggedInError) as refused:
        cloud_worker.upload(str(Path(__file__)))
    assert cloud_worker.classify(refused.value)[0] == "not_logged_in"
    for status, code in [(401, "not_logged_in"), (413, "quota"), (500, "failed")]:
        monkeypatch.setattr(urllib.request, "urlopen", refuse(status))
        assert cloud_worker.classify(raised(backend().quota))[0] == code, status

    def unreachable(request: Any, timeout: float | None = None) -> Any:
        raise urllib.error.URLError("nodename nor servname provided")

    monkeypatch.setattr(urllib.request, "urlopen", unreachable)
    assert cloud_worker.classify(raised(backend().quota))[0] == "network"
    assert cloud_worker.classify(FileNotFoundError(2, "gone", "/x.mcap"))[0] == "file_missing"


def test_login_and_account_are_dimos_own(no_login: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    answers = iter(
        [
            {
                "device_code": "dc",
                "user_code": "AB",
                "verification_uri": "u",
                "interval": 1,
                "expires_in": 60,
            },
            {"status": "authorization_pending"},
            {"status": "ok", "api_key": "dimos_sk_x", "key_id": "dimos_sk_x", "email": "e@x"},
        ]
    )
    monkeypatch.setattr(cloud, "_post", lambda path, **params: next(answers))
    monkeypatch.setattr(time, "sleep", lambda s: None)
    emitted: list[dict[str, Any]] = []
    monkeypatch.setattr(cloud_worker, "emit", emitted.append)
    assert cloud_worker.login() == {"event": "done", "status": "ok", "email": "e@x"}
    assert emitted[0]["code"] == "AB" and no_login.read_text().strip() == "dimos_sk_x"

    monkeypatch.setattr(
        urllib.request,
        "urlopen",
        lambda request, timeout=None: io.BytesIO(
            json.dumps({"email": "e@x", "scopes": "data"}).encode()
        ),
    )
    assert cloud_worker.account()["email"] == "e@x"
    monkeypatch.setattr(urllib.request, "urlopen", refuse(401))
    revoked = cloud_worker.account()
    assert revoked["loggedIn"] is False and "revoked" in revoked["error"]
    assert cloud_worker.logout() == {"loggedOut": True}
    assert cloud_worker.account()["loggedIn"] is False
