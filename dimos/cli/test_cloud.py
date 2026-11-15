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

from collections.abc import Iterator
import io
import json
from pathlib import Path
import threading
import time
from typing import Any
import urllib.error
import urllib.request

import pytest
import typer

from dimos.cli import cloud


class FakeKeyring:
    """Dict-backed stand-in for the OS keyring."""

    def __init__(self) -> None:
        self.store: dict[tuple[str, str], str] = {}

    def get_password(self, service: str, user: str) -> str | None:
        return self.store.get((service, user))

    def set_password(self, service: str, user: str, value: str) -> None:
        self.store[(service, user)] = value

    def delete_password(self, service: str, user: str) -> None:
        del self.store[(service, user)]


@pytest.fixture
def filestore(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> Path:
    """Force the headless path: no keyring backend, credentials in a temp file."""
    monkeypatch.setattr(cloud, "_keyring", lambda: None)
    cred = tmp_path / "dimos" / "credentials"
    cred.parent.mkdir()
    monkeypatch.setattr(cloud, "CREDENTIALS_PATH", cred)
    monkeypatch.delenv("DIMOS_CLOUD_API_KEY", raising=False)
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

    assert filestore.read_text().strip() == "dimos_sk_x"
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


def test_cloud_config_key_overrides_stored(
    monkeypatch: pytest.MonkeyPatch, filestore: Path
) -> None:
    filestore.write_text("dimos_sk_stored\n")
    monkeypatch.setenv("DIMOS_CLOUD_API_KEY", "dimos_sk_env")
    assert cloud.api_key() == "dimos_sk_env"


def test_keyring_store_load_forget(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    kr = FakeKeyring()
    monkeypatch.setattr(cloud, "_keyring", lambda: kr)
    monkeypatch.setattr(cloud, "CREDENTIALS_PATH", tmp_path / "never-written.json")
    monkeypatch.delenv("DIMOS_CLOUD_API_KEY", raising=False)

    assert cloud._store("dimos_sk_kr") == "system keyring"
    assert not (tmp_path / "never-written.json").exists()  # keyring won; no file
    assert cloud.api_key() == "dimos_sk_kr"
    assert cloud._forget() is True
    assert cloud.api_key() is None and cloud._forget() is False


def test_whoami_displays_account(
    monkeypatch: pytest.MonkeyPatch, filestore: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    filestore.write_text("dimos_sk_w\n")
    seen: dict[str, str] = {}

    def fake_urlopen(req: Any, timeout: float | None = None) -> io.BytesIO:
        seen["auth"] = req.get_header("Authorization")
        return io.BytesIO(json.dumps({"email": "e@x", "scopes": "data"}).encode())

    monkeypatch.setattr(urllib.request, "urlopen", fake_urlopen)
    cloud.whoami()
    assert seen["auth"] == "Bearer dimos_sk_w"
    assert "e@x (scopes: data)" in capsys.readouterr().out


def test_whoami_revoked_key_exits(monkeypatch: pytest.MonkeyPatch, filestore: Path) -> None:
    filestore.write_text("dimos_sk_dead\n")

    def fake_urlopen(req: Any, timeout: float | None = None) -> io.BytesIO:
        raise urllib.error.HTTPError(req.full_url, 401, "unauthorized", {}, None)  # type: ignore[arg-type]

    monkeypatch.setattr(urllib.request, "urlopen", fake_urlopen)
    with pytest.raises(typer.Exit):
        cloud.whoami()


def test_whoami_not_logged_in_exits(filestore: Path) -> None:
    with pytest.raises(typer.Exit):
        cloud.whoami()


class StalledKeyring(FakeKeyring):
    """A keyring that never answers, the way a locked one waits on an unlock prompt."""

    def __init__(self) -> None:
        super().__init__()
        self.release = threading.Event()
        self.tries = 0

    def get_password(self, service: str, user: str) -> str | None:
        self.tries += 1
        self.release.wait()  # only the test ever lets go
        return super().get_password(service, user)


@pytest.fixture
def stalled(monkeypatch: pytest.MonkeyPatch) -> Iterator[StalledKeyring]:
    """A keyring that hangs, wired in as the backend, with a deadline short enough to test."""
    kr = StalledKeyring()
    monkeypatch.setattr(cloud, "_backend", None)
    monkeypatch.setattr(cloud, "_import_keyring", lambda: kr)
    monkeypatch.setenv("DIMOS_CLOUD_KEYRING_TIMEOUT", "0.2")
    monkeypatch.delenv("DIMOS_CLOUD_API_KEY", raising=False)
    yield kr
    kr.release.set()  # never leave the parked thread behind


def test_a_keyring_that_never_answers_falls_back_to_the_file(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, stalled: StalledKeyring
) -> None:
    """The defect that wedged a gateway: a locked keyring blocked api_key() with no deadline, so
    the account route never answered and the upload worker stopped at its first byte."""
    cred = tmp_path / "credentials"
    cred.write_text("dimos_sk_file\n")
    monkeypatch.setattr(cloud, "CREDENTIALS_PATH", cred)

    started = time.monotonic()
    assert cloud.api_key() == "dimos_sk_file"
    assert time.monotonic() - started < 5.0  # gave up on the prompt instead of waiting for it
    assert stalled.tries == 1  # and it did try the keyring first


def test_a_stalled_keyring_is_tried_once_not_once_per_lookup(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, stalled: StalledKeyring
) -> None:
    monkeypatch.setattr(cloud, "CREDENTIALS_PATH", tmp_path / "no-credentials")
    for _ in range(5):
        assert cloud.api_key() is None
    assert stalled.tries == 1  # remembered for KEYRING_RETRY_S, so a poll costs nothing


def test_a_stalled_keyring_is_retried_after_its_window(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, stalled: StalledKeyring
) -> None:
    """A login stored after this process started still has to be findable."""
    monkeypatch.setattr(cloud, "CREDENTIALS_PATH", tmp_path / "no-credentials")
    monkeypatch.setattr(cloud, "KEYRING_RETRY_S", 0.0)
    assert cloud.api_key() is None
    assert cloud.api_key() is None
    assert stalled.tries == 2
