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

from concurrent.futures import ThreadPoolExecutor
import hashlib
import os
import subprocess
import sys
from threading import Event

import pytest
import requests

from dimos.utils import assets, cache

URL = "https://example.com/model.pt"


@pytest.fixture(autouse=True)
def cache_root(tmp_path, monkeypatch):
    root = tmp_path / "cache"
    monkeypatch.setattr(assets, "CACHE_DIR", root)
    monkeypatch.setattr(cache, "CACHE_DIR", root)
    monkeypatch.setattr(cache, "_CACHE_LOCK_DIR", tmp_path / "state" / "cache-users")
    monkeypatch.setattr(cache, "_CACHE_GATE_PATH", tmp_path / "state" / "cache-clean.lock")
    return root


def test_https_cache_reuses_verified_file_offline(requests_mock, cache_root):
    contents = b"model weights"
    checksum = hashlib.sha256(contents).hexdigest()
    requests_mock.get(URL, content=contents)
    path = assets.download_http_asset(URL, sha256=checksum)
    assert path.is_relative_to(cache_root / "assets" / "http")
    assert path.read_bytes() == contents
    assert assets.download_http_asset(URL, sha256=checksum) == path
    assert requests_mock.call_count == 1
    path.write_bytes(b"corrupt")
    assert assets.download_http_asset(URL, sha256=checksum).read_bytes() == contents
    assert requests_mock.call_count == 2


def test_distinct_sources_with_same_filename_do_not_collide(requests_mock):
    second = "https://example.org/model.pt"
    requests_mock.get(URL, content=b"one")
    requests_mock.get(second, content=b"two")
    first = assets.download_http_asset(URL)
    other = assets.download_http_asset(second)
    assert first != other
    assert (first.read_bytes(), other.read_bytes()) == (b"one", b"two")


@pytest.mark.parametrize("failure", ["http", "timeout", "checksum"])
def test_failed_download_publishes_no_partial_file_and_can_retry(
    requests_mock, cache_root, failure
):
    checksum = hashlib.sha256(b"complete").hexdigest()
    if failure == "http":
        requests_mock.get(URL, status_code=503)
    elif failure == "timeout":
        requests_mock.get(URL, exc=requests.Timeout("timed out"))
    else:
        requests_mock.get(URL, content=b"bad")
    with pytest.raises(RuntimeError, match="Could not download model asset"):
        assets.download_http_asset(URL, sha256=checksum)
    assert not list(cache_root.rglob("model.pt"))
    assert not list(cache_root.rglob("*.part"))
    requests_mock.get(URL, content=b"complete")
    assert assets.download_http_asset(URL, sha256=checksum).read_bytes() == b"complete"


def test_interrupt_removes_partial_download(requests_mock, mocker, cache_root):
    def interrupted(*args, **kwargs):
        yield b"partial"
        raise KeyboardInterrupt

    requests_mock.get(URL, content=b"")
    mocker.patch.object(requests.Response, "iter_content", side_effect=interrupted)
    with pytest.raises(KeyboardInterrupt):
        assets.download_http_asset(URL)
    assert not list(cache_root.rglob("*.part"))
    assert not list(cache_root.rglob("model.pt"))


def test_concurrent_download_and_cleanup_protection(requests_mock):
    downloading, release = Event(), Event()

    def response(request, context):
        downloading.set()
        assert release.wait(5)
        return b"complete"

    requests_mock.get(URL, content=response)
    with ThreadPoolExecutor(max_workers=2) as pool:
        first = pool.submit(assets.download_http_asset, URL)
        try:
            assert downloading.wait(5)
            second = pool.submit(assets.download_http_asset, URL)
            with pytest.raises(cache.CacheInUseError), cache.cache_cleanup_guard():
                pass
        finally:
            release.set()
        assert first.result(timeout=5) == second.result(timeout=5)
        assert first.result().read_bytes() == b"complete"
    assert requests_mock.call_count == 1


def test_cache_clean_preserves_external_checkpoint_and_waits_for_consumer(requests_mock, tmp_path):
    external = tmp_path / "user-checkpoint.pt"
    external.write_bytes(b"user data")
    requests_mock.get(URL, content=b"model")
    with cache.cache_usage_guard():
        path = assets.download_http_asset(URL)
        with pytest.raises(cache.CacheInUseError), cache.cache_cleanup_guard():
            pass
        assert path.read_bytes() == b"model"
    with cache.cache_cleanup_guard():
        assert cache.clean_caches().complete
    assert not path.exists()
    assert external.read_bytes() == b"user data"


@pytest.fixture
def hub(mocker):
    hub = mocker.MagicMock()
    mocker.patch.object(assets.importlib, "import_module", return_value=hub)
    return hub


def test_hf_reuses_dimos_cache_without_network(hub, cache_root):
    path = cache_root / "assets" / "huggingface" / "model"
    hub.try_to_load_from_cache.return_value = str(path)
    assert assets.download_hf_asset(repo_id="owner/model", revision="abc", filename="model") == path
    hub.try_to_load_from_cache.assert_called_once_with(
        "owner/model", "model", revision="abc", cache_dir=cache_root / "assets" / "huggingface"
    )
    hub.hf_hub_download.assert_not_called()


def test_hf_download_uses_explicit_revision_and_dimos_cache(hub, cache_root):
    hub.try_to_load_from_cache.return_value = None
    path = cache_root / "assets" / "huggingface" / "model"
    hub.hf_hub_download.return_value = str(path)
    assert assets.download_hf_asset(repo_id="owner/model", revision="abc", filename="model") == path
    hub.hf_hub_download.assert_called_once_with(
        "owner/model", "model", revision="abc", cache_dir=cache_root / "assets" / "huggingface"
    )


def test_hf_failure_identifies_source_and_can_retry(hub, cache_root):
    hub.try_to_load_from_cache.return_value = None
    hub.hf_hub_download.side_effect = [OSError("offline"), str(cache_root / "model")]
    with pytest.raises(RuntimeError, match="owner/model@abc/model: offline"):
        assets.download_hf_asset(repo_id="owner/model", revision="abc", filename="model")
    assert (
        assets.download_hf_asset(repo_id="owner/model", revision="abc", filename="model")
        == cache_root / "model"
    )


def test_xdg_cache_and_state_roots(tmp_path):
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            """
from dimos.constants import CACHE_DIR, STATE_DIR
from dimos.utils.cache import cache_usage_guard
with cache_usage_guard():
    print(CACHE_DIR)
    print(STATE_DIR)
    assert list((STATE_DIR / 'cache-users').glob('*.lock'))
""",
        ],
        env={
            **os.environ,
            "XDG_CACHE_HOME": str(tmp_path / "xdg-cache"),
            "XDG_STATE_HOME": str(tmp_path / "xdg-state"),
        },
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == [
        str(tmp_path / "xdg-cache/dimos"),
        str(tmp_path / "xdg-state/dimos"),
    ]


@pytest.mark.parametrize("name", [".download.lock", ".download.part"])
def test_remote_filename_cannot_overlap_download_bookkeeping(name, requests_mock):
    url = f"https://example.com/{name}"
    requests_mock.get(url, content=b"asset")
    path = assets.download_http_asset(url)
    assert path.read_bytes() == b"asset"
    assert assets.download_http_asset(url).read_bytes() == b"asset"
    assert requests_mock.call_count == 1
