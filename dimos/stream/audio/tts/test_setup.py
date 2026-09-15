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

import hashlib
import subprocess
import sys

import pytest
import requests

from dimos.stream.audio.tts import setup


@pytest.fixture
def assets(monkeypatch):
    contents = {"model.onnx": b"model bytes", "voices.bin": b"voice bytes"}
    monkeypatch.setattr(
        setup,
        "ASSET_SHA256",
        {name: hashlib.sha256(data).hexdigest() for name, data in contents.items()},
    )
    return contents


def test_downloads_assets_and_reuses_verified_cache(tmp_path, assets, requests_mock):
    for name, data in assets.items():
        requests_mock.get(f"{setup.RELEASE_URL}/{name}", content=data)
    setup.setup_assets(tmp_path)
    assert {p.name: p.read_bytes() for p in tmp_path.iterdir()} == assets
    assert requests_mock.call_count == 2
    requests_mock.reset_mock()
    setup.setup_assets(tmp_path)
    assert requests_mock.call_count == 0


def test_repairs_only_missing_or_corrupt_assets(tmp_path, assets, requests_mock):
    (tmp_path / "voices.bin").write_bytes(assets["voices.bin"])
    for old in (None, b"broken model"):
        if old is not None:
            (tmp_path / "model.onnx").write_bytes(old)
        requests_mock.get(f"{setup.RELEASE_URL}/model.onnx", content=assets["model.onnx"])
        setup.setup_assets(tmp_path)
        assert (tmp_path / "model.onnx").read_bytes() == assets["model.onnx"]
    assert requests_mock.call_count == 2


@pytest.mark.parametrize("failure", ["http", "timeout", "checksum"])
def test_failed_download_preserves_existing_file(tmp_path, assets, requests_mock, failure):
    destination = tmp_path / "model.onnx"
    destination.write_bytes(b"existing file")
    url = f"{setup.RELEASE_URL}/model.onnx"
    if failure == "http":
        requests_mock.get(url, status_code=503)
    elif failure == "timeout":
        requests_mock.get(url, exc=requests.Timeout("download timed out"))
    else:
        requests_mock.get(url, content=b"wrong checksum")
    with pytest.raises((requests.RequestException, ValueError)):
        setup.setup_assets(tmp_path)
    assert list(tmp_path.iterdir()) == [destination]
    assert destination.read_bytes() == b"existing file"


def test_interrupted_download_removes_partial_file(tmp_path, assets, requests_mock, mocker):
    def interrupted(*args, **kwargs):
        yield b"partial download"
        raise KeyboardInterrupt

    requests_mock.get(f"{setup.RELEASE_URL}/model.onnx", content=b"")
    mocker.patch.object(requests.Response, "iter_content", side_effect=interrupted)
    with pytest.raises(KeyboardInterrupt):
        setup.setup_assets(tmp_path)
    assert list(tmp_path.iterdir()) == []


def test_main_reports_failure(tmp_path, assets, requests_mock, monkeypatch, capsys):
    monkeypatch.setattr(setup.setup_assets, "__defaults__", (tmp_path,))
    requests_mock.get(f"{setup.RELEASE_URL}/model.onnx", status_code=503)
    assert setup.main() == 1
    assert "TTS setup failed:" in capsys.readouterr().err


def test_setup_imports_without_robot_or_inference_dependencies():
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            """
import importlib.abc
import sys
class BlockHeavyImports(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'kokoro_onnx', 'onnxruntime'} or fullname.startswith(('dimos.robot', 'dimos.cli', 'dimos.core')):
            raise AssertionError(f'Unexpected setup dependency: {fullname}')
sys.meta_path.insert(0, BlockHeavyImports())
from dimos.stream.audio.tts.setup import main
""",
        ],
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stderr
