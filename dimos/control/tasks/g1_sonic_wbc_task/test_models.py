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

import pytest

from dimos.control.tasks.g1_sonic_wbc_task import models


@pytest.fixture
def assets(monkeypatch):
    contents = {
        "planner_sonic.onnx": b"planner",
        "sonic_v1_1/model_encoder.onnx": b"encoder",
        "low_latency/model_encoder.onnx": b"low latency encoder",
    }
    monkeypatch.setattr(
        models,
        "MODEL_FILES",
        {name: hashlib.sha256(data).hexdigest() for name, data in contents.items()},
    )
    monkeypatch.setattr(models, "MOTION_FILES", {})
    return contents


def test_setup_downloads_pinned_planner_and_selected_profile(tmp_path, assets, mocker):
    def download(*, filename, local_dir, **kwargs):
        path = local_dir / filename
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(assets[filename])
        return str(path)

    fetch = mocker.patch.object(models, "hf_hub_download", side_effect=download)

    paths = models.setup_models(tmp_path)

    assert [path.relative_to(tmp_path).as_posix() for path in paths] == [
        "planner_sonic.onnx",
        "sonic_v1_1/model_encoder.onnx",
    ]
    assert all(path.read_bytes() == assets[path.relative_to(tmp_path).as_posix()] for path in paths)
    assert [call.kwargs for call in fetch.call_args_list] == [
        {
            "repo_id": models.MODEL_REPOSITORY,
            "filename": name,
            "revision": models.MODEL_REVISION,
            "local_dir": tmp_path,
            "force_download": False,
        }
        for name in ("planner_sonic.onnx", "sonic_v1_1/model_encoder.onnx")
    ]


def test_check_and_repeat_setup_use_verified_files_without_network(tmp_path, assets, mocker):
    for name, data in assets.items():
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
    fetch = mocker.patch.object(models, "hf_hub_download")
    http = mocker.patch.object(models.requests, "get")

    checked = models.setup_models(tmp_path, check=True)
    repeated = models.setup_models(tmp_path)

    assert checked == repeated
    assert len(checked) == 2
    fetch.assert_not_called()
    http.assert_not_called()


def test_check_fails_on_corrupt_asset_without_repair(tmp_path, assets, mocker):
    path = tmp_path / "planner_sonic.onnx"
    path.write_bytes(b"corrupt")
    fetch = mocker.patch.object(models, "hf_hub_download")

    with pytest.raises(ValueError, match="SHA-256 mismatch"):
        models.setup_models(tmp_path, check=True)

    fetch.assert_not_called()
    assert path.read_bytes() == b"corrupt"


def test_setup_does_not_accept_wrong_downloaded_weights(tmp_path, assets, mocker):
    def download(*, filename, local_dir, **kwargs):
        path = local_dir / filename
        path.write_bytes(b"wrong weights")
        return str(path)

    mocker.patch.object(models, "hf_hub_download", side_effect=download)

    with pytest.raises(ValueError, match="SHA-256 mismatch"):
        models.setup_models(tmp_path)


def test_bad_motion_download_keeps_existing_file(tmp_path, mocker):
    path = tmp_path / "joint_pos.csv"
    path.write_bytes(b"previous file")
    response = mocker.MagicMock()
    response.__enter__.return_value = response
    response.iter_content.return_value = [b"wrong download"]
    mocker.patch.object(models.requests, "get", return_value=response)

    with pytest.raises(ValueError, match="SHA-256 mismatch"):
        models._download_motion(
            "macarena/joint_pos.csv", path, hashlib.sha256(b"correct").hexdigest()
        )

    assert path.read_bytes() == b"previous file"
    assert not path.with_suffix(".download").exists()
