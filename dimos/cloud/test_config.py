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

from pathlib import Path

import pytest

from dimos.cloud.config import CloudConfig
from dimos.cloud.data import CloudData


def test_cloud_configuration_precedence_is_lazy(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    dotenv = tmp_path / ".env"
    dotenv.write_text("DIMOS_CLOUD_URL=https://file.example\nDIMOS_CLOUD_HTTP_TIMEOUT=12\n")
    monkeypatch.setenv("DIMOS_CLOUD_URL", "https://env.example")
    monkeypatch.setenv("DIMOS_CLOUD_API_KEY", "test-key")
    from_env = CloudConfig(_env_file=dotenv)
    explicit = CloudConfig(url="https://explicit.example", _env_file=dotenv)

    assert from_env.url == "https://env.example"
    assert from_env.http_timeout == 12
    assert explicit.url == "https://explicit.example"
    assert "test-key" not in repr(from_env)
    monkeypatch.setenv("DIMOS_CLOUD_URL", "https://next.example")
    assert CloudConfig(_env_file=dotenv).url == "https://next.example"
    assert from_env.url == "https://env.example"


def test_explicit_cloud_configuration_reaches_request_transport(mocker) -> None:
    request = mocker.patch("dimos.cloud.data.HttpCloudRequest")
    config = CloudConfig(url="https://cloud.example", api_key="test-key", http_timeout=17)

    CloudData(config=config).backend.quota()

    request.assert_called_once_with("https://cloud.example", "test-key", 17)
    request.return_value.request.assert_called_once_with("GET", "/v1/data/quota")
