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

from pydantic import ValidationError
import pytest

from dimos.robot.drone.px4.config import MavsdkConfig, mavsdk_config_from_environment


@pytest.mark.parametrize(
    "connection_url",
    (
        "serial:///dev/ttyTHS3:921600",
        "serial:///dev/ttyUSB0:57600",
        "udpin://0.0.0.0:14540",
        "udpout://192.168.1.20:18570",
    ),
)
def test_config_accepts_supported_connection_urls(connection_url: str) -> None:
    assert MavsdkConfig(connection_url=connection_url).connection_url == connection_url


@pytest.mark.parametrize(
    "connection_url",
    (
        "serial://dev/ttyTHS3:921600",
        "serial:///tmp/ttyTHS3:921600",
        "udp://0.0.0.0:14540",
        "udpin://localhost:14540",
        "udpin://0.0.0.0:0",
        "",
    ),
)
def test_config_rejects_unsupported_connection_urls(connection_url: str) -> None:
    with pytest.raises(ValidationError, match="connection_url"):
        _ = MavsdkConfig(connection_url=connection_url)


def test_environment_override_sets_connection_url(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("DIMOS_MAVSDK_CONNECTION_URL", "udpin://0.0.0.0:14540")

    assert mavsdk_config_from_environment().connection_url == "udpin://0.0.0.0:14540"


def test_environment_override_sets_connection_timeout(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("DIMOS_MAVSDK_CONNECTION_TIMEOUT_S", "3.5")

    assert mavsdk_config_from_environment().connection_timeout_s == 3.5


def test_blueprint_default_url_preserves_environment_timeout_override(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.delenv("DIMOS_MAVSDK_CONNECTION_URL", raising=False)
    monkeypatch.setenv("DIMOS_MAVSDK_CONNECTION_TIMEOUT_S", "7.5")

    config = mavsdk_config_from_environment(default_connection_url="udpin://0.0.0.0:14540")

    assert config == MavsdkConfig(
        connection_url="udpin://0.0.0.0:14540",
        connection_timeout_s=7.5,
    )
