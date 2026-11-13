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

from pathlib import Path

import pytest

from dimos.hosted.config import DEFAULT_LISTEN, HostConfig, write_settings


def test_a_host_is_closed_by_default() -> None:
    assert HostConfig().listen == DEFAULT_LISTEN == "tcp/127.0.0.1:7447"
    assert HostConfig().autodiscovery is False


def test_write_settings_sets_and_removes_keeping_the_rest(tmp_path: Path) -> None:
    env = tmp_path / ".env"
    env.write_text("ROBOT_IP=10.0.0.2\nHOST__TAGS=old\n")

    write_settings({"listen": "iface/wlan0", "HOST__CONNECT": "tcp/10.0.0.9:7447"}, env)
    write_settings({"tags": ""}, env)

    text = env.read_text()
    assert "ROBOT_IP=10.0.0.2" in text
    assert "HOST__LISTEN=iface/wlan0" in text
    assert "HOST__CONNECT=tcp/10.0.0.9:7447" in text
    assert "HOST__TAGS" not in text


@pytest.mark.parametrize(
    ("values", "message"),
    (
        ({"lisen": "x"}, "Unknown settings \\['lisen'\\]"),
        ({"startup_timeout": "-1"}, "greater than 0"),
    ),
)
def test_write_settings_refuses_what_a_host_would_not_start_with(
    tmp_path: Path, values: dict[str, str], message: str
) -> None:
    env = tmp_path / ".env"
    with pytest.raises(ValueError, match=message):
        write_settings(values, env)
    assert not env.exists()
