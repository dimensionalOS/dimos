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
import socket
from types import SimpleNamespace

import pytest

from dimos.hosted import config
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


def _interfaces(monkeypatch: pytest.MonkeyPatch, **ifaces: list[str]) -> None:
    addrs = {
        name: [SimpleNamespace(family=socket.AF_INET, address=ip) for ip in ips]
        for name, ips in ifaces.items()
    }
    monkeypatch.setattr(config.psutil, "net_if_addrs", lambda: addrs)


def test_iface_shorthand_listens_on_the_interface_addresses(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _interfaces(monkeypatch, wlan0=["10.0.0.5"], tailscale0=["100.97.180.50", "100.64.0.9"])
    listen = "iface/wlan0, iface/tailscale0:7450, tcp/127.0.0.1:7447, tcp/0.0.0.0, tcp/[::], tcp/[::1]:7450"
    assert HostConfig(listen=listen).listen_endpoints == [
        "tcp/10.0.0.5:7447",
        "tcp/100.97.180.50:7450",
        "tcp/100.64.0.9:7450",
        "tcp/127.0.0.1:7447",
        "tcp/0.0.0.0:7447",
        "tcp/[::]:7447",
        "tcp/[::1]:7450",
    ]


def test_loopback_and_an_interface_share_a_port(monkeypatch: pytest.MonkeyPatch) -> None:
    _interfaces(monkeypatch, lo=["127.0.0.1"], tailscale0=["100.97.180.50"])
    host = HostConfig(listen="tcp/127.0.0.1:7447,iface/tailscale0,iface/lo")
    assert host.listen_endpoints == ["tcp/127.0.0.1:7447", "tcp/100.97.180.50:7447"]


def test_an_interface_without_ipv4_is_refused(monkeypatch: pytest.MonkeyPatch) -> None:
    _interfaces(monkeypatch, tailscale0=[])
    with pytest.raises(ValueError, match="'tailscale0' has no IPv4"):
        HostConfig(listen="tcp/127.0.0.1:7447,iface/tailscale0").listen_endpoints  # noqa: B018
    with pytest.raises(ValueError, match="'wlan0' has no IPv4"):
        HostConfig(listen="iface/wlan0").listen_endpoints  # noqa: B018
