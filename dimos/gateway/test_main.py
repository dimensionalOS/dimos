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

"""$DIMOS_GATEWAY, the one variable Desktop starts the gateway with."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.gateway import config, desktop, main


def test_everything_comes_from_dimos_gateway(
    server_home: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.delenv(config.GATEWAY_ENV)
    assert main.socket_path() == config.gateway_dir() / "dimos-gateway.sock"
    assert main.dimos_dir() == DIMOS_PROJECT_ROOT
    assert desktop.desktop_url() == "http://127.0.0.1:5555"  # config.yaml's port, never called here
    given = {"socket": "/s/g.sock", "dimosDir": "/c", "desktopUrl": "http://127.0.0.1:9001/"}
    monkeypatch.setenv(config.GATEWAY_ENV, json.dumps(given))
    assert main.socket_path() == Path("/s/g.sock")
    assert main.dimos_dir() == Path("/c")
    assert desktop.desktop_url() == "http://127.0.0.1:9001"


def test_not_an_object_is_refused(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv(config.GATEWAY_ENV, "[1]")
    with pytest.raises(ValueError):
        config.gateway_env()


def test_print_socket(monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]) -> None:
    monkeypatch.setenv(config.GATEWAY_ENV, json.dumps({"socket": "/s/g.sock"}))
    main.gateway(detach_=False, print_socket=True, zenoh=True, write_provides=False)
    assert capsys.readouterr().out == "/s/g.sock\n"
