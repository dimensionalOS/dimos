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

"""CLI arg handling for `dimos spy`: --transport parsing, validation, web mode."""

from typing import Any

import pytest

from dimos.protocol.pubsub.spy import SOURCE_FACTORIES
from dimos.utils.cli.spy.run_spy import SpyApp, _parse_transports, _web_command


def test_parse_transports_default_is_none():
    assert _parse_transports([]) is None
    assert _parse_transports(["web"]) is None


def test_parse_transports_collects_repeated_flags():
    assert _parse_transports(["--transport", "lcm"]) == ["lcm"]
    assert _parse_transports(["--transport=zenoh"]) == ["zenoh"]
    assert _parse_transports(["--transport", "lcm", "--transport", "zenoh"]) == ["lcm", "zenoh"]


def test_parse_transports_rejects_unknown_transport():
    with pytest.raises(SystemExit, match="zneoh"):
        _parse_transports(["--transport", "zneoh"])


def test_parse_transports_error_lists_valid_choices():
    with pytest.raises(SystemExit, match="lcm"):
        _parse_transports(["--transport=nope"])


class _StubSource:
    name = "zenoh"

    def __init__(self) -> None:
        self.started = False

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.started = False

    def tap(self, callback: Any) -> Any:
        return lambda: None

    def subscribe_decoded(self, topic: str, callback: Any) -> Any:
        raise NotImplementedError


def test_spy_app_default_skips_unavailable_backend(monkeypatch, capsys):
    def unavailable():
        raise ImportError("lcm backend missing")

    created: list[_StubSource] = []

    def stub_factory() -> _StubSource:
        source = _StubSource()
        created.append(source)
        return source

    monkeypatch.setitem(SOURCE_FACTORIES, "lcm", unavailable)
    monkeypatch.setitem(SOURCE_FACTORIES, "zenoh", stub_factory)
    app = SpyApp()  # no --transport: degrades to the available backend
    try:
        assert [s.started for s in created] == [True]
    finally:
        app.spy.stop()
    assert "lcm" in capsys.readouterr().err


def test_spy_app_explicit_unavailable_transport_is_hard_error(monkeypatch):
    def unavailable():
        raise ImportError("zenoh backend missing")

    monkeypatch.setitem(SOURCE_FACTORIES, "zenoh", unavailable)
    with pytest.raises(ImportError, match="zenoh backend missing"):
        SpyApp(transports=["zenoh"])


def test_web_command_forwards_transport_filters():
    cmd = _web_command(["--transport", "zenoh"])
    assert cmd.endswith("run_spy.py --transport zenoh")


def test_web_command_without_filters_serves_bare_script():
    assert _web_command([]).endswith("run_spy.py")
