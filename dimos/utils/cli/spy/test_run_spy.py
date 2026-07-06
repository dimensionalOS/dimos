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

"""CLI arg handling for `dimos spy`: --transport parsing, validation, LCM-only alias."""

import sys
from typing import Any

import pytest

from dimos.utils.cli.spy import run_spy
from dimos.utils.cli.spy.core import SOURCE_FACTORIES
from dimos.utils.cli.spy.run_spy import SpyApp, _parse_transports, lcm_only_argv


def test_parse_transports_default_is_none():
    assert _parse_transports([]) is None


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


def test_parse_transports_rejects_unexpected_positional():
    # A stray positional must fail loudly, not be silently ignored.
    with pytest.raises(SystemExit, match="unexpected"):
        _parse_transports(["foo"])
    with pytest.raises(SystemExit, match="unexpected"):
        _parse_transports(["--transport", "lcm", "foo"])


def test_parse_transports_missing_value_errors():
    with pytest.raises(SystemExit, match="requires a transport name"):
        _parse_transports(["--transport"])


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


def test_spy_app_default_skips_non_import_backend_failure(monkeypatch, capsys):
    # Any construction failure (e.g. a native init error), not just ImportError,
    # must degrade rather than kill the default spy.
    def broken():
        raise RuntimeError("zenoh native init failed")

    created: list[_StubSource] = []

    def stub_factory() -> _StubSource:
        source = _StubSource()
        created.append(source)
        return source

    monkeypatch.setitem(SOURCE_FACTORIES, "lcm", broken)
    monkeypatch.setitem(SOURCE_FACTORIES, "zenoh", stub_factory)
    app = SpyApp()  # no --transport: degrades past the broken backend
    try:
        assert [s.started for s in created] == [True]
    finally:
        app.spy.stop()
    err = capsys.readouterr().err
    assert "lcm" in err and "native init failed" in err


def test_spy_app_explicit_unavailable_transport_is_hard_error(monkeypatch):
    def unavailable():
        raise RuntimeError("zenoh backend missing")

    monkeypatch.setitem(SOURCE_FACTORIES, "zenoh", unavailable)
    with pytest.raises(RuntimeError, match="zenoh backend missing"):
        SpyApp(transports=["zenoh"])


class _FailingStartSource(_StubSource):
    name = "zenoh"

    def start(self) -> None:
        raise RuntimeError("zenoh start failed")


def test_spy_app_default_survives_backend_start_failure(monkeypatch, capsys):
    # A backend that constructs but dies during start() must not kill the
    # default spy — the survivor keeps running.
    survivors: list[_StubSource] = []

    def good_factory() -> _StubSource:
        source = _StubSource()
        source.name = "lcm"
        survivors.append(source)
        return source

    monkeypatch.setitem(SOURCE_FACTORIES, "lcm", good_factory)
    monkeypatch.setitem(SOURCE_FACTORIES, "zenoh", _FailingStartSource)
    app = SpyApp()  # default path: best-effort
    try:
        assert survivors and survivors[0].started
    finally:
        app.spy.stop()
    assert "zenoh" in capsys.readouterr().err


def test_spy_app_explicit_start_failure_is_hard_error(monkeypatch):
    monkeypatch.setitem(SOURCE_FACTORIES, "zenoh", _FailingStartSource)
    with pytest.raises(RuntimeError, match="zenoh start failed"):
        SpyApp(transports=["zenoh"])  # explicit: strict, no degrade


def test_lcm_only_argv_forwards_plain_args():
    assert lcm_only_argv([]) == ["spy", "--transport", "lcm"]
    assert lcm_only_argv(["--foo", "bar"]) == ["spy", "--transport", "lcm", "--foo", "bar"]


def test_lcm_only_argv_rejects_transport_override():
    with pytest.raises(SystemExit, match="LCM-only"):
        lcm_only_argv(["--transport", "zenoh"])
    with pytest.raises(SystemExit, match="LCM-only"):
        lcm_only_argv(["--transport=zenoh"])


def test_lcm_main_forwards_args_to_spy(monkeypatch):
    seen: list[str] = []
    monkeypatch.setattr(run_spy, "main", lambda: seen.extend(sys.argv))
    monkeypatch.setattr(sys, "argv", ["lcmspy", "--foo"])
    run_spy.lcm_main()
    assert seen == ["spy", "--transport", "lcm", "--foo"]


def test_lcm_main_rejects_transport_override(monkeypatch):
    monkeypatch.setattr(sys, "argv", ["lcmspy", "--transport", "zenoh"])
    with pytest.raises(SystemExit, match="LCM-only"):
        run_spy.lcm_main()
