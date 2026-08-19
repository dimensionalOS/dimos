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

"""Rendering guarantees for `dimos.cli.art`.

The two that matter: nothing ever exceeds the terminal width, and the device
code and verification URL survive at every width — a clipped credential makes
`dimos login` unusable.
"""

import re
import sys

import pytest

from dimos.cli import art

_ESC = re.compile(r"\x1b\[[0-9;]*m")

URI = "login.dimensional.org/activate"
CODE = "FQZM-VWKT"

# 30 is the width of the verification URL — the hard floor below which no
# layout can help.
WIDTHS = [30, 34, 40, 44, 48, 56, 62, 66, 72, 80, 84, 100, 120]


@pytest.fixture
def tty(monkeypatch: pytest.MonkeyPatch) -> None:
    """Pretend we are on a colour terminal, so the art actually renders."""
    monkeypatch.setattr(art, "enabled", lambda: True)


def plain(rows: list[str]) -> list[str]:
    return [_ESC.sub("", r) for r in rows]


def bodies(cols: int) -> dict[str, list[str]]:
    return {
        "login": art.login_rows(URI, CODE, "⠋", "14:58"),
        "signed_in": art.signed_in_rows("you@dimensionalos.com", "ab12cd34", "system keyring"),
        "denied": art.refused_rows("Denied", "The code was rejected in the browser."),
        "expired": art.refused_rows("Code expired", "Codes are valid for 15 minutes."),
        "logout": art.logged_out_rows("login.dimensional.org/keys"),
    }


@pytest.mark.parametrize("cols", WIDTHS)
def test_never_exceeds_terminal_width(
    cols: int, tty: None, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(art, "width", lambda: cols)
    for name, rows in bodies(cols).items():
        widest = max(len(r) for r in plain(rows))
        assert widest <= cols, f"{name} overflowed at {cols}: {widest}"


@pytest.mark.parametrize("cols", WIDTHS)
def test_code_and_url_are_never_truncated(
    cols: int, tty: None, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(art, "width", lambda: cols)
    text = " ".join(plain(art.login_rows(URI, CODE, "⠋", "14:58")))
    assert CODE in text, f"device code lost at {cols} columns"
    assert URI in text, f"verification URL lost at {cols} columns"


def test_no_art_and_no_colour_off_terminal(monkeypatch: pytest.MonkeyPatch) -> None:
    """CI logs and journald get the words, not a doorway."""
    monkeypatch.setattr(art, "enabled", lambda: False)
    rows = art.login_rows(URI, CODE, "⠋", "14:58")
    blob = "\n".join(rows)
    assert "\x1b[" not in blob
    assert not any(ch in blob for ch in "╭╰│░▒█")
    assert CODE in blob and URI in blob


def test_no_colour_honours_env(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(sys.stdout, "isatty", lambda: True)
    monkeypatch.setenv("NO_COLOR", "1")
    assert art.enabled() is False


def test_rendering_is_deterministic(tty: None) -> None:
    assert art.signed_in_rows("a@b", "k", "keyring") == art.signed_in_rows("a@b", "k", "keyring")


def test_sigil_is_rectangular(tty: None) -> None:
    rows = plain(art.sigil("ok"))
    assert len(rows) == 15
    assert {len(r) for r in rows} == {30}


def test_sigil_reads_without_colour(monkeypatch: pytest.MonkeyPatch) -> None:
    """The picture must live in the characters — that is what lets `beams`
    animate it, and what makes the mono fallback legible."""
    monkeypatch.setattr(art, "enabled", lambda: True)
    glyphs = set("".join(plain(art.sigil("ok")))) - {" "}
    assert len(glyphs) >= 3


def test_whoami_stays_one_line(tty: None) -> None:
    """It is a lookup people script against, not a moment."""
    rows = art.whoami_rows("e@x", "data")
    assert len(rows) == 1
    assert plain(rows)[0] == "e@x (scopes: data)"


def test_states_are_visually_distinct(tty: None) -> None:
    ok = "\n".join(art.signed_in_rows("a@b", "k", "keyring"))
    bad = "\n".join(art.refused_rows("Denied", "no"))
    idle = "\n".join(art.login_rows(URI, CODE))
    assert ok != bad != idle
    assert "38;2;136;255;136" in ok  # green once signed in
    assert "38;2;255;106;95" in bad  # red when refused
    assert "38;2;126;142;142" in idle  # grey while nothing has happened
