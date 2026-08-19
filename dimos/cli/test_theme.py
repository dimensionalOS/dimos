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

"""Rendering guarantees for the theme components.

The two that matter: nothing ever exceeds the terminal width, and a long value —
a URL, a one-time code — is never truncated. A clipped credential makes the
command that printed it unusable.
"""

import io
import re
import sys

import pytest

from dimos.cli import cloud, theme

_ESC = re.compile(r"\x1b\[[0-9;]*m")

URI = "login.dimensional.org/activate"
CODE = "FQZM-VWKT"

# 30 is the width of the verification URL — the floor below which no layout helps.
WIDTHS = [30, 34, 40, 44, 48, 56, 62, 66, 72, 80, 84, 100, 120]


@pytest.fixture
def tty(monkeypatch: pytest.MonkeyPatch) -> None:
    """Pretend we are on a colour terminal, so components actually draw."""
    monkeypatch.setattr(theme, "enabled", lambda: True)


def plain(rows: list[str]) -> list[str]:
    return [_ESC.sub("", r) for r in rows]


def cards() -> dict[str, list[str]]:
    return {
        "login": cloud._login_card(URI, CODE, "⠋", "14:58"),
        "signed_in": cloud._signed_in_card("you@dimensionalos.com", "ab12cd34", "system keyring"),
        "refused": cloud._refused_card("Denied", "The code was rejected in the browser."),
        "logged_out": cloud._logged_out_card("login.dimensional.org/keys"),
    }


# --------------------------------------------------------------- tokens


def test_portal_gradient_comes_from_the_tcss() -> None:
    """The gradient is theme data, not a constant in a Python file."""
    assert len(theme.PORTAL) == 11
    assert theme.COLORS["portal-0"] == "#f4f19f"
    assert theme.rgb("portal-0") == (0xF4, 0xF1, 0x9F)


def test_rgb_handles_short_hex() -> None:
    assert theme.rgb("nope", "#abc") == (0xAA, 0xBB, 0xCC)


# --------------------------------------------------------------- width


@pytest.mark.parametrize("cols", WIDTHS)
def test_never_exceeds_terminal_width(
    cols: int, tty: None, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(theme, "term_width", lambda: cols)
    for name, rows in cards().items():
        widest = max(len(r) for r in plain(rows))
        assert widest <= cols, f"{name} overflowed at {cols}: {widest}"


@pytest.mark.parametrize("cols", WIDTHS)
def test_code_and_url_are_never_truncated(
    cols: int, tty: None, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(theme, "term_width", lambda: cols)
    text = " ".join(plain(cloud._login_card(URI, CODE, "⠋", "14:58")))
    assert CODE in text, f"device code lost at {cols} columns"
    assert URI in text, f"verification URL lost at {cols} columns"


def test_card_grows_for_a_body_taller_than_the_art(tty: None) -> None:
    tall = [theme.paint(f"line {i}", theme.MUTED) for i in range(40)]
    assert len(plain(theme.card(tall, "ok", cols=100))) >= 40


def test_wrap_never_splits_a_long_token() -> None:
    assert URI in " ".join(theme.wrap([URI], 10))


# --------------------------------------------------------------- streams


def test_no_art_and_no_colour_off_terminal(monkeypatch: pytest.MonkeyPatch) -> None:
    """CI logs and journald get the words, not a doorway."""
    monkeypatch.setattr(theme, "enabled", lambda: False)
    blob = "\n".join(cloud._login_card(URI, CODE, "⠋", "14:58"))
    assert "\x1b[" not in blob
    assert not any(ch in blob for ch in "╭╰│░▒█")
    assert CODE in blob and URI in blob


def test_no_colour_honours_env(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(sys.stdout, "isatty", lambda: True)
    monkeypatch.setenv("NO_COLOR", "1")
    assert theme.enabled() is False


def test_show_checks_the_stream_it_writes_to(monkeypatch: pytest.MonkeyPatch) -> None:
    """A redirected stderr must not receive colour just because stdout is a TTY."""

    class FakeOut(io.StringIO):
        def isatty(self) -> bool:
            return True

    class FakeErr(io.StringIO):
        def isatty(self) -> bool:
            return False

    out, err = FakeOut(), FakeErr()
    monkeypatch.setattr(sys, "stdout", out)
    monkeypatch.setattr(sys, "stderr", err)
    row = "\x1b[38;2;255;0;0mboom\x1b[0m"
    theme.show([row], err=True)
    assert "\x1b[" not in err.getvalue(), "colour leaked into redirected stderr"
    assert "boom" in err.getvalue()


# --------------------------------------------------------------- sigil


def test_sigil_is_rectangular(tty: None) -> None:
    rows = plain(theme.sigil("ok"))
    assert len(rows) == 15
    assert {len(r) for r in rows} == {30}


def test_sigil_reads_without_colour(tty: None) -> None:
    """The picture must live in the characters — that is what makes the mono
    fallback legible and lets a text effect animate it unflattened."""
    assert len(set("".join(plain(theme.sigil("ok")))) - {" "}) >= 3


def test_states_are_visually_distinct(tty: None) -> None:
    ok = "\n".join(theme.sigil("ok"))
    bad = "\n".join(theme.sigil("bad"))
    idle = "\n".join(theme.sigil("idle"))
    assert ok != bad != idle
    assert "38;2;136;255;136" in "\n".join(cloud._signed_in_card("a@b", "k", "keyring"))


def test_rendering_is_deterministic(tty: None) -> None:
    assert theme.sigil("ok") == theme.sigil("ok")


def test_whoami_stays_one_line(tty: None) -> None:
    rows = cloud._whoami_line("e@x", "data")
    assert len(rows) == 1
    assert plain(rows)[0] == "e@x (scopes: data)"
