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

"""Dimensional cloud auth: `dimos login` / `dimos logout` / `dimos whoami`.

Device-code flow (RFC 8628 shaped) against api.dimensional.org — built for robots:
no browser or clipboard needed on this machine. On a local desktop the CLI opens the
verification page for you; anywhere else (over SSH, headless, a robot) it just prints
the URL and an 8-character code, which you approve from any signed-in browser, and the
minted API key is
stored in the system keyring, falling back to a plain-text 0600 file
(`CREDENTIALS_PATH`, just the key) on headless machines with no keyring
backend. `DIMOS_API_KEY` (via GlobalConfig) overrides any stored login.
"""

import importlib.metadata
import json
import os
from pathlib import Path
import socket
import sys
import textwrap
import time
from types import ModuleType
from typing import Any, cast
import urllib.error
import urllib.parse
import urllib.request

import typer

from dimos.cli import theme
from dimos.constants import CREDENTIALS_PATH
from dimos.core.global_config import global_config

_KEYRING_SERVICE = "dimos-cloud"
_KEYRING_USER = "default"


# Presentation. Copy lives here; the components that draw it live in `theme`. `reveal` is the
# only thing needing terminaltexteffects, so it is imported inside the function
# — `dimos --help` must not pay for an animation it will never run.

ABOUT = (
    "DimOS is the open-source operating system for physical space. Our platform lets "
    "developers build physical AI applications with natural-language control, across "
    "any hardware form factor: humanoids, quadrupeds, drones, and robotic arms."
)
COMMUNITY = "discord.gg/dimos"
CAPABILITIES: list[tuple[str, str]] = [
    ("Navigation", "SLAM, obstacle avoidance, route planning, exploration"),
    ("Perception", "detection, tracking, spatial memory"),
    ("Manipulation", "grasp synthesis, pick and place, constraints"),
]


def _version() -> str:
    """The installed version, for the signed-in card. Never fatal."""
    try:
        return importlib.metadata.version("dimos")
    except importlib.metadata.PackageNotFoundError:
        return "dev"


def _open_browser(url: str) -> bool:
    """Open the verification page for the user, when it is safe to.

    Only on a local terminal with a display. Over SSH ``webbrowser.open`` would
    launch a browser on the far end, a robot; headless it can seize the terminal
    with a text browser. The URL is printed regardless, so the fallback is the
    same everywhere. ``NO_BROWSER`` opts out.
    """
    if not sys.stdout.isatty() or os.environ.get("NO_BROWSER"):
        return False
    if os.environ.get("SSH_CONNECTION") or os.environ.get("SSH_TTY"):
        return False
    if sys.platform not in ("darwin", "win32") and not (
        os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
    ):
        return False
    try:
        import webbrowser

        return webbrowser.open(url)
    except Exception:
        return False


def _login_card(
    uri: str, code: str, spin: str = "", clock: str = "", opened: bool = False
) -> list[str]:
    """The one card the whole wait lives in: URL, code, and a ticking last line."""
    body = [
        theme.paint("Sign in to Dimensional", theme.GREY_RAMP[2]),
        theme.paint(f"{'Opened' if opened else 'Open':<10}{uri}", theme.MUTED),
        "",
        theme.paint(f"Code      {code}", theme.rgb("white")),
    ]
    if spin:
        body.append(
            theme.paint(f"{spin} waiting for approval", theme.rgb("cyan"))
            + theme.paint(f"   expires in {clock}", theme.MUTED)
        )
    return theme.card(body, "idle")


def _signed_in_card(email: str, key_id: str, where: str) -> list[str]:
    """One card, not two: who you are, then what DimOS is.

    The prose ladders down with the width. Wrapped into a narrow terminal the
    About paragraph becomes a wall taller than the window meant to hold it, and
    it pushes the two lines that matter — who you are, and that it worked — off
    the top of the scrollback. So the blurbs go first, then the paragraph, and
    the identity block is what always survives.
    """
    cols = min(theme.term_width(), 110)
    width = max(24, cols - 2 - (22 + 3) - 4)
    # A robot without a keyring stores the key in a file; "~" keeps that path
    # short enough that the card keeps its art.
    where = where.replace(str(Path.home()), "~", 1)
    body = [
        theme.paint("Signed in", theme.rgb("agent")),
        theme.paint(email, theme.rgb("white")),
        "",
        theme.paint(f"key      {key_id}… · {where}", theme.MUTED),
        theme.paint(f"version  DimOS v{_version()}", theme.MUTED),
    ]
    if cols >= 48:
        body += [
            "",
            theme.paint("Join our community: ", theme.MUTED)
            + theme.paint(COMMUNITY, theme.rgb("cyan")),
        ]
    if cols < 72:
        # Too little room for the About prose; identity and the community link
        # are what a narrow terminal keeps.
        return theme.card(body, "ok", cols, cap=110)

    body += ["", theme.paint("About DimOS", theme.rgb("white"))]
    body += [theme.paint(ln, theme.MUTED) for ln in textwrap.wrap(ABOUT, width)]
    body.append("")
    for name, blurb in CAPABILITIES:
        body.append(theme.paint("▸ ", theme.SALMON) + theme.paint(name, theme.rgb("white")))
        body += [theme.paint("   " + ln, theme.MUTED) for ln in textwrap.wrap(blurb, width - 3)]
        body.append("")
    return theme.card(body[:-1], "ok", cols, cap=110)


def _refused_card(title: str, *details: str, hint: str = "dimos login") -> list[str]:
    body = [theme.paint(title, theme.RED_RAMP[2])]
    body += [theme.paint(d, theme.MUTED) for d in details]
    return theme.card([*body, "", theme.paint(hint, theme.rgb("cyan"))], "bad")


def _reveal(rows: list[str]) -> None:
    """Sweep the wordmark in. Silent off a terminal, and skipped when it will
    not fit.

    The effect is told to ignore terminal dimensions so it stops cropping the
    wordmark's last letter — which means it will happily overflow instead. A
    wordmark wider than the terminal wraps into nonsense, so below its own width
    we skip the flourish entirely; the signed-in card still carries the brand.
    """
    if not theme.enabled():
        return
    if theme.term_width() < max(theme.visible_len(r) for r in rows):
        return
    from terminaltexteffects.effects.effect_beams import Beams

    effect = Beams("\n".join(rows))
    # Without this the effect crops to the width it detects and the wordmark
    # loses its last letter. The art carries its own dimensions.
    effect.terminal_config.ignore_terminal_dimensions = True
    effect.terminal_config.frame_rate = 100_000
    frames = list(effect)
    started, n, fps = time.time(), 60, 60  # 60 frames at 60fps -> a ~1s sweep
    # Redraw in place, relative to where the cursor is. An absolute cursor-home
    # would paint the wordmark over whatever sits at the top of the window.
    with theme.Live() as live:
        for k in range(n):
            live.update(frames[min(len(frames) - 1, int(len(frames) * k / (n - 1)))].split("\n"))
            slack = started + (k + 1) / fps - time.time()
            if slack > 0:
                time.sleep(slack)


def _wordmark() -> list[str]:
    lines = theme.ascii_logo.strip("\n").split("\n")
    return [
        theme.paint(ln, theme.ramp(theme.PORTAL, i / max(1, len(lines) - 1)))
        for i, ln in enumerate(lines)
    ]


def _logged_out_card() -> list[str]:
    return theme.card(
        [
            theme.paint("Logged out", theme.GREY_RAMP[2]),
            theme.paint("Key removed from this machine.", theme.MUTED),
            theme.paint("It stays valid until you revoke it in the console.", theme.MUTED),
        ],
        "idle",
    )


def _whoami_line(email: str, scopes: str) -> list[str]:
    """One line, deliberately. `whoami` is a lookup people script against, not a
    moment — a card with art here would be ceremony for an answer to a question."""
    return [
        theme.paint(email, theme.rgb("white")) + theme.paint(f" (scopes: {scopes})", theme.MUTED)
    ]


def _base() -> str:
    return global_config.dimos_cloud_url.rstrip("/")


def _post(path: str, **params: str | int) -> dict[str, Any]:
    url = f"{_base()}{path}?" + urllib.parse.urlencode(params)
    with urllib.request.urlopen(
        urllib.request.Request(url, method="POST"), timeout=global_config.dimos_http_timeout
    ) as r:
        return cast("dict[str, Any]", json.load(r))


def _keyring() -> ModuleType | None:
    """The OS keyring, or None on machines without a usable backend (headless robots)."""
    try:
        import keyring

        keyring.get_password(_KEYRING_SERVICE, "probe")
        return keyring
    except Exception:
        return None


def _store(key: str) -> str:
    """Persist the API key; returns a human-readable location for the login message."""
    if kr := _keyring():
        kr.set_password(_KEYRING_SERVICE, _KEYRING_USER, key)
        return "system keyring"
    CREDENTIALS_PATH.parent.mkdir(parents=True, exist_ok=True)
    # No keyring backend (typical on robots): owner-only file, the same convention
    # gh / aws / kubectl use for exactly this situation.
    fd = os.open(CREDENTIALS_PATH, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, "w") as f:
        f.write(key + "\n")
    return str(CREDENTIALS_PATH)


def _load() -> str | None:
    if kr := _keyring():
        if key := kr.get_password(_KEYRING_SERVICE, _KEYRING_USER):
            return cast("str", key)
    try:
        return CREDENTIALS_PATH.read_text().strip() or None
    except OSError:
        return None


def _forget() -> bool:
    found = False
    if kr := _keyring():
        if kr.get_password(_KEYRING_SERVICE, _KEYRING_USER):
            kr.delete_password(_KEYRING_SERVICE, _KEYRING_USER)
            found = True
    if CREDENTIALS_PATH.exists():
        CREDENTIALS_PATH.unlink()
        found = True
    return found


def api_key() -> str | None:
    """The credential for cloud calls: DIMOS_API_KEY first, then the stored login."""
    if global_config.dimos_api_key:
        return global_config.dimos_api_key
    return _load()


def login() -> None:
    """Sign this machine in to Dimensional cloud."""
    d = _post("/auth/device", label=socket.gethostname())
    deadline = time.time() + d["expires_in"]
    opened = _open_browser(d["verification_uri"])

    # The wait runs on the alternate screen, so resizing the window repaints the
    # card cleanly instead of smearing it. The outcome is carried back out and
    # rendered on the normal screen below, where it stays in the scrollback.
    signed_in: tuple[str, str, str] | None = None
    outcome = "expired"
    with theme.Live(fullscreen=True) as live:
        spin = live.spinner()

        def frame() -> list[str]:
            left = max(0, int(deadline - time.time()))
            return _login_card(
                d["verification_uri"],
                d["user_code"],
                next(spin),
                f"{left // 60}:{left % 60:02d}",
                opened=opened,
            )

        live.update(frame())
        while time.time() < deadline:
            live.pause(d["interval"], frame)
            r = _post("/auth/token", device_code=d["device_code"])
            if r["status"] == "ok":
                signed_in = (r["email"], r["key_id"], _store(r["api_key"]))
                outcome = "ok"
                break
            if r["status"] in ("denied", "expired"):
                outcome = r["status"]
                break

    if outcome == "ok" and signed_in is not None:
        _reveal(_wordmark())
        theme.show(_signed_in_card(*signed_in))
        return
    if outcome == "denied":
        theme.show(
            _refused_card(
                "Denied",
                "The code was rejected in the browser.",
                "No key was created.",
            ),
            err=True,
        )
    else:
        theme.show(
            _refused_card(
                "Code expired",
                f"{d['user_code']} was never approved.",
                "Codes are valid for 15 minutes.",
            ),
            err=True,
        )
    raise typer.Exit(1)


def logout() -> None:
    """Forget the stored key. The key itself stays valid until revoked in the console."""
    if _forget():
        theme.show(_logged_out_card())
    else:
        typer.echo("Not logged in.")


def whoami() -> None:
    """Show which account this machine's key belongs to."""
    key = api_key()
    if not key:
        typer.echo("Not logged in — run `dimos login`.", err=True)
        raise typer.Exit(1)
    req = urllib.request.Request(
        f"{_base()}/auth/whoami", headers={"Authorization": f"Bearer {key}"}
    )
    try:
        with urllib.request.urlopen(req, timeout=global_config.dimos_http_timeout) as r:
            who = json.load(r)
    except urllib.error.HTTPError as e:
        typer.echo(
            "Key invalid or revoked — run `dimos login`."
            if e.code == 401
            else f"Cloud error: {e.code}",
            err=True,
        )
        raise typer.Exit(1) from e
    theme.show(_whoami_line(who["email"], str(who["scopes"])))
