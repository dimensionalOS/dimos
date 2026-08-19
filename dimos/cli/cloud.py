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

Device-code flow (RFC 8628 shaped) against login.dimensional.org — built for robots:
no browser or clipboard needed on this machine. The CLI prints an 8-character code,
you approve it from any signed-in browser (laptop, phone), and the minted API key is
stored in the system keyring, falling back to a plain-text 0600 file
(`CREDENTIALS_PATH`, just the key) on headless machines with no keyring
backend. `DIMOS_API_KEY` (via GlobalConfig) overrides any stored login.
"""

import importlib.metadata
import json
import os
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


# --- presentation -----------------------------------------------------------
# Copy lives here; the components that draw it live in `theme`. `reveal` is the
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


def _login_card(uri: str, code: str, spin: str = "", clock: str = "") -> list[str]:
    """The one card the whole wait lives in: URL, code, and a ticking last line."""
    body = [
        theme.paint("Sign in to Dimensional", theme.GREY_RAMP[2]),
        theme.paint(f"Open      {uri}", theme.MUTED),
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
    """One card, not two: who you are, then what DimOS is."""
    cols = min(theme.term_width(), 110)
    width = max(24, cols - 2 - (22 + 3) - 4)
    body = [
        theme.paint("Signed in", theme.rgb("agent")),
        theme.paint(email, theme.rgb("white")),
        "",
        theme.paint(f"key      {key_id}… · {where}", theme.MUTED),
        theme.paint(f"version  DimOS v{_version()}", theme.MUTED),
        "",
        theme.paint("About DimOS", theme.rgb("white")),
    ]
    body += [theme.paint(ln, theme.MUTED) for ln in textwrap.wrap(ABOUT, width)]
    body += [
        "",
        theme.paint("Join our community: ", theme.MUTED)
        + theme.paint(COMMUNITY, theme.rgb("cyan")),
        "",
    ]
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
    """Sweep the wordmark in. Silent off a terminal."""
    if not theme.enabled():
        return
    from terminaltexteffects.effects.effect_beams import Beams

    effect = Beams("\n".join(rows))
    # Without this the effect crops to the width it detects and the wordmark
    # loses its last letter. The art carries its own dimensions.
    effect.terminal_config.ignore_terminal_dimensions = True
    effect.terminal_config.frame_rate = 100_000
    frames = list(effect)
    started, n = time.time(), 60
    for k in range(n):
        sys.stdout.write(
            "\033[H" + frames[min(len(frames) - 1, int(len(frames) * k / (n - 1)))] + "\n"
        )
        sys.stdout.flush()
        slack = started + (k + 1) / 30 - time.time()
        if slack > 0:
            time.sleep(slack)


def _wordmark() -> list[str]:
    lines = theme.ascii_logo.strip("\n").split("\n")
    return [
        theme.paint(ln, theme.ramp(theme.PORTAL, i / max(1, len(lines) - 1)))
        for i, ln in enumerate(lines)
    ]


def _logged_out_card(revoke_url: str) -> list[str]:
    return theme.card(
        [
            theme.paint("Logged out", theme.GREY_RAMP[2]),
            theme.paint("Key removed from this machine.", theme.MUTED),
            theme.paint("It stays valid until revoked.", theme.MUTED),
            "",
            theme.paint(revoke_url, theme.rgb("cyan")),
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
    with urllib.request.urlopen(urllib.request.Request(url, method="POST")) as r:
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
    with theme.Live() as live:
        spin = live.spinner()

        def frame() -> list[str]:
            left = max(0, int(deadline - time.time()))
            return _login_card(
                d["verification_uri"], d["user_code"], next(spin), f"{left // 60}:{left % 60:02d}"
            )

        live.update(frame())
        while time.time() < deadline:
            live.pause(d["interval"], frame)
            r = _post("/auth/token", device_code=d["device_code"])
            if r["status"] == "ok":
                where = _store(r["api_key"])
                live.clear()
                _reveal(_wordmark())
                theme.show(_signed_in_card(r["email"], r["key_id"], where))
                return
            if r["status"] in ("denied", "expired"):
                denied = r["status"] == "denied"
                live.clear()
                theme.show(
                    _refused_card(
                        "Denied" if denied else "Code expired",
                        "The code was rejected in the browser."
                        if denied
                        else f"{d['user_code']} was never approved.",
                        "No key was created." if denied else "Codes are valid for 15 minutes.",
                    ),
                    err=True,
                )
                raise typer.Exit(1)
        live.clear()
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
    """Forget the stored key. Revoke it fully at login.dimensional.org/keys."""
    if _forget():
        theme.show(_logged_out_card(f"{_base()}/keys"))
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
        with urllib.request.urlopen(req) as r:
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
