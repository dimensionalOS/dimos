from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import time
from typing import Any


@dataclass
class XPosterStatus:
    dry_run: bool
    installed: bool
    authenticated: bool
    username: str | None = None
    detail: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class XPostResult:
    post_id: str | None
    post_url: str | None
    dry_run: bool = False


class XPoster:
    """Post Kora dispatches through the local xurl CLI."""

    def __init__(self) -> None:
        self.dry_run = _env_bool("KORA_SOCIAL_X_DRY_RUN", default=True, legacy="INFLUENCER_X_DRY_RUN")
        self.timeout_sec = float(
            _env("KORA_SOCIAL_X_TIMEOUT_SEC", "45", legacy="INFLUENCER_X_TIMEOUT_SEC")
        )
        self._status_cache: XPosterStatus | None = None
        self._status_cache_ts = 0.0
        self._status_cache_ttl_sec = float(
            _env("KORA_SOCIAL_X_STATUS_TTL_SEC", "120", legacy="INFLUENCER_X_STATUS_TTL_SEC")
        )

    def status(self) -> XPosterStatus:
        now = time.monotonic()
        if self._status_cache is not None and now - self._status_cache_ts < self._status_cache_ttl_sec:
            return self._status_cache

        status = self._fresh_status()
        self._status_cache = status
        self._status_cache_ts = now
        return status

    def set_dry_run(self, dry_run: bool) -> XPosterStatus:
        self.dry_run = dry_run
        self._status_cache = None
        self._status_cache_ts = 0.0
        return self.status()

    def _fresh_status(self) -> XPosterStatus:
        installed = shutil.which("xurl") is not None
        if self.dry_run:
            detail = "dry run on"
            if not installed:
                detail = "dry run on; xurl not installed"
            return XPosterStatus(
                dry_run=True,
                installed=installed,
                authenticated=False,
                detail=detail,
            )

        if not installed:
            return XPosterStatus(
                dry_run=False,
                installed=False,
                authenticated=False,
                detail="xurl not installed",
            )

        auth = self._run(["xurl", "auth", "status"], timeout=10)
        if auth.returncode != 0:
            return XPosterStatus(
                dry_run=False,
                installed=True,
                authenticated=False,
                detail=_safe_output(auth.stderr or auth.stdout),
            )

        whoami = self._run(["xurl", "whoami"], timeout=10)
        username = _extract_username(whoami.stdout)
        return XPosterStatus(
            dry_run=False,
            installed=True,
            authenticated=True,
            username=username,
            detail=None if whoami.returncode == 0 else _safe_output(whoami.stderr or whoami.stdout),
        )

    def post_image(self, caption: str, image_path: Path) -> XPostResult:
        if self.dry_run:
            return XPostResult(post_id="dry-run", post_url=None, dry_run=True)

        status = self.status()
        if not status.installed:
            raise RuntimeError("xurl is not installed.")
        if not status.authenticated:
            detail = f" {status.detail}" if status.detail else ""
            raise RuntimeError(f"xurl is not authenticated.{detail}")

        upload = self._run(
            [
                "xurl",
                "media",
                "upload",
                "--media-type",
                "image/jpeg",
                "--category",
                "tweet_image",
                str(image_path),
            ],
            timeout=self.timeout_sec,
        )
        if upload.returncode != 0:
            raise RuntimeError(f"xurl media upload failed: {_safe_output(upload.stderr or upload.stdout)}")

        media_id = _extract_id(upload.stdout, preferred_keys=("media_id", "media_id_string", "id"))
        if media_id is None:
            raise RuntimeError(f"xurl media upload returned no media id: {_safe_output(upload.stdout)}")

        post = self._run(
            ["xurl", "post", caption, "--media-id", media_id],
            timeout=self.timeout_sec,
        )
        if post.returncode != 0:
            raise RuntimeError(f"xurl post failed: {_safe_output(post.stderr or post.stdout)}")

        post_id = _extract_id(post.stdout, preferred_keys=("id", "post_id", "tweet_id"))
        post_url = _extract_url(post.stdout)
        if post_url is None and post_id is not None and status.username:
            post_url = f"https://x.com/{status.username}/status/{post_id}"

        return XPostResult(post_id=post_id, post_url=post_url, dry_run=False)

    def _run(self, cmd: list[str], *, timeout: float) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )


def _env(name: str, default: str, *, legacy: str | None = None) -> str:
    raw = os.getenv(name)
    if raw is not None:
        return raw
    if legacy is not None:
        raw = os.getenv(legacy)
        if raw is not None:
            return raw
    return default


def _env_bool(name: str, *, default: bool, legacy: str | None = None) -> bool:
    raw = _env(name, "1" if default else "0", legacy=legacy)
    return raw.strip().lower() in {"1", "true", "yes", "on"}


def _safe_output(value: str, *, max_len: int = 400) -> str:
    compact = " ".join(value.strip().split())
    if len(compact) <= max_len:
        return compact
    return f"{compact[:max_len]}..."


def _json_loads_maybe(value: str) -> Any | None:
    value = value.strip()
    if not value:
        return None
    try:
        return json.loads(value)
    except json.JSONDecodeError:
        return None


def _walk_json(value: Any) -> list[Any]:
    items = [value]
    if isinstance(value, dict):
        for child in value.values():
            items.extend(_walk_json(child))
    elif isinstance(value, list):
        for child in value:
            items.extend(_walk_json(child))
    return items


def _extract_id(value: str, *, preferred_keys: tuple[str, ...]) -> str | None:
    parsed = _json_loads_maybe(value)
    if parsed is not None:
        for item in _walk_json(parsed):
            if not isinstance(item, dict):
                continue
            for key in preferred_keys:
                candidate = item.get(key)
                if isinstance(candidate, str | int):
                    return str(candidate)

    patterns = [
        r"\bmedia[_ ]id(?:_string)?\b[^0-9]*(\d{6,})",
        r"\b(?:post|tweet)?[_ ]?id\b[^0-9]*(\d{6,})",
        r"\b(\d{12,})\b",
    ]
    for pattern in patterns:
        match = re.search(pattern, value, flags=re.IGNORECASE)
        if match:
            return match.group(1)
    return None


def _extract_url(value: str) -> str | None:
    match = re.search(r"https://(?:x|twitter)\.com/[A-Za-z0-9_]+/status/\d+", value)
    if match:
        return match.group(0).replace("https://twitter.com/", "https://x.com/")
    return None


def _extract_username(value: str) -> str | None:
    parsed = _json_loads_maybe(value)
    if parsed is not None:
        for item in _walk_json(parsed):
            if not isinstance(item, dict):
                continue
            for key in ("username", "screen_name", "handle"):
                candidate = item.get(key)
                if isinstance(candidate, str) and candidate:
                    return candidate.removeprefix("@")

    match = re.search(r"@([A-Za-z0-9_]{1,15})", value)
    if match:
        return match.group(1)
    match = re.search(r"\busername\b[^A-Za-z0-9_@]*@?([A-Za-z0-9_]{1,15})", value, re.IGNORECASE)
    if match:
        return match.group(1)
    return None
