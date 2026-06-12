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

"""DimOS Visual Console — Reflex app for the Rerun topic selector.

Implements the "DimOS Visual Console" design handoff: a dark mission-control
layout with a header bar, a fixed-width LCM topic catalog rail (search,
filter chips, grouped topic table), an embedded Rerun web viewer, and a
stage → apply selection tray.
"""

from __future__ import annotations

import asyncio
import os
import time
from typing import Any
from urllib.parse import urlsplit

import reflex as rx
import requests

# ── Configuration ────────────────────────────────────────────────


def _selector_api_url() -> str:
    return os.environ.get("DIMOS_SELECTOR_API_URL", "http://127.0.0.1:9880").rstrip("/")


def _selector_title() -> str:
    return os.environ.get("DIMOS_SELECTOR_TITLE", "DimOS Visual Console")


def _rerun_url() -> str:
    return os.environ.get("DIMOS_SELECTOR_RERUN_URL", "http://127.0.0.1:9878/")


# ── Catalog classification & formatting (pure helpers) ──────────

GROUP_ORDER = [
    "Perception",
    "Robot state",
    "Navigation",
    "Control",
    "Text / logs",
    "Other",
]

HEAVY_BANDWIDTH_BPS = 50_000.0
HEAVY_TYPE_KEYWORDS = ("image", "pointcloud", "occupancygrid", "video", "laserscan")

PERCEPTION_KEYWORDS = (
    "image",
    "camera",
    "pointcloud",
    "laser",
    "lidar",
    "occupancy",
    "costmap",
    "detection",
    "video",
    "marker",
    "track",
)
NAVIGATION_KEYWORDS = ("goal", "path", "plan", "waypoint")
CONTROL_KEYWORDS = ("cmd", "twist", "sport_mode", "motor")
ROBOT_STATE_KEYWORDS = (
    "odom",
    "tf",
    "imu",
    "joint",
    "battery",
    "gps",
    "navsat",
    "pose",
    "point",
    "transform",
    "robot",
)
TEXT_KEYWORDS = ("log", "text", "str", "diagnostic", "thought", "status")


def topic_group(channel: str, type_name: str | None) -> str:
    """Assign a catalog row to a fixed display group (best-effort keywords)."""

    haystack = f"{channel} {type_name}".lower()
    if any(keyword in haystack for keyword in PERCEPTION_KEYWORDS):
        return "Perception"
    if any(keyword in channel.lower() for keyword in NAVIGATION_KEYWORDS):
        return "Navigation"
    if any(keyword in haystack for keyword in CONTROL_KEYWORDS):
        return "Control"
    if any(keyword in haystack for keyword in ROBOT_STATE_KEYWORDS):
        return "Robot state"
    if any(keyword in haystack for keyword in TEXT_KEYWORDS):
        return "Text / logs"
    return "Other"


def is_heavy(row: dict[str, Any]) -> bool:
    """Heavy = sustained high bandwidth or a known bulky message type."""

    if float(row.get("bandwidth_bps") or 0.0) >= HEAVY_BANDWIDTH_BPS:
        return True
    type_name = (row.get("type_name") or "").lower()
    return any(keyword in type_name for keyword in HEAVY_TYPE_KEYWORDS)


def render_badge(renderability: str | None, render_reason: str | None) -> tuple[str, str]:
    """Return the (css kind, label) for a row's render badge."""

    if renderability == "renderable":
        if "converter" in (render_reason or ""):
            return "converter", "converter"
        return "native", "renderable"
    if renderability == "unsupported":
        return "unsupported", "unsupported"
    return "unsupported", "unsupported"


def fmt_hz(hz: float | None) -> str:
    if not hz or hz <= 0:
        return "—"
    if hz >= 10:
        return f"{hz:.0f} Hz"
    return f"{hz:.1f} Hz"


def fmt_bw(bps: float | None) -> str:
    if not bps or bps <= 0:
        return "—"
    if bps >= 1e6:
        return f"{bps / 1e6:.1f} MB/s"
    if bps >= 1e3:
        return f"{bps / 1e3:.0f} kB/s"
    return f"{round(bps)} B/s"


def fmt_ago(age_s: float | None) -> str:
    if age_s is None:
        return "never"
    if age_s < 2:
        return "now"
    if age_s < 60:
        return f"{int(age_s)}s ago"
    if age_s < 3600:
        return f"{int(age_s / 60)}m ago"
    return f"{int(age_s / 3600)}h ago"


def enrich_row(raw: dict[str, Any], *, now_monotonic: float | None = None) -> dict[str, Any]:
    """Precompute every display field the UI needs for one catalog row."""

    now = now_monotonic if now_monotonic is not None else time.monotonic()
    channel = str(raw.get("channel") or "")
    name = str(raw.get("name") or channel)
    type_name = raw.get("type_name")
    renderability = raw.get("renderability")
    render_reason = str(raw.get("render_reason") or "")
    selected = bool(raw.get("selected"))
    logging = bool(raw.get("logging"))
    live = bool(raw.get("live"))
    selectable = renderability == "renderable"
    heavy = is_heavy(raw)
    badge_kind, badge_label = render_badge(renderability, render_reason)
    last_seen = raw.get("last_seen_monotonic")
    age = max(0.0, now - float(last_seen)) if last_seen is not None else None
    last_error = str(raw.get("last_error") or "")

    row_class = "vc-row"
    if selected:
        row_class += " is-staged"
    if not selectable:
        row_class += " is-disabled"
    check_class = "vc-check"
    if selected:
        check_class += " is-checked"
    if logging:
        check_class += " is-applied"

    return {
        "kind": "topic",
        "channel": channel,
        "name": name,
        "type_name": type_name or "",
        "group": topic_group(channel, type_name),
        "renderability": renderability,
        "selectable": selectable,
        "selected": selected,
        "logging": logging,
        "live": live,
        "heavy": heavy,
        "bandwidth_bps": float(raw.get("bandwidth_bps") or 0.0),
        "row_class": row_class,
        "check_class": check_class,
        "badge_class": f"vc-badge b-{badge_kind}",
        "badge_label": badge_label,
        "render_reason": render_reason,
        "rate_text": fmt_hz(raw.get("rate_hz")),
        "bw_text": fmt_bw(raw.get("bandwidth_bps")),
        "bw_class": "num mono is-heavy" if heavy else "num mono",
        "bw_title": (
            "heavy — high bandwidth. Selecting starts decode + Rerun logging." if heavy else ""
        ),
        "state_class": "vc-state s-live" if live else "vc-state s-idle",
        "state_label": "live" if live else "idle",
        "state_title": f"last seen {fmt_ago(age)}",
        "name_title": last_error or channel,
        "row_title": "" if selectable else render_reason,
        "last_error": last_error,
    }


def enrich_rows(raw_rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    now = time.monotonic()
    return [enrich_row(raw, now_monotonic=now) for raw in raw_rows]


def grouped_display_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Flatten rows into group-header + topic items in fixed group order."""

    out: list[dict[str, Any]] = []
    for group in GROUP_ORDER:
        members = [row for row in rows if row.get("group") == group]
        if not members:
            continue
        out.append({"kind": "group", "group": group, "count": len(members)})
        out.extend(members)
    return out


def filter_rows(
    rows: list[dict[str, Any]],
    *,
    search_query: str = "",
    renderable_only: bool = False,
    live_only: bool = False,
    heavy_only: bool = False,
    selected_only: bool = False,
) -> list[dict[str, Any]]:
    """Return catalog rows matching the selector UI filters."""

    query = search_query.lower().strip()
    filtered = rows
    if query:
        filtered = [
            row
            for row in filtered
            if query in str(row.get("channel", "")).lower()
            or query in str(row.get("type_name", "")).lower()
            or query in str(row.get("name", "")).lower()
        ]
    if renderable_only:
        filtered = [row for row in filtered if row.get("renderability") == "renderable"]
    if live_only:
        filtered = [row for row in filtered if row.get("live")]
    if heavy_only:
        filtered = [row for row in filtered if row.get("heavy")]
    if selected_only:
        filtered = [row for row in filtered if row.get("selected") or row.get("logging")]
    return filtered


def row_counts(rows: list[dict[str, Any]]) -> tuple[int, int, int, int]:
    """Return staged, logging, live, and renderable row counts."""

    return (
        sum(1 for row in rows if row.get("selected")),
        sum(1 for row in rows if row.get("logging")),
        sum(1 for row in rows if row.get("live")),
        sum(1 for row in rows if row.get("renderability") == "renderable"),
    )


def status_text(rows: list[dict[str, Any]], *, live_count: int, renderable_count: int) -> str:
    """Return degraded/normal selector status copy for catalog rows."""

    if not rows:
        return "No decodable LCM data observed yet. Start a stack that publishes typed DimOS LCM topics."
    if renderable_count == 0:
        return "LCM traffic is visible, but no observed topic has a Rerun converter yet."
    return f"LCM {len(rows)} channels · {live_count} live · {renderable_count} renderable"


def staged_topics_after_toggle(
    rows: list[dict[str, Any]], channel: str, selected: bool
) -> list[str]:
    """Return the staged topic list after toggling one topic row."""

    staged = {
        str(row.get("channel"))
        for row in rows
        if row.get("selected") and row.get("channel") != channel and row.get("channel") is not None
    }
    if selected:
        staged.add(channel)
    return sorted(staged)


def selection_dirty(rows: list[dict[str, Any]]) -> bool:
    """True when the staged selection differs from the applied/logging set."""

    staged = {row.get("channel") for row in rows if row.get("selected")}
    applied = {row.get("channel") for row in rows if row.get("logging")}
    return staged != applied


# ── State ────────────────────────────────────────────────────────


class SelectorState(rx.State):
    """Reflex state for the DimOS Rerun topic selector."""

    api_url: str = _selector_api_url()
    title: str = _selector_title()
    rerun_url: str = _rerun_url()
    catalog_rows: list[dict[str, Any]] = []
    search_query: str = ""
    filter_renderable: bool = False
    filter_live: bool = False
    filter_heavy: bool = False
    filter_selected: bool = False
    last_error: str = ""
    rerun_ok: bool = True
    viewer_visible: bool = True
    _refresh_tick: int = 0

    @rx.var
    def filtered_rows(self) -> list[dict[str, Any]]:
        return filter_rows(
            self.catalog_rows,
            search_query=self.search_query,
            renderable_only=self.filter_renderable,
            live_only=self.filter_live,
            heavy_only=self.filter_heavy,
            selected_only=self.filter_selected,
        )

    @rx.var
    def display_rows(self) -> list[dict[str, Any]]:
        return grouped_display_rows(self.filtered_rows)

    @rx.var
    def total_count(self) -> int:
        return len(self.catalog_rows)

    @rx.var
    def visible_count(self) -> int:
        return len(self.filtered_rows)

    @rx.var
    def has_observed(self) -> bool:
        return bool(self.catalog_rows)

    @rx.var
    def has_visible(self) -> bool:
        return bool(self.filtered_rows)

    @rx.var
    def staged_count(self) -> int:
        return row_counts(self.catalog_rows)[0]

    @rx.var
    def logging_count(self) -> int:
        return row_counts(self.catalog_rows)[1]

    @rx.var
    def live_count(self) -> int:
        return row_counts(self.catalog_rows)[2]

    @rx.var
    def logging_rows(self) -> list[dict[str, Any]]:
        return [row for row in self.catalog_rows if row.get("logging")]

    @rx.var
    def dirty(self) -> bool:
        return selection_dirty(self.catalog_rows)

    @rx.var
    def apply_disabled(self) -> bool:
        return not selection_dirty(self.catalog_rows)

    @rx.var
    def clear_disabled(self) -> bool:
        staged, logging, _, _ = row_counts(self.catalog_rows)
        return staged == 0 and logging == 0

    @rx.var
    def staged_bw_text(self) -> str:
        total = sum(
            float(row.get("bandwidth_bps") or 0.0)
            for row in self.catalog_rows
            if row.get("selected")
        )
        return f"~{fmt_bw(total)}" if total > 0 else ""

    @rx.var
    def heavy_staged_count(self) -> int:
        return sum(1 for row in self.catalog_rows if row.get("selected") and row.get("heavy"))

    @rx.var
    def heavy_staged_title(self) -> str:
        names = [
            str(row.get("name"))
            for row in self.catalog_rows
            if row.get("selected") and row.get("heavy")
        ]
        return ", ".join(names)

    @rx.var
    def lcm_chip_text(self) -> str:
        if not self.catalog_rows:
            if self.last_error:
                return "selector API unreachable"
            return "LCM no traffic"
        return f"LCM {len(self.catalog_rows)} ch · {self.live_count} live"

    @rx.var
    def lcm_chip_class(self) -> str:
        if not self.catalog_rows:
            return "hdr-status bad" if self.last_error else "hdr-status warn"
        return "hdr-status ok"

    @rx.var
    def rerun_chip_text(self) -> str:
        return "Rerun connected" if self.rerun_ok else "Rerun unreachable"

    @rx.var
    def rerun_chip_class(self) -> str:
        return "hdr-status ok" if self.rerun_ok else "hdr-status bad"

    @rx.var
    def empty_title(self) -> str:
        if self.last_error:
            return "Selector API unreachable"
        return "No LCM traffic observed"

    @rx.var
    def empty_body(self) -> str:
        if self.last_error:
            return f"Waiting for the DimOS bridge at {self.api_url}. {self.last_error}"
        return status_text(self.catalog_rows, live_count=0, renderable_count=0)

    @rx.var
    def api_host_chip(self) -> str:
        return self.api_url.removeprefix("http://").removeprefix("https://")

    def _probe_rerun(self) -> None:
        parts = urlsplit(self.rerun_url)
        try:
            requests.get(f"{parts.scheme}://{parts.netloc}/", timeout=0.8)
            self.rerun_ok = True
        except Exception:
            self.rerun_ok = False

    def _post(self, path: str, payload: dict[str, Any] | None = None) -> None:
        response = requests.post(f"{self.api_url}/{path}", json=payload or {}, timeout=1.5)
        response.raise_for_status()

    @rx.event
    def refresh_catalog(self) -> None:
        try:
            response = requests.get(f"{self.api_url}/catalog", timeout=1.5)
            response.raise_for_status()
            payload = response.json()
            self.catalog_rows = enrich_rows(list(payload.get("catalog") or []))
            self.title = str(payload.get("title") or self.title)
            self.rerun_url = str(payload.get("rerun_url") or self.rerun_url)
            self.last_error = ""
        except Exception as exc:
            self.last_error = str(exc)
        # The Rerun web viewer probe is cheap but still an extra request; do
        # it on the first tick and then every 5 ticks.
        if self._refresh_tick % 5 == 0:
            self._probe_rerun()
        self._refresh_tick += 1

    @rx.event
    def set_search_query(self, value: str) -> None:
        self.search_query = value

    @rx.event
    def toggle_filter(self, name: str) -> None:
        if name == "renderable":
            self.filter_renderable = not self.filter_renderable
        elif name == "live":
            self.filter_live = not self.filter_live
        elif name == "heavy":
            self.filter_heavy = not self.filter_heavy
        elif name == "selected":
            self.filter_selected = not self.filter_selected

    @rx.event
    def toggle_topic(self, channel: str) -> None:
        row = next(
            (row for row in self.catalog_rows if row.get("channel") == channel),
            None,
        )
        if row is None or not row.get("selectable"):
            return
        try:
            self._post(
                "stage",
                {
                    "topics": staged_topics_after_toggle(
                        self.catalog_rows, channel, not row.get("selected")
                    )
                },
            )
            self.refresh_catalog()
        except Exception as exc:
            self.last_error = str(exc)

    @rx.event
    def apply_staged(self) -> None:
        try:
            self._post("apply")
            self.refresh_catalog()
        except Exception as exc:
            self.last_error = str(exc)

    @rx.event
    def clear_staged(self) -> None:
        try:
            self._post("clear")
            self.refresh_catalog()
        except Exception as exc:
            self.last_error = str(exc)

    @rx.event
    async def reconnect_viewer(self) -> Any:
        """Force-remount the viewer iframe and re-probe the Rerun server."""

        self.viewer_visible = False
        yield
        await asyncio.sleep(0.1)
        self._probe_rerun()
        self.viewer_visible = True


# ── Stylesheet (design tokens from the Visual Console handoff) ──

GLOBAL_CSS = """
:root {
  --bg0: #0b0e13;
  --bg1: #10141b;
  --bg2: #151b24;
  --bg3: #1b222e;
  --line: #232b38;
  --line-soft: #1b2230;
  --text: #d9e0ea;
  --dim: #8b95a5;
  --faint: #5b6473;
  --accent: #34d399;
  --warn: #fbbf24;
  --bad: #f87171;
  --sans: 'IBM Plex Sans', system-ui, sans-serif;
  --mono: 'JetBrains Mono', ui-monospace, monospace;
}
html, body { margin: 0; height: 100%; background: var(--bg0); }
body { color: var(--text); font-family: var(--sans); font-size: 13px; }
.mono { font-family: var(--mono); }
.dim { color: var(--dim); }
button { font-family: var(--sans); cursor: pointer; }
button:disabled { cursor: not-allowed; opacity: 0.45; }

.vc-root { height: 100vh; display: flex; flex-direction: column; overflow: hidden; background: var(--bg0); color: var(--text); font-family: var(--sans); font-size: 13px; }

/* ── Header ─────────────────────────────────────── */
.vc-header { display: flex; align-items: center; gap: 12px; height: 46px; padding: 0 14px; background: var(--bg1); border-bottom: 1px solid var(--line); flex: none; }
.hdr-mark { width: 10px; height: 10px; background: var(--accent); flex: none; }
.vc-header h1 { font-size: 13px; font-weight: 600; letter-spacing: 0.04em; text-transform: uppercase; margin: 0; }
.hdr-chip { font-size: 11px; font-family: var(--mono); color: var(--dim); background: var(--bg2); border: 1px solid var(--line); padding: 3px 8px; border-radius: 3px; }
.hdr-spacer { flex: 1; }
.hdr-status { display: inline-flex; align-items: center; gap: 6px; font-size: 11px; font-weight: 500; letter-spacing: 0.02em; color: var(--dim); border: 1px solid var(--line); padding: 3px 9px; border-radius: 3px; background: var(--bg2); }
.hdr-status .dot { width: 7px; height: 7px; border-radius: 50%; }
.hdr-status.ok .dot { background: var(--accent); box-shadow: 0 0 6px color-mix(in oklab, var(--accent) 60%, transparent); }
.hdr-status.warn .dot { background: var(--warn); }
.hdr-status.bad .dot { background: var(--bad); }
.hdr-status.bad { color: var(--bad); border-color: color-mix(in oklab, var(--bad) 35%, var(--line)); }
.hdr-status.warn { color: var(--warn); }

/* ── Main split ─────────────────────────────────── */
.vc-main { flex: 1; display: flex; min-height: 0; }
.vc-rail { width: 500px; flex: none; display: flex; flex-direction: column; min-height: 0; background: var(--bg1); border-right: 1px solid var(--line); }
.rail-controls { padding: 10px 10px 8px; border-bottom: 1px solid var(--line-soft); display: flex; flex-direction: column; gap: 8px; flex: none; }
.vc-search { width: 100%; box-sizing: border-box; background: var(--bg0); border: 1px solid var(--line); color: var(--text); font-family: var(--mono); font-size: 12px; padding: 7px 10px; border-radius: 3px; outline: none; }
.vc-search:focus { border-color: color-mix(in oklab, var(--accent) 55%, var(--line)); }
.vc-search::placeholder { color: var(--faint); }
.vc-filterrow { display: flex; align-items: center; gap: 6px; }
.filter-meta { margin-left: auto; font-size: 10px; font-family: var(--mono); color: var(--faint); }

.vc-chipbtn { background: var(--bg2); border: 1px solid var(--line); color: var(--dim); font-size: 11px; padding: 3px 9px; border-radius: 3px; line-height: 1.4; }
.vc-chipbtn:hover { border-color: var(--faint); color: var(--text); }
.vc-chipbtn.is-on { background: color-mix(in oklab, var(--accent) 16%, var(--bg2)); border-color: color-mix(in oklab, var(--accent) 50%, var(--line)); color: var(--accent); }

.rail-scroll { flex: 1; overflow-y: auto; min-height: 0; }
.rail-scroll::-webkit-scrollbar { width: 8px; }
.rail-scroll::-webkit-scrollbar-thumb { background: var(--line); border-radius: 4px; }

/* ── Table ──────────────────────────────────────── */
.vc-thead, .vc-row { display: grid; grid-template-columns: 26px minmax(0,1fr) 80px 54px 72px 58px; align-items: center; gap: 4px; padding: 0 8px 0 6px; }
.vc-thead { position: sticky; top: 0; z-index: 2; background: var(--bg1); height: 26px; font-size: 9.5px; text-transform: uppercase; letter-spacing: 0.08em; color: var(--faint); border-bottom: 1px solid var(--line-soft); }
.vc-thead .num, .vc-row .num { text-align: right; }
.vc-row { min-height: 32px; border-bottom: 1px solid var(--line-soft); font-size: 11px; cursor: pointer; }
.vc-row:hover { background: var(--bg2); }
.vc-row.is-staged { background: color-mix(in oklab, var(--accent) 7%, var(--bg1)); }
.vc-row.is-staged:hover { background: color-mix(in oklab, var(--accent) 11%, var(--bg1)); }
.vc-row.is-disabled { cursor: default; }
.vc-row.is-disabled .vc-topicname, .vc-row.is-disabled .num, .vc-row.is-disabled .vc-state { opacity: 0.55; }
.vc-row .num { font-family: var(--mono); font-size: 10.5px; }
.vc-row .num.is-heavy { color: var(--warn); font-weight: 600; }
.cell-name { display: flex; align-items: center; gap: 6px; min-width: 0; }

.vc-group-h { display: flex; align-items: baseline; gap: 7px; padding: 10px 10px 4px; font-size: 10px; font-weight: 600; text-transform: uppercase; letter-spacing: 0.1em; color: var(--dim); }
.gh-count { color: var(--faint); font-weight: 400; }

.vc-topicname { display: flex; flex-direction: column; gap: 1px; min-width: 0; }
.tn-channel { font-family: var(--mono); font-size: 11.5px; font-weight: 500; color: var(--text); white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
.tn-type { font-family: var(--mono); font-size: 9.5px; color: var(--faint); white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
/* ── Badges ─────────────────────────────────────── */
.vc-badge { display: inline-flex; align-items: center; font-size: 9px; font-weight: 600; letter-spacing: 0.06em; text-transform: uppercase; padding: 2px 6px; border-radius: 3px; border: 1px solid transparent; white-space: nowrap; }
.b-native { color: var(--accent); border-color: color-mix(in oklab, var(--accent) 45%, transparent); background: color-mix(in oklab, var(--accent) 10%, transparent); }
.b-converter { color: color-mix(in oklab, var(--accent) 70%, var(--text)); border-color: color-mix(in oklab, var(--accent) 28%, var(--line)); background: transparent; }
.b-unsupported { color: var(--faint); border-color: var(--line); }
.b-logging { color: var(--bg0); background: var(--accent); border-color: var(--accent); }

.vc-state { display: inline-flex; align-items: center; gap: 5px; font-family: var(--mono); font-size: 10px; color: var(--dim); }
.vc-state .dot { width: 6px; height: 6px; border-radius: 50%; background: var(--faint); }
.vc-state.s-live { color: var(--accent); }
.vc-state.s-live .dot { background: var(--accent); box-shadow: 0 0 5px color-mix(in oklab, var(--accent) 55%, transparent); }

/* ── Checkbox ───────────────────────────────────── */
.vc-check { width: 16px; height: 16px; border-radius: 3px; border: 1px solid var(--faint); background: transparent; color: var(--bg0); display: inline-flex; align-items: center; justify-content: center; font-size: 11px; font-weight: 700; line-height: 1; flex: none; pointer-events: none; }
.vc-check.is-checked { background: color-mix(in oklab, var(--accent) 80%, var(--bg0)); border-color: var(--accent); }
.vc-check.is-checked.is-applied { background: var(--accent); }
.vc-row:hover .vc-check { border-color: var(--accent); }
.vc-row.is-disabled .vc-check { border-color: var(--line); }

/* ── Notices / empty states ─────────────────────── */
.vc-notice { margin: 8px; padding: 9px 11px; font-size: 11.5px; line-height: 1.5; color: var(--warn); background: color-mix(in oklab, var(--warn) 7%, var(--bg2)); border: 1px solid color-mix(in oklab, var(--warn) 30%, var(--line)); border-radius: 3px; }
.vc-notice .mono { font-size: 10.5px; }
.vc-empty { display: flex; flex-direction: column; align-items: center; gap: 10px; padding: 56px 32px; text-align: center; }
.empty-scope { display: flex; gap: 6px; }
.empty-scope span { width: 8px; height: 8px; border-radius: 50%; border: 1px solid var(--faint); animation: vc-pulse 1.6s infinite; }
.empty-scope span:nth-child(2) { animation-delay: 0.25s; }
.empty-scope span:nth-child(3) { animation-delay: 0.5s; }
@keyframes vc-pulse { 0%, 100% { background: transparent; } 50% { background: var(--faint); } }
@media (prefers-reduced-motion: reduce) { .empty-scope span { animation: none; } }
.empty-title { font-size: 13px; font-weight: 600; }
.empty-body { font-size: 11.5px; line-height: 1.6; color: var(--dim); max-width: 320px; overflow-wrap: anywhere; }

/* ── Viewer panel ───────────────────────────────── */
.vc-viewer { flex: 1; display: flex; flex-direction: column; min-width: 0; background: var(--bg0); }
.viewer-bar { display: flex; align-items: center; gap: 10px; height: 38px; padding: 0 12px; background: var(--bg1); border-bottom: 1px solid var(--line); flex: none; }
.conn-dot { width: 8px; height: 8px; border-radius: 50%; flex: none; }
.conn-dot.ok { background: var(--accent); box-shadow: 0 0 6px color-mix(in oklab, var(--accent) 60%, transparent); }
.conn-dot.bad { background: var(--bad); }
.viewer-title { font-size: 11px; font-weight: 600; text-transform: uppercase; letter-spacing: 0.06em; }
.viewer-url { font-family: var(--mono); font-size: 10.5px; color: var(--faint); white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
.viewer-spacer { flex: 1; }

.vc-btn { font-size: 11px; font-weight: 500; padding: 5px 12px; border-radius: 3px; border: 1px solid var(--line); background: var(--bg2); color: var(--text); text-decoration: none; display: inline-flex; align-items: center; }
.vc-btn:hover:not(:disabled) { border-color: var(--faint); }
.vc-btn.ghost { background: transparent; color: var(--dim); }
.vc-btn.primary { background: var(--bg3); border-color: var(--line); color: var(--dim); }
.vc-btn.primary.is-dirty { background: var(--accent); border-color: var(--accent); color: #08110d; font-weight: 600; }
.vc-btn.primary.is-dirty:hover { background: color-mix(in oklab, var(--accent) 88%, white); }

.viewer-body { flex: 1; min-height: 0; padding: 10px; display: flex; }
.viewer-iframe { flex: 1; width: 100%; min-height: 0; border: 1px solid var(--line); border-radius: 4px; background: var(--bg0); }
.viewer-placeholder { flex: 1; border: 1px solid var(--line); border-radius: 4px; background: repeating-linear-gradient(-45deg, var(--bg1) 0 14px, var(--bg0) 14px 28px); }

.viewer-down { align-items: center; justify-content: center; }
.down-box { max-width: 460px; border: 1px solid color-mix(in oklab, var(--bad) 35%, var(--line)); border-radius: 4px; background: var(--bg1); padding: 22px 26px; display: flex; flex-direction: column; gap: 10px; }
.down-title { font-size: 14px; font-weight: 600; color: var(--bad); }
.down-sub { font-size: 12px; color: var(--dim); line-height: 1.5; overflow-wrap: anywhere; }
.down-hints { margin: 2px 0 6px; padding-left: 18px; display: flex; flex-direction: column; gap: 6px; font-size: 11.5px; color: var(--dim); line-height: 1.5; }
.down-box .mono { font-size: 10.5px; color: var(--text); }
.down-box .vc-btn.primary { align-self: flex-start; background: var(--accent); border-color: var(--accent); color: #08110d; font-weight: 600; }

.viewer-foot { display: flex; align-items: center; gap: 10px; min-height: 34px; padding: 5px 12px; border-top: 1px solid var(--line); background: var(--bg1); flex: none; }
.foot-label { font-size: 9.5px; font-weight: 600; text-transform: uppercase; letter-spacing: 0.1em; color: var(--faint); flex: none; }
.foot-empty { font-size: 11px; color: var(--faint); }
.foot-entities { display: flex; flex-wrap: wrap; gap: 5px; }
.entity-chip { font-family: var(--mono); font-size: 10px; color: var(--accent); border: 1px solid color-mix(in oklab, var(--accent) 30%, var(--line)); background: color-mix(in oklab, var(--accent) 7%, transparent); padding: 2px 7px; border-radius: 3px; }

/* ── Selection tray ─────────────────────────────── */
.vc-tray { display: flex; align-items: center; gap: 16px; min-height: 50px; padding: 7px 14px; background: var(--bg1); border-top: 1px solid var(--line); flex: none; }
.tray-status { display: flex; align-items: center; gap: 8px; font-size: 12px; flex: none; }
.tray-count strong { color: var(--accent); font-size: 14px; }
.tray-sep { color: var(--faint); }
.tray-warn { color: var(--warn); font-size: 11px; font-weight: 600; }
.tray-spacer { flex: 1; }
.tray-actions { display: flex; gap: 8px; flex: none; }
"""

FONTS_URL = (
    "https://fonts.googleapis.com/css2"
    "?family=IBM+Plex+Sans:wght@400;500;600"
    "&family=JetBrains+Mono:wght@400;500;600&display=swap"
)


# ── Components ───────────────────────────────────────────────────


def header_bar() -> rx.Component:
    return rx.el.header(
        rx.el.span(class_name="hdr-mark"),
        rx.el.h1(SelectorState.title),
        rx.el.span(SelectorState.api_host_chip, class_name="hdr-chip"),
        rx.el.span(class_name="hdr-spacer"),
        rx.el.span(
            rx.el.span(class_name="dot"),
            SelectorState.lcm_chip_text,
            class_name=SelectorState.lcm_chip_class,
        ),
        rx.el.span(
            rx.el.span(class_name="dot"),
            SelectorState.rerun_chip_text,
            class_name=SelectorState.rerun_chip_class,
        ),
        class_name="vc-header",
    )


def filter_chip(label: str, active: rx.Var, key: str) -> rx.Component:
    return rx.el.button(
        label,
        class_name=rx.cond(active, "vc-chipbtn is-on", "vc-chipbtn"),
        on_click=SelectorState.toggle_filter(key),
    )


def rail_controls() -> rx.Component:
    return rx.el.div(
        rx.el.input(
            placeholder="Search LCM topics or types…",
            default_value=SelectorState.search_query,
            on_change=SelectorState.set_search_query.debounce(250),
            class_name="vc-search",
            type="search",
        ),
        rx.el.div(
            filter_chip("renderable", SelectorState.filter_renderable, "renderable"),
            filter_chip("live", SelectorState.filter_live, "live"),
            filter_chip("heavy", SelectorState.filter_heavy, "heavy"),
            filter_chip("selected", SelectorState.filter_selected, "selected"),
            rx.el.span(
                SelectorState.visible_count,
                "/",
                SelectorState.total_count,
                class_name="filter-meta",
            ),
            class_name="vc-filterrow",
        ),
        class_name="rail-controls",
    )


def table_head() -> rx.Component:
    return rx.el.div(
        rx.el.span(),
        rx.el.span("channel"),
        rx.el.span("render"),
        rx.el.span("rate", class_name="num"),
        rx.el.span("b/w", class_name="num"),
        rx.el.span("status"),
        class_name="vc-thead",
    )


def group_header(item: rx.Var[dict[str, Any]]) -> rx.Component:
    return rx.el.div(
        rx.el.span(item["group"]),
        rx.el.span(item["count"], class_name="gh-count"),
        class_name="vc-group-h",
    )


def topic_row(row: rx.Var[dict[str, Any]]) -> rx.Component:
    return rx.el.div(
        rx.el.span(
            rx.el.span(
                rx.cond(row["selected"], "✓", ""),
                class_name=row["check_class"].to(str),
            )
        ),
        rx.el.span(
            rx.el.div(
                rx.el.span(row["name"], class_name="tn-channel"),
                rx.el.span(row["type_name"], class_name="tn-type"),
                class_name="vc-topicname",
                title=row["name_title"].to(str),
            ),
            rx.cond(
                row["logging"],
                rx.el.span("logging", class_name="vc-badge b-logging"),
                rx.fragment(),
            ),
            class_name="cell-name",
        ),
        rx.el.span(
            rx.el.span(
                row["badge_label"],
                class_name=row["badge_class"].to(str),
                title=row["render_reason"].to(str),
            )
        ),
        rx.el.span(row["rate_text"], class_name="num"),
        rx.el.span(
            row["bw_text"], class_name=row["bw_class"].to(str), title=row["bw_title"].to(str)
        ),
        rx.el.span(
            rx.el.span(
                rx.el.span(class_name="dot"),
                row["state_label"],
                class_name=row["state_class"].to(str),
            ),
            title=row["state_title"].to(str),
        ),
        class_name=row["row_class"].to(str),
        title=row["row_title"].to(str),
        on_click=SelectorState.toggle_topic(row["channel"]),
    )


def display_item(item: rx.Var[dict[str, Any]]) -> rx.Component:
    return rx.cond(item["kind"] == "group", group_header(item), topic_row(item))


def empty_state(title: Any, body: Any) -> rx.Component:
    return rx.el.div(
        rx.el.div(rx.el.span(), rx.el.span(), rx.el.span(), class_name="empty-scope"),
        rx.el.div(title, class_name="empty-title"),
        rx.el.div(body, class_name="empty-body"),
        class_name="vc-empty",
    )


def catalog_rail() -> rx.Component:
    return rx.el.aside(
        rail_controls(),
        rx.el.div(
            rx.cond(
                SelectorState.has_observed,
                rx.cond(
                    SelectorState.has_visible,
                    rx.fragment(
                        table_head(),
                        rx.foreach(SelectorState.display_rows, display_item),
                    ),
                    empty_state(
                        "No topics match",
                        rx.fragment(
                            SelectorState.total_count,
                            " channels observed. Adjust search or filters.",
                        ),
                    ),
                ),
                empty_state(SelectorState.empty_title, SelectorState.empty_body),
            ),
            class_name="rail-scroll",
        ),
        class_name="vc-rail",
    )


def viewer_down_box() -> rx.Component:
    return rx.el.div(
        rx.el.div(
            rx.el.div("Rerun web viewer unreachable", class_name="down-title"),
            rx.el.div(
                "The viewer iframe could not connect to ",
                rx.el.span(SelectorState.rerun_url, class_name="mono"),
                ".",
                class_name="down-sub",
            ),
            rx.el.ul(
                rx.el.li(
                    "Check that the Rerun bridge is running with the web viewer "
                    "enabled in this blueprint's ",
                    rx.el.span("vis_module()", class_name="mono"),
                    ".",
                ),
                rx.el.li("Verify the viewer port is not bound by another instance."),
                rx.el.li(
                    "Pinned SDK is ",
                    rx.el.span("rerun-sdk==0.32.0a1", class_name="mono"),
                    " — the viewer version must match.",
                ),
                class_name="down-hints",
            ),
            rx.el.button(
                "Retry connection",
                class_name="vc-btn primary",
                on_click=SelectorState.reconnect_viewer,
            ),
            class_name="down-box",
        ),
        class_name="viewer-body viewer-down",
    )


def viewer_panel() -> rx.Component:
    return rx.el.section(
        rx.el.header(
            rx.el.span(class_name=rx.cond(SelectorState.rerun_ok, "conn-dot ok", "conn-dot bad")),
            rx.el.span("Rerun viewer", class_name="viewer-title"),
            rx.el.span(SelectorState.rerun_url, class_name="viewer-url"),
            rx.el.span(class_name="viewer-spacer"),
            rx.el.button(
                "Reconnect",
                class_name="vc-btn ghost",
                on_click=SelectorState.reconnect_viewer,
            ),
            rx.cond(
                SelectorState.rerun_ok,
                rx.el.a(
                    "Open in tab ↗",
                    href=SelectorState.rerun_url,
                    target="_blank",
                    class_name="vc-btn ghost",
                ),
                rx.el.button("Open in tab ↗", class_name="vc-btn ghost", disabled=True),
            ),
            class_name="viewer-bar",
        ),
        rx.cond(
            SelectorState.rerun_ok,
            rx.el.div(
                rx.cond(
                    SelectorState.viewer_visible,
                    rx.el.iframe(src=SelectorState.rerun_url, class_name="viewer-iframe"),
                    rx.el.div(class_name="viewer-placeholder"),
                ),
                class_name="viewer-body",
            ),
            viewer_down_box(),
        ),
        rx.el.footer(
            rx.el.span("bridge", class_name="foot-label"),
            rx.cond(
                SelectorState.logging_count > 0,
                rx.el.div(
                    rx.foreach(
                        SelectorState.logging_rows,
                        lambda row: rx.el.span(
                            row["name"],
                            class_name="entity-chip",
                            title=row["channel"].to(str),
                        ),
                    ),
                    class_name="foot-entities",
                ),
                rx.el.span(
                    "no topics logged — select topics in the catalog and apply",
                    class_name="foot-empty",
                ),
            ),
            class_name="viewer-foot",
        ),
        class_name="vc-viewer",
    )


def selection_tray() -> rx.Component:
    return rx.el.div(
        rx.el.div(
            rx.el.span(
                rx.el.strong(SelectorState.staged_count),
                " staged",
                class_name="tray-count",
            ),
            rx.el.span("·", class_name="tray-sep"),
            rx.el.span(SelectorState.logging_count, " logging", class_name="dim"),
            rx.cond(
                SelectorState.staged_bw_text != "",
                rx.fragment(
                    rx.el.span("·", class_name="tray-sep"),
                    rx.el.span(SelectorState.staged_bw_text, class_name="mono dim"),
                ),
                rx.fragment(),
            ),
            rx.cond(
                SelectorState.heavy_staged_count > 0,
                rx.el.span(
                    "⚠ ",
                    SelectorState.heavy_staged_count,
                    " heavy",
                    class_name="tray-warn",
                    title=SelectorState.heavy_staged_title,
                ),
                rx.fragment(),
            ),
            class_name="tray-status",
        ),
        rx.el.span(class_name="tray-spacer"),
        rx.el.div(
            rx.el.button(
                "Clear",
                class_name="vc-btn ghost",
                on_click=SelectorState.clear_staged,
                disabled=SelectorState.clear_disabled,
            ),
            rx.el.button(
                rx.cond(SelectorState.dirty, "Apply selection", "Applied"),
                class_name=rx.cond(
                    SelectorState.dirty, "vc-btn primary is-dirty", "vc-btn primary"
                ),
                on_click=SelectorState.apply_staged,
                disabled=SelectorState.apply_disabled,
            ),
            class_name="tray-actions",
        ),
        class_name="vc-tray",
    )


def index() -> rx.Component:
    return rx.el.div(
        rx.el.style(GLOBAL_CSS),
        header_bar(),
        rx.el.div(catalog_rail(), viewer_panel(), class_name="vc-main"),
        selection_tray(),
        rx.moment(interval=1000, on_change=SelectorState.refresh_catalog, display="none"),
        class_name="vc-root",
    )


app = rx.App(stylesheets=[FONTS_URL])
app.add_page(index, route="/", title="DimOS Visual Console", on_load=SelectorState.refresh_catalog)
