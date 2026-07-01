#!/usr/bin/env python3
# Browser-controlled network conditions: GET/POST /netem flips scoped tc netem profiles on the
# gateway host via the root wrapper (gateway/scripts/dimos-netem, installed to /usr/local/bin).
# The endpoint passes a whitelisted profile NAME only — the wrapper owns the tc/iptables arguments —
# and is OFF unless NETEM_CTL=1 (never expose net-shaping on a public box by accident). The wrapper
# self-heals (auto-clears) so a crashed browser can't leave the box degraded.
from __future__ import annotations

import asyncio
import os
import sys

from fastapi import Request
from fastapi.responses import JSONResponse

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

WRAPPER = os.environ.get("NETEM_WRAPPER", "/usr/local/bin/dimos-netem")
NETEM_CTL = os.environ.get("NETEM_CTL", "0") == "1"
HEAL_S = int(os.environ.get("DIMOS_NETEM_HEAL_S", "900"))

# Mirrors the wrapper's hardcoded table (single wire format: the id). Shaping profiles set the
# persistent condition; outage profiles are momentary (auto-restore) and don't change `active`.
PROFILES = [
    {"id": "clean", "label": "clean", "desc": "no shaping — the real path as-is (baseline)"},
    {"id": "wifi-normal", "label": "wifi-normal", "desc": "10mbit · 40±20ms · 0.3% loss — office Wi-Fi"},
    {"id": "wifi-crowded", "label": "wifi-crowded", "desc": "2mbit · 100±60ms · 1.5% bursty loss — conference/factory"},
    {"id": "wifi-edge", "label": "wifi-edge", "desc": "700kbit · 200±150ms · 5% bursty loss — robot far from AP"},
    {"id": "disaster", "label": "disaster", "desc": "200kbit · 500±300ms · 8% loss — degrade-gracefully test"},
    {"id": "loss-3", "label": "loss-3%", "desc": "100mbit · 40ms · 3% loss — HoL isolation (no bw cap)"},
    {"id": "loss-5", "label": "loss-5%", "desc": "100mbit · 40ms · 5% loss — HoL isolation (no bw cap)"},
]
OUTAGES = [
    {"id": "outage-udp-3s", "label": "UDP 3s", "desc": "drop QUIC/WT for 3s — Auto fallback test"},
    {"id": "outage-udp-10s", "label": "UDP 10s", "desc": "drop QUIC/WT for 10s"},
    {"id": "outage-all-10s", "label": "all 10s", "desc": "drop TCP+UDP for 10s — full reconnect test"},
]
_IDS = {p["id"] for p in PROFILES} | {o["id"] for o in OUTAGES}


async def _wrapper(*args: str) -> tuple[int, str]:
    proc = await asyncio.create_subprocess_exec(
        "sudo", "-n", WRAPPER, *args,
        stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT,
    )
    out, _ = await proc.communicate()
    return proc.returncode or 0, out.decode(errors="replace").strip()


async def _state() -> dict:
    enabled = NETEM_CTL and sys.platform == "linux" and os.path.exists(WRAPPER)
    active = "clean"
    if enabled:
        rc, out = await _wrapper("status")
        if rc == 0 and out:
            active = out.splitlines()[0].strip()
        else:
            enabled = False  # sudo not configured / wrapper broken → hide the UI section
    return {
        "enabled": enabled,
        "active": active,
        "healsInS": HEAL_S,
        "profiles": PROFILES,
        "outages": OUTAGES,
    }


async def get_state() -> JSONResponse:
    return JSONResponse(await _state())


async def set_profile(req: Request) -> JSONResponse:
    if not NETEM_CTL:
        return JSONResponse({"error": "netem control disabled (set NETEM_CTL=1)"}, status_code=403)
    try:
        profile = str((await req.json()).get("profile", ""))
    except Exception:
        return JSONResponse({"error": "body must be JSON: {\"profile\": \"<id>\"}"}, status_code=400)
    if profile not in _IDS:
        return JSONResponse({"error": f"unknown profile: {profile}"}, status_code=400)
    rc, out = await _wrapper(profile)
    if rc != 0:
        logger.warning("netem apply failed", profile=profile, out=out[:200])
        return JSONResponse({"error": out[:200] or f"wrapper exited {rc}"}, status_code=500)
    logger.info("netem", profile=profile, result=out[:120])
    return JSONResponse(await _state())
