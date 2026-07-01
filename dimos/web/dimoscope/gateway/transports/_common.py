#!/usr/bin/env python3
# Shared helpers for the bench delivery transports: the [f64 gateway-send-ms][LC02] framing (the same
# frame the WS data plane sends, so every path decodes through the browser's identical frameToSample)
# plus subscription matching + the ?topics= query parse.
from __future__ import annotations

import struct
import time

from ..bus import Sample


def frame(s: Sample) -> bytes:
    return struct.pack(">d", time.time() * 1000.0) + s.lc02  # [f64 gateway-send-ms][LC02]


def wants(subs: set[str], topic: str) -> bool:
    return "*" in subs or topic in subs


def subs_from_query(tp: str | None) -> set[str]:
    if tp is None or tp == "*":
        return {"*"}
    return {t for t in tp.split(",") if t}
