#!/usr/bin/env python3
# Per-client priority outbox — the QoS enforcement point at the browser-link bottleneck. Replaces the
# data plane's single FIFO queue with priority + conflation + weighted round-robin:
#   • each topic lands in a priority CLASS (command > sensor > default > bulk), via default_priority()
#   • best_effort topics keep only the LATEST frame (conflation); reliable topics keep a bounded deque
#   • the writer drains by WEIGHTED round-robin: high classes get most of the budget, low keeps a floor
#     (pose/teleop win under contention but lidar never starves to zero)
# Shedding is explicit (bounded memory), so it does not rely on ws.send() blocking.
from __future__ import annotations

import asyncio
from collections import OrderedDict, deque
from dataclasses import dataclass
import fnmatch
import json
from pathlib import Path
import re

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# lane → (priority rank, conflate?, keep_last depth). Mirrors packages/web/src/qos.ts LANES + defaultLane.
_CMD = re.compile(r"(^|/)(cmd_vel|cmd|teleop|goal|clicked_point|move_base|nav_goal)(/|$)", re.I)
_BULK = re.compile(
    r"(^|/)(lidar|laser|scan|points?|point_?cloud|cloud|camera|image|img|depth|rgb|map|costmap|occupancy|grid)(/|$)",
    re.I,
)
_SENSOR = re.compile(
    r"(^|/)(pose|odom|imu|tf|joint|battery|twist|velocity|state|wrench|p\d+)(/|$)", re.I
)
_CMD_T = re.compile(r"(Twist|Goal)", re.I)
_BULK_T = re.compile(r"(PointCloud|LaserScan|Image|OccupancyGrid|CompressedImage)", re.I)
_SENSOR_T = re.compile(r"(Pose|Odometry|Imu|Transform|JointState|Quaternion|Vector3)", re.I)

# (rank, conflate, depth) per lane
LANE_COMMAND = (3, False, 8)
LANE_SENSOR = (2, True, 1)
LANE_DEFAULT = (1, False, 16)
LANE_BULK = (0, True, 1)
CONTROL = (3, False, 256)  # json control (hello/topic/rpc-res) — top priority, generously buffered

_RANK = 4  # number of priority classes (0..3)
_WEIGHTS = {
    3: 8,
    2: 4,
    1: 2,
    0: 1,
}  # weighted round-robin budget per class (low keeps a floor of 1)


# ── optional operator config map: topic-glob → lane ─────────────────────────────────────────────
# dimos itself does QoS as a first-match glob→rule list (global_config.zenoh_qos / ZenohQoS); this is the
# dimoscope-lane analogue for the long tail of custom per-blueprint topics the name/type heuristic below
# can't classify. Off by default (no rules file → empty). It slots BETWEEN the client override and the
# heuristic — final precedence at the data plane: client override > these rules > heuristic > LANE_DEFAULT.
LANE_BY_NAME = {
    "command": LANE_COMMAND,
    "sensor": LANE_SENSOR,
    "default": LANE_DEFAULT,
    "bulk": LANE_BULK,
}


@dataclass(frozen=True)
class QosRule:
    pattern: str  # fnmatch glob vs the topic — or vs "<topic>#<type>" when the pattern contains '#'
    lane: str  # one of LANE_BY_NAME


_RULES: list[QosRule] = []


def set_qos_rules(rules: list[QosRule]) -> None:
    """Replace the active rule list (first match wins). Empty → pure name/type heuristic. Rules naming an
    unknown lane are dropped."""
    global _RULES
    _RULES = [r for r in rules if r.lane in LANE_BY_NAME]


def load_qos_rules(path) -> int:  # type: ignore[no-untyped-def]
    """Load an optional JSON rule file: `[{"topic": "/foo/**", "lane": "sensor"}, ...]`. Missing file →
    no-op; malformed → warn and skip (never crash the service). Returns the count of rules applied."""
    p = Path(path)
    if not p.is_file():
        return 0
    try:
        raw = json.loads(p.read_text())
        rules = [
            QosRule(str(r["topic"]), str(r["lane"]))
            for r in raw
            if isinstance(r, dict) and "topic" in r and "lane" in r  # skip comment/partial entries
        ]
    except (OSError, ValueError, TypeError) as e:
        logger.warning("ignoring bad qos rules file", path=str(p), error=str(e))
        return 0
    set_qos_rules(rules)
    logger.info("loaded qos rules", count=len(_RULES), path=str(p))
    return len(_RULES)


def _rule_lane(topic: str, typ: str) -> tuple[int, bool, int] | None:
    """First matching config rule → its lane tuple, else None. A '#' in the pattern matches "<topic>#<type>"."""
    for r in _RULES:
        target = f"{topic}#{typ}" if "#" in r.pattern else topic
        if fnmatch.fnmatch(target, r.pattern):
            return LANE_BY_NAME[r.lane]
    return None


def default_priority(topic: str, typ: str = "") -> tuple[int, bool, int]:
    """Server default → (rank, conflate, depth). Precedence: operator config rules (set_qos_rules /
    load_qos_rules) first, then the name/type heuristic (mirror of qos.ts defaultLane), then LANE_DEFAULT.
    A client's declared QoS still overrides all of this at the data plane (see data.py)."""
    ruled = _rule_lane(topic, typ)
    if ruled is not None:
        return ruled
    if _CMD.search(topic) or _CMD_T.search(typ):
        return LANE_COMMAND
    if _BULK.search(topic) or _BULK_T.search(typ):
        return LANE_BULK  # heavy/big beats the generic sensor match
    if _SENSOR.search(topic) or _SENSOR_T.search(typ):
        return LANE_SENSOR
    return LANE_DEFAULT


_RANK_OF = {"low": 0, "normal": 1, "high": 2, "critical": 3}  # mirror of qos.ts PRIORITY_RANK


def declared_to_class(
    priority: str | None,
    reliability: str | None,
    depth: int | None,
    default: tuple[int, bool, int],
) -> tuple[int, bool, int]:
    """A client's declared QoS (from the subscribe op) → (rank, conflate, depth), merged onto the topic's
    default — only the fields the client set are overridden. `best-effort` → conflate (latest-wins)."""
    rank = _RANK_OF.get(priority, default[0]) if priority is not None else default[0]
    conflate = (reliability == "best-effort") if reliability is not None else default[1]
    d = int(depth) if depth else default[2]
    return (rank, conflate, d)


class PriorityOutbox:
    """A per-client outbox the writer drains: the priority + conflation + WRR scheduler above.
    Never blocks on put."""

    def __init__(self) -> None:
        # per class: OrderedDict[topic -> deque(frames)]; ordered for round-robin across topics
        self._cls: list[OrderedDict[str, deque]] = [OrderedDict() for _ in range(_RANK)]
        self._credits = dict(_WEIGHTS)
        self._event = asyncio.Event()

    # ── put (loop thread, cheap, never blocks) ──────────────────────────────
    def put_control(self, item) -> None:  # type: ignore[no-untyped-def]
        self._put("\x00ctl", CONTROL[0], CONTROL[1], CONTROL[2], item)

    def put_data(self, topic: str, prio: int, conflate: bool, depth: int, item) -> None:  # type: ignore[no-untyped-def]
        self._put(topic, prio, conflate, depth, item)

    def _put(self, topic: str, prio: int, conflate: bool, depth: int, item) -> None:  # type: ignore[no-untyped-def]
        bucket = self._cls[prio].get(topic)
        if bucket is None:
            bucket = deque(maxlen=1 if conflate else max(1, depth))
            self._cls[prio][topic] = bucket
        bucket.append(item)  # conflate → maxlen=1 overwrites; reliable → bounded keep_last
        self._event.set()

    # ── get (writer) ────────────────────────────────────────────────────────
    async def get(self):  # type: ignore[no-untyped-def]
        while True:
            item = self._pick()
            if item is not None:
                return item
            self._event.clear()
            await self._event.wait()

    def _pick(self):  # type: ignore[no-untyped-def]
        # weighted round-robin: serve a class while it has credit; refill when all credited classes drained
        for _ in range(2):  # at most one refill pass
            for c in range(_RANK - 1, -1, -1):
                if self._cls[c] and self._credits[c] > 0:
                    self._credits[c] -= 1
                    return self._pop_rr(c)
            # nothing served under current credits → refill and serve highest non-empty (the floor)
            if any(self._cls[c] for c in range(_RANK)):
                self._credits = dict(_WEIGHTS)
                continue
            return None
        return None

    def _pop_rr(self, c: int):  # type: ignore[no-untyped-def]
        # round-robin across topics within a class: take the front topic, rotate it to the back
        topic, bucket = next(iter(self._cls[c].items()))
        item = bucket.popleft()
        self._cls[c].move_to_end(topic)
        if not bucket:
            del self._cls[c][topic]
        return item
