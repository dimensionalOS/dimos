#!/usr/bin/env python3
# Per-client priority outbox — the QoS enforcement point. The browser link (wifi/cellular) is the
# bottleneck where the fast bus meets the slow client; declaring a topic "important" only matters if
# something *enforces* it here. This replaces the data plane's single FIFO queue (which tail-drops
# indiscriminately under a slow client) with a priority + conflation + weighted-round-robin discipline:
#
#   • each topic lands in a priority CLASS (command > sensor > default > bulk), via default_priority()
#   • best_effort topics keep only the LATEST frame (conflation — a backed-up lidar collapses, never grows)
#   • reliable topics keep a bounded deque (DDS keep_last depth)
#   • the writer drains by WEIGHTED round-robin: high classes get most of the budget, low classes keep a
#     floor (so pose/teleop win under contention but lidar never starves to exactly zero)
#
# This is not novel — it's Foxglove's per-client bounded drop-queue + Reactive-Streams onBackpressureLatest
# (conflation) + DDS/Zenoh priority-with-drop, composed at the per-client egress. The shedding is explicit
# (bounded memory) so it does not rely on ws.send() blocking.
from __future__ import annotations

import asyncio
import re
from collections import OrderedDict, deque

# lane → (priority rank, conflate?, keep_last depth). Mirrors packages/topics/src/qos.ts LANES + defaultLane.
_CMD = re.compile(r"(^|/)(cmd_vel|cmd|teleop|goal|clicked_point|move_base|nav_goal)(/|$)", re.I)
_BULK = re.compile(
    r"(^|/)(lidar|laser|scan|points?|point_?cloud|cloud|camera|image|img|depth|rgb|map|costmap|occupancy|grid)(/|$)",
    re.I,
)
_SENSOR = re.compile(r"(^|/)(pose|odom|imu|tf|joint|battery|twist|velocity|state|wrench|p\d+)(/|$)", re.I)
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
_WEIGHTS = {3: 8, 2: 4, 1: 2, 0: 1}  # weighted round-robin budget per class (low keeps a floor of 1)


def default_priority(topic: str, typ: str = "") -> tuple[int, bool, int]:
    """Server-side mirror of qos.ts defaultLane → (rank, conflate, depth)."""
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
    """A per-client outbox the writer drains. `fifo=True` reproduces the old single bounded FIFO (the A/B
    baseline); otherwise it's the priority + conflation + WRR scheduler above. Never blocks on put."""

    def __init__(self, fifo: bool = False, fifo_max: int = 4096) -> None:
        self.fifo = fifo
        self._fifo_max = fifo_max
        self._fifo: deque = deque()
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
        if self.fifo:
            self._fifo.append(item)
            while len(self._fifo) > self._fifo_max:
                self._fifo.popleft()  # bounded: drop oldest (Foxglove-style)
        else:
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
        if self.fifo:
            return self._fifo.popleft() if self._fifo else None
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
