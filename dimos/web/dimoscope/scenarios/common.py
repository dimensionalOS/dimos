#!/usr/bin/env python3
# Shared scaffolding for the 3 "real-world" scenario blueprints (nav / arm / cam).
#
# Each scenario is a standalone dimos Module that publishes a distinct namespace of topics at a
# distinct data profile, so the browser SDK can discover → visualize → type → benchmark them, and
# you can run one, Ctrl-C, run another (the gateway taps the bus, so the topic set just swaps).
#
# This module is a TWIN of bench/bench_source.py's scaffolding (same _tn/_mk transport helpers,
# same "build the big payload once, restamp ts+frame_id per publish" trick, same __main__ runner)
# — factored out here so nav.py/arm.py/cam.py stay to just their topic definitions.
#
#   ts        (publish wall-clock, seconds)  → one-way latency in the bench
#   frame_id  (per-topic monotonic seq)      → exact drop/gap detection in the bench
#
# Launch (from dimos/web/dimoscope):  DIMOS_TRANSPORT=zenoh uv run python scenarios/nav.py
import math
import os
import time

import numpy as np
import reactivex as rx

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray

__all__ = [
    # dimos plumbing re-exported so scenario files import from one place
    "Module", "ModuleConfig", "Out", "rpc", "rx", "np", "math", "time",
    # helpers
    "TRANSPORT", "env_f", "env_i", "tn", "mk", "Seq", "make_image", "stamp_header", "run_standalone",
    # message types
    "Pose", "PoseStamped", "Quaternion", "Vector3", "Imu",
    "OccupancyGrid", "Path", "Image", "ImageFormat", "JointState", "PointCloud2",
    "Header", "JointTrajectory", "TrajectoryPoint", "Detection2DArray", "IDENT",
]

TRANSPORT = os.environ.get("DIMOS_TRANSPORT", "lcm")
IDENT = Quaternion.from_euler(Vector3(0.0, 0.0, 0.0))


def env_f(key: str, default: float) -> float:
    return float(os.environ.get(key, default))


def env_i(key: str, default: int) -> int:
    return int(os.environ.get(key, default))


def tn(topic: str) -> str:
    # Zenoh key-exprs cannot start with "/"; LCM channels keep it. The Zenoh gateway re-adds the
    # leading "/" so the browser sees "/nav/pose" either way. (verbatim from bench_source.py)
    return topic[1:] if (TRANSPORT == "zenoh" and topic.startswith("/")) else topic


def mk(topic: str, typ: type):  # type: ignore[no-untyped-def]
    if TRANSPORT == "zenoh":
        from dimos.core.transport import ZenohTransport

        return ZenohTransport(topic, typ)
    from dimos.core.transport import LCMTransport

    return LCMTransport(topic, typ)


class Seq:
    """Per-topic monotonic counter → frame_id=str(seq) for exact drop/gap detection."""

    def __init__(self) -> None:
        self._d: dict[str, int] = {}

    def __call__(self, topic: str) -> str:
        n = self._d.get(topic, 0)
        self._d[topic] = n + 1
        return str(n)


def make_image(nbytes: int, fmt: ImageFormat = ImageFormat.RGB) -> Image:
    """An Image of ~nbytes bytes (RGB=3ch, GRAY=1ch). Built ONCE then restamped per publish so
    generation cost never dominates the send loop."""
    ch = 1 if fmt in (ImageFormat.GRAY, ImageFormat.GRAY16) else 3
    px = max(1, nbytes // ch)
    h = max(1, int(px**0.5))
    w = max(1, px // h)
    shape = (h, w) if ch == 1 else (h, w, ch)
    data = (np.arange(h * w * ch, dtype=np.int64) % 256).astype(np.uint8).reshape(shape)
    return Image(data=data, format=fmt)


def stamp_header(seq_val: str) -> Header:
    """A Header stamped with wall-clock now + frame_id=seq — for messages whose `.ts` is a
    read-only property off header.stamp (e.g. Detection2DArray)."""
    now = time.time()
    h = Header()
    h.stamp.sec = int(now)
    h.stamp.nsec = int((now % 1.0) * 1e9)
    h.frame_id = seq_val
    return h


def run_standalone(mod: Module, ports: list[tuple[str, str, type]], label: str) -> None:
    """Wire each Out port to a transport, start the module, and idle until Ctrl-C — the __main__
    body every scenario shares. `ports` = [(attr_name, topic, MsgType), ...]."""
    for attr, topic, typ in ports:
        getattr(mod, attr).transport = mk(tn(topic), typ)
    mod.start()
    print(
        f"{label}: publishing {', '.join(t for _, t, _ in ports)} over {TRANSPORT}"
        " — Ctrl-C to stop, then run another scenario.",
        flush=True,
    )
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        mod.stop()
