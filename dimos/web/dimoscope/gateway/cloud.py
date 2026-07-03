#!/usr/bin/env python3
# dimoscope cloud plane: server-side PointCloud2 thinning/coding, so the browser sees a
# browser-appropriate representation of heavy geometry instead of the raw 320 KB/frame firehose.
# A Bus consumer (like media.py) that, for every source PointCloud2, republishes derived variants
# ONTO THE BUS so they flow through every existing transport (WS/WT/WebRTC), lane, and the LVC —
# no new endpoint, no client wiring for the downsampled path:
#   <topic>_ds     — stride-decimated to CLOUD_DS_MAX_POINTS, still a *standard* PointCloud2
#                    (re-encoded via lcm_encode) → the existing @dimos/msgs decode + WorldView render
#                    work unchanged. This is the guaranteed bandwidth win (~10×, lossy decimation).
#   <topic>_draco  — full-resolution, Draco-quantized (DracoPy), on a custom type "draco.PointCloud2":
#                    payload = [u32be seq][u32be n_points][f64be ts_s][draco bytes]. The client
#                    measures it with a cheap header parse (bench) and decodes geometry with draco3d
#                    only for rendering.
# Decoding/encoding is CPU-heavy (open3d + Draco) so it runs in a single-worker executor, off the loop.
from __future__ import annotations

import asyncio
from concurrent.futures import ThreadPoolExecutor
import os
import struct

from dimos.utils.logging_config import setup_logger

from .bus import Bus, Sample

logger = setup_logger()

# PointCloud2 pulls open3d + numpy (heavy). A dimos build without them → no cloud plane (degrade).
try:
    import numpy as np

    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

    HAS_CLOUD = True
except Exception:  # pragma: no cover - depends on the optional open3d/numpy install
    HAS_CLOUD = False

# Draco point-cloud codec (optional): geometry quantization at the source. Preflight-gated —
# a missing DracoPy just disables the _draco variant; _ds still ships.
try:
    import DracoPy  # type: ignore[import-untyped]

    HAS_DRACO = True
except Exception:  # pragma: no cover - optional native dep
    HAS_DRACO = False

DS_ON = os.environ.get("CLOUD_DS", "1") == "1"
DS_MAX_POINTS = int(os.environ.get("CLOUD_DS_MAX_POINTS", "2000"))
DRACO_ON = os.environ.get("CLOUD_DRACO", "1") == "1" and HAS_DRACO
DRACO_QUANT_BITS = int(os.environ.get("CLOUD_QUANT_BITS", "11"))
DRACO_TYPE = "draco.PointCloud2"  # custom wire type; the client branches on it before @dimos/msgs


def _seq_of(frame_id: str | None) -> int:
    """The bench stamps frame_id = str(seq); recover it so derived variants keep offered/delivered."""
    if frame_id and frame_id.isdigit() and len(frame_id) < 10:
        return int(frame_id)
    return 0


class CloudPlane:
    def __init__(self, bus: Bus) -> None:
        self.bus = bus
        self._loop: asyncio.AbstractEventLoop | None = None
        self._in_q: asyncio.Queue = asyncio.Queue(maxsize=64)  # (topic, payload) → transcode worker
        self._exec = ThreadPoolExecutor(max_workers=1)  # serialise: one open3d/Draco op at a time
        self.enabled = HAS_CLOUD and (DS_ON or DRACO_ON)
        if self.enabled:
            bus.subscribe(self._on_sample)
            logger.info(
                "cloud plane on",
                ds=DS_ON,
                ds_max=DS_MAX_POINTS,
                draco=DRACO_ON,
                quant_bits=DRACO_QUANT_BITS,
            )
        elif not HAS_CLOUD:
            logger.warning("cloud plane off — PointCloud2/open3d unavailable")

    # bus tap (loop thread, cheap): only source PointCloud2 topics; skip our own derived outputs.
    def _on_sample(self, s: Sample) -> None:
        if "PointCloud2" not in (s.type or ""):
            return
        if s.topic.endswith("_ds") or s.topic.endswith("_draco"):
            return  # feedback guard — never transcode a derived cloud
        try:
            self._in_q.put_nowait((s.topic, s.payload))
        except asyncio.QueueFull:
            pass  # geometry is freshest-wins; drop under backpressure

    async def run(self) -> None:
        self._loop = asyncio.get_running_loop()
        while True:
            topic, payload = await self._in_q.get()
            await self._loop.run_in_executor(self._exec, self._process, topic, payload, self._loop)

    def _process(self, topic: str, payload: bytes, loop) -> None:
        """Executor thread: decode once, emit the downsampled + Draco variants."""
        try:
            pc = PointCloud2.lcm_decode(payload)
            pts = pc.points_f32()  # (N, 3) float32
        except Exception:
            return
        if pts is None or len(pts) == 0:
            return
        frame_id = getattr(pc, "frame_id", "0")
        ts = getattr(pc, "ts", None)

        if DS_ON:
            self._emit_ds(topic, pts, frame_id, ts, loop)
        if DRACO_ON:
            self._emit_draco(topic, pts, frame_id, ts, loop)

    def _emit_ds(self, topic: str, pts, frame_id: str, ts, loop) -> None:
        try:
            if len(pts) > DS_MAX_POINTS:
                step = -(-len(pts) // DS_MAX_POINTS)  # ceil division → deterministic stride
                ds = pts[::step]
            else:
                ds = pts
            out = PointCloud2.from_numpy(ds, frame_id=frame_id, timestamp=ts)
            payload = out.lcm_encode(frame_id=frame_id)  # keep frame_id → seq continuity for the bench
        except Exception:
            return
        ds_topic = topic + "_ds"
        lc02 = self.bus._make_lc02(f"{ds_topic}#sensor_msgs.PointCloud2", payload)
        loop.call_soon_threadsafe(
            self.bus._publish, ds_topic, "sensor_msgs.PointCloud2", lc02, payload
        )

    def _emit_draco(self, topic: str, pts, frame_id: str, ts, loop) -> None:
        try:
            draco = DracoPy.encode(
                pts.astype(np.float32),
                quantization_bits=DRACO_QUANT_BITS,
            )
            # envelope: [u32be seq][u32be nPoints][f64be ts_s] then the Draco blob. ts → the client's
            # srcTs so the Draco variant reports end-to-end latency like the standard clouds.
            payload = struct.pack(">IId", _seq_of(frame_id), len(pts), float(ts or 0.0)) + bytes(draco)
        except Exception:
            return
        dr_topic = topic + "_draco"
        lc02 = self.bus._make_lc02(f"{dr_topic}#{DRACO_TYPE}", payload)
        loop.call_soon_threadsafe(self.bus._publish, dr_topic, DRACO_TYPE, lc02, payload)
