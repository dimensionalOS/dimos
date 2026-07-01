#!/usr/bin/env python3
# scope-cam: a perception (multi-camera) scenario. Namespace: /cam/*
#
#   /cam/rgb         Image            @30 Hz   ~550 KB   RGB camera        (≈16 MB/s)
#   /cam/depth       Image            @15 Hz   ~300 KB   GRAY depth        (≈4.5 MB/s)
#   /cam/points      PointCloud2      @10 Hz   ~1 MB     dense cloud       (≈10 MB/s)
#   /cam/detections  Detection2DArray @30 Hz   tiny      per-frame metadata
#
# Data-path axis: bandwidth / bufferbloat (~30 MB/s of bulk), the WebTransport head-of-line story where
# big frames must not stall the small /cam/detections topic (the important topic that must survive the
# bulk, so its payload is minimal by design).
#
# Run (from dimos/web/dimoscope): DIMOS_TRANSPORT=zenoh uv run python scenarios/cam.py
from common import (
    Detection2DArray,
    Image,
    ImageFormat,
    Module,
    ModuleConfig,
    Out,
    PointCloud2,
    Seq,
    env_f,
    env_i,
    make_image,
    np,
    rpc,
    run_standalone,
    rx,
    stamp_header,
    time,
)


class ScopeCamConfig(ModuleConfig):
    rgb_hz: float = env_f("SCOPE_CAM_RGB_HZ", 30.0)
    rgb_bytes: int = env_i("SCOPE_CAM_RGB_BYTES", 550_000)
    depth_hz: float = env_f("SCOPE_CAM_DEPTH_HZ", 15.0)
    depth_bytes: int = env_i("SCOPE_CAM_DEPTH_BYTES", 300_000)
    points_hz: float = env_f("SCOPE_CAM_POINTS_HZ", 10.0)
    points_pts: int = env_i("SCOPE_CAM_POINTS_PTS", 62500)  # ×16 B/pt ≈ 1 MB
    det_hz: float = env_f("SCOPE_CAM_DET_HZ", 30.0)


class ScopeCam(Module):
    """Perception scenario: RGB + depth cameras + dense point cloud + detections."""

    config: ScopeCamConfig
    rgb: Out[Image]
    depth: Out[Image]
    points: Out[PointCloud2]
    detections: Out[Detection2DArray]

    @rpc
    def start(self) -> None:
        c = self.config
        seq = Seq()

        if c.rgb_hz > 0:
            rgb = make_image(c.rgb_bytes, ImageFormat.RGB)

            def tick_rgb(_: int) -> None:
                rgb.ts = time.time()
                rgb.frame_id = seq("rgb")
                self.rgb.publish(rgb)

            self.register_disposable(rx.interval(1.0 / c.rgb_hz).subscribe(tick_rgb))

        if c.depth_hz > 0:
            depth = make_image(c.depth_bytes, ImageFormat.GRAY)

            def tick_depth(_: int) -> None:
                depth.ts = time.time()
                depth.frame_id = seq("depth")
                self.depth.publish(depth)

            self.register_disposable(rx.interval(1.0 / c.depth_hz).subscribe(tick_depth))

        if c.points_hz > 0:
            span = np.array([4.0, 4.0, 3.0], dtype=np.float32)
            arr = np.random.rand(c.points_pts, 3).astype(np.float32) * span
            pc = PointCloud2.from_numpy(arr, frame_id="0", timestamp=time.time())

            def tick_points(_: int) -> None:
                pc.ts = time.time()
                pc.frame_id = seq("points")
                self.points.publish(pc)

            self.register_disposable(rx.interval(1.0 / c.points_hz).subscribe(tick_points))

        if c.det_hz > 0:

            def tick_det(_: int) -> None:
                d = Detection2DArray()
                d.header = stamp_header(seq("det"))
                d.detections = []
                d.detections_length = 0
                self.detections.publish(d)

            self.register_disposable(rx.interval(1.0 / c.det_hz).subscribe(tick_det))


scope_cam = ScopeCam.blueprint()

# (attr, topic, MsgType) — the shared topic↔type source of truth (runtime wiring + `gen_types.py`).
PORTS = [
    ("rgb", "/cam/rgb", Image),
    ("depth", "/cam/depth", Image),
    ("points", "/cam/points", PointCloud2),
    ("detections", "/cam/detections", Detection2DArray),
]

if __name__ == "__main__":
    run_standalone(ScopeCam(), PORTS, "scope-cam")
