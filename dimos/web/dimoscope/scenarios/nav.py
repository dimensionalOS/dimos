#!/usr/bin/env python3
# scope-nav — a mobile-navigation robot scenario. Namespace: /nav/*
#
#   /nav/pose   PoseStamped    @100 Hz   tiny, high-rate   (robot driving a slow circle)
#   /nav/path   Path           @  5 Hz   small             (a 20-pose planned path)
#   /nav/cloud  PointCloud2    @ 10 Hz   ~200 KB           (a lidar-like sweep)
#   /nav/map    OccupancyGrid  @  1 Hz   ~40 KB            (a 200×200 costmap)
#
# Data-path axis: SIZE-MIX — a tiny high-rate pose interleaved with a periodic big map/cloud. This
# is the QoS-priority story (keep pose crisp while the bulk degrades). All four auto-render in the
# app's WorldView 2D (arrow + trail, path, height-coloured points, grey grid) — by message TYPE, so
# the topic names don't matter.
#
# Run (from dimos/web/dimoscope):  DIMOS_TRANSPORT=zenoh uv run python scenarios/nav.py
from common import (
    IDENT,
    Module,
    ModuleConfig,
    OccupancyGrid,
    Out,
    Path,
    PointCloud2,
    Pose,
    PoseStamped,
    Seq,
    env_f,
    env_i,
    math,
    np,
    rpc,
    run_standalone,
    rx,
    time,
)


class ScopeNavConfig(ModuleConfig):
    pose_hz: float = env_f("SCOPE_NAV_POSE_HZ", 100.0)
    path_hz: float = env_f("SCOPE_NAV_PATH_HZ", 5.0)
    cloud_hz: float = env_f("SCOPE_NAV_CLOUD_HZ", 10.0)
    cloud_pts: int = env_i("SCOPE_NAV_CLOUD_PTS", 12500)  # ×16 B/pt ≈ 200 KB
    map_hz: float = env_f("SCOPE_NAV_MAP_HZ", 1.0)
    map_cells: int = env_i("SCOPE_NAV_MAP_CELLS", 200)  # N×N int8 grid


class ScopeNav(Module):
    """Mobile-navigation scenario: pose + path + lidar cloud + occupancy map."""

    config: ScopeNavConfig
    pose: Out[PoseStamped]
    path: Out[Path]
    cloud: Out[PointCloud2]
    map: Out[OccupancyGrid]

    @rpc
    def start(self) -> None:
        c = self.config
        seq = Seq()
        t0 = time.time()

        def tick_pose(_: int) -> None:
            t = time.time() - t0
            self.pose.publish(
                PoseStamped(
                    ts=time.time(),
                    frame_id=seq("pose"),
                    position=(3.0 * math.cos(0.2 * t), 3.0 * math.sin(0.2 * t), 0.0),
                    orientation=IDENT,
                )
            )

        def tick_path(_: int) -> None:
            poses = [
                PoseStamped(
                    ts=time.time(), frame_id="p", position=(0.3 * i, 0.0, 0.0), orientation=IDENT
                )
                for i in range(20)
            ]
            self.path.publish(Path(ts=time.time(), frame_id=seq("path"), poses=poses))

        if c.pose_hz > 0:
            self.register_disposable(rx.interval(1.0 / c.pose_hz).subscribe(tick_pose))
        if c.path_hz > 0:
            self.register_disposable(rx.interval(1.0 / c.path_hz).subscribe(tick_path))

        if c.cloud_hz > 0:
            span = np.array([6.0, 6.0, 2.0], dtype=np.float32)
            off = np.array([3.0, 3.0, 0.0], dtype=np.float32)
            pts = np.random.rand(c.cloud_pts, 3).astype(np.float32) * span - off
            cloud = PointCloud2.from_numpy(pts, frame_id="0", timestamp=time.time())

            def tick_cloud(_: int) -> None:
                cloud.ts = time.time()
                cloud.frame_id = seq("cloud")
                self.cloud.publish(cloud)

            self.register_disposable(rx.interval(1.0 / c.cloud_hz).subscribe(tick_cloud))

        if c.map_hz > 0:
            n = c.map_cells
            grid = (np.random.rand(n, n) > 0.85).astype(np.int8) * 100

            def tick_map(_: int) -> None:
                self.map.publish(
                    OccupancyGrid(
                        grid=grid,
                        resolution=0.1,
                        origin=Pose(-n * 0.05, -n * 0.05, 0.0),
                        frame_id=seq("map"),
                        ts=time.time(),
                    )
                )

            self.register_disposable(rx.interval(1.0 / c.map_hz).subscribe(tick_map))

    @rpc
    def navigate_to(self, goal: PoseStamped) -> bool:
        """Demo command with a typed message arg — codegen emits
        `navigate_to(goal: geometry_msgs.PoseStamped): Promise<boolean>`."""
        self.pose.publish(goal)
        return True


scope_nav = ScopeNav.blueprint()

# (attr, topic, MsgType) — the shared topic↔type source of truth (runtime wiring + `gen_types.py`).
PORTS = [
    ("pose", "/nav/pose", PoseStamped),
    ("path", "/nav/path", Path),
    ("cloud", "/nav/cloud", PointCloud2),
    ("map", "/nav/map", OccupancyGrid),
]

if __name__ == "__main__":
    run_standalone(ScopeNav(), PORTS, "scope-nav")
