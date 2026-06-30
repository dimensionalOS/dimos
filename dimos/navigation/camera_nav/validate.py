# Copyright 2025-2026 Dimensional Inc.
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

"""Standalone validation: webcam → depth → accumulate → costmap → Rerun.

    python -m dimos.navigation.camera_nav.validate [--stride 8] [--fov 70]

Watch /depth, /frame_cloud, /global_map, and /costmap grow in the browser.
"""

from __future__ import annotations

import argparse
import time

import cv2
import numpy as np
import rerun as rr

from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.mapping.pointclouds.accumulators.general import GeneralPointCloudAccumulator
from dimos.mapping.pointclouds.occupancy import height_cost_occupancy
from dimos.perception.depth.monocular_depth_module import MonocularDepthModule
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _log_depth(depth_np: np.ndarray) -> None:
    rr.log("/depth", rr.DepthImage(depth_np, meter=1.0))


def _log_cloud(path: str, points: np.ndarray, colors: np.ndarray | None) -> None:
    if len(points) == 0:
        return
    if colors is not None and len(colors) == len(points):
        rr.log(path, rr.Points3D(positions=points, colors=(colors * 255).astype(np.uint8)))
    else:
        rr.log(path, rr.Points3D(positions=points))


def _log_costmap(grid: OccupancyGrid) -> None:
    cost = grid.grid.astype(np.float32)
    cost[cost < 0] = 50.0
    rr.log("/costmap", rr.Image(((cost / 100.0) * 255).astype(np.uint8)))


def main() -> None:
    parser = argparse.ArgumentParser(description="Camera-nav validation")
    parser.add_argument("--device", default=None)
    parser.add_argument("--cam", type=int, default=0)
    parser.add_argument("--fov", type=float, default=70.0)
    parser.add_argument("--stride", type=int, default=8)
    parser.add_argument("--max-depth", type=float, default=8.0)
    args = parser.parse_args()

    rr.init("camera_nav_validate")
    server_uri = rr.serve_grpc()
    rr.serve_web_viewer(connect_to=server_uri, open_browser=True)

    cap = cv2.VideoCapture(args.cam)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera {args.cam}")
    ret, frame = cap.read()
    if not ret:
        raise RuntimeError("Cannot read from camera")
    H, W = frame.shape[:2]
    logger.info("Camera: %dx%d", W, H)

    camera_info = CameraInfo.from_fov(args.fov, W, H, axis="horizontal", frame_id="camera_optical")
    depth_module = MonocularDepthModule(device=args.device, stride=args.stride, max_depth=args.max_depth, max_freq=5.0)
    depth_module._load_model()
    depth_module._latest_camera_info = camera_info

    from dimos.core.global_config import GlobalConfig
    accum = GeneralPointCloudAccumulator(voxel_size=0.05, global_config=GlobalConfig())

    logger.info("Model ready — check browser. Press Ctrl+C to stop.")

    frame_count = 0
    t_last = time.perf_counter()

    try:
        while True:
            ret, bgr = cap.read()
            if not ret:
                time.sleep(0.05)
                continue

            ts = time.time()
            image = Image(data=bgr, format=ImageFormat.BGR, frame_id="camera_optical", ts=ts)

            try:
                pts_cam, cols = depth_module._run_inference(image)
            except Exception:
                logger.exception("Inference error")
                time.sleep(0.1)
                continue

            _log_cloud("/frame_cloud", pts_cam, cols)

            if len(pts_cam) > 0:
                import open3d as o3d
                frame_pcd = o3d.geometry.PointCloud()
                frame_pcd.points = o3d.utility.Vector3dVector(pts_cam.astype(float))
                if cols is not None:
                    frame_pcd.colors = o3d.utility.Vector3dVector(cols.astype(float))
                accum.add(frame_pcd)

                global_pcd = accum.get_point_cloud()
                pts_g = np.asarray(global_pcd.points, dtype=np.float32)
                cols_g = np.asarray(global_pcd.colors, dtype=np.float32) if global_pcd.has_colors() else None
                _log_cloud("/global_map", pts_g, cols_g)

                try:
                    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
                    cloud = PointCloud2.from_numpy(pts_g, frame_id="camera_optical", timestamp=ts)
                    grid = height_cost_occupancy(cloud, resolution=0.05)
                    _log_costmap(grid)
                except Exception:
                    logger.exception("Costmap error")

            frame_count += 1
            now = time.perf_counter()
            if now - t_last >= 5.0:
                n_pts = len(accum.get_point_cloud().points)
                logger.info("%.1f fps  |  frame pts: %d  |  global pts: %d", frame_count / (now - t_last), len(pts_cam), n_pts)
                frame_count = 0
                t_last = now

            time.sleep(max(0.0, 0.2 - (time.perf_counter() - now)))

    except KeyboardInterrupt:
        logger.info("Stopped.")
    finally:
        cap.release()


if __name__ == "__main__":
    main()
