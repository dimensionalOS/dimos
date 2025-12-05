from __future__ import annotations

import sys
import time
import types

import numpy as np

try:
    import yaml  # type: ignore[unused-import]
except ModuleNotFoundError:
    # Minimal stub so open3d imports don't fail when PyYAML is absent in local envs.
    yaml_stub = types.ModuleType("yaml")
    yaml_stub.safe_load = lambda *args, **kwargs: None  # type: ignore[attr-defined]
    yaml_stub.load = yaml_stub.safe_load  # type: ignore[attr-defined]
    yaml_stub.full_load = yaml_stub.safe_load  # type: ignore[attr-defined]
    yaml_stub.dump = lambda *args, **kwargs: ""  # type: ignore[attr-defined]
    yaml_stub.Loader = object  # type: ignore[attr-defined]
    sys.modules["yaml"] = yaml_stub

# used but no mapping:
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

# generated:
from dimos.msgs.sensor_msgs.PointCloud2                  import PointCloud2                # — maps directly to `rr.Points3D` from numpy points.
from dimos.msgs.nav_msgs.Path                            import Path                       # — maps to `rr.LineStrips3D` from pose positions.

# mapped but not being generated yet:
from dimos.msgs.sensor_msgs.Image                        import Image                      # — maps directly to `rr.Image` (RGB conversion via impl).
from dimos.msgs.geometry_msgs.Transform                  import Transform                  # — maps to `rr.Transform3D` (translation + quaternion).
from dimos.msgs.nav_msgs.Odometry                        import Odometry                   # — maps to `rr.Transform3D` from pose.
from dimos.msgs.nav_msgs.OccupancyGrid                   import OccupancyGrid              # — maps to `rr.SegmentationImage` from grid.
from dimos.msgs.geometry_msgs.Twist                      import Twist                      # — maps to `rr.Arrows3D` (linear & angular vectors).
from dimos.msgs.geometry_msgs.TwistStamped               import TwistStamped               # — maps to `rr.Arrows3D` (stamped twist).
from dimos.msgs.geometry_msgs.TwistWithCovariance        import TwistWithCovariance        # — maps to `rr.Arrows3D` (covariance ignored).
from dimos.msgs.geometry_msgs.TwistWithCovarianceStamped import TwistWithCovarianceStamped # — maps to `rr.Arrows3D` (covariance ignored).
from dimos.msgs.geometry_msgs.Vector3                    import Vector3                    # — maps to `rr.Vectors3D`.
from dimos.msgs.geometry_msgs.Quaternion                 import Quaternion                 # — maps to `rr.Transform3D` (rotation only).
from dimos.msgs.geometry_msgs.PoseWithCovarianceStamped  import PoseWithCovarianceStamped  # — duplicate of 6; included for completeness.
from dimos.msgs.sensor_msgs.Joy                          import Joy                        # — coarse mapping to `rr.AnyValues` (axes.buttons telemetry).
from dimos.msgs.sensor_msgs.CameraInfo                   import CameraInfo                 # — mapped to `rr.TextDocument` summary of intrinsics.
from dimos.msgs.foxglove_msgs.Color                      import Color                      # — maps to `rr.Color`.

def make_pose(x: float, y: float, z: float, frame: str = "map", ts: float | None = None) -> PoseStamped:
    """Helper to build a PoseStamped with identity orientation."""
    return PoseStamped(
        ts=ts or time.time(),
        frame_id=frame,
        position=[x, y, z],
        orientation=[0.0, 0.0, 0.0, 1.0],
    )


def build_sample_paths() -> list[Path]:
    """Create a few example Path instances with different frames and shapes."""
    square = [
        make_pose(0.0, 0.0, 0.0),
        make_pose(1.0, 0.0, 0.0),
        make_pose(1.0, 1.0, 0.0),
        make_pose(0.0, 1.0, 0.0),
    ]
    line = [make_pose(x * 0.5, 0.2 * x, 0.0, frame="odom") for x in range(6)]
    zigzag = [
        make_pose(x, (-1) ** x * 0.5, 0.1 * x, frame="base_link") for x in range(5)
    ]
    return [
        Path(frame_id="map", poses=square),
        Path(frame_id="odom", poses=line),
        Path(frame_id="base_link", poses=zigzag),
    ]


def build_sample_pointclouds() -> list[PointCloud2]:
    """Create a couple of small PointCloud2 examples."""
    # Grid of points on XY plane
    xs, ys = np.meshgrid(np.linspace(-1, 1, 5), np.linspace(-1, 1, 5))
    grid_points = np.column_stack((xs.flatten(), ys.flatten(), np.zeros_like(xs).flatten()))

    # Random cluster around (1, 1, 0.5)
    rng = np.random.default_rng(seed=42)
    cluster_points = rng.normal(loc=(1.0, 1.0, 0.5), scale=0.1, size=(50, 3))

    return [
        PointCloud2.from_numpy(grid_points, frame_id="map"),
        PointCloud2.from_numpy(cluster_points, frame_id="camera_link"),
    ]


def main() -> None:
    paths = build_sample_paths()
    clouds = build_sample_pointclouds()

    print("=== Paths ===")
    for idx, path in enumerate(paths, start=1):
        print(f"[{idx}] {path}")
        for pose_idx, pose in enumerate(path.poses[:3], start=1):
            print(f"    pose{pose_idx}: {pose}")

    print("\n=== PointCloud2 ===")
    for idx, cloud in enumerate(clouds, start=1):
        print(f"[{idx}] {cloud}")
        pts = cloud.as_numpy()
        if len(pts):
            print(f"    sample points: {pts[:3]}")

def create_example_msgs():
    # Create a few example Path instances with different frames and shapes.
    for each in build_sample_paths():
        yield each

    # Create a couple of small PointCloud2 examples.
    for each in build_sample_pointclouds():
        yield each

if __name__ == "__main__":
    main()
