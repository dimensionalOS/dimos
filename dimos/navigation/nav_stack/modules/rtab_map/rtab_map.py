# Copyright 2026 Dimensional Inc.
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

from __future__ import annotations

from pathlib import Path
import time

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.nav_stack.frames import FRAME_BODY, FRAME_MAP, FRAME_ODOM
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class RtabMapConfig(NativeModuleConfig):
    cwd: str | None = str(Path(__file__).resolve().parent / "cpp")
    executable: str = "result/bin/rtab_map"
    build_command: str | None = "nix build .#default --no-write-lock-file"

    # Frame names.
    world_frame: str = FRAME_MAP
    local_frame: str = FRAME_ODOM
    body_frame: str = FRAME_BODY

    # OctoMap / Grid defaults (the six required by the locked spec).
    grid_3d: bool = True
    grid_ray_tracing: bool = True
    # 0 = laser scan, 1 = depth, 2 = both. We feed only laser scans;
    # default must be 0 or rtabmap's internal LocalGridMaker skips every
    # signature and the OctoMap stays empty. Supersedes the
    # rtabmap-pre-0.20.15 `grid_from_depth: bool` knob.
    grid_sensor: int = 0
    grid_cell_size: float = 0.1
    grid_max_ground_angle: float = 45.0
    grid_ground_is_obstacle: bool = False
    grid_flat_obstacle_detected: bool = True

    # Additional rtabmap Grid/* params surfaced so callers can tune
    # ground/obstacle segmentation, max range, and keyframe admission.
    # Defaults mirror the C++ binary's defaults.
    grid_normals_segmentation: bool = False
    grid_max_obstacle_height: float = 2.0
    grid_max_ground_height: float = 0.05
    grid_range_max: float = 8.0

    # OctoMap log-odds knobs. rtabmap defaults make the OctoMap behave
    # like a long-term static map (ClampingMax=0.971 → cells saturate
    # at +3.5 log-odds, ~9 empty-cell observations needed to flip back
    # to free). For a "treat everything as dynamic" feel — where a chair
    # that moves out of view actually disappears from the map within
    # a second or two — we lower ClampingMax. Trade-off: walls flicker
    # if not constantly observed.
    #   ProbHit=0.7 → +0.85 log-odds per hit
    #   ProbMiss=0.4 → -0.41 log-odds per miss
    #   ProbClampingMax=0.75 → saturation at +1.1 log-odds → ~3 misses
    #     to clear a saturated cell (~0.6 s at 5 Hz octomap update rate)
    octomap_prob_hit: float = 0.7
    octomap_prob_miss: float = 0.4
    octomap_prob_clamping_max: float = 0.75
    octomap_prob_clamping_min: float = 0.12
    octomap_occupancy_thr: float = 0.5
    # Project the cloud into the world's gravity-aligned frame before
    # ground/obstacle segmentation. rtabmap defaults this to false (height
    # threshold applied in the sensor's local frame, where z=0 is the
    # sensor mount). For a tall robot like the G1 — whose pose.z is ~1.2 m
    # above ground — that misclassifies the floor and anything shorter
    # than the sensor (chairs, tables) as "ground", which then never
    # appears in the obstacle-only OctoMap output. Setting this to true
    # makes segmentation happen in world-z so MaxGroundHeight=0.05 is
    # measured from the actual floor. **Assumes the pose passed to rtabmap
    # is gravity-aligned and z=0 of the world frame is at floor level.**
    grid_map_frame_projection: bool = True

    # Keyframe admission rate. With rgbd_linear_update=0 (no motion gate),
    # rtab.process admits every scan as a keyframe; this gate bounds the
    # keyframe rate at the main-loop level (rtabmap's own
    # Rtabmap/DetectionRate is consumed by RtabmapThread, not by direct
    # process() calls). 0.5 s = 2 Hz keyframes — enough for dynamic
    # clearing (a ghost trail clears in ~3 misses ≈ 1.5 s with the default
    # ClampingMax=0.75) without paying ICP/Memory cost at 10 Hz scan rate.
    # Set to 0 to process every scan (synthetic tests).
    rtabmap_process_period: float = 0.5
    # Motion-gate keyframe admission. Default 0 = let the time gate above
    # do the rate limiting. A non-zero value gates *additionally* on
    # spatial motion, which is wrong for a stationary robot watching a
    # dynamic scene (no motion → no keyframes → no dynamic clearing).
    rgbd_linear_update: float = 0.0
    rgbd_angular_update: float = 0.0
    # Keep signatures (and their local grids) around after rtabmap thinks
    # they're disconnected. The OctoMap reads grids out of Memory's
    # signatures, so they must stay accessible across loop-closure-driven
    # graph reshuffles. Default true; only set false if you understand
    # the memory-lifecycle implications.
    mem_not_linked_nodes_kept: bool = True
    # If true (default), the scan callback drops any older queued scans
    # when a new one arrives — keep only the latest. Protects against the
    # buffer growing unboundedly when per-frame processing is slower than
    # the input scan rate. Set False to preserve every scan (slow but
    # lossless; pairs with rgbd_linear_update=0 for unit-test scenarios).
    drop_stale_scans: bool = True
    # rtabmap's one-to-many proximity detection neighbor count; enables
    # geometric (scan-based) loop closure in lidar-only mode.
    rgbd_proximity_path_max_neighbors: int = 10
    # Spatial proximity detection — search the local pose-graph for
    # candidates within rgbd_local_radius of the current keyframe. Must
    # be on for lidar-only loop closure (visual BoW is unavailable).
    rgbd_proximity_by_space: bool = True
    # Spatial search radius (m) for proximity candidate selection.
    # rtabmap default 10m; KITTI-360-style outdoor scenes use ≥10, tight
    # indoor scenes use 2-3.
    rgbd_local_radius: float = 10.0
    # Max pose-graph depth for proximity candidate selection. rtabmap's
    # default is 50, which silently kills loop closure on long
    # trajectories: any candidate that's >50 keyframes ago in the graph
    # gets rejected even when it's spatially right next to the current
    # pose. For a 500-scan KITTI-360 run we saw 16 candidates within 10m
    # of the current pose, all but one with graph depth 200+ — that one
    # qualifying because it happened to be the only loop candidate close
    # enough. 0 disables the depth filter entirely (the "find loops
    # anywhere in the graph" mode), at the cost of a per-process search
    # that scales linearly with graph size. For benchmark / long-run
    # contexts this is the right default.
    rgbd_proximity_max_graph_depth: int = 0
    # ICP correspondence distance threshold (m). rtabmap default 0.05m is
    # very tight for outdoor LiDAR; 0.5m is forgiving and gives proximity
    # ICP a chance at slightly-misaligned candidates.
    icp_max_correspondence_distance: float = 0.5

    # Verbose per-frame stderr diagnostics from the C++ binary. Useful for
    # debugging things like "global_map_slam isn't updating": surfaces
    # whether scans are queueing, whether rtabmap admits each frame as a
    # keyframe, what LocalGridMaker produces, when global_map / octomap
    # publish events fire and to which topics.
    debug: bool = False

    # Publishing cadence. Each publish triggers
    # ``OctoMap::update(octomap_poses)`` first, which integrates every
    # scan accumulated since the last update — so this also controls how
    # often new scans get baked into the global map. Faster updates =
    # snappier dynamic-obstacle clearing but more octree write cost.
    octomap_publish_period: float = 0.5
    global_map_publish_period: float = 1.0
    global_map_voxel_size: float = 0.15

    # Input handling.
    # Input scans arrive in the world (map) frame; the binary undoes the
    # current odom transform so rtabmap sees body-frame scans.
    unregister_input: bool = True
    # Drop scans whose timestamp differs from the latest odometry's by more
    # than this many seconds — guards against rtabmap getting stale/fresh
    # mismatched pairs.
    scan_odom_max_dt: float = 0.2

    # RGB camera support. When `color_image_enabled=True` AND camera_fx>0,
    # the binary subscribes to the `color_image` stream and attaches the
    # latest RGB frame (with a CameraModel built from these intrinsics) to
    # each SensorData passed to rtab.process. rtabmap then stores the image
    # on the signature for visualization and — when feature extraction is
    # configured (Kp/DetectorStrategy != -1) — uses it for visual
    # bag-of-words loop closure on top of the lidar-only ICP path.
    #
    # Default off: existing lidar-only setups keep working unchanged.
    color_image_enabled: bool = False
    # Pinhole intrinsics in pixels. fx=0 (the default) is interpreted as
    # "no intrinsics provided"; the C++ binary will then ignore RGB frames
    # even if the stream is connected.
    camera_fx: float = 0.0
    camera_fy: float = 0.0
    camera_cx: float = 0.0
    camera_cy: float = 0.0
    camera_image_width: int = 0
    camera_image_height: int = 0
    # Rigid transform from body frame to camera optical frame
    # (quat is xyzw, defaults to identity = camera coincides with body).
    camera_local_x: float = 0.0
    camera_local_y: float = 0.0
    camera_local_z: float = 0.0
    camera_local_qx: float = 0.0
    camera_local_qy: float = 0.0
    camera_local_qz: float = 0.0
    camera_local_qw: float = 1.0
    # Drop RGB frames whose timestamp differs from the scan's by more than
    # this many seconds. RGB and lidar usually run at different rates;
    # this widens to whatever your camera framerate + jitter allows.
    rgb_max_dt: float = 0.2


class RtabMap(NativeModule):
    """RtabMap NativeModule — librtabmap behind an LCM wrapper.

    Plays the same role in the nav stack as :class:`PGO`: consumes
    """

    config: RtabMapConfig

    registered_scan: In[PointCloud2]
    odometry: In[Odometry]
    # Optional RGB feed. If left unconnected, the binary stays in
    # lidar-only mode. Connecting an Image source and setting
    # `color_image_enabled=True` (plus camera intrinsics) makes rtabmap
    # store the RGB on each keyframe's signature and — when feature
    # extraction is enabled — use it for visual loop closure on top of
    # the lidar/ICP path.
    color_image: In[Image]

    corrected_odometry: Out[Odometry]
    global_map: Out[PointCloud2]
    rtab_tf: Out[Odometry]
    octomap: Out[PointCloud2]
    projected_2d_grid: Out[PointCloud2]
    # Pose-graph outputs — same wire contract as PGO so the existing
    # KITTI-360 benchmark + Rerun bridge consume RtabMap unchanged.
    # `pose_graph_edges` carries (start, end) PoseStamped pairs whose
    # orientation.w encodes edge type (1.0 = odom, 0.4 = loop closure).
    # `loop_closure` fires once per detected closure (empty Path payload
    # is fine — scorers count events).
    pose_graph_edges: Out[NavPath]
    loop_closure: Out[NavPath]

    @rpc
    def start(self) -> None:
        super().start()
        # Fan map->odom corrections from the C++ binary to the Python TF
        # bridge — same pattern PGO uses for pgo_tf. Subscribing through
        # the transport (not the Out's local subscribers) is required
        # because the publisher lives in the C++ subprocess.
        self.register_disposable(
            Disposable(self.rtab_tf.transport.subscribe(self._on_tf_correction, self.rtab_tf))
        )
        # Seed identity TF so downstream consumers can resolve map->body
        # before the first loop closure shifts the map frame.
        self._publish_tf(
            translation=(0.0, 0.0, 0.0),
            rotation=(0.0, 0.0, 0.0, 1.0),
            ts=time.time(),
        )
        logger.info("RtabMap native module started (C++ librtabmap + LCM)")

    def _on_tf_correction(self, msg: Odometry) -> None:
        self._publish_tf(
            translation=(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),
            rotation=(
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ),
            ts=msg.ts or time.time(),
        )

    def _publish_tf(
        self,
        translation: tuple[float, float, float],
        rotation: tuple[float, float, float, float],
        ts: float,
    ) -> None:
        self.tf.publish(
            Transform(
                frame_id=self.config.world_frame,
                child_frame_id=self.config.local_frame,
                translation=Vector3(*translation),
                rotation=Quaternion(*rotation),
                ts=ts,
            )
        )
