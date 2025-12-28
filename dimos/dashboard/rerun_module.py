# Copyright 2025 Dimensional Inc.
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

"""Centralized Rerun visualization module.

This module provides a single point of Rerun initialization and logging,
eliminating race conditions from multiple processes trying to start
the Rerun gRPC server.

Usage in blueprints:
    from dimos.dashboard.rerun_module import rerun_module

    blueprint = autoconnect(
        go2_connection(),
        rerun_module(),
        ...
    )

Input stream names are designed to match existing module outputs:
    - color_image: matches GO2Connection.color_image
    - odom: matches GO2Connection.odom
    - tf: matches GO2Connection.tf (TF transforms for frame connections)
    - global_map: matches VoxelGridMapper.global_map
    - path: matches ReplanningAStarPlanner.path
    - global_costmap: matches CostMapper.global_costmap
    - camera_info: matches GO2Connection.camera_info (for frustum)
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path as FilePath
from typing import TYPE_CHECKING

from reactivex.disposable import Disposable

from dimos.core import In, Module, rpc
from dimos.core.module import ModuleConfig
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs import PoseStamped
    from dimos.msgs.nav_msgs import OccupancyGrid, Path
    from dimos.msgs.sensor_msgs import CameraInfo, Image, PointCloud2
    from dimos.msgs.tf2_msgs import TFMessage

logger = setup_logger()

# Known robot URDF paths (relative to dimos package root)
_DIMOS_ROOT = FilePath(__file__).parent.parent
ROBOT_URDFS = {
    "unitree_go2": _DIMOS_ROOT / "robot/unitree/go2/go2.urdf",
    "unitree_g1": _DIMOS_ROOT / "robot/unitree/g1/g1.urdf",
}


@dataclass
class RerunModuleConfig(ModuleConfig):
    """Configuration for the RerunModule."""

    app_id: str = "dimos"
    web_port: int = 9090
    open_browser: bool = False
    serve_web: bool = True

    # Logging paths for each stream type
    color_image_path: str = "world/robot/camera/rgb"
    camera_info_path: str = "world/robot/camera"
    odom_path: str = "world/robot"
    global_map_path: str = "world/map"
    path_path: str = "world/nav/path"
    global_costmap_path: str = "world/nav/costmap"

    # Visual configuration
    show_grid: bool = True
    grid_size: float = 20.0
    grid_divisions: int = 20
    grid_color: tuple[int, int, int, int] = (100, 150, 200, 80)

    show_robot_axes: bool = True
    robot_axes_length: float = 0.5

    # Robot URDF path - if empty, will try to look up from robot_model global config
    # Can be absolute path or robot model key (e.g., "unitree_go2")
    urdf_path: str = ""

    # URDF root link name (for connecting transforms)
    urdf_root_link: str = "base_link"

    # Camera frame name (for connecting to robot)
    camera_frame: str = "camera_link"

    # Camera transform relative to robot body (for frustum visualization)
    camera_offset: tuple[float, float, float] = (0.3, 0.0, 0.15)  # front of Go2


class RerunModule(Module):
    """Centralized Rerun visualization module.

    This module is the ONLY place in the system that initializes Rerun
    and manages the gRPC server. All visualization data flows to this
    module via transports (LCM/SHM) and gets logged to Rerun here.

    This architecture eliminates race conditions from multiple Dask workers
    trying to start the Rerun server simultaneously.

    Input stream names match existing module outputs for autoconnect:
        - color_image: Image from GO2Connection
        - camera_info: CameraInfo from GO2Connection (for frustum)
        - odom: PoseStamped from GO2Connection (for axes)
        - global_map: PointCloud2 from VoxelGridMapper
        - path: Path from ReplanningAStarPlanner
        - global_costmap: OccupancyGrid from CostMapper

    TF transforms are received via the Module.tf service (LCMTF on /tf topic),
    NOT as an In stream. This allows receiving TF published by GO2Connection
    via self.tf.publish() which uses the standard LCM TF pubsub.
    """

    default_config = RerunModuleConfig
    config: RerunModuleConfig

    # Input streams for visualization data
    # Names match existing module outputs for autoconnect compatibility
    color_image: In[Image] = None  # type: ignore[assignment]
    camera_info: In[CameraInfo] = None  # type: ignore[assignment]
    odom: In[PoseStamped] = None  # type: ignore[assignment]
    # NOTE: TF is NOT an In stream - we use the Module.tf service (LCMTF) instead
    global_map: In[PointCloud2] = None  # type: ignore[assignment]
    path: In[Path] = None  # type: ignore[assignment]
    global_costmap: In[OccupancyGrid] = None  # type: ignore[assignment]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._rr_initialized = False
        self._camera_info_logged = False
        self._urdf_logged = False
        self._odom_frame_logged = False
        self._camera_optical_frame_id: str | None = None

    def _resolve_urdf_path(self) -> FilePath | None:
        """Resolve the URDF path from config or global robot_model."""
        # If explicit path provided
        if self.config.urdf_path:
            path = FilePath(self.config.urdf_path)
            if path.exists():
                return path
            # Maybe it's a robot model key
            if self.config.urdf_path in ROBOT_URDFS:
                return ROBOT_URDFS[self.config.urdf_path]
            logger.warning(f"URDF path not found: {self.config.urdf_path}")
            return None

        # Try to get from global config robot_model
        robot_model = getattr(self.config, "robot_model", None)
        if robot_model and robot_model in ROBOT_URDFS:
            return ROBOT_URDFS[robot_model]

        return None

    def _init_rerun(self) -> None:
        """Initialize Rerun server (called once in start())."""
        if self._rr_initialized:
            return

        import rerun as rr
        import rerun.blueprint as rrb

        logger.info(f"Initializing Rerun with app_id='{self.config.app_id}'")

        # Create blueprint for nice panel layout
        blueprint = rrb.Blueprint(
            rrb.Horizontal(
                rrb.Spatial3DView(
                    name="3D View",
                    origin="world",
                ),
                rrb.Vertical(
                    rrb.Spatial2DView(
                        name="Camera",
                        origin=self.config.color_image_path,
                    ),
                    rrb.Spatial2DView(
                        name="Costmap",
                        origin=self.config.global_costmap_path,
                    ),
                    row_shares=[2, 1],
                ),
                column_shares=[3, 1],
            ),
            rrb.TimePanel(state="collapsed"),
        )

        rr.init(self.config.app_id, default_blueprint=blueprint)

        # Start gRPC server
        server_uri = rr.serve_grpc()
        logger.info(f"Rerun gRPC server started at {server_uri}")

        # Optionally serve web viewer
        if self.config.serve_web:
            rr.serve_web_viewer(
                connect_to=server_uri,
                open_browser=self.config.open_browser,
                web_port=self.config.web_port,
            )
            logger.info(f"Rerun web viewer serving on port {self.config.web_port}")

        # Set up world coordinate system (Z-up, right-handed)
        # Register "world" as a named frame so TF transforms can connect to it
        rr.log(
            "world",
            rr.ViewCoordinates.RIGHT_HAND_Z_UP,
            rr.CoordinateFrame("world"),
            static=True,
        )

        # Bridge the named frame "world" to the implicit frame hierarchy "tf#/world"
        # This allows child entities (world/grid, world/map) to connect through
        # their implicit parent frame to our named "world" frame
        rr.log(
            "world",
            rr.Transform3D(
                parent_frame="world",
                child_frame="tf#/world",
            ),
            static=True,
        )

        # Log ground grid
        if self.config.show_grid:
            self._log_ground_grid()

        # Log robot URDF (static, will move with transform at odom_path)
        self._log_robot_urdf()

        self._rr_initialized = True

    def _log_robot_urdf(self) -> None:
        """Log the robot URDF model using Rerun's built-in URDF loader."""
        import rerun as rr

        urdf_path = self._resolve_urdf_path()
        if urdf_path is None:
            logger.info("No URDF path configured, skipping robot model visualization")
            return

        if not urdf_path.exists():
            logger.warning(f"URDF file not found: {urdf_path}")
            return

        try:
            # Use Rerun's built-in URDF data-loader with entity_path_prefix
            # This places the URDF at world/robot/{robot_name}/...
            # The URDF loader will create CoordinateFrame for each link
            rr.log_file_from_path(
                str(urdf_path),
                entity_path_prefix="world/robot",
                static=True,
            )
            self._urdf_logged = True
            logger.info(f"Logged robot URDF from {urdf_path} at world/robot/")
        except Exception as e:
            logger.warning(f"Failed to log URDF: {e}")

    def _log_ground_grid(self) -> None:
        """Log a ground plane grid for spatial reference."""
        import rerun as rr

        size = self.config.grid_size
        divisions = self.config.grid_divisions
        half = size / 2
        step = size / divisions

        lines = []
        for i in range(divisions + 1):
            pos = -half + i * step
            # Lines parallel to Y axis
            lines.append([[pos, -half, 0], [pos, half, 0]])
            # Lines parallel to X axis
            lines.append([[-half, pos, 0], [half, pos, 0]])

        rr.log(
            "world/grid",
            rr.LineStrips3D(
                lines,
                colors=[self.config.grid_color],
                radii=0.01,
            ),
            static=True,
        )
        logger.info(f"Logged ground grid ({size}m x {size}m)")

    @rpc
    def start(self) -> None:
        """Start the module and initialize Rerun."""
        super().start()

        # Initialize Rerun - this is the ONLY place in the codebase
        self._init_rerun()

        # Set up subscriptions for each input stream
        self._setup_subscriptions()

    def _setup_subscriptions(self) -> None:
        """Subscribe to input streams and log to Rerun."""
        import rerun as rr

        # Color image stream (from GO2Connection.color_image)
        # Image must be logged at a CHILD path of the Pinhole entity
        if self.color_image is not None:
            image_path = self.config.color_image_path  # world/robot/camera/rgb

            def on_image(img: Image) -> None:
                try:
                    rr.log(image_path, img.to_rerun())
                except Exception as e:
                    logger.warning(f"Failed to log image to rerun: {e}")

            self._disposables.add(Disposable(self.color_image.subscribe(on_image)))
            logger.info(f"Subscribed to color_image -> {image_path}")

        # Camera info stream (from GO2Connection.camera_info) - for frustum
        # Pinhole is logged at camera path; image is at child path (above)
        if self.camera_info is not None:
            camera_path = self.config.camera_info_path  # world/robot/camera

            def on_camera_info(info: CameraInfo) -> None:
                # Only log once as static since intrinsics don't change
                if self._camera_info_logged:
                    return
                try:
                    # Store frame_id for TF lookup (usually "camera_optical")
                    self._camera_optical_frame_id = info.frame_id

                    # Extract intrinsics from K matrix
                    # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
                    k = info.K
                    if len(k) >= 9:
                        fx, fy = k[0], k[4]
                        cx, cy = k[2], k[5]

                        # Log Pinhole only (Transform3D will come from TF callback)
                        rr.log(
                            camera_path,
                            rr.Pinhole(
                                focal_length=[fx, fy],
                                principal_point=[cx, cy],
                                width=info.width,
                                height=info.height,
                                image_plane_distance=0.5,
                            ),
                            static=True,
                        )
                        self._camera_info_logged = True
                        logger.info(
                            f"Logged camera pinhole: {info.width}x{info.height}, "
                            f"fx={fx:.1f}, fy={fy:.1f}, frame={info.frame_id}"
                        )
                except Exception as e:
                    logger.warning(f"Failed to log camera_info to rerun: {e}")

            self._disposables.add(Disposable(self.camera_info.subscribe(on_camera_info)))
            logger.info(f"Subscribed to camera_info -> {camera_path}")

        # Odom stream (from GO2Connection.odom) - just for axes visualization
        # The actual transform (world -> base_link) comes from TF
        if self.odom is not None:
            odom_path = self.config.odom_path
            show_axes = self.config.show_robot_axes
            axes_length = self.config.robot_axes_length
            urdf_root_link = self.config.urdf_root_link

            # Log the CoordinateFrame once (static) to associate entity with base_link
            def on_first_odom(pose: PoseStamped) -> None:
                # Associate this entity with the base_link named frame
                rr.log(
                    odom_path,
                    rr.CoordinateFrame(urdf_root_link),
                    static=True,
                )
                # Unsubscribe after first message
                self._odom_frame_logged = True

            def on_odom(pose: PoseStamped) -> None:
                try:
                    # Log axes at the robot position (via base_link frame)
                    if show_axes:
                        rr.log(odom_path, rr.TransformAxes3D(axes_length))
                except Exception as e:
                    logger.warning(f"Failed to log odom axes to rerun: {e}")

            # Log coordinate frame once, then just axes
            self._odom_frame_logged = False

            def on_odom_wrapper(pose: PoseStamped) -> None:
                if not self._odom_frame_logged:
                    on_first_odom(pose)
                on_odom(pose)

            self._disposables.add(Disposable(self.odom.subscribe(on_odom_wrapper)))
            logger.info(f"Subscribed to odom -> {self.config.odom_path}")

        # TF service subscription (LCMTF on /tf topic) - drives entity transforms
        # NOTE: We use Module.tf (the TF service) NOT an In[TFMessage] stream
        # GO2Connection publishes via self.tf.publish() which goes to LCM /tf
        #
        # Key insight: We must log Transform3D to entity paths (world/robot, world/robot/camera)
        # to make them move. Named frames (CoordinateFrame) alone don't animate entities.
        try:
            tf_topic = getattr(self.tf.config, "topic", None)
            if tf_topic is not None:
                # Capture config values for closure
                base_link = self.config.urdf_root_link  # "base_link"
                odom_path = self.config.odom_path  # "world/robot"
                cam_path = self.config.camera_info_path  # "world/robot/camera"
                cam_link = self.config.camera_frame  # "camera_link"

                def on_tf_msg(tf_msg: TFMessage, _topic: object) -> None:
                    try:
                        # Build quick lookup for transforms in this message
                        edges: dict[tuple[str, str], object] = {
                            (t.frame_id, t.child_frame_id): t for t in tf_msg.transforms
                        }

                        cam_opt = self._camera_optical_frame_id or "camera_optical"

                        # 1) Move the whole robot subtree: world -> base_link
                        t_world_base = (
                            edges.get(("world", base_link))
                            or edges.get(("map", base_link))
                            or edges.get(("odom", base_link))
                        )
                        if t_world_base is not None:
                            rr.log(
                                odom_path,
                                rr.Transform3D(
                                    translation=[
                                        t_world_base.translation.x,
                                        t_world_base.translation.y,
                                        t_world_base.translation.z,
                                    ],
                                    rotation=rr.Quaternion(
                                        xyzw=[
                                            t_world_base.rotation.x,
                                            t_world_base.rotation.y,
                                            t_world_base.rotation.z,
                                            t_world_base.rotation.w,
                                        ]
                                    ),
                                ),
                            )

                        # 2) Move/orient the camera frustum: base_link -> camera_optical
                        t_base_cam = edges.get((base_link, cam_opt))
                        if t_base_cam is None:
                            # Compose base_link -> camera_link -> camera_optical
                            t1 = edges.get((base_link, cam_link))
                            t2 = edges.get((cam_link, cam_opt))
                            if t1 is not None and t2 is not None:
                                t_base_cam = t1 + t2  # Transform composition

                        if t_base_cam is not None:
                            rr.log(
                                cam_path,
                                rr.Transform3D(
                                    translation=[
                                        t_base_cam.translation.x,
                                        t_base_cam.translation.y,
                                        t_base_cam.translation.z,
                                    ],
                                    rotation=rr.Quaternion(
                                        xyzw=[
                                            t_base_cam.rotation.x,
                                            t_base_cam.rotation.y,
                                            t_base_cam.rotation.z,
                                            t_base_cam.rotation.w,
                                        ]
                                    ),
                                ),
                            )

                        # Also log raw TF for debugging in Rerun's "tf" entity
                        for transform in tf_msg.transforms:
                            rr.log("tf", transform.to_rerun())

                    except Exception as e:
                        logger.warning(f"Failed to process tf for rerun: {e}")

                unsub = self.tf.pubsub.subscribe(tf_topic, on_tf_msg)
                self._disposables.add(Disposable(unsub))
                logger.info(f"Subscribed to TF service {tf_topic} -> rerun transforms")
        except Exception as e:
            logger.warning(f"Failed to subscribe to TF service for rerun: {e}")

        # Global map stream (from VoxelGridMapper.global_map)
        if self.global_map is not None:

            def on_global_map(pc: PointCloud2) -> None:
                try:
                    rr.log(self.config.global_map_path, pc.to_rerun())
                except Exception as e:
                    logger.warning(f"Failed to log global_map to rerun: {e}")

            self._disposables.add(Disposable(self.global_map.subscribe(on_global_map)))
            logger.info(f"Subscribed to global_map -> {self.config.global_map_path}")

        # Path stream (from ReplanningAStarPlanner.path)
        if self.path is not None:

            def on_path(nav_path: Path) -> None:
                try:
                    rr.log(self.config.path_path, nav_path.to_rerun())
                except Exception as e:
                    logger.warning(f"Failed to log path to rerun: {e}")

            self._disposables.add(Disposable(self.path.subscribe(on_path)))
            logger.info(f"Subscribed to path -> {self.config.path_path}")

        # Global costmap stream (from CostMapper.global_costmap)
        if self.global_costmap is not None:

            def on_costmap(grid: OccupancyGrid) -> None:
                try:
                    rr.log(self.config.global_costmap_path, grid.to_rerun())
                except Exception as e:
                    logger.warning(f"Failed to log global_costmap to rerun: {e}")

            self._disposables.add(Disposable(self.global_costmap.subscribe(on_costmap)))
            logger.info(f"Subscribed to global_costmap -> {self.config.global_costmap_path}")

    @rpc
    def stop(self) -> None:
        """Stop the module."""
        logger.info("Stopping RerunModule")
        super().stop()


# Blueprint helper function
def rerun_module(
    app_id: str = "dimos",
    web_port: int = 9090,
    open_browser: bool = False,
    serve_web: bool = True,
    urdf_path: str = "",
    urdf_root_link: str = "base_link",
    camera_frame: str = "camera_link",
    camera_offset: tuple[float, float, float] = (0.3, 0.0, 0.15),
    **kwargs: object,
) -> RerunModule:
    """Create a RerunModule blueprint.

    Args:
        app_id: Application identifier for Rerun
        web_port: Port for the Rerun web viewer
        open_browser: Whether to open browser automatically
        serve_web: Whether to serve the web viewer
        urdf_path: Path to robot URDF, or robot model key (e.g., "unitree_go2")
        urdf_root_link: Name of the URDF root link frame (for transform connection)
        camera_frame: Name of the camera frame (for image association)
        camera_offset: Camera position relative to robot body (x, y, z)
        **kwargs: Additional configuration options

    Returns:
        RerunModule blueprint
    """
    return RerunModule.blueprint(
        app_id=app_id,
        web_port=web_port,
        open_browser=open_browser,
        serve_web=serve_web,
        urdf_path=urdf_path,
        urdf_root_link=urdf_root_link,
        camera_frame=camera_frame,
        camera_offset=camera_offset,
        **kwargs,
    )


__all__ = ["RerunModule", "RerunModuleConfig", "rerun_module"]
