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
    - global_map: matches VoxelGridMapper.global_map
    - path: matches ReplanningAStarPlanner.path
    - global_costmap: matches CostMapper.global_costmap
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from reactivex.disposable import Disposable

from dimos.core import In, Module, rpc
from dimos.core.module import ModuleConfig
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs import PoseStamped
    from dimos.msgs.nav_msgs import OccupancyGrid, Path
    from dimos.msgs.sensor_msgs import Image, PointCloud2

logger = setup_logger()


@dataclass
class RerunModuleConfig(ModuleConfig):
    """Configuration for the RerunModule."""

    app_id: str = "dimos"
    web_port: int = 9090
    open_browser: bool = False
    serve_web: bool = True
    # Logging paths for each stream type
    color_image_path: str = "robot/camera/rgb"
    odom_path: str = "robot/odom"
    global_map_path: str = "world/map"
    path_path: str = "world/nav/path"
    global_costmap_path: str = "world/nav/costmap"


class RerunModule(Module):
    """Centralized Rerun visualization module.

    This module is the ONLY place in the system that initializes Rerun
    and manages the gRPC server. All visualization data flows to this
    module via transports (LCM/SHM) and gets logged to Rerun here.

    This architecture eliminates race conditions from multiple Dask workers
    trying to start the Rerun server simultaneously.

    Input stream names match existing module outputs for autoconnect:
        - color_image: Image from GO2Connection
        - odom: PoseStamped from GO2Connection
        - global_map: PointCloud2 from VoxelGridMapper
        - path: Path from ReplanningAStarPlanner
        - global_costmap: OccupancyGrid from CostMapper
    """

    default_config = RerunModuleConfig
    config: RerunModuleConfig

    # Input streams for visualization data
    # Names match existing module outputs for autoconnect compatibility
    color_image: In[Image] = None  # type: ignore[assignment]
    odom: In[PoseStamped] = None  # type: ignore[assignment]
    global_map: In[PointCloud2] = None  # type: ignore[assignment]
    path: In[Path] = None  # type: ignore[assignment]
    global_costmap: In[OccupancyGrid] = None  # type: ignore[assignment]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._rr_initialized = False

    def _init_rerun(self) -> None:
        """Initialize Rerun server (called once in start())."""
        if self._rr_initialized:
            return

        import rerun as rr

        logger.info(f"Initializing Rerun with app_id='{self.config.app_id}'")

        rr.init(self.config.app_id)

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

        self._rr_initialized = True

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
        if self.color_image is not None:

            def on_image(img: Image) -> None:
                try:
                    rr.log(self.config.color_image_path, img.to_rerun())
                except Exception as e:
                    logger.warning(f"Failed to log image to rerun: {e}")

            self._disposables.add(Disposable(self.color_image.subscribe(on_image)))
            logger.info(f"Subscribed to color_image -> {self.config.color_image_path}")

        # Odom stream (from GO2Connection.odom)
        if self.odom is not None:

            def on_odom(pose: PoseStamped) -> None:
                try:
                    rr.log(self.config.odom_path, pose.to_rerun())
                except Exception as e:
                    logger.warning(f"Failed to log odom to rerun: {e}")

            self._disposables.add(Disposable(self.odom.subscribe(on_odom)))
            logger.info(f"Subscribed to odom -> {self.config.odom_path}")

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
    **kwargs: object,
) -> RerunModule:
    """Create a RerunModule blueprint.

    Args:
        app_id: Application identifier for Rerun
        web_port: Port for the Rerun web viewer
        open_browser: Whether to open browser automatically
        serve_web: Whether to serve the web viewer
        **kwargs: Additional configuration options

    Returns:
        RerunModule blueprint
    """
    return RerunModule.blueprint(
        app_id=app_id,
        web_port=web_port,
        open_browser=open_browser,
        serve_web=serve_web,
        **kwargs,
    )


__all__ = ["RerunModule", "RerunModuleConfig", "rerun_module"]
