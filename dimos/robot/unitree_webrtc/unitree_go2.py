#!/usr/bin/env python3

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


import functools
import logging
import os
import time
import warnings
from typing import List, Optional

from dimos import core
from dimos.core import In, Module, Out, rpc
from dimos.msgs.std_msgs import Header
from dimos.msgs.geometry_msgs import PoseStamped, Transform, Twist, Vector3, Quaternion
from dimos.msgs.nav_msgs import OccupancyGrid, Path
from dimos.msgs.sensor_msgs import Image
from dimos_lcm.std_msgs import String
from dimos_lcm.sensor_msgs import CameraInfo
from dimos_lcm.vision_msgs import Detection2DArray, Detection3DArray
from dimos.perception.spatial_perception import SpatialMemory
from dimos.perception.common.utils import (
    extract_pose_from_detection3d,
    load_camera_info,
    load_camera_info_opencv,
    rectify_image,
)
from dimos.protocol import pubsub
from dimos.protocol.pubsub.lcmpubsub import LCM, Topic
from dimos.protocol.tf import TF
from dimos.robot.foxglove_bridge import FoxgloveBridge
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule
from dimos.navigation.global_planner import AstarPlanner
from dimos.navigation.local_planner.holonomic_local_planner import HolonomicLocalPlanner
from dimos.navigation.bt_navigator.navigator import BehaviorTreeNavigator, NavigatorState
from dimos.navigation.frontier_exploration import WavefrontFrontierExplorer
from dimos.robot.unitree_webrtc.connection import UnitreeWebRTCConnection
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.map import Map
from dimos.robot.unitree_webrtc.type.odometry import Odometry
from dimos.robot.unitree_webrtc.unitree_skills import MyUnitreeSkills
from dimos.robot.unitree_webrtc.depth_module import DepthModule
from dimos.skills.skills import AbstractRobotSkill, SkillLibrary
from dimos.utils.data import get_data
from dimos.utils.logging_config import setup_logger
from dimos.utils.testing import TimedSensorReplay
from dimos.utils.transform_utils import offset_distance
from dimos.perception.object_tracker import ObjectTracking
from dimos_lcm.std_msgs import Bool
from dimos.robot.robot import Robot
from dimos.types.robot_capabilities import RobotCapability


logger = setup_logger("dimos.robot.unitree_webrtc.unitree_go2", level=logging.INFO)

# Suppress verbose loggers
logging.getLogger("aiortc.codecs.h264").setLevel(logging.ERROR)
logging.getLogger("lcm_foxglove_bridge").setLevel(logging.ERROR)
logging.getLogger("websockets.server").setLevel(logging.ERROR)
logging.getLogger("FoxgloveServer").setLevel(logging.ERROR)
logging.getLogger("asyncio").setLevel(logging.ERROR)
logging.getLogger("root").setLevel(logging.WARNING)

# Suppress warnings
warnings.filterwarnings("ignore", message="coroutine.*was never awaited")
warnings.filterwarnings("ignore", message="H264Decoder.*failed to decode")


class FakeRTC:
    """Fake WebRTC connection for testing with recorded data."""

    def __init__(self, *args, **kwargs):
        get_data("unitree_office_walk")  # Preload data for testing

    def connect(self):
        pass

    def standup(self):
        print("standup suppressed")

    def liedown(self):
        print("liedown suppressed")

    @functools.cache
    def lidar_stream(self):
        print("lidar stream start")
        lidar_store = TimedSensorReplay("unitree_office_walk/lidar", autocast=LidarMessage.from_msg)
        return lidar_store.stream()

    @functools.cache
    def odom_stream(self):
        print("odom stream start")
        odom_store = TimedSensorReplay("unitree_office_walk/odom", autocast=Odometry.from_msg)
        return odom_store.stream()

    @functools.cache
    def video_stream(self):
        print("video stream start")
        video_store = TimedSensorReplay(
            "unitree_office_walk/video", autocast=lambda x: Image.from_numpy(x).to_rgb()
        )
        return video_store.stream()

    def move(self, twist: Twist, duration: float = 0.0):
        pass

    def publish_request(self, topic: str, data: dict):
        """Fake publish request for testing."""
        return {"status": "ok", "message": "Fake publish"}


class ConnectionModule(Module):
    """Module that handles robot sensor data, movement commands, and camera information."""

    movecmd: In[Twist] = None
    odom: Out[PoseStamped] = None
    lidar: Out[LidarMessage] = None
    video: Out[Image] = None
    camera_info: Out[CameraInfo] = None
    camera_pose: Out[PoseStamped] = None
    ip: str
    connection_type: str = "webrtc"

    _odom: PoseStamped = None
    _lidar: LidarMessage = None
    _last_image: Image = None

    def __init__(
        self,
        ip: str = None,
        connection_type: str = "webrtc",
        rectify_image: bool = True,
        *args,
        **kwargs,
    ):
        self.ip = ip
        self.connection_type = connection_type
        self.rectify_image = rectify_image
        self.tf = TF()
        self.connection = None

        # Load camera parameters from YAML
        base_dir = os.path.dirname(os.path.abspath(__file__))

        # Use sim camera parameters for mujoco, real camera for others
        if connection_type == "mujoco":
            camera_params_path = os.path.join(base_dir, "params", "sim_camera.yaml")
        else:
            camera_params_path = os.path.join(base_dir, "params", "front_camera_720.yaml")

        self.lcm_camera_info = load_camera_info(camera_params_path, frame_id="camera_link")

        # Load OpenCV matrices for rectification if enabled
        if rectify_image:
            self.camera_matrix, self.dist_coeffs = load_camera_info_opencv(camera_params_path)
            self.lcm_camera_info.D = [0.0] * len(
                self.lcm_camera_info.D
            )  # zero out distortion coefficients for rectification
        else:
            self.camera_matrix = None
            self.dist_coeffs = None

        Module.__init__(self, *args, **kwargs)

    @rpc
    def start(self):
        """Start the connection and subscribe to sensor streams."""
        match self.connection_type:
            case "webrtc":
                self.connection = UnitreeWebRTCConnection(self.ip)
            case "fake":
                self.connection = FakeRTC(self.ip)
            case "mujoco":
                from dimos.robot.unitree_webrtc.mujoco_connection import MujocoConnection

                self.connection = MujocoConnection()
                self.connection.start()
            case _:
                raise ValueError(f"Unknown connection type: {self.connection_type}")

        # Connect sensor streams to outputs
        self.connection.lidar_stream().subscribe(self.lidar.publish)
        self.connection.odom_stream().subscribe(self._publish_tf)
        self.connection.video_stream().subscribe(self._on_video)
        self.movecmd.subscribe(self.move)

    def _on_video(self, msg: Image):
        """Handle incoming video frames and publish synchronized camera data."""
        # Apply rectification if enabled
        if self.rectify_image:
            rectified_msg = rectify_image(msg, self.camera_matrix, self.dist_coeffs)
            self._last_image = rectified_msg
            self.video.publish(rectified_msg)
        else:
            self._last_image = msg
            self.video.publish(msg)

        # Publish camera info and pose synchronized with video
        timestamp = msg.ts if msg.ts else time.time()
        self._publish_camera_info(timestamp)
        self._publish_camera_pose(timestamp)

    def _publish_tf(self, msg):
        self._odom = msg
        self.odom.publish(msg)
        self.tf.publish(Transform.from_pose("base_link", msg))
        camera_link = Transform(
            translation=Vector3(0.3, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id="base_link",
            child_frame_id="camera_link",
            ts=time.time(),
        )
        self.tf.publish(camera_link)

    def _publish_camera_info(self, timestamp: float):
        header = Header(timestamp, "camera_link")
        self.lcm_camera_info.header = header
        self.camera_info.publish(self.lcm_camera_info)

    def _publish_camera_pose(self, timestamp: float):
        """Publish camera pose from TF lookup."""
        try:
            # Look up transform from world to camera_link
            transform = self.tf.get(
                parent_frame="world",
                child_frame="camera_link",
                time_point=timestamp,
                time_tolerance=1.0,
            )

            if transform:
                pose_msg = PoseStamped(
                    ts=timestamp,
                    frame_id="camera_link",
                    position=transform.translation,
                    orientation=transform.rotation,
                )
                self.camera_pose.publish(pose_msg)
            else:
                logger.debug("Could not find transform from world to camera_link")

        except Exception as e:
            logger.error(f"Error publishing camera pose: {e}")

    @rpc
    def get_odom(self) -> Optional[PoseStamped]:
        """Get the robot's odometry.

        Returns:
            The robot's odometry
        """
        return self._odom

    @rpc
    def move(self, twist: Twist, duration: float = 0.0):
        """Send movement command to robot."""
        self.connection.move(twist, duration)

    @rpc
    def standup(self):
        """Make the robot stand up."""
        return self.connection.standup()

    @rpc
    def liedown(self):
        """Make the robot lie down."""
        return self.connection.liedown()

    @rpc
    def publish_request(self, topic: str, data: dict):
        """Publish a request to the WebRTC connection.
        Args:
            topic: The RTC topic to publish to
            data: The data dictionary to publish
        Returns:
            The result of the publish request
        """
        return self.connection.publish_request(topic, data)


class UnitreeGo2(Robot):
    """Full Unitree Go2 robot with navigation and perception capabilities."""

    def __init__(
        self,
        ip: str,
        output_dir: str = None,
        websocket_port: int = 7779,
        skill_library: Optional[SkillLibrary] = None,
        connection_type: Optional[str] = "webrtc",
    ):
        """Initialize the robot system.

        Args:
            ip: Robot IP address (or None for fake connection)
            output_dir: Directory for saving outputs (default: assets/output)
            websocket_port: Port for web visualization
            skill_library: Skill library instance
            connection_type: webrtc, fake, or mujoco
        """
        super().__init__()
        self.ip = ip
        self.connection_type = connection_type or "webrtc"
        if ip is None and self.connection_type == "webrtc":
            self.connection_type = "fake"  # Auto-enable playback if no IP provided
        self.output_dir = output_dir or os.path.join(os.getcwd(), "assets", "output")
        self.websocket_port = websocket_port
        self.lcm = LCM()

        # Initialize skill library
        if skill_library is None:
            skill_library = MyUnitreeSkills()
        self.skill_library = skill_library

        # Set capabilities
        self.capabilities = [RobotCapability.LOCOMOTION, RobotCapability.VISION]

        self.dimos = None
        self.connection = None
        self.mapper = None
        self.global_planner = None
        self.local_planner = None
        self.navigator = None
        self.frontier_explorer = None
        self.websocket_vis = None
        self.foxglove_bridge = None
        self.spatial_memory_module = None
        self.depth_module = None
        self.object_tracker = None

        self._setup_directories()

    def _setup_directories(self):
        """Setup directories for spatial memory storage."""
        os.makedirs(self.output_dir, exist_ok=True)
        logger.info(f"Robot outputs will be saved to: {self.output_dir}")

        # Initialize memory directories
        self.memory_dir = os.path.join(self.output_dir, "memory")
        os.makedirs(self.memory_dir, exist_ok=True)

        # Initialize spatial memory properties
        self.spatial_memory_dir = os.path.join(self.memory_dir, "spatial_memory")
        self.spatial_memory_collection = "spatial_memory"
        self.db_path = os.path.join(self.spatial_memory_dir, "chromadb_data")
        self.visual_memory_path = os.path.join(self.spatial_memory_dir, "visual_memory.pkl")

        # Create spatial memory directories
        os.makedirs(self.spatial_memory_dir, exist_ok=True)
        os.makedirs(self.db_path, exist_ok=True)

    def start(self):
        """Start the robot system with all modules."""
        self.dimos = core.start(8)

        self._deploy_connection()
        self._deploy_mapping()
        self._deploy_navigation()
        self._deploy_visualization()
        self._deploy_perception()
        self._deploy_camera()

        self._start_modules()

        self.lcm.start()

        logger.info("UnitreeGo2 initialized and started")
        logger.info(f"WebSocket visualization available at http://localhost:{self.websocket_port}")

    def _deploy_connection(self):
        """Deploy and configure the connection module."""
        self.connection = self.dimos.deploy(
            ConnectionModule, self.ip, connection_type=self.connection_type
        )

        self.connection.lidar.transport = core.LCMTransport("/lidar", LidarMessage)
        self.connection.odom.transport = core.LCMTransport("/odom", PoseStamped)
        self.connection.video.transport = core.LCMTransport("/go2/color_image", Image)
        self.connection.movecmd.transport = core.LCMTransport("/cmd_vel", Twist)
        self.connection.camera_info.transport = core.LCMTransport("/go2/camera_info", CameraInfo)
        self.connection.camera_pose.transport = core.LCMTransport("/go2/camera_pose", PoseStamped)

    def _deploy_mapping(self):
        """Deploy and configure the mapping module."""
        min_height = 0.3 if self.connection_type == "mujoco" else 0.15
        self.mapper = self.dimos.deploy(
            Map, voxel_size=0.5, global_publish_interval=2.5, min_height=min_height
        )

        self.mapper.global_map.transport = core.LCMTransport("/global_map", LidarMessage)
        self.mapper.global_costmap.transport = core.LCMTransport("/global_costmap", OccupancyGrid)
        self.mapper.local_costmap.transport = core.LCMTransport("/local_costmap", OccupancyGrid)

        self.mapper.lidar.connect(self.connection.lidar)

    def _deploy_navigation(self):
        """Deploy and configure navigation modules."""
        self.global_planner = self.dimos.deploy(AstarPlanner)
        self.local_planner = self.dimos.deploy(HolonomicLocalPlanner)
        self.navigator = self.dimos.deploy(
            BehaviorTreeNavigator,
            reset_local_planner=self.local_planner.reset,
            check_goal_reached=self.local_planner.is_goal_reached,
        )
        self.frontier_explorer = self.dimos.deploy(WavefrontFrontierExplorer)

        self.navigator.goal.transport = core.LCMTransport("/navigation_goal", PoseStamped)
        self.navigator.goal_request.transport = core.LCMTransport("/goal_request", PoseStamped)
        self.navigator.goal_reached.transport = core.LCMTransport("/goal_reached", Bool)
        self.navigator.navigation_state.transport = core.LCMTransport("/navigation_state", String)
        self.navigator.global_costmap.transport = core.LCMTransport(
            "/global_costmap", OccupancyGrid
        )
        self.global_planner.path.transport = core.LCMTransport("/global_path", Path)
        self.local_planner.cmd_vel.transport = core.LCMTransport("/cmd_vel", Twist)
        self.frontier_explorer.goal_request.transport = core.LCMTransport(
            "/goal_request", PoseStamped
        )
        self.frontier_explorer.goal_reached.transport = core.LCMTransport("/goal_reached", Bool)
        self.frontier_explorer.explore_cmd.transport = core.LCMTransport("/explore_cmd", Bool)
        self.frontier_explorer.stop_explore_cmd.transport = core.LCMTransport(
            "/stop_explore_cmd", Bool
        )

        self.global_planner.target.connect(self.navigator.goal)

        self.global_planner.global_costmap.connect(self.mapper.global_costmap)
        self.global_planner.odom.connect(self.connection.odom)

        self.local_planner.path.connect(self.global_planner.path)
        self.local_planner.local_costmap.connect(self.mapper.local_costmap)
        self.local_planner.odom.connect(self.connection.odom)

        self.connection.movecmd.connect(self.local_planner.cmd_vel)

        self.navigator.odom.connect(self.connection.odom)

        self.frontier_explorer.costmap.connect(self.mapper.global_costmap)
        self.frontier_explorer.odometry.connect(self.connection.odom)

    def _deploy_visualization(self):
        """Deploy and configure visualization modules."""
        self.websocket_vis = self.dimos.deploy(WebsocketVisModule, port=self.websocket_port)
        self.websocket_vis.click_goal.transport = core.LCMTransport("/goal_request", PoseStamped)
        self.websocket_vis.explore_cmd.transport = core.LCMTransport("/explore_cmd", Bool)
        self.websocket_vis.stop_explore_cmd.transport = core.LCMTransport("/stop_explore_cmd", Bool)
        self.websocket_vis.movecmd.transport = core.LCMTransport("/cmd_vel", Twist)

        self.websocket_vis.robot_pose.connect(self.connection.odom)
        self.websocket_vis.path.connect(self.global_planner.path)
        self.websocket_vis.global_costmap.connect(self.mapper.global_costmap)

        self.foxglove_bridge = FoxgloveBridge()

    def _deploy_perception(self):
        """Deploy and configure perception modules."""
        # Deploy spatial memory
        self.spatial_memory_module = self.dimos.deploy(
            SpatialMemory,
            collection_name=self.spatial_memory_collection,
            db_path=self.db_path,
            visual_memory_path=self.visual_memory_path,
            output_dir=self.spatial_memory_dir,
        )

        self.spatial_memory_module.video.transport = core.LCMTransport("/go2/color_image", Image)
        self.spatial_memory_module.odom.transport = core.LCMTransport(
            "/go2/camera_pose", PoseStamped
        )

        logger.info("Spatial memory module deployed and connected")

        # Deploy object tracker
        self.object_tracker = self.dimos.deploy(
            ObjectTracking,
            frame_id="camera_link",
        )

        # Set up transports
        self.object_tracker.detection2darray.transport = core.LCMTransport(
            "/go2/detection2d", Detection2DArray
        )
        self.object_tracker.detection3darray.transport = core.LCMTransport(
            "/go2/detection3d", Detection3DArray
        )
        self.object_tracker.tracked_overlay.transport = core.LCMTransport(
            "/go2/tracked_overlay", Image
        )

        logger.info("Object tracker module deployed")

    def _deploy_camera(self):
        """Deploy and configure the camera module."""
        gt_depth_scale = 1.0 if self.connection_type == "mujoco" else 0.5
        self.depth_module = self.dimos.deploy(DepthModule, gt_depth_scale=gt_depth_scale)

        # Set up transports
        self.depth_module.color_image.transport = core.LCMTransport("/go2/color_image", Image)
        self.depth_module.depth_image.transport = core.LCMTransport("/go2/depth_image", Image)
        self.depth_module.camera_info.transport = core.LCMTransport("/go2/camera_info", CameraInfo)

        logger.info("Camera module deployed and connected")

        # Connect object tracker inputs after camera module is deployed
        if self.object_tracker:
            self.object_tracker.color_image.connect(self.connection.video)
            self.object_tracker.depth.connect(self.depth_module.depth_image)
            self.object_tracker.camera_info.connect(self.connection.camera_info)
            logger.info("Object tracker connected to camera module")

    def _start_modules(self):
        """Start all deployed modules in the correct order."""
        self.connection.start()
        self.mapper.start()
        self.global_planner.start()
        self.local_planner.start()
        self.navigator.start()
        self.frontier_explorer.start()
        self.websocket_vis.start()
        self.foxglove_bridge.start()
        self.spatial_memory_module.start()
        self.depth_module.start()
        self.object_tracker.start()

        # Initialize skills after connection is established
        if self.skill_library is not None:
            for skill in self.skill_library:
                if isinstance(skill, AbstractRobotSkill):
                    self.skill_library.create_instance(skill.__name__, robot=self)
            if isinstance(self.skill_library, MyUnitreeSkills):
                self.skill_library._robot = self
                self.skill_library.init()
                self.skill_library.initialize_skills()

    def get_single_rgb_frame(self, timeout: float = 2.0) -> Image:
        topic = Topic("/go2/color_image", Image)
        return self.lcm.wait_for_message(topic, timeout=timeout)

    def move(self, twist: Twist, duration: float = 0.0):
        """Send movement command to robot."""
        self.connection.move(twist, duration)

    def explore(self) -> bool:
        """Start autonomous frontier exploration.

        Returns:
            True if exploration started successfully
        """
        return self.frontier_explorer.explore()

    def navigate_to(self, pose: PoseStamped, blocking: bool = True):
        """Navigate to a target pose.

        Args:
            pose: Target pose to navigate to
            blocking: If True, block until goal is reached. If False, return immediately.

        Returns:
            If blocking=True: True if navigation was successful, False otherwise
            If blocking=False: True if goal was accepted, False otherwise
        """

        logger.info(
            f"Navigating to pose: ({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f})"
        )
        self.navigator.set_goal(pose)
        time.sleep(1.0)

        if blocking:
            while self.navigator.get_state() == NavigatorState.FOLLOWING_PATH:
                time.sleep(0.25)

            time.sleep(1.0)
            if not self.navigator.is_goal_reached():
                logger.info("Navigation was cancelled or failed")
                return False
            else:
                logger.info("Navigation goal reached")
                return True

        return True

    def stop_exploration(self) -> bool:
        """Stop autonomous exploration.

        Returns:
            True if exploration was stopped
        """
        self.navigator.cancel_goal()
        return self.frontier_explorer.stop_exploration()

    def cancel_navigation(self) -> bool:
        """Cancel the current navigation goal.

        Returns:
            True if goal was cancelled
        """
        return self.navigator.cancel_goal()

    @property
    def spatial_memory(self) -> Optional[SpatialMemory]:
        """Get the robot's spatial memory module.

        Returns:
            SpatialMemory module instance or None if perception is disabled
        """
        return self.spatial_memory_module

    def get_odom(self) -> PoseStamped:
        """Get the robot's odometry.

        Returns:
            The robot's odometry
        """
        return self.connection.get_odom()

    def navigate_to_object(self, bbox: List[float], distance: float = 0.5, timeout: float = 30.0):
        """Navigate to an object by tracking it and maintaining a specified distance.

        Args:
            bbox: Bounding box of the object to track [x1, y1, x2, y2]
            distance: Distance to maintain from the object (meters)
            timeout: Total timeout for the navigation (seconds)

        Returns:
            True if navigation completed successfully, False otherwise
        """
        if not self.object_tracker:
            logger.error("Object tracker not initialized")
            return False

        logger.info(f"Starting object tracking with bbox: {bbox}")
        self.object_tracker.track(bbox)

        start_time = time.time()
        goal_set = False

        while time.time() - start_time < timeout:
            if self.navigator.get_state() == NavigatorState.IDLE and goal_set:
                logger.info("Waiting for goal result")
                time.sleep(1.0)
                if not self.navigator.is_goal_reached():
                    logger.info("Goal cancelled, object tracking failed")
                    return False
                else:
                    logger.info("Object tracking goal reached")
                    return True

            if goal_set and not self.object_tracker.is_tracking():
                continue

            detection_topic = Topic("/go2/detection3d", Detection3DArray)
            detection_msg = self.lcm.wait_for_message(detection_topic, timeout=1.0)

            if detection_msg and len(detection_msg.detections) > 0:
                target_pose = extract_pose_from_detection3d(detection_msg.detections[0])

                retracted_pose = offset_distance(
                    target_pose, distance, approach_vector=Vector3(-1, 0, 0)
                )

                goal_pose = PoseStamped(
                    frame_id=detection_msg.header.frame_id,
                    position=retracted_pose.position,
                    orientation=retracted_pose.orientation,
                )
                self.navigator.set_goal(goal_pose)
                goal_set = True

            time.sleep(0.25)

        logger.info("Object tracking timed out")
        return False


def main():
    """Main entry point."""
    ip = os.getenv("ROBOT_IP")
    connection_type = os.getenv("CONNECTION_TYPE", "webrtc")

    pubsub.lcm.autoconf()

    robot = UnitreeGo2(ip=ip, websocket_port=7779, connection_type=connection_type)
    robot.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Shutting down...")


if __name__ == "__main__":
    main()
