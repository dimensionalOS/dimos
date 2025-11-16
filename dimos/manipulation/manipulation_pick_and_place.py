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

"""
Manipulation module for VLM-guided pick and place operations.
Outputs end-effector coordinates for robot execution.
"""

import threading
import time
from typing import Any, Optional

import numpy as np
import reactivex as rx

from dimos.core import Module, rpc
from dimos.manipulation.manip_aio_pipeline_new import ManipulationPipeline
from dimos.manipulation.Task_Planner import VLMTaskPlanner
from dimos.models.vl.qwen import QwenVlModel
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.manipulation.manipulation_module")


class ManipulationModule(Module):
    """
    Manipulation module that uses VLM for task planning.
    
    Takes natural language instructions and outputs end-effector coordinates
    for pick and place operations using multi-camera perception.
    """
    
    def __init__(
        self,
        camera_calibrations: list[dict],
        grasp_server_url: str = "ws://localhost:8765",
        vlm_model: Optional[QwenVlModel] = None,
        **kwargs
    ):
        """
        Initialize ManipulationModule with calibrated cameras.
        
        Args:
            camera_calibrations: List of camera configs with calibration:
                [{
                    "camera_id": 0,
                    "intrinsics": [fx, fy, cx, cy],
                    "extrinsics": 4x4 numpy array (from ArUco calibration)
                }, ...]
            grasp_server_url: WebSocket URL for grasp generation server
            vlm_model: Optional VLM model (defaults to Qwen)
        """
        super().__init__(**kwargs)
        
        # Store camera configurations
        self.camera_configs = camera_calibrations
        self.num_cameras = len(camera_calibrations)
        
        # Initialize manipulation pipeline with calibrated cameras
        logger.info("Initializing ManipulationPipeline...")
        self.pipeline = ManipulationPipeline(
            camera_configs=self.camera_configs,
            min_confidence=0.6,
            max_objects=20,
            grasp_server_url=grasp_server_url,
            enable_grasp_generation=True,
            enable_segmentation=True,
            max_grasps_per_object=5,
            min_points_for_grasp=100
        )
        
        # Initialize VLM task planner
        logger.info("Initializing VLMTaskPlanner...")
        self.planner = VLMTaskPlanner(
            manipulation_pipeline=self.pipeline,
            vlm_model=vlm_model or QwenVlModel(),
            camera_id=0  # Use first camera for VLM queries
        )
        
        # Camera streams (will be set by start_camera_streams)
        self.camera_streams = []
        self.pipeline_streams = None
        self.stream_subscriptions = []
        
        # Latest frame buffers for each camera
        self.latest_rgb_images = [None] * self.num_cameras
        self.latest_depth_images = [None] * self.num_cameras
        
        # Current task state
        self.current_task_type = None
        self.current_object = None
        self.object_held = False
        
        # Grasp execution parameters (from visual servoing)
        self.gripper_max_opening = 0.07     # 7cm maximum gripper opening
        self.grasp_width_offset = 0.03      # Add 3cm to object width for gripper
        self.approach_distance = 0.10       # Approach from 10cm above/away
        self.retract_distance = 0.12        # Retract 12cm after grasp/place
        
        # Workspace safety limits
        self.workspace_min_radius = 0.2     # 20cm minimum reach
        self.workspace_max_radius = 0.75    # 75cm maximum reach
        self.workspace_min_z = 0.05         # 5cm minimum height (table level)
        self.workspace_max_z = 0.6          # 60cm maximum height
        
        # Task control flags
        self.task_running = False
        self.cameras_ready = False
        
        logger.info("ManipulationModule initialized successfully")
    
    @rpc
    def start_camera_streams(
        self,
        camera_stream_0: rx.Observable,
        camera_stream_1: Optional[rx.Observable] = None
    ) -> dict[str, Any]:
        """
        Start camera streams and initialize pipeline processing.
        
        Args:
            camera_stream_0: First camera stream (rx.Observable emitting RGB-D frames)
            camera_stream_1: Optional second camera stream for dual-camera setup
            
        Returns:
            Status dictionary
        """
        try:
            # Prepare list of camera streams
            self.camera_streams = [camera_stream_0]
            if camera_stream_1 and self.num_cameras > 1:
                self.camera_streams.append(camera_stream_1)
            
            if len(self.camera_streams) != self.num_cameras:
                return {
                    "status": "error",
                    "message": f"Expected {self.num_cameras} streams, got {len(self.camera_streams)}"
                }
            
            # Subscribe to each camera for frame buffering
            for idx, stream in enumerate(self.camera_streams):
                def create_frame_callback(cam_id):
                    def on_frame(frame_data):
                        if frame_data:
                            self.latest_rgb_images[cam_id] = frame_data.get("rgb")
                            self.latest_depth_images[cam_id] = frame_data.get("depth")
                    return on_frame
                
                subscription = stream.subscribe(on_next=create_frame_callback(idx))
                self.stream_subscriptions.append(subscription)
            
            # Create pipeline processing streams
            self.pipeline_streams = self.pipeline.create_streams(self.camera_streams)
            
            # Subscribe to objects with grasps output
            self.pipeline_streams["objects_with_grasps"].subscribe(
                on_next=lambda objs: logger.debug(f"Pipeline detected {len(objs)} graspable objects")
            )
            
            self.cameras_ready = True
            logger.info(f"Started {len(self.camera_streams)} camera streams successfully")
            
            return {
                "status": "success",
                "message": f"Started {len(self.camera_streams)} camera streams",
                "cameras": len(self.camera_streams)
            }
            
        except Exception as e:
            logger.error(f"Failed to start camera streams: {e}")
            return {"status": "error", "message": str(e)}
    
    @rpc
    def pick_task(self, instruction: str) -> dict[str, Any]:
        """
        Plan a pick task and return end-effector coordinates.
        
        Args:
            instruction: Natural language command (e.g., "pick up the blue mug")
            
        Returns:
            Dictionary containing:
                - status: "success" or "error"
                - position: [x, y, z] end-effector target position in meters
                - orientation: 3x3 rotation matrix for end-effector
                - gripper_width: Target gripper opening in meters
                - approach_position: [x, y, z] approach waypoint
                - retract_position: [x, y, z] retract waypoint
                - object: Detected object class name
                - confidence: VLM confidence score
        """
        if self.task_running:
            return {"status": "error", "message": "Another task is already running"}
        
        if not self.cameras_ready:
            return {"status": "error", "message": "Camera streams not initialized"}
        
        if not self.latest_rgb_images[0]:
            return {"status": "error", "message": "No camera data available"}
        
        try:
            self.task_running = True
            self.current_task_type = "pick"
            
            # Get current RGB image for VLM
            rgb_image = self.latest_rgb_images[0]
            
            # Use VLM to identify target object and select grasp
            logger.info(f"Planning pick task: '{instruction}'")
            task_plan = self.planner.plan_pick_task(
                instruction=instruction,
                rgb_image=rgb_image,
                retry_on_failure=True
            )
            
            if not task_plan:
                logger.error("VLM failed to identify target object")
                return {
                    "status": "error",
                    "message": "Could not identify target object from instruction"
                }
            
            logger.info(f"VLM identified: {task_plan.target_object.get('class_name', 'unknown')} "
                       f"at pixel {task_plan.pixel_location} "
                       f"with confidence {task_plan.confidence:.2f}")
            logger.info(f"VLM reasoning: {task_plan.reasoning}")
            
            # Extract grasp information
            grasp = task_plan.selected_grasp
            grasp_position = grasp["translation"]  # [x, y, z] in world/robot frame
            
            # Validate workspace limits
            if not self._validate_position(grasp_position):
                return {
                    "status": "error",
                    "message": f"Grasp position {grasp_position} outside workspace limits"
                }
            
            # Calculate approach position (above the grasp)
            approach_position = grasp_position.copy()
            approach_position[2] += self.approach_distance
            
            # Calculate retract position (lift after grasping)
            retract_position = grasp_position.copy()
            retract_position[2] += self.retract_distance
            
            # Calculate gripper width
            object_width = grasp.get("width", 0.04)
            gripper_width = min(
                object_width + self.grasp_width_offset,
                self.gripper_max_opening
            )
            
            # Store current object info
            self.current_object = task_plan.target_object
            
            logger.info(f"Pick task planned successfully:")
            logger.info(f"  Target position: {grasp_position}")
            logger.info(f"  Gripper width: {gripper_width*1000:.1f}mm")
            
            return {
                "status": "success",
                "task_type": "pick",
                "position": grasp_position,
                "orientation": grasp["rotation_matrix"],
                "gripper_width": gripper_width,
                "approach_position": approach_position,
                "retract_position": retract_position,
                "object": task_plan.target_object.get("class_name", "unknown"),
                "object_id": task_plan.target_object.get("temp_id", ""),
                "confidence": task_plan.confidence,
                "reasoning": task_plan.reasoning,
                "grasp_score": grasp.get("score", 0.0)
            }
            
        except Exception as e:
            logger.error(f"Error in pick task planning: {e}")
            return {"status": "error", "message": str(e)}
        finally:
            self.task_running = False
    
    @rpc
    def place_task(self, instruction: str) -> dict[str, Any]:
        """
        Plan a place task and return end-effector coordinates.
        
        Args:
            instruction: Natural language command (e.g., "place it on the table")
            
        Returns:
            Dictionary containing:
                - status: "success" or "error"
                - position: [x, y, z] end-effector target position in meters
                - orientation: 3x3 rotation matrix for end-effector
                - approach_position: [x, y, z] approach waypoint
                - retract_position: [x, y, z] retract waypoint
                - confidence: VLM confidence score
        """
        if self.task_running:
            return {"status": "error", "message": "Another task is already running"}
        
        if not self.cameras_ready:
            return {"status": "error", "message": "Camera streams not initialized"}
        
        if not self.latest_rgb_images[0] or not self.latest_depth_images[0]:
            return {"status": "error", "message": "No camera data available"}
        
        try:
            self.task_running = True
            self.current_task_type = "place"
            
            # Get current frames for VLM
            rgb_image = self.latest_rgb_images[0]
            depth_image = self.latest_depth_images[0]
            
            # Use VLM to identify place location
            logger.info(f"Planning place task: '{instruction}'")
            place_plan = self.planner.plan_place_task(
                instruction=instruction,
                rgb_image=rgb_image,
                depth_image=depth_image
            )
            
            if not place_plan:
                logger.error("VLM failed to identify place location")
                return {
                    "status": "error",
                    "message": "Could not identify place location from instruction"
                }
            
            logger.info(f"VLM identified place location at pixel {place_plan['pixel_location']}")
            logger.info(f"World position: {place_plan['world_position']}")
            logger.info(f"VLM reasoning: {place_plan.get('reasoning', '')}")
            
            # Extract place position
            place_position = place_plan['world_position']  # [x, y, z] in world/robot frame
            
            # Add clearance if placing on a surface
            place_position[2] += 0.02  # 2cm clearance above surface
            
            # Validate workspace limits
            if not self._validate_position(place_position):
                return {
                    "status": "error",
                    "message": f"Place position {place_position} outside workspace limits"
                }
            
            # Calculate approach position (above the place location)
            approach_position = place_position.copy()
            approach_position[2] += self.approach_distance
            
            # Calculate retract position (lift after placing)
            retract_position = place_position.copy()
            retract_position[2] += self.retract_distance
            
            # Default orientation for placing (straight down)
            place_orientation = [
                [1, 0, 0],
                [0, 1, 0],
                [0, 0, 1]
            ]
            
            logger.info(f"Place task planned successfully:")
            logger.info(f"  Target position: {place_position}")
            
            return {
                "status": "success",
                "task_type": "place",
                "position": place_position,
                "orientation": place_orientation,
                "approach_position": approach_position,
                "retract_position": retract_position,
                "confidence": place_plan.get('confidence', 0.5),
                "reasoning": place_plan.get('reasoning', ''),
                "pixel_location": place_plan['pixel_location']
            }
            
        except Exception as e:
            logger.error(f"Error in place task planning: {e}")
            return {"status": "error", "message": str(e)}
        finally:
            self.task_running = False
    
    @rpc
    def pick_and_place_sequence(
        self,
        pick_instruction: str,
        place_instruction: str
    ) -> dict[str, Any]:
        """
        Plan a complete pick and place sequence.
        
        Args:
            pick_instruction: Natural language pick command
            place_instruction: Natural language place command
            
        Returns:
            Dictionary containing both pick and place coordinates
        """
        logger.info(f"Planning pick and place sequence:")
        logger.info(f"  Pick: '{pick_instruction}'")
        logger.info(f"  Place: '{place_instruction}'")
        
        # Plan pick
        pick_result = self.pick_task(pick_instruction)
        
        if pick_result["status"] != "success":
            return {
                "status": "error",
                "message": f"Pick planning failed: {pick_result.get('message', 'Unknown error')}",
                "phase": "pick"
            }
        
        # Wait a moment for pipeline to update
        time.sleep(0.5)
        
        # Plan place
        place_result = self.place_task(place_instruction)
        
        if place_result["status"] != "success":
            return {
                "status": "error",
                "message": f"Place planning failed: {place_result.get('message', 'Unknown error')}",
                "phase": "place"
            }
        
        # Combine results
        return {
            "status": "success",
            "pick": {
                "position": pick_result["position"],
                "orientation": pick_result["orientation"],
                "gripper_width": pick_result["gripper_width"],
                "approach_position": pick_result["approach_position"],
                "retract_position": pick_result["retract_position"],
                "object": pick_result["object"]
            },
            "place": {
                "position": place_result["position"],
                "orientation": place_result["orientation"],
                "approach_position": place_result["approach_position"],
                "retract_position": place_result["retract_position"]
            },
            "execution_sequence": [
                {"action": "move", "position": pick_result["approach_position"], "description": "Approach object"},
                {"action": "open_gripper", "width": pick_result["gripper_width"], "description": "Open gripper"},
                {"action": "move", "position": pick_result["position"], "description": "Move to grasp"},
                {"action": "close_gripper", "description": "Grasp object"},
                {"action": "move", "position": pick_result["retract_position"], "description": "Lift object"},
                {"action": "move", "position": place_result["approach_position"], "description": "Move to place area"},
                {"action": "move", "position": place_result["position"], "description": "Lower to place"},
                {"action": "open_gripper", "description": "Release object"},
                {"action": "move", "position": place_result["retract_position"], "description": "Retract from place"}
            ]
        }
    
    def _validate_position(self, position: list) -> bool:
        """
        Validate that a position is within workspace limits.
        
        Args:
            position: [x, y, z] position to validate
            
        Returns:
            True if position is valid and reachable
        """
        x, y, z = position
        
        # Check height limits
        if not (self.workspace_min_z <= z <= self.workspace_max_z):
            logger.error(f"Position z={z:.3f}m outside limits "
                        f"[{self.workspace_min_z:.2f}, {self.workspace_max_z:.2f}]")
            return False
        
        # Check radial distance
        distance = np.sqrt(x**2 + y**2 + z**2)
        if not (self.workspace_min_radius <= distance <= self.workspace_max_radius):
            logger.error(f"Position distance={distance:.3f}m outside limits "
                        f"[{self.workspace_min_radius:.2f}, {self.workspace_max_radius:.2f}]")
            return False
        
        return True
    
    @rpc
    def set_object_held(self, held: bool) -> dict[str, Any]:
        """
        Update whether an object is currently held.
        Used by robot controller to inform module of grasp success/failure.
        
        Args:
            held: True if object is held, False otherwise
            
        Returns:
            Status dictionary
        """
        self.object_held = held
        
        if not held:
            self.current_object = None
        
        return {
            "status": "success",
            "object_held": self.object_held,
            "current_object": self.current_object.get("class_name") if self.current_object else None
        }
    
    @rpc
    def get_status(self) -> dict[str, Any]:
        """
        Get current status of the manipulation module.
        
        Returns:
            Dictionary with current module state
        """
        return {
            "cameras_ready": self.cameras_ready,
            "num_cameras": len(self.camera_streams),
            "task_running": self.task_running,
            "current_task": self.current_task_type,
            "object_held": self.object_held,
            "current_object": self.current_object.get("class_name") if self.current_object else None,
            "pipeline_active": self.pipeline_streams is not None,
            "workspace_limits": {
                "min_radius": self.workspace_min_radius,
                "max_radius": self.workspace_max_radius,
                "min_z": self.workspace_min_z,
                "max_z": self.workspace_max_z
            }
        }
    
    @rpc
    def get_detected_objects(self) -> dict[str, Any]:
        """
        Get list of currently detected objects with grasps.
        
        Returns:
            Dictionary with detected objects information
        """
        try:
            objects = self.pipeline.get_objects_with_grasps()
            
            object_list = []
            for obj in objects:
                object_list.append({
                    "class_name": obj.get("class_name", "unknown"),
                    "id": obj.get("temp_id", ""),
                    "num_grasps": len(obj.get("grasps", [])),
                    "best_grasp_score": obj["grasps"][0]["score"] if obj.get("grasps") else 0.0,
                    "position": obj["grasps"][0]["translation"] if obj.get("grasps") else None
                })
            
            return {
                "status": "success",
                "num_objects": len(object_list),
                "objects": object_list
            }
            
        except Exception as e:
            logger.error(f"Error getting detected objects: {e}")
            return {"status": "error", "message": str(e)}
    
    @rpc
    def shutdown(self) -> dict[str, Any]:
        """
        Shutdown the manipulation module and cleanup resources.
        
        Returns:
            Status dictionary
        """
        logger.info("Shutting down ManipulationModule")
        
        try:
            # Unsubscribe from all streams
            for subscription in self.stream_subscriptions:
                subscription.dispose()
            self.stream_subscriptions.clear()
            
            # Clear camera streams
            self.camera_streams.clear()
            self.cameras_ready = False
            
            # Cleanup pipeline
            if hasattr(self.pipeline, "cleanup"):
                self.pipeline.cleanup()
            
            logger.info("ManipulationModule shutdown complete")
            return {"status": "success", "message": "Module shutdown complete"}
            
        except Exception as e:
            logger.error(f"Error during shutdown: {e}")
            return {"status": "error", "message": str(e)}