"""
Task planner that uses VLM for object selection and grasp execution planning.
"""

import json
import re
from dataclasses import dataclass
from typing import Any, Optional

import numpy as np

from dimos.models.vl.qwen import QwenVlModel
from dimos.manipulation.manip_aio_pipeline_new import ManipulationPipeline
from dimos.msgs.sensor_msgs import Image
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.planning.task_planner")


@dataclass
class TaskPlan:
    """Result of task planning."""
    instruction: str
    target_object: dict
    selected_grasp: dict
    pixel_location: tuple[int, int]
    confidence: float
    reasoning: str


class VLMTaskPlanner:
    """
    Task planner that uses VLM to identify target objects and select grasps.
    
    Workflow:
    1. VLM analyzes scene and instruction
    2. VLM returns pixel coordinates of target object
    3. Map pixel to detected object
    4. Select best grasp for that object
    """
    
    def __init__(
        self,
        manipulation_pipeline: ManipulationPipeline,
        vlm_model: Optional[QwenVlModel] = None,
        camera_id: int = 0,
    ):
        """
        Initialize task planner.
        
        Args:
            manipulation_pipeline: Pipeline with object detection and grasps
            vlm_model: VLM model for visual reasoning (defaults to Qwen)
            camera_id: Which camera to use for VLM queries
        """
        self.pipeline = manipulation_pipeline
        self.vlm = vlm_model or QwenVlModel()
        self.camera_id = camera_id
        
    def plan_pick_task(
        self,
        instruction: str,
        rgb_image: np.ndarray,
        retry_on_failure: bool = True
    ) -> Optional[TaskPlan]:
        """
        Plan a pick task based on natural language instruction.
        
        Args:
            instruction: Natural language command (e.g., "pick up the blue mug")
            rgb_image: Current camera view
            retry_on_failure: Whether to retry with different prompts if parsing fails
            
        Returns:
            TaskPlan with grasp information, or None if planning failed
        """
        # Get current objects with grasps from pipeline
        objects_with_grasps = self.pipeline.get_objects_with_grasps()
        
        if not objects_with_grasps:
            logger.warning("No objects with grasps available")
            return None
            
        # Query VLM to identify target object
        vlm_response = self._query_vlm_for_object(
            rgb_image, 
            instruction,
            objects_with_grasps
        )
        
        if not vlm_response:
            logger.error("VLM failed to identify target object")
            return None
            
        # Parse VLM response to get pixel coordinates
        pixel_coords = self._parse_pixel_from_vlm(vlm_response)
        
        if not pixel_coords and retry_on_failure:
            # Retry with more explicit prompt
            vlm_response = self._query_vlm_with_explicit_format(
                rgb_image,
                instruction
            )
            pixel_coords = self._parse_pixel_from_vlm(vlm_response)
            
        if not pixel_coords:
            logger.error(f"Could not parse pixel coordinates from VLM response: {vlm_response}")
            return None
            
        # Find object at pixel location
        target_object = self.pipeline.find_object_at_pixel(pixel_coords, self.camera_id)
        
        if not target_object:
            logger.warning(f"No object found at pixel {pixel_coords}")
            return None
            
        # Get best grasp for object
        best_grasp = None
        if target_object.get('grasps'):
            best_grasp = target_object['grasps'][0]  # Already sorted by score
        else:
            logger.warning(f"Object at {pixel_coords} has no grasps")
            return None
            
        # Extract confidence and reasoning from VLM
        confidence = self._extract_confidence(vlm_response)
        reasoning = self._extract_reasoning(vlm_response)
        
        return TaskPlan(
            instruction=instruction,
            target_object=target_object,
            selected_grasp=best_grasp,
            pixel_location=pixel_coords,
            confidence=confidence,
            reasoning=reasoning
        )
        
    def _query_vlm_for_object(
        self,
        rgb_image: np.ndarray,
        instruction: str,
        available_objects: list[dict]
    ) -> str:
        """Query VLM to identify target object."""
        
        # Build context about available objects
        object_descriptions = []
        for i, obj in enumerate(available_objects):
            desc = f"Object {i}: {obj.get('class_name', 'unknown')}"
            if obj.get('grasps'):
                desc += f" (graspable, {len(obj['grasps'])} grasp options)"
            object_descriptions.append(desc)
            
        prompt = f"""You are a robotic manipulation assistant. Your task is to identify which object to pick based on the instruction.

Instruction: {instruction}

Available objects detected in scene:
{chr(10).join(object_descriptions)}

Please identify the target object by clicking on it in the image. Respond with:
1. The pixel coordinates [x, y] where you would click
2. Your confidence (0-1)
3. Brief reasoning for your selection

Format your response as:
PIXEL: [x, y]
CONFIDENCE: 0.X
REASONING: <your explanation>
"""
        
        # Convert numpy image to Image message
        image_msg = Image.from_numpy(rgb_image)
        
        return self.vlm.query(image_msg, prompt)
        
    def _query_vlm_with_explicit_format(
        self,
        rgb_image: np.ndarray,
        instruction: str
    ) -> str:
        """Retry with more explicit formatting instructions."""
        
        prompt = f"""Click on the object described: "{instruction}"

IMPORTANT: Your response MUST include pixel coordinates in this EXACT format:
PIXEL: [x, y]

Where x is the horizontal position (0 to image width)
And y is the vertical position (0 to image height)

Example response:
PIXEL: [320, 240]
CONFIDENCE: 0.9
REASONING: The blue mug is clearly visible in the center of the image.

Now identify and click on: {instruction}"""

        image_msg = Image.from_numpy(rgb_image)
        return self.vlm.query(image_msg, prompt)
        
    def _parse_pixel_from_vlm(self, vlm_response: str) -> Optional[tuple[int, int]]:
        """Parse pixel coordinates from VLM response."""
        
        # Try multiple parsing patterns
        patterns = [
            r"PIXEL:\s*\[(\d+),\s*(\d+)\]",
            r"pixel.*?(\d{1,4})[,\s]+(\d{1,4})",
            r"\[(\d{1,4}),\s*(\d{1,4})\]",
            r"click.*?(\d{1,4})[,\s]+(\d{1,4})",
        ]
        
        for pattern in patterns:
            match = re.search(pattern, vlm_response, re.IGNORECASE)
            if match:
                x, y = int(match.group(1)), int(match.group(2))
                logger.info(f"Parsed pixel coordinates: ({x}, {y})")
                return (x, y)
                
        return None
    
    def plan_place_task(
        self,
        instruction: str,
        rgb_image: np.ndarray,
        depth_image: np.ndarray
    ) -> Optional[dict]:
        """
        Plan placement location using VLM.
        
        Returns:
            Dict with placement position in world coordinates
        """
        # Query VLM for placement location
        prompt = f"""Instruction: {instruction}
        
    Where should I place the object? Provide pixel coordinates.

    PIXEL: [x, y]
    REASONING: <why this location>"""

        response = self.vlm.query(Image.from_numpy(rgb_image), prompt)
        pixel = self._parse_pixel_from_vlm(response)
        
        if not pixel:
            return None
        
        # Convert pixel to 3D using depth
        x, y = pixel
        depth_value = depth_image[y, x]
        
        # Use camera intrinsics to get 3D point
        fx, fy, cx, cy = self.pipeline.camera_configs[self.camera_id]['intrinsics']
        
        z = depth_value
        x_3d = (x - cx) * z / fx
        y_3d = (y - cy) * z / fy
        
        # Transform to world coordinates
        extrinsics = self.pipeline.camera_configs[self.camera_id]['extrinsics']
        cam_point = np.array([x_3d, y_3d, z, 1.0])
        world_point = extrinsics @ cam_point
        
        return {
            "instruction": instruction,
            "pixel_location": pixel,
            "world_position": world_point[:3].tolist(),
            "confidence": self._extract_confidence(response),
            "reasoning": self._extract_reasoning(response)
        }
        
    def _extract_confidence(self, vlm_response: str) -> float:
        """Extract confidence value from VLM response."""
        
        patterns = [
            r"CONFIDENCE:\s*(0?\.\d+)",
            r"confidence.*?(0?\.\d+)",
        ]
        
        for pattern in patterns:
            match = re.search(pattern, vlm_response, re.IGNORECASE)
            if match:
                return float(match.group(1))
                
        return 0.5  # Default confidence
        
    def _extract_reasoning(self, vlm_response: str) -> str:
        """Extract reasoning from VLM response."""
        
        match = re.search(r"REASONING:\s*(.+?)(?:\n|$)", vlm_response, re.IGNORECASE | re.DOTALL)
        if match:
            return match.group(1).strip()
            
        return vlm_response  # Return full response if no specific reasoning found
        
    def plan_complex_task(
        self,
        instruction: str,
        rgb_image: np.ndarray
    ) -> list[TaskPlan]:
        """
        Plan a complex multi-step task.
        
        Args:
            instruction: Complex instruction (e.g., "Sort the tools by size")
            rgb_image: Current scene
            
        Returns:
            List of TaskPlans to execute in sequence
        """
        # This could decompose complex tasks into multiple pick operations
        # For now, just handle as single pick
        
        single_plan = self.plan_pick_task(instruction, rgb_image)
        return [single_plan] if single_plan else []
        
    def validate_grasp_reachability(self, grasp: dict) -> bool:
        """
        Check if a grasp is reachable by the robot.
        
        Args:
            grasp: Grasp dictionary with translation and rotation
            
        Returns:
            True if grasp is within robot workspace
        """
        translation = grasp.get('translation', [0, 0, 0])
        
        # Define robot workspace limits (example)
        workspace_limits = {
            'x': (-0.5, 0.5),
            'y': (-0.5, 0.5),
            'z': (0.1, 0.8)
        }
        
        return (
            workspace_limits['x'][0] <= translation[0] <= workspace_limits['x'][1] and
            workspace_limits['y'][0] <= translation[1] <= workspace_limits['y'][1] and
            workspace_limits['z'][0] <= translation[2] <= workspace_limits['z'][1]
        )