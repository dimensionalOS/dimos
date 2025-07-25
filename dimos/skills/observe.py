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
Observer skill for an agent.

This module provides a skill that sends a single image from any
Robot Data Stream to the Qwen VLM for inference and adds the response
to the agent's conversation history.
"""

import time
from typing import Optional
import base64
import cv2
import numpy as np
import reactivex as rx
from reactivex import operators as ops
from pydantic import Field

from dimos.skills.skills import AbstractRobotSkill
from dimos.agents.agent import LLMAgent
from dimos.models.qwen.video_query import query_single_frame
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.skills.observe")


class Observe(AbstractRobotSkill):
    """
    A skill that captures a single frame from a Robot Video Stream, sends it to a VLM,
    and adds the response to the agent's conversation history.

    This skill is used for visual reasoning, spatial understanding, or any queries involving visual information that require critical thinking.
    """

    query_text: str = Field(
        "What do you see in this image? Describe the environment in detail.",
        description="Query text to send to the VLM model with the image",
    )

    def __init__(self, robot=None, agent: Optional[LLMAgent] = None, **data):
        """
        Initialize the Observe skill.

        Args:
            robot: The robot instance
            agent: The agent to store results in
            **data: Additional data for configuration
        """
        super().__init__(robot=robot, **data)
        self._agent = agent
        self._model_name = "qwen2.5-vl-72b-instruct"

        # Get the video stream from the robot
        self._video_stream = self._robot.video_stream
        if self._video_stream is None:
            logger.error("Failed to get video stream from robot")

    def __call__(self):
        """
        Capture a single frame, process it with Qwen, and add the result to conversation history.

        Returns:
            A message indicating the observation result
        """
        super().__call__()

        if self._agent is None:
            error_msg = "No agent provided to Observe skill"
            logger.error(error_msg)
            return error_msg

        if self._robot is None:
            error_msg = "No robot instance provided to Observe skill"
            logger.error(error_msg)
            return error_msg

        if self._video_stream is None:
            error_msg = "No video stream available"
            logger.error(error_msg)
            return error_msg

        try:
            logger.info("Capturing frame for Qwen observation")

            # Get a single frame from the video stream
            frame = self._get_frame_from_stream()

            if frame is None:
                error_msg = "Failed to capture frame from video stream"
                logger.error(error_msg)
                return error_msg

            # Process the frame with Qwen
            response = self._process_frame_with_qwen(frame)

            logger.info(f"Added Qwen observation to conversation history")
            return f"Observation complete: {response}"

        except Exception as e:
            error_msg = f"Error in Observe skill: {e}"
            logger.error(error_msg)
            return error_msg

    def _get_frame_from_stream(self):
        """
        Get a single frame from the video stream.

        Returns:
            A single frame from the video stream, or None if no frame is available
        """
        if self._video_stream is None:
            logger.error("Video stream is None")
            return None

        frame = None
        frame_subject = rx.subject.Subject()

        subscription = self._video_stream.pipe(
            ops.take(1)  # Take just one frame
        ).subscribe(
            on_next=lambda x: frame_subject.on_next(x),
            on_error=lambda e: logger.error(f"Error getting frame: {e}"),
        )

        # Wait up to 5 seconds for a frame
        timeout = 5.0
        start_time = time.time()

        def on_frame(f):
            nonlocal frame
            frame = f

        frame_subject.subscribe(on_frame)

        while frame is None and time.time() - start_time < timeout:
            time.sleep(0.1)

        subscription.dispose()
        return frame

    def _process_frame_with_qwen(self, frame):
        """
        Process a frame with the Qwen model using query_single_frame.

        Args:
            frame: The video frame to process (numpy array)

        Returns:
            The response from Qwen
        """
        logger.info(f"Processing frame with Qwen model: {self._model_name}")

        try:
            # Ensure frame is in RGB format for Qwen
            if isinstance(frame, np.ndarray):
                # OpenCV uses BGR, convert to RGB if needed
                if frame.shape[-1] == 3:  # Check if it has color channels
                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                else:
                    frame_rgb = frame
            else:
                raise ValueError("Frame must be a numpy array")

            # Query Qwen with the frame (direct function call)
            response = query_single_frame(
                frame_rgb,
                self.query_text,
                model_name=self._model_name,
            )

            logger.info(f"Qwen response received: {response[:100]}...")
            return response

        except Exception as e:
            logger.error(f"Error processing frame with Qwen: {e}")
            raise
