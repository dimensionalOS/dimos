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
Robot Capabilities Module

Provides low-level robot capabilities such as speech, gestures, and other
basic robot operations that can be used by higher-level skills.

This module can be extended with additional robot capabilities in the future.
"""

import time

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class RobotCapabilities(Module):
    """Low-level capabilities for basic robot operations.

    Provides foundational robot capabilities like speech that can be used
    by higher-level manipulation skills. In a real setting, this would
    interface with actual text-to-speech hardware or services.

    This module can be extended with additional capabilities such as:
    - Gestures and body language
    - LED/display control
    - Sound effects
    - Sensor feedback
    """

    rpc_calls: list[str] = []

    @rpc
    def speak(self, text: str) -> str:
        """Speak text out loud through the robot's speakers.

        In a real implementation, this would interface with text-to-speech
        hardware or services. Currently logs the speech and simulates
        execution time.

        Args:
            text: The text to speak.

        Returns:
            Status message confirming what was spoken.
        """
        time.sleep(0.1)  # Simulate execution time
        logger.info(f"[Robot Speaking]: {text}")
        return f"Robot said: {text}"

    @rpc
    def start(self) -> None:
        """Start the robot capabilities module."""
        super().start()
        logger.info("RobotCapabilities module started")

    @rpc
    def stop(self) -> None:
        """Stop the robot capabilities module."""
        super().stop()
        logger.info("RobotCapabilities module stopped")

    def __getstate__(self):
        """Serialize for Dask - no state to preserve."""
        return {}

    def __setstate__(self, state):
        """Deserialize for Dask."""
        pass


# Expose blueprint for declarative composition
robot_capabilities = RobotCapabilities.blueprint

__all__ = ["RobotCapabilities", "robot_capabilities"]
