"""Abstract base class for all DIMOS robot implementations.

This module defines the AbstractRobot class which serves as the foundation for
all robot implementations in DIMOS, establishing a common interface regardless
of the underlying hardware or communication protocol (ROS, WebRTC, etc).
"""

from abc import ABC, abstractmethod
from reactivex.observable import Observable


class AbstractRobot(ABC):
    """Abstract base class for all robot implementations.

    This class defines the minimal interface that all robot implementations
    must provide, regardless of whether they use ROS, WebRTC, or other
    communication protocols.
    """

    @abstractmethod
    def connect(self) -> bool:
        """Establish a connection to the robot.

        This method should handle all necessary setup to establish
        communication with the robot hardware.

        Returns:
            bool: True if connection was successful, False otherwise.
        """
        pass

    @abstractmethod
    def move(self, *args, **kwargs) -> bool:
        """Move the robot.

        This is a generic movement interface that should be implemented
        by all robot classes. The exact parameters will depend on the
        specific robot implementation.

        Returns:
            bool: True if movement command was successfully sent.
        """
        pass

    @abstractmethod
    def get_video_stream(self, fps: int = 30) -> Observable:
        """Get a video stream from the robot's camera.

        Args:
            fps: Frames per second for the video stream. Defaults to 30.

        Returns:
            Observable: An observable stream of video frames.
        """
        pass

    @abstractmethod
    def stop(self) -> None:
        """Clean up resources and stop the robot.

        This method should handle all necessary cleanup when shutting down
        the robot connection, including stopping any ongoing movements.
        """
        pass
