from typing import Optional

from reactivex import Observable

from dimos.perception.person_tracker import PersonTrackingStream
from dimos.perception.object_tracker import ObjectTrackingStream


class RobotStreamManager:
    """Manage optional perception streams for a robot.

    This class lazily creates and stores commonly used perception streams such as
    person or object tracking. It requires a video stream and camera parameters
    when enabling the streams.
    """

    def __init__(
        self,
        *,
        video_stream: Optional[Observable] = None,
        camera_intrinsics: Optional[list] = None,
        camera_pitch: float = 0.0,
        camera_height: float = 0.0,
    ) -> None:
        self._video_stream = video_stream
        self._camera_intrinsics = camera_intrinsics
        self._camera_pitch = camera_pitch
        self._camera_height = camera_height

        self._person_tracker: Optional[PersonTrackingStream] = None
        self._object_tracker: Optional[ObjectTrackingStream] = None
        self.person_tracking_stream: Optional[Observable] = None
        self.object_tracking_stream: Optional[Observable] = None

    @property
    def video_stream(self) -> Optional[Observable]:
        return self._video_stream

    @video_stream.setter
    def video_stream(self, stream: Observable) -> None:
        self._video_stream = stream

    def enable_person_tracking(self, *, video_stream: Optional[Observable] = None) -> Observable:
        """Create or return the person tracking stream."""
        if video_stream is not None:
            self._video_stream = video_stream
        if self.person_tracking_stream is None:
            if self._video_stream is None:
                raise ValueError("Video stream required for person tracking")
            self._person_tracker = PersonTrackingStream(
                camera_intrinsics=self._camera_intrinsics,
                camera_pitch=self._camera_pitch,
                camera_height=self._camera_height,
            )
            self.person_tracking_stream = self._person_tracker.create_stream(self._video_stream)
        return self.person_tracking_stream

    def enable_object_tracking(self, *, video_stream: Optional[Observable] = None) -> Observable:
        """Create or return the object tracking stream."""
        if video_stream is not None:
            self._video_stream = video_stream
        if self.object_tracking_stream is None:
            if self._video_stream is None:
                raise ValueError("Video stream required for object tracking")
            self._object_tracker = ObjectTrackingStream(
                camera_intrinsics=self._camera_intrinsics,
                camera_pitch=self._camera_pitch,
                camera_height=self._camera_height,
            )
            self.object_tracking_stream = self._object_tracker.create_stream(self._video_stream)
        return self.object_tracking_stream

    def cleanup(self) -> None:
        if self._person_tracker:
            self._person_tracker.cleanup()
        if self._object_tracker:
            self._object_tracker.cleanup()
