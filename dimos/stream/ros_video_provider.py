from reactivex import Subject, Observable
from reactivex import operators as ops
from reactivex.scheduler import ThreadPoolScheduler
import multiprocessing
import logging
import time

from dimos.stream.video_provider import AbstractVideoProvider

logging.basicConfig(level=logging.INFO)

# Create thread pool scheduler
pool_scheduler = ThreadPoolScheduler(multiprocessing.cpu_count())

class ROSVideoProvider(AbstractVideoProvider):
    """Video provider that uses a Subject to broadcast frames pushed by ROS."""

    def __init__(self, dev_name: str = "ros_video"):
        super().__init__(dev_name)
        self.logger = logging.getLogger(dev_name)
        self._subject = Subject()
        self._last_frame_time = None
        print("ROSVideoProvider initialized")

    def push_data(self, frame):
        """Push a new frame into the provider."""
        try:
            current_time = time.time()
            if self._last_frame_time:
                frame_interval = current_time - self._last_frame_time
                print(f"Frame interval: {frame_interval:.3f}s ({1/frame_interval:.1f} FPS)")
            self._last_frame_time = current_time
            
            print(f"Pushing frame type: {type(frame)}")
            self._subject.on_next(frame)
            print("Frame pushed")
        except Exception as e:
            print(f"Push error: {e}")

    def capture_video_as_observable(self, fps: int = 30) -> Observable:
        """Return an observable of frames."""
        print(f"Creating observable with {fps} FPS rate limiting")
        return self._subject.pipe(
            ops.share()
        )
