from datetime import timedelta
import time
import cv2
import reactivex as rx
from reactivex import operators as ops
from reactivex.observable import Observable
from reactivex.disposable import CompositeDisposable
from threading import Lock
import logging
from abc import ABC, abstractmethod

logging.basicConfig(level=logging.INFO)

class AbstractVideoProvider(ABC):
    """Abstract base class for video providers managing video capture resources."""

    def __init__(self, dev_name: str = "NA"):
        """Initializes the video provider with a device name.

        Args:
            dev_name: The name of the device. Defaults to "NA".
        """
        self.dev_name = dev_name
        self.disposables = CompositeDisposable()

    @abstractmethod
    def capture_video_as_observable(self, fps: int) -> Observable:
        """Abstract method to create an observable from video capture.

        Args:
            fps: Frames per second to emit.

        Returns:
            Observable: An observable emitting frames at the specified rate.
        """
        pass

    def dispose_all(self):
        """Disposes of all active subscriptions managed by this provider."""
        if self.disposables:
            self.disposables.dispose()
        else:
            logging.info("No disposables to dispose.")

    def __del__(self):
        """Destructor to ensure resources are cleaned up if not explicitly disposed."""
        self.dispose_all()

class VideoProvider(AbstractVideoProvider):
    """Video provider implementation for capturing video as an observable."""

    def __init__(self, dev_name: str, video_source: str = "/app/assets/video-f30-480p.mp4"):
        """Initializes the video provider with a device name and video source.

        Args:
            dev_name: The name of the device.
            video_source: The path to the video source. Defaults to a sample video.
        """
        super().__init__(dev_name)
        self.video_source = video_source
        self.cap = None
        self.lock = Lock()

    def _initialize_capture(self):
        """Initializes the video capture object if not already initialized."""
        if self.cap is None or not self.cap.isOpened():
            if self.cap:
                self.cap.release()
                logging.info("Released previous capture")
            self.cap = cv2.VideoCapture(self.video_source)
            if self.cap is None or not self.cap.isOpened():
                logging.error(f"Failed to open video source: {self.video_source}")
                raise Exception(f"Failed to open video source: {self.video_source}")
            logging.info("Opened new capture")

    def capture_video_as_observable(self, realtime: bool = True, fps: int = 30) -> Observable:
        """Creates an observable from video capture that emits frames at specified FPS or the video's native FPS.

        Args:
            realtime: If True, use the video's native FPS. Defaults to True.
            fps: Frames per second to emit. Defaults to 30fps. Only used if realtime is False and the video's native FPS is not available.

        Returns:
            Observable: An observable emitting frames at the specified or native rate.
        """
        def emit_frames(observer, scheduler):
            try:
                self._initialize_capture()
                
                # Determine the FPS to use
                local_fps = fps
                if realtime:
                    native_fps = self.cap.get(cv2.CAP_PROP_FPS)
                    if native_fps > 0:
                        local_fps = native_fps
                    else:
                        logging.warning("Native FPS not available, defaulting to specified FPS")
                
                frame_interval = 1.0 / local_fps
                frame_time = time.monotonic()

                while self.cap.isOpened():
                    with self.lock:  # Thread-safe access
                        ret, frame = self.cap.read()
                    
                    if not ret:
                        logging.warning("Failed to read frame, resetting capture position")
                        with self.lock:
                            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Loop video
                        continue

                    # Control frame rate
                    now = time.monotonic()
                    next_frame_time = frame_time + frame_interval
                    sleep_time = next_frame_time - now
                    
                    if sleep_time > 0:
                        time.sleep(sleep_time)
                    
                    observer.on_next(frame)
                    frame_time = next_frame_time

            except Exception as e:
                logging.error(f"Error during frame emission: {e}")
                observer.on_error(e)
            finally:
                with self.lock:
                    if self.cap and self.cap.isOpened():
                        self.cap.release()
                        logging.info("Capture released")
                observer.on_completed()

        return rx.create(emit_frames).pipe(
            ops.share()
        )

    def dispose_all(self):
        """Disposes of all resources."""
        with self.lock:
            if self.cap and self.cap.isOpened():
                self.cap.release()
                logging.info("Capture released in dispose_all")
        super().dispose_all()

    def __del__(self):
        """Destructor to ensure resources are cleaned up if not explicitly disposed."""
        self.dispose_all()
