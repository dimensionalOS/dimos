from datetime import timedelta
import time
import cv2
import reactivex as rx
from reactivex import operators as ops
from reactivex.observable import Observable
from reactivex.disposable import CompositeDisposable
from reactivex.scheduler import ThreadPoolScheduler, CurrentThreadScheduler
from threading import Lock

from dimos.types.videostream import FrameProcessor

class VideoProvider:
    def __init__(self, dev_name:str="NA"):
        self.dev_name = dev_name
        self.disposables = CompositeDisposable()

    def dispose_all(self):
        """Disposes of all active subscriptions managed by this agent."""
        if self.disposables:
            self.disposables.dispose()
        else:
            print("No disposables to dispose.")


# TODO: Test threading concurrency and instanciation more fully
class VideoProviderExample(VideoProvider):
    def __init__(self, dev_name: str, video_source:str="/app/assets/video-f30-480p.mp4"):
        super().__init__(dev_name)
        self.video_source = video_source
        self.cap = None
        self.lock = Lock()  # Ensure thread-safe access

    def get_capture(self):
        """Ensure that the capture device is correctly initialized and open."""
        if self.cap is None or not self.cap.isOpened():
            if self.cap:
                self.cap.release()
                print("Released Capture")
            self.cap = cv2.VideoCapture(self.video_source)
            print("Opened Capture")
            if not self.cap.isOpened():
                raise Exception("Failed to open video source")
        return self.cap

    # def video_capture_to_observable(self):
    #     cap = self.get_capture()

    #     def emit_frames(observer, scheduler):
    #         try:
    #             while cap.isOpened():
    #                 with self.lock:  # Ensure thread-safe access to the capture
    #                     ret, frame = cap.read()
    #                 if ret:
    #                     observer.on_next(frame)
    #                 else:
    #                     with self.lock:
    #                         cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # If loading from a video, loop it
    #                     continue
    #                     # observer.on_completed()
    #                     # break
    #         except Exception as e:
    #             observer.on_error(e)
    #         finally:
    #             with self.lock:
    #                 if cap.isOpened():
    #                     cap.release()
    #                 print("Capture released")
    #             observer.on_completed()

    #     return rx.create(emit_frames).pipe(
    #         ops.share()
    #     )

    def video_capture_to_observable(self, fps=30):
        """Creates an Observable from video capture that emits at specified FPS.

        Args:
            fps: Frames per second to emit. Defaults to 30fps.

        Returns:
            Observable emitting frames at the specified rate.
        """
        cap = self.get_capture()
        frame_interval = 1.0 / fps

        def emit_frames(observer, scheduler):
            try:
                frame_time = time.monotonic()
                while cap.isOpened():
                    with self.lock:  # Thread-safe access
                        ret, frame = cap.read()
                    
                    if ret:
                        # Control frame rate
                        now = time.monotonic()
                        next_frame_time = frame_time + frame_interval
                        sleep_time = next_frame_time - now
                        
                        if sleep_time > 0:
                            time.sleep(sleep_time)
                        
                        observer.on_next(frame)
                        frame_time = next_frame_time
                    else:
                        with self.lock:
                            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Loop video
                        continue
            except Exception as e:
                observer.on_error(e)
            finally:
                with self.lock:
                    if cap.isOpened():
                        cap.release()
                    print("Capture released")
                observer.on_completed()

        return rx.create(emit_frames).pipe(
            ops.share()
        )

    def dispose_all(self):
        """Disposes of all resources."""
        with self.lock:
            if self.cap and self.cap.isOpened():
                self.cap.release()
                print("Capture released in dispose_all")
        super().dispose_all()

    def __del__(self):
        """Destructor to ensure resources are cleaned up if not explicitly disposed."""
        self.dispose_all()
