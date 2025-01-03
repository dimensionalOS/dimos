from time import sleep
import cv2
import reactivex as rx
from reactivex import operators as ops
from reactivex.disposable import CompositeDisposable
from reactivex.scheduler import ThreadPoolScheduler, CurrentThreadScheduler


class MediaProvider:
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
class VideoProviderExample(MediaProvider):
    def __init__(self, dev_name: str, video_source:str="/app/dimos/assets/video-f30-480p.mp4"):
        super().__init__(dev_name)
        self.video_source = video_source
        # self.scheduler = ThreadPoolScheduler(1) # CurrentThreadScheduler
        self.cap = None

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

    def video_capture_to_observable(self):
        cap = self.get_capture()

        def emit_frames(observer, scheduler):
            try:
                while cap.isOpened():
                    ret, frame = cap.read()
                    if ret:
                        observer.on_next(frame)
                    else:
                        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # If loading from a video, loop it
                        continue
                        # observer.on_completed()
                        # break
            except Exception as e:
                observer.on_error(e)
            finally:
                cap.release()

        return rx.create(emit_frames).pipe(
            # ops.observe_on(self.scheduler),  #
            # ops.subscribe_on(self.scheduler),  #
            ops.share()
        )
    
    def dispose_all(self):
        """Disposes of all resources."""
        if self.cap and self.cap.isOpened():
            self.cap.release()
        super().dispose_all()

    def __del__(self):
        """Destructor to ensure resources are cleaned up if not explicitly disposed."""
        self.dispose_all()

    




# class VideoProviderExample(MediaProvider):
#     def __init__(self, dev_name: str, provider_type:str="Video", video_source:str="/app/dimos/assets/video-f30-480p.mp4"):
#         super().__init__(dev_name)
#         self.provider_type = provider_type
#         self.video_source = video_source

#     def video_capture_to_observable(self, cap):
#         """Creates an observable from a video capture source."""
#         def on_subscribe(observer, scheduler=None):
            
#             def read_frame(): # scheduler, state):
#                 while True:
#                     try:
#                         ret, frame = cap.read()
#                         if ret:
#                             observer.on_next(frame)
#                             # cv2.waitKey(1)
#                             # Reschedule reading the next frame
#                             #if scheduler:
#                             #scheduler.schedule(read_frame)
#                         else:
#                             cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
#                             continue
#                             # observer.on_completed()
#                             # cap.release()
#                     except Exception as e:
#                         observer.on_error(e)
#                         cap.release()
            
#             # Schedule the first frame read
#             #if scheduler:
#             #scheduler.schedule(read_frame)
#             #else:
#             read_frame()  # Direct call on the same thread
#         return rx.create(on_subscribe).pipe(
#             ops.publish(),  # Convert the observable from cold to hot
#             ops.ref_count()  # Start emitting when the first subscriber subscribes and stop when the last unsubscribes
#         )

#     def get_capture(self): # , video_source="/app/dimos/assets/video-f30-480p.mp4"):
#         # video_source = root_dir + '' # "udp://0.0.0.0:23000" # "/dev/video0"
#         cap = cv2.VideoCapture(self.video_source)
#         print("Opening video source")
#         print(f"Source: {self.video_source}")
#         if not cap.isOpened():
#             print("Failed to open video source")
#             exit()
#         print("Opened video source")
#         return cap
    
#     def video_capture_to_observable(self): # , video_source="/app/dimos/assets/video-f30-480p.mp4"):
#         cap = self.get_capture()
#         return self.video_capture_to_observable(cap)

#     # def dispose():
#     #     self.disposeables.dispose()
#             # from time import sleep
#             # while True:
#             #     sleep(1)
#         # if cv2.waitKey(1) & 0xFF == ord('q'):
#         #     # disposable.dispose()
#         #     disposable_flask.dispose()
#         #     disposable_oai.dispose()
#         #     for _ in disposablables:
#         #         disposablables.dispose()

#         #     cv2.destroyAllWindows()
#         #     break
