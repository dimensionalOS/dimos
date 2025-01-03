from datetime import timedelta
import cv2
import numpy as np
import os
from reactivex import Observable
from reactivex import operators as ops

class StreamUtils:
    def limit_emission_rate(frame_stream, time_delta=timedelta(milliseconds=40)):
        return frame_stream.pipe(
            ops.throttle_first(time_delta)
        )


# TODO: Reorganize, filenaming
class FrameProcessor:
    def __init__(self, output_dir='/app/dimos/assets/frames'):
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)
        self.image_count = 0 
        # TODO: Add randomness to jpg folder storage naming. 
        # Will overwrite between sessions.

    def to_grayscale(self, frame):
        if frame is None:
            print("Received None frame for grayscale conversion.")
            return None
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    def edge_detection(self, frame):
        return cv2.Canny(frame, 100, 200)

    def resize(self, frame, scale=0.5):
        return cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

    def export_to_jpeg(self, frame, save_limit=100, suffix=""):
        if frame is None:
            print("Error: Attempted to save a None image.")
            return None
        
        # Check if the image has an acceptable number of channels
        if len(frame.shape) == 3 and frame.shape[2] not in [1, 3, 4]:
            print(f"Error: Frame with shape {frame.shape} has unsupported number of channels.")
            return None

        # If save_limit is not 0, only export a maximum number of frames
        if self.image_count > save_limit:
            return frame
        
        filepath = os.path.join(self.output_dir, f'{suffix}_image_{self.image_count}.jpg')
        cv2.imwrite(filepath, frame)
        self.image_count += 1
        return frame

    def compute_optical_flow(self, acc, current_frame):
        prev_frame, _ = acc  # acc (accumulator) contains the previous frame and its flow (which is ignored here)

        if prev_frame is None:
            # Skip processing for the first frame as there's no previous frame to compare against.
            return (current_frame, None)

        # Convert frames to grayscale (if not already done)
        gray_current = self.to_grayscale(current_frame)
        gray_prev = self.to_grayscale(prev_frame)

        # Compute optical flow
        flow = cv2.calcOpticalFlowFarneback(gray_prev, gray_current, None, 0.5, 3, 15, 3, 5, 1.2, 0)

        # Relevancy calulation (average magnitude of flow vectors)
        mag, _ = cv2.cartToPolar(flow[..., 0], flow[..., 1])
        relevancy = np.mean(mag)

        # Return the current frame as the new previous frame and the processed optical flow, with relevancy score
        return (current_frame, flow, relevancy)

    def visualize_flow(self, flow):
        if flow is None:
            return None
        hsv = np.zeros((flow.shape[0], flow.shape[1], 3), dtype=np.uint8)
        hsv[..., 1] = 255
        mag, ang = cv2.cartToPolar(flow[..., 0], flow[..., 1])
        hsv[..., 0] = ang * 180 / np.pi / 2
        hsv[..., 2] = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX)
        rgb = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        return rgb

    # ==============================

    def process_stream_edge_detection(self, frame_stream):
        return frame_stream.pipe(
            ops.map(self.edge_detection),
        )

    def process_stream_resize(self, frame_stream):
        return frame_stream.pipe(
            ops.map(self.resize),
        )

    def process_stream_to_greyscale(self, frame_stream):
        return frame_stream.pipe(
            ops.map(self.to_grayscale),
        )

    # TODO: Propogate up relevancy score from compute_optical_flow
    def process_stream_optical_flow(self, frame_stream):
        return frame_stream.pipe(
            ops.scan(self.compute_optical_flow, (None, None)),  # Initial value for scan is (None, None)
            ops.map(lambda result: result[1]),  # Extract only the flow part from the tuple
            ops.filter(lambda flow: flow is not None),
            ops.map(self.visualize_flow),
        )

    def process_stream_export_to_jpeg(self, frame_stream, suffix=""):
        return frame_stream.pipe(
            ops.map(lambda frame: self.export_to_jpeg(frame, suffix=suffix)),
        )

class VideoStream:
    def __init__(self, source=0):
        """
        Initialize the video stream from a camera source.
        
        Args:
            source (int or str): Camera index or video file path.
        """
        self.capture = cv2.VideoCapture(source)
        if not self.capture.isOpened():
            raise ValueError(f"Unable to open video source {source}")

    def __iter__(self):
        return self

    def __next__(self):
        ret, frame = self.capture.read()
        if not ret:
            self.capture.release()
            raise StopIteration
        return frame

    def release(self):
        self.capture.release()