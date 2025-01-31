from datetime import timedelta
import cv2
import numpy as np
import os
from reactivex import Observable
from reactivex import operators as ops
from typing import Callable, Tuple, Optional

# TODO: Reorganize, filenaming - Consider merger with VideoOperators class
class FrameProcessor:
    def __init__(self, output_dir='/app/assets/frames', delete_on_init=False):
        """Initializes the FrameProcessor.

        Sets up the output directory for frame storage and optionally cleans up
        existing JPG files.

        Args:
            output_dir: Directory path for storing processed frames.
                Defaults to '/app/assets/frames'.
            delete_on_init: If True, deletes all existing JPG files in output_dir.
                Defaults to False.

        Raises:
            OSError: If directory creation fails or if file deletion fails.
            PermissionError: If lacking permissions for directory/file operations.
        """
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)

        if delete_on_init:
            try:
                jpg_files = [f for f in os.listdir(self.output_dir) 
                           if f.lower().endswith('.jpg')]
                for file in jpg_files:
                    file_path = os.path.join(self.output_dir, file)
                    os.remove(file_path)
                print(f"Cleaned up {len(jpg_files)} existing JPG files from {self.output_dir}")
            except Exception as e:
                print(f"Error cleaning up JPG files: {e}")
                raise
        
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
        if self.image_count > save_limit and save_limit != 0:
            return frame
        
        # filepath = os.path.join(self.output_dir, f'{suffix}_image_{self.image_count}.jpg')
        filepath = os.path.join(self.output_dir, f'{self.image_count}_{suffix}.jpg')
        cv2.imwrite(filepath, frame)
        self.image_count += 1
        return frame

    def compute_optical_flow(
        self,
        acc: Tuple[np.ndarray, np.ndarray, Optional[float]],
        current_frame: np.ndarray,
        compute_relevancy: bool = True
    ) -> Tuple[np.ndarray, np.ndarray, Optional[float]]:
        """Computes optical flow between consecutive frames.

        Uses the Farneback algorithm to compute dense optical flow between the
        previous and current frame. Optionally calculates a relevancy score
        based on the mean magnitude of motion vectors.

        Args:
            acc: Accumulator tuple containing:
                prev_frame: Previous video frame (np.ndarray)
                prev_flow: Previous optical flow (np.ndarray)
                prev_relevancy: Previous relevancy score (float or None)
            current_frame: Current video frame as BGR image (np.ndarray)
            compute_relevancy: If True, calculates mean magnitude of flow vectors.
                Defaults to True.

        Returns:
            A tuple containing:
                current_frame: Current frame for next iteration
                flow: Computed optical flow array or None if first frame
                relevancy: Mean magnitude of flow vectors or None if not computed

        Raises:
            ValueError: If input frames have invalid dimensions or types.
            TypeError: If acc is not a tuple of correct types.
        """
        prev_frame, prev_flow, prev_relevancy = acc

        if prev_frame is None:
            return (current_frame, None, None)

        # Convert frames to grayscale
        gray_current = self.to_grayscale(current_frame)
        gray_prev = self.to_grayscale(prev_frame)

        # Compute optical flow
        flow = cv2.calcOpticalFlowFarneback(gray_prev, gray_current, None, 0.5, 3, 15, 3, 5, 1.2, 0)

        # Relevancy calulation (average magnitude of flow vectors)
        relevancy = None
        if compute_relevancy:
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

    def process_stream_optical_flow(
        self, 
        frame_stream: Observable
    ) -> Observable:
        """Processes video stream to compute and visualize optical flow.

        Computes optical flow between consecutive frames and generates a color-coded
        visualization where hue represents flow direction and intensity represents
        flow magnitude. This method optimizes performance by disabling relevancy
        computation.

        Args:
            frame_stream: An Observable emitting video frames as numpy arrays.
                Each frame should be in BGR format with shape (height, width, 3).

        Returns:
            An Observable emitting visualized optical flow frames as BGR images
            (np.ndarray). Hue indicates flow direction, intensity shows magnitude.

        Raises:
            TypeError: If frame_stream is not an Observable.
            ValueError: If frames have invalid dimensions or format.

        Note:
            Flow visualization uses HSV color mapping where:
            - Hue: Direction of motion (0-360 degrees)
            - Saturation: Fixed at 255
            - Value: Magnitude of motion (0-255)

        Examples:
            >>> flow_stream = processor.process_stream_optical_flow(frame_stream)
            >>> flow_stream.subscribe(lambda flow: cv2.imshow('Flow', flow))
        """
        return frame_stream.pipe(
            ops.scan(
                lambda acc, frame: self.compute_optical_flow(acc, frame, compute_relevancy=False),
                (None, None, None)
            ),
            ops.map(lambda result: result[1]),  # Extract flow component
            ops.filter(lambda flow: flow is not None),
            ops.map(self.visualize_flow),
        )
    
    def process_stream_optical_flow_with_relevancy(
        self, 
        frame_stream: Observable
    ) -> Observable:
        """Processes video stream to compute optical flow with movement relevancy.

        Applies optical flow computation to each frame and returns both the 
        visualized flow and a relevancy score indicating the amount of movement.
        The relevancy score is calculated as the mean magnitude of flow vectors.
        This method includes relevancy computation for motion detection.

        Args:
            frame_stream: An Observable emitting video frames as numpy arrays.
                Each frame should be in BGR format with shape (height, width, 3).

        Returns:
            An Observable emitting tuples of (visualized_flow, relevancy_score):
                visualized_flow: np.ndarray, BGR image visualizing optical flow
                relevancy_score: float, mean magnitude of flow vectors,
                    higher values indicate more motion

        Raises:
            TypeError: If frame_stream is not an Observable.
            ValueError: If frames have invalid dimensions or format.

        Examples:
            >>> flow_stream = processor.process_stream_optical_flow_with_relevancy(
            ...     frame_stream
            ... )
            >>> flow_stream.subscribe(
            ...     lambda result: print(f"Motion score: {result[1]}")
            ... )

        Note:
            Relevancy scores are computed using mean magnitude of flow vectors.
            Higher scores indicate more movement in the frame.
        """
        return frame_stream.pipe(
            ops.scan(
                lambda acc, frame: self.compute_optical_flow(acc, frame, compute_relevancy=True),
                (None, None, None)
            ),
            # Result is (current_frame, flow, relevancy)
            ops.filter(lambda result: result[1] is not None),  # Filter out None flows
            ops.map(lambda result: (
                self.visualize_flow(result[1]),  # Visualized flow
                result[2]                        # Relevancy score
            )),
            ops.filter(lambda result: result[0] is not None)  # Ensure valid visualization
        )

    def process_stream_with_jpeg_export(
        self, 
        frame_stream: Observable,
        suffix: str = ""
    ) -> Observable:
        """Processes stream by saving frames as JPEGs while passing them through.

        Saves each frame from the stream as a JPEG file and passes the frame
        downstream unmodified. Files are saved sequentially with optional suffix
        in the configured output directory (self.output_dir).

        Args:
            frame_stream: An Observable emitting video frames as numpy arrays.
                Each frame should be in BGR format with shape (height, width, 3).
            suffix: Optional string to append to filename before index.
                Defaults to empty string. Example: "optical" -> "optical_image_1.jpg"

        Returns:
            An Observable emitting the same frames that were saved. Returns None
            for frames that could not be saved due to format issues or save_limit.

        Raises:
            TypeError: If frame_stream is not an Observable.
            ValueError: If frames have invalid format or output directory
                is not writable.
            OSError: If there are file system permission issues.

        Note:
            Frames are saved as '{suffix}_image_{index}.jpg' where index
            increments for each saved frame. Saving stops after reaching
            the configured save_limit (default: 100).
        """
        return frame_stream.pipe(
            ops.map(lambda frame: self.export_to_jpeg(frame, suffix=suffix)),
        )
