from datetime import timedelta
import cv2
import numpy as np
import os
from reactivex import Observable
from reactivex import operators as ops
from typing import Callable, Tuple, Optional

from dimos.stream.frame_processor import FrameProcessor

class VideoOperators:
    """Collection of video processing operators for reactive video streams."""
    
    @staticmethod
    def with_fps_sampling(
        fps: int = 25,
        *,
        sample_interval: Optional[timedelta] = None,
        use_latest: bool = True
    ) -> Callable[[Observable], Observable]:
        """Creates an operator that samples frames at a specified rate.

        Creates a transformation operator that samples frames either by taking
        the latest frame or the first frame in each interval. Provides frame
        rate control through time-based selection.

        Args:
            fps: Desired frames per second, defaults to 25 FPS.
                Ignored if sample_interval is provided.
            sample_interval: Optional explicit interval between samples.
                If provided, overrides the fps parameter.
            use_latest: If True, uses the latest frame in interval.
                If False, uses the first frame. Defaults to True.

        Returns:
            A function that transforms an Observable[np.ndarray] stream to a sampled 
            Observable[np.ndarray] stream with controlled frame rate.

        Raises:
            ValueError: If fps is not positive or sample_interval is negative.
            TypeError: If sample_interval is provided but not a timedelta object.

        Examples:
            Sample latest frame at 30 FPS (good for real-time):
                >>> video_stream.pipe(
                ...     VideoOperators.with_fps_sampling(fps=30)
                ... )

            Sample first frame with custom interval (good for consistent timing):
                >>> video_stream.pipe(
                ...     VideoOperators.with_fps_sampling(
                ...         sample_interval=timedelta(milliseconds=40),
                ...         use_latest=False
                ...     )
                ... )

        Note:
            This operator helps manage high-speed video streams through time-based
            frame selection. It reduces the frame rate by selecting frames at 
            specified intervals.

            When use_latest=True:
                - Uses sampling to select the most recent frame at fixed intervals
                - Discards intermediate frames, keeping only the latest
                - Best for real-time video where latest frame is most relevant
                - Uses ops.sample internally

            When use_latest=False:
                - Uses throttling to select the first frame in each interval
                - Ignores subsequent frames until next interval
                - Best for scenarios where you want consistent frame timing
                - Uses ops.throttle_first internally

            This is an approropriate solution for managing video frame rates and
            memory usage in many scenarios.
        """
        if sample_interval is None:
            if fps <= 0:
                raise ValueError("FPS must be positive")
            sample_interval = timedelta(microseconds=int(1_000_000 / fps))

        def _operator(source: Observable) -> Observable:
            return source.pipe(
                ops.sample(sample_interval) if use_latest else ops.throttle_first(sample_interval)
            )
        return _operator

    @staticmethod
    def with_jpeg_export(
        frame_processor: 'FrameProcessor',
        save_limit: int = 100,
        suffix: str = ""
    ) -> Callable[[Observable], Observable]:
        """Creates an operator that saves video frames as JPEG files.

        Creates a transformation operator that saves each frame from the video
        stream as a JPEG file while passing the frame through unchanged.

        Args:
            frame_processor: FrameProcessor instance that handles the JPEG export
                operations and maintains file count.
            save_limit: Maximum number of frames to save before stopping.
                Defaults to 100. Set to 0 for unlimited saves.
            suffix: Optional string to append to filename before index.
                Example: "raw" creates "1_raw.jpg".
                Defaults to empty string.

        Returns:
            A function that transforms an Observable of frames into another
            Observable of the same frames, with side effect of saving JPEGs.

        Raises:
            ValueError: If save_limit is negative.
            TypeError: If frame_processor is not a FrameProcessor instance.

        Example:
            >>> video_stream.pipe(
            ...     VideoOperators.with_jpeg_export(processor, suffix="raw")
            ... )
        """
        def _operator(source: Observable) -> Observable:
            return source.pipe(
                ops.map(lambda frame: frame_processor.export_to_jpeg(frame, save_limit, suffix))
            )
        return _operator
    
    @staticmethod
    def with_optical_flow_filtering(
        threshold: float = 1.0
    ) -> Callable[[Observable], Observable]:
        """Creates an operator that filters optical flow frames by relevancy score.

        Filters a stream of optical flow results (frame, relevancy_score) tuples,
        passing through only frames that meet the relevancy threshold.

        Args:
            threshold: Minimum relevancy score required for frames to pass through.
                Defaults to 1.0. Higher values mean more motion required.

        Returns:
            A function that transforms an Observable of (frame, score) tuples
            into an Observable of frames that meet the threshold.

        Raises:
            ValueError: If threshold is negative.
            TypeError: If input stream items are not (frame, float) tuples.

        Examples:
            Basic filtering:
                >>> optical_flow_stream.pipe(
                ...     VideoOperators.with_optical_flow_filtering(threshold=1.0)
                ... )

            With custom threshold:
                >>> optical_flow_stream.pipe(
                ...     VideoOperators.with_optical_flow_filtering(threshold=2.5)
                ... )

        Note:
            Input stream should contain tuples of (frame, relevancy_score) where
            frame is a numpy array and relevancy_score is a float or None.
            None scores are filtered out.
        """
        return lambda source: source.pipe(
            ops.filter(lambda result: result[1] is not None),
            ops.filter(lambda result: result[1] > threshold),
            ops.map(lambda result: result[0])
        )

